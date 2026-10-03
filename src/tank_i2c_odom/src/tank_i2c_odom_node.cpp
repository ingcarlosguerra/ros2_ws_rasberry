#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>


class TankI2COdom : public rclcpp::Node
{
public:

    TankI2COdom()
        : Node("tank_i2c_odom")
    {
        // =========================================================
        // PARAMETROS
        // =========================================================

        declare_parameter<std::string>(
            "i2c_device",
            "/dev/i2c-1");

        declare_parameter<int>(
            "i2c_address",
            0x34);

        declare_parameter<double>(
            "wheel_diameter",
            0.05);

        declare_parameter<double>(
            "track_width",
            0.10);

        declare_parameter<double>(
            "encoder_ticks_per_revolution",
            1980.0);

        declare_parameter<int>(
            "left_encoder_sign",
            1);

        declare_parameter<int>(
            "right_encoder_sign",
            1);

        declare_parameter<double>(
            "update_rate",
            50.0);

        declare_parameter<std::string>(
            "odom_frame",
            "odom");

        declare_parameter<std::string>(
            "base_frame",
            "base_link");

        declare_parameter<bool>(
            "publish_tf",
            true);

        declare_parameter<bool>(
            "debug_encoders",
            true);

        // =========================================================
        // LEER PARAMETROS
        // =========================================================

        i2c_device_ =
            get_parameter(
                "i2c_device").as_string();

        i2c_address_ =
            get_parameter(
                "i2c_address").as_int();

        wheel_diameter_ =
            get_parameter(
                "wheel_diameter").as_double();

        track_width_ =
            get_parameter(
                "track_width").as_double();

        encoder_ticks_per_revolution_ =
            get_parameter(
                "encoder_ticks_per_revolution").as_double();

        left_encoder_sign_ =
            get_parameter(
                "left_encoder_sign").as_int();

        right_encoder_sign_ =
            get_parameter(
                "right_encoder_sign").as_int();

        update_rate_ =
            get_parameter(
                "update_rate").as_double();

        odom_frame_ =
            get_parameter(
                "odom_frame").as_string();

        base_frame_ =
            get_parameter(
                "base_frame").as_string();

        publish_tf_ =
            get_parameter(
                "publish_tf").as_bool();

        debug_encoders_ =
            get_parameter(
                "debug_encoders").as_bool();

        // =========================================================
        // VALIDACIONES
        // =========================================================

        if (wheel_diameter_ <= 0.0)
        {
            throw std::runtime_error(
                "wheel_diameter debe ser > 0");
        }

        if (track_width_ <= 0.0)
        {
            throw std::runtime_error(
                "track_width debe ser > 0");
        }

        if (encoder_ticks_per_revolution_ <= 0.0)
        {
            throw std::runtime_error(
                "encoder_ticks_per_revolution debe ser > 0");
        }

        if (update_rate_ <= 0.0)
        {
            throw std::runtime_error(
                "update_rate debe ser > 0");
        }

        if (left_encoder_sign_ != 1 &&
            left_encoder_sign_ != -1)
        {
            RCLCPP_WARN(
                get_logger(),
                "left_encoder_sign invalido. "
                "Usando 1.");

            left_encoder_sign_ = 1;
        }

        if (right_encoder_sign_ != 1 &&
            right_encoder_sign_ != -1)
        {
            RCLCPP_WARN(
                get_logger(),
                "right_encoder_sign invalido. "
                "Usando 1.");

            right_encoder_sign_ = 1;
        }

        // =========================================================
        // ABRIR I2C
        // =========================================================

        if (!open_i2c())
        {
            RCLCPP_FATAL(
                get_logger(),
                "No se pudo abrir I2C");

            throw std::runtime_error(
                "No se pudo abrir I2C");
        }

        // =========================================================
        // INFORMACION
        // =========================================================

        const double wheel_circumference =
            M_PI * wheel_diameter_;

        RCLCPP_INFO(
            get_logger(),
            "==========================================");

        RCLCPP_INFO(
            get_logger(),
            "       TANK I2C ODOMETRY");

        RCLCPP_INFO(
            get_logger(),
            "==========================================");

        RCLCPP_INFO(
            get_logger(),
            "I2C device: %s",
            i2c_device_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "I2C address: 0x%02X",
            i2c_address_);

        RCLCPP_INFO(
            get_logger(),
            "Wheel diameter: %.4f m",
            wheel_diameter_);

        RCLCPP_INFO(
            get_logger(),
            "Wheel circumference: %.6f m",
            wheel_circumference);

        RCLCPP_INFO(
            get_logger(),
            "Track width: %.4f m",
            track_width_);

        RCLCPP_INFO(
            get_logger(),
            "Encoder ticks/revolution: %.2f",
            encoder_ticks_per_revolution_);

        RCLCPP_INFO(
            get_logger(),
            "Left encoder sign: %d",
            left_encoder_sign_);

        RCLCPP_INFO(
            get_logger(),
            "Right encoder sign: %d",
            right_encoder_sign_);

        RCLCPP_INFO(
            get_logger(),
            "Update rate: %.1f Hz",
            update_rate_);

        RCLCPP_INFO(
            get_logger(),
            "==========================================");

        // =========================================================
        // ODOM
        // =========================================================

        odom_pub_ =
            create_publisher<nav_msgs::msg::Odometry>(
                "/odom",
                10);

        // =========================================================
        // TF
        // =========================================================

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(
                *this);

        // =========================================================
        // PRIMERA LECTURA
        // =========================================================

        int32_t encoder_counts[4] = {};

        if (!read_encoder_counts(encoder_counts))
        {
            RCLCPP_FATAL(
                get_logger(),
                "No se pudo leer el encoder inicialmente");

            throw std::runtime_error(
                "No se pudo leer encoder");
        }

        previous_left_encoder_ =
            encoder_counts[0];

        previous_right_encoder_ =
            encoder_counts[1];

        initialized_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Encoder inicial M1=%d M2=%d",
            encoder_counts[0],
            encoder_counts[1]);

        // =========================================================
        // TIMER
        // =========================================================

        auto period =
            std::chrono::duration<double>(
                1.0 / update_rate_);

        timer_ =
            create_wall_timer(
                std::chrono::duration_cast<
                    std::chrono::milliseconds>(period),
                std::bind(
                    &TankI2COdom::update,
                    this));
    }


    ~TankI2COdom()
    {
        if (fd_ >= 0)
        {
            close(fd_);
        }
    }


private:

    // =============================================================
    // REGISTRO ENCODER
    // =============================================================

    static constexpr uint8_t
        MOTOR_ENCODER_TOTAL_ADDR = 60;


    // =============================================================
    // I2C
    // =============================================================

    int fd_ = -1;

    std::string i2c_device_;

    int i2c_address_;


    // =============================================================
    // ROBOT
    // =============================================================

    double wheel_diameter_;

    double track_width_;

    double encoder_ticks_per_revolution_;


    // =============================================================
    // SIGNOS
    // =============================================================

    int left_encoder_sign_;

    int right_encoder_sign_;


    // =============================================================
    // TIMER
    // =============================================================

    double update_rate_;

    rclcpp::TimerBase::SharedPtr timer_;


    // =============================================================
    // ODOM
    // =============================================================

    rclcpp::Publisher<
        nav_msgs::msg::Odometry>::SharedPtr odom_pub_;


    // =============================================================
    // TF
    // =============================================================

    std::unique_ptr<
        tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string odom_frame_;

    std::string base_frame_;

    bool publish_tf_;


    // =============================================================
    // DEBUG
    // =============================================================

    bool debug_encoders_;


    // =============================================================
    // ENCODER ANTERIOR
    // =============================================================

    int32_t previous_left_encoder_ = 0;

    int32_t previous_right_encoder_ = 0;

    bool initialized_ = false;


    // =============================================================
    // POSE
    // =============================================================

    double x_ = 0.0;

    double y_ = 0.0;

    double theta_ = 0.0;


    // =============================================================
    // VELOCIDADES
    // =============================================================

    double linear_velocity_ = 0.0;

    double angular_velocity_ = 0.0;


    // =============================================================
    // ABRIR I2C
    // =============================================================

    bool open_i2c()
    {
        fd_ =
            open(
                i2c_device_.c_str(),
                O_RDWR);

        if (fd_ < 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Error abriendo %s: %s",
                i2c_device_.c_str(),
                strerror(errno));

            return false;
        }

        if (
            ioctl(
                fd_,
                I2C_SLAVE,
                i2c_address_) < 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Error seleccionando direccion "
                "I2C 0x%02X: %s",
                i2c_address_,
                strerror(errno));

            close(fd_);

            fd_ = -1;

            return false;
        }

        return true;
    }


    // =============================================================
    // LEER ENCODERS
    //
    // Registro 60 = 0x3C
    //
    // El controlador devuelve:
    //
    // M1 = bytes 0-3
    // M2 = bytes 4-7
    // M3 = bytes 8-11
    // M4 = bytes 12-15
    //
    // Little endian
    // =============================================================

    bool read_encoder_counts(
        int32_t counts[4])
    {
        uint8_t command =
            MOTOR_ENCODER_TOTAL_ADDR;

        uint8_t buffer[16] = {};

        // ---------------------------------------------------------
        // Seleccionar registro 60
        // ---------------------------------------------------------

        ssize_t written =
            write(
                fd_,
                &command,
                1);

        if (written != 1)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Error escribiendo registro encoder: %s",
                strerror(errno));

            return false;
        }

        // ---------------------------------------------------------
        // Leer 16 bytes
        // ---------------------------------------------------------

        ssize_t received =
            read(
                fd_,
                buffer,
                sizeof(buffer));

        if (received != sizeof(buffer))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Error leyendo encoder. "
                "Recibidos %ld de 16 bytes. %s",
                received,
                strerror(errno));

            return false;
        }

        // ---------------------------------------------------------
        // Convertir 4 x uint32 little endian
        // a int32
        // ---------------------------------------------------------

        for (int i = 0; i < 4; i++)
        {
            uint32_t value =
                static_cast<uint32_t>(
                    buffer[i * 4]);

            value |=
                static_cast<uint32_t>(
                    buffer[i * 4 + 1]) << 8;

            value |=
                static_cast<uint32_t>(
                    buffer[i * 4 + 2]) << 16;

            value |=
                static_cast<uint32_t>(
                    buffer[i * 4 + 3]) << 24;

            std::memcpy(
                &counts[i],
                &value,
                sizeof(int32_t));
        }

        return true;
    }


    // =============================================================
    // DELTA DE ENCODER
    //
    // Maneja overflow de int32.
    // =============================================================

    int64_t encoder_delta(
        int32_t current,
        int32_t previous)
    {
        int64_t delta =
            static_cast<int64_t>(current) -
            static_cast<int64_t>(previous);

        const int64_t encoder_range =
            4294967296LL;

        if (delta >
            2147483647LL)
        {
            delta -=
                encoder_range;
        }

        else if (
            delta <
            -2147483648LL)
        {
            delta +=
                encoder_range;
        }

        return delta;
    }


    // =============================================================
    // ACTUALIZAR ODOM
    // =============================================================

    void update()
    {
        int32_t counts[4] = {};

        if (!read_encoder_counts(counts))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "No se pudieron leer los encoders");

            return;
        }

        // =========================================================
        // DELTA TICKS
        // =========================================================

        int64_t delta_left =
            encoder_delta(
                counts[0],
                previous_left_encoder_);

        int64_t delta_right =
            encoder_delta(
                counts[1],
                previous_right_encoder_);

        previous_left_encoder_ =
            counts[0];

        previous_right_encoder_ =
            counts[1];

        // =========================================================
        // APLICAR SIGNOS
        // =========================================================

        delta_left *=
            left_encoder_sign_;

        delta_right *=
            right_encoder_sign_;

        // =========================================================
        // TICKS -> REVOLUCIONES
        // =========================================================

        double left_revolutions =
            static_cast<double>(
                delta_left) /
            encoder_ticks_per_revolution_;

        double right_revolutions =
            static_cast<double>(
                delta_right) /
            encoder_ticks_per_revolution_;

        // =========================================================
        // REVOLUCIONES -> DISTANCIA
        // =========================================================

        double wheel_circumference =
            M_PI *
            wheel_diameter_;

        double left_distance =
            left_revolutions *
            wheel_circumference;

        double right_distance =
            right_revolutions *
            wheel_circumference;

        // =========================================================
        // CINEMATICA DIFFERENTIAL / TANK
        //
        // delta_s =
        //      (right + left) / 2
        //
        // delta_theta =
        //      (right - left) / track_width
        // =========================================================

        double delta_distance =
            (right_distance +
             left_distance) / 2.0;

        double delta_theta =
            (right_distance -
             left_distance) /
            track_width_;

        // =========================================================
        // ACTUALIZAR POSE
        //
        // Usamos el angulo medio durante el intervalo.
        // =========================================================

        double theta_mid =
            theta_ +
            delta_theta / 2.0;

        x_ +=
            delta_distance *
            std::cos(theta_mid);

        y_ +=
            delta_distance *
            std::sin(theta_mid);

        theta_ +=
            delta_theta;

        // Mantener theta entre -PI y PI
        theta_ =
            std::atan2(
                std::sin(theta_),
                std::cos(theta_));

        // =========================================================
        // VELOCIDADES
        // =========================================================

        double dt =
            1.0 / update_rate_;

        linear_velocity_ =
            delta_distance /
            dt;

        angular_velocity_ =
            delta_theta /
            dt;

        // =========================================================
        // PUBLICAR
        // =========================================================

        publish_odometry();

        // =========================================================
        // DEBUG
        // =========================================================

        if (debug_encoders_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "ENC M1=%d M2=%d | "
                "dM1=%ld dM2=%ld | "
                "L=%.4f R=%.4f | "
                "x=%.3f y=%.3f th=%.3f",
                counts[0],
                counts[1],
                delta_left,
                delta_right,
                left_distance,
                right_distance,
                x_,
                y_,
                theta_);
        }
    }


    // =============================================================
    // PUBLICAR ODOM + TF
    // =============================================================

    void publish_odometry()
    {
        auto stamp =
            now();

        // =========================================================
        // QUATERNION
        // =========================================================

        tf2::Quaternion quaternion;

        quaternion.setRPY(
            0.0,
            0.0,
            theta_);

        quaternion.normalize();

        // =========================================================
        // MENSAJE ODOM
        // =========================================================

        nav_msgs::msg::Odometry odom;

        odom.header.stamp =
            stamp;

        odom.header.frame_id =
            odom_frame_;

        odom.child_frame_id =
            base_frame_;

        // Pose
        odom.pose.pose.position.x =
            x_;

        odom.pose.pose.position.y =
            y_;

        odom.pose.pose.position.z =
            0.0;

        odom.pose.pose.orientation.x =
            quaternion.x();

        odom.pose.pose.orientation.y =
            quaternion.y();

        odom.pose.pose.orientation.z =
            quaternion.z();

        odom.pose.pose.orientation.w =
            quaternion.w();

        // Velocidad
        odom.twist.twist.linear.x =
            linear_velocity_;

        odom.twist.twist.linear.y =
            0.0;

        odom.twist.twist.linear.z =
            0.0;

        odom.twist.twist.angular.z =
            angular_velocity_;

        // =========================================================
        // COVARIANZAS
        //
        // Valores iniciales conservadores.
        // Se pueden calibrar posteriormente.
        // =========================================================

        odom.pose.covariance[0] =
            0.01;

        odom.pose.covariance[7] =
            0.01;

        odom.pose.covariance[35] =
            0.05;

        odom.twist.covariance[0] =
            0.01;

        odom.twist.covariance[35] =
            0.05;

        odom_pub_->publish(
            odom);

        // =========================================================
        // TF odom -> base_link
        // =========================================================

        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped transform;

            transform.header.stamp =
                stamp;

            transform.header.frame_id =
                odom_frame_;

            transform.child_frame_id =
                base_frame_;

            transform.transform.translation.x =
                x_;

            transform.transform.translation.y =
                y_;

            transform.transform.translation.z =
                0.0;

            transform.transform.rotation.x =
                quaternion.x();

            transform.transform.rotation.y =
                quaternion.y();

            transform.transform.rotation.z =
                quaternion.z();

            transform.transform.rotation.w =
                quaternion.w();

            tf_broadcaster_->sendTransform(
                transform);
        }
    }
};


// =================================================================
// MAIN
// =================================================================

int main(
    int argc,
    char *argv[])
{
    rclcpp::init(
        argc,
        argv);

    try
    {
        auto node =
            std::make_shared<TankI2COdom>();

        rclcpp::spin(node);
    }

    catch (
        const std::exception &e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger(
                "tank_i2c_odom"),
            "%s",
            e.what());
    }

    rclcpp::shutdown();

    return 0;
}
