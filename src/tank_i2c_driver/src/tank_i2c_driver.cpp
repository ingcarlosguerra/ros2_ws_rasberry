#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>


class TankI2CDriver : public rclcpp::Node
{
public:

    TankI2CDriver()
        : Node("tank_i2c_driver")
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
            "max_linear_speed",
            0.30);

        declare_parameter<double>(
            "max_angular_speed",
            6.0);

        declare_parameter<double>(
            "track_width",
            0.10);

        declare_parameter<double>(
            "wheel_diameter",
            0.05);

        declare_parameter<double>(
            "ticks_per_revolution",
            1980.0);

        declare_parameter<int>(
            "max_motor_speed",
            50);

        declare_parameter<int>(
            "left_motor_sign",
            1);

        declare_parameter<int>(
            "right_motor_sign",
            1);

        declare_parameter<int>(
            "left_encoder_sign",
            1);

        declare_parameter<int>(
            "right_encoder_sign",
            1);

        declare_parameter<double>(
            "motor_rate",
            100.0);

        declare_parameter<double>(
            "encoder_rate",
            20.0);

        declare_parameter<bool>(
            "publish_tf",
            true);

        declare_parameter<std::string>(
            "odom_frame",
            "odom");

        declare_parameter<std::string>(
            "base_frame",
            "base_link");

        declare_parameter<double>(
            "watchdog_timeout",
            0.25);


        // =========================================================
        // LEER PARAMETROS
        // =========================================================

        i2c_device_ =
            get_parameter("i2c_device").as_string();

        i2c_address_ =
            get_parameter("i2c_address").as_int();

        max_linear_speed_ =
            get_parameter("max_linear_speed").as_double();

        max_angular_speed_ =
            get_parameter("max_angular_speed").as_double();

        track_width_ =
            get_parameter("track_width").as_double();

        wheel_diameter_ =
            get_parameter("wheel_diameter").as_double();

        ticks_per_revolution_ =
            get_parameter("ticks_per_revolution").as_double();

        max_motor_speed_ =
            get_parameter("max_motor_speed").as_int();

        left_motor_sign_ =
            get_parameter("left_motor_sign").as_int();

        right_motor_sign_ =
            get_parameter("right_motor_sign").as_int();

        left_encoder_sign_ =
            get_parameter("left_encoder_sign").as_int();

        right_encoder_sign_ =
            get_parameter("right_encoder_sign").as_int();

        motor_rate_ =
            get_parameter("motor_rate").as_double();

        encoder_rate_ =
            get_parameter("encoder_rate").as_double();

        publish_tf_ =
            get_parameter("publish_tf").as_bool();

        odom_frame_ =
            get_parameter("odom_frame").as_string();

        base_frame_ =
            get_parameter("base_frame").as_string();

        watchdog_timeout_ =
            get_parameter("watchdog_timeout").as_double();


        // =========================================================
        // VALIDACIONES
        // =========================================================

        if (max_linear_speed_ <= 0.0)
            throw std::runtime_error(
                "max_linear_speed debe ser > 0");

        if (track_width_ <= 0.0)
            throw std::runtime_error(
                "track_width debe ser > 0");

        if (wheel_diameter_ <= 0.0)
            throw std::runtime_error(
                "wheel_diameter debe ser > 0");

        if (ticks_per_revolution_ <= 0.0)
            throw std::runtime_error(
                "ticks_per_revolution debe ser > 0");

        if (max_motor_speed_ <= 0)
            throw std::runtime_error(
                "max_motor_speed debe ser > 0");


        // =========================================================
        // CALCULOS
        // =========================================================

        wheel_circumference_ =
            M_PI * wheel_diameter_;

        meters_per_tick_ =
            wheel_circumference_ /
            ticks_per_revolution_;


        // =========================================================
        // ABRIR I2C
        // =========================================================

        if (!open_i2c())
        {
            throw std::runtime_error(
                "No se pudo abrir el bus I2C");
        }


        // =========================================================
        // TF
        // =========================================================

        tf_broadcaster_ =
            std::make_unique<
                tf2_ros::TransformBroadcaster>(*this);


        // =========================================================
        // /cmd_vel
        // =========================================================

        cmd_vel_sub_ =
            create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                rclcpp::QoS(10),
                std::bind(
                    &TankI2CDriver::cmd_vel_callback,
                    this,
                    std::placeholders::_1));


        // =========================================================
        // /odom
        // =========================================================

        odom_pub_ =
            create_publisher<nav_msgs::msg::Odometry>(
                "/odom",
                rclcpp::QoS(10));


        // =========================================================
        // TIMER MOTORES
        //
        // 100 Hz
        // =========================================================

        auto motor_period =
            std::chrono::duration_cast<
                std::chrono::nanoseconds>(
                std::chrono::duration<double>(
                    1.0 / motor_rate_));

        motor_timer_ =
            create_wall_timer(
                motor_period,
                std::bind(
                    &TankI2CDriver::motor_timer_callback,
                    this));


        // =========================================================
        // TIMER ENCODERS
        //
        // 20 Hz
        // =========================================================

        auto encoder_period =
            std::chrono::duration_cast<
                std::chrono::nanoseconds>(
                std::chrono::duration<double>(
                    1.0 / encoder_rate_));

        encoder_timer_ =
            create_wall_timer(
                encoder_period,
                std::bind(
                    &TankI2CDriver::encoder_timer_callback,
                    this));


        // =========================================================
        // WATCHDOG
        // =========================================================

        last_cmd_time_ = now();


        // =========================================================
        // ESTADO INICIAL
        // =========================================================

        previous_left_ticks_ = 0;
        previous_right_ticks_ = 0;

        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;

        target_left_motor_ = 0;
        target_right_motor_ = 0;


        // =========================================================
        // INFORMACION
        // =========================================================

        RCLCPP_INFO(
            get_logger(),
            "========================================");

        RCLCPP_INFO(
            get_logger(),
            "       TANK I2C DRIVER");

        RCLCPP_INFO(
            get_logger(),
            "========================================");

        RCLCPP_INFO(
            get_logger(),
            "I2C: %s",
            i2c_device_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Address: 0x%02X",
            i2c_address_);

        RCLCPP_INFO(
            get_logger(),
            "Max linear: %.3f m/s",
            max_linear_speed_);

        RCLCPP_INFO(
            get_logger(),
            "Track width: %.3f m",
            track_width_);

        RCLCPP_INFO(
            get_logger(),
            "Wheel diameter: %.3f m",
            wheel_diameter_);

        RCLCPP_INFO(
            get_logger(),
            "Ticks/revolution: %.1f",
            ticks_per_revolution_);

        RCLCPP_INFO(
            get_logger(),
            "Meters/tick: %.9f",
            meters_per_tick_);

        RCLCPP_INFO(
            get_logger(),
            "Motor rate: %.1f Hz",
            motor_rate_);

        RCLCPP_INFO(
            get_logger(),
            "Encoder rate: %.1f Hz",
            encoder_rate_);

        RCLCPP_INFO(
            get_logger(),
            "========================================");


        // =========================================================
        // MOTOR STOP
        // =========================================================

        stop_motors();
    }


    ~TankI2CDriver()
    {
        stop_motors();

        if (fd_ >= 0)
            close(fd_);
    }


private:

    // =============================================================
    // REGISTROS HIWONDER
    // =============================================================

    static constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 0x33;

    static constexpr uint8_t
        MOTOR_ENCODER_TOTAL_ADDR = 0x3C;


    // =============================================================
    // I2C
    // =============================================================

    int fd_ = -1;

    std::string i2c_device_;

    int i2c_address_;

    std::mutex i2c_mutex_;


    // =============================================================
    // PARAMETROS
    // =============================================================

    double max_linear_speed_;

    double max_angular_speed_;

    double track_width_;

    double wheel_diameter_;

    double ticks_per_revolution_;

    double wheel_circumference_;

    double meters_per_tick_;

    int max_motor_speed_;

    int left_motor_sign_;

    int right_motor_sign_;

    int left_encoder_sign_;

    int right_encoder_sign_;

    double motor_rate_;

    double encoder_rate_;

    double watchdog_timeout_;

    bool publish_tf_;

    std::string odom_frame_;

    std::string base_frame_;


    // =============================================================
    // ROS
    // =============================================================

    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_sub_;

    rclcpp::Publisher<
        nav_msgs::msg::Odometry>::SharedPtr
        odom_pub_;

    rclcpp::TimerBase::SharedPtr
        motor_timer_;

    rclcpp::TimerBase::SharedPtr
        encoder_timer_;

    std::unique_ptr<
        tf2_ros::TransformBroadcaster>
        tf_broadcaster_;


    // =============================================================
    // CMD VEL
    // =============================================================

    int target_left_motor_ = 0;

    int target_right_motor_ = 0;

    rclcpp::Time last_cmd_time_;


    // =============================================================
    // ENCODERS
    // =============================================================

    int32_t previous_left_ticks_ = 0;

    int32_t previous_right_ticks_ = 0;

    bool encoder_initialized_ = false;


    // =============================================================
    // ODOMETRIA
    // =============================================================

    double x_ = 0.0;

    double y_ = 0.0;

    double theta_ = 0.0;


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
                "No se pudo abrir %s: %s",
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
                "No se pudo seleccionar "
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
    // ESCRIBIR REGISTRO I2C
    // =============================================================

    bool write_register(
        uint8_t reg,
        const uint8_t *data,
        size_t length)
    {
        std::lock_guard<std::mutex> lock(
            i2c_mutex_);


        std::vector<uint8_t> buffer(
            length + 1);

        buffer[0] = reg;

        for (size_t i = 0; i < length; i++)
            buffer[i + 1] = data[i];


        ssize_t result =
            write(
                fd_,
                buffer.data(),
                buffer.size());


        if (result !=
            static_cast<ssize_t>(
                buffer.size()))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Error I2C write: %s",
                strerror(errno));

            return false;
        }

        return true;
    }


    // =============================================================
    // LEER REGISTRO I2C
    // =============================================================

    bool read_register(
        uint8_t reg,
        uint8_t *data,
        size_t length)
    {
        std::lock_guard<std::mutex> lock(
            i2c_mutex_);


        uint8_t register_byte = reg;


        ssize_t result =
            write(
                fd_,
                &register_byte,
                1);


        if (result != 1)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Error seleccionando registro "
                "I2C 0x%02X: %s",
                reg,
                strerror(errno));

            return false;
        }


        result =
            read(
                fd_,
                data,
                length);


        if (result !=
            static_cast<ssize_t>(length))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Error leyendo I2C: %s",
                strerror(errno));

            return false;
        }

        return true;
    }


    // =============================================================
    // ESCRIBIR MOTORES
    // =============================================================

    bool write_motor_speeds(
        int8_t m1,
        int8_t m2,
        int8_t m3,
        int8_t m4)
    {
        uint8_t data[4];

        data[0] =
            static_cast<uint8_t>(m1);

        data[1] =
            static_cast<uint8_t>(m2);

        data[2] =
            static_cast<uint8_t>(m3);

        data[3] =
            static_cast<uint8_t>(m4);


        return write_register(
            MOTOR_FIXED_SPEED_ADDR,
            data,
            4);
    }


    // =============================================================
    // LEER ENCODERS
    //
    // 0x3C
    //
    // 16 bytes:
    //
    // Encoder 1 = bytes 0-3
    // Encoder 2 = bytes 4-7
    // Encoder 3 = bytes 8-11
    // Encoder 4 = bytes 12-15
    // =============================================================

    bool read_encoders(
        std::array<int32_t, 4> &encoders)
    {
        uint8_t data[16];


        if (!read_register(
                MOTOR_ENCODER_TOTAL_ADDR,
                data,
                16))
        {
            return false;
        }


        for (int i = 0; i < 4; i++)
        {
            uint32_t value =
                static_cast<uint32_t>(data[i * 4]) |
                (static_cast<uint32_t>(data[i * 4 + 1]) << 8) |
                (static_cast<uint32_t>(data[i * 4 + 2]) << 16) |
                (static_cast<uint32_t>(data[i * 4 + 3]) << 24);

            encoders[i] =
                static_cast<int32_t>(value);
        }

        return true;
    }


    // =============================================================
    // CMD VEL
    // =============================================================

    void cmd_vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = now();


        double linear =
            std::clamp(
                msg->linear.x,
                -max_linear_speed_,
                max_linear_speed_);


        double angular =
            -std::clamp(
                msg->angular.z,
                -max_angular_speed_,
                max_angular_speed_);


        // =========================================================
        // CINEMATICA TANK
        //
        // left  = v - wL/2
        // right = v + wL/2
        // =========================================================

        double left =
            linear -
            angular * track_width_ / 2.0;


        double right =
            linear +
            angular * track_width_ / 2.0;


        left =
            std::clamp(
                left,
                -max_linear_speed_,
                max_linear_speed_);


        right =
            std::clamp(
                right,
                -max_linear_speed_,
                max_linear_speed_);


        // =========================================================
        // m/s -> MOTOR VALUE
        // =========================================================

        int left_motor =
            static_cast<int>(
                std::lround(
                    left /
                    max_linear_speed_ *
                    max_motor_speed_));


        int right_motor =
            static_cast<int>(
                std::lround(
                    right /
                    max_linear_speed_ *
                    max_motor_speed_));


        left_motor =
            std::clamp(
                left_motor,
                -max_motor_speed_,
                max_motor_speed_);


        right_motor =
            std::clamp(
                right_motor,
                -max_motor_speed_,
                max_motor_speed_);


        // =========================================================
        // POLARIDAD MOTOR
        // =========================================================

        left_motor *=
            left_motor_sign_;

        right_motor *=
            right_motor_sign_;


        target_left_motor_ =
            left_motor;

        target_right_motor_ =
            right_motor;
    }


    // =============================================================
    // TIMER MOTORES
    // =============================================================

    void motor_timer_callback()
    {
        // =========================================================
        // WATCHDOG
        // =========================================================

        if (
            (now() - last_cmd_time_).seconds()
            > watchdog_timeout_)
        {
            target_left_motor_ = 0;
            target_right_motor_ = 0;
        }


        // =========================================================
        // ENVIAR
        // =========================================================

        write_motor_speeds(
            static_cast<int8_t>(
                target_left_motor_),

            static_cast<int8_t>(
                target_right_motor_),

            0,
            0);
    }


    // =============================================================
    // TIMER ENCODERS
    // =============================================================

    void encoder_timer_callback()
    {
        std::array<int32_t, 4> encoders;


        if (!read_encoders(encoders))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "No se pudieron leer los encoders");

            return;
        }


        // =========================================================
        // M1 = IZQUIERDA
        // M2 = DERECHA
        // =========================================================

        int32_t left_ticks =
            encoders[0] *
            left_encoder_sign_;


        int32_t right_ticks =
            encoders[1] *
            right_encoder_sign_;


        // =========================================================
        // PRIMERA LECTURA
        // =========================================================

        if (!encoder_initialized_)
        {
            previous_left_ticks_ =
                left_ticks;

            previous_right_ticks_ =
                right_ticks;

            encoder_initialized_ = true;

            return;
        }


        // =========================================================
        // DELTA TICKS
        // =========================================================

        int32_t delta_left =
            left_ticks -
            previous_left_ticks_;


        int32_t delta_right =
            right_ticks -
            previous_right_ticks_;


        previous_left_ticks_ =
            left_ticks;

        previous_right_ticks_ =
            right_ticks;


        // =========================================================
        // TICKS -> METROS
        // =========================================================

        double delta_left_m =
            static_cast<double>(
                delta_left) *
            meters_per_tick_;


        double delta_right_m =
            static_cast<double>(
                delta_right) *
            meters_per_tick_;


        // =========================================================
        // CINEMATICA DIFERENCIAL
        // =========================================================

        double delta_s =
            (delta_left_m +
             delta_right_m) / 2.0;


        double delta_theta =
            (delta_right_m -
             delta_left_m) /
            track_width_;


        // =========================================================
        // INTEGRACION
        // =========================================================

        double theta_mid =
            theta_ +
            delta_theta / 2.0;


        x_ +=
            delta_s *
            std::cos(theta_mid);


        y_ +=
            delta_s *
            std::sin(theta_mid);


        theta_ +=
            delta_theta;


        // Normalizar theta

        while (theta_ > M_PI)
            theta_ -= 2.0 * M_PI;

        while (theta_ < -M_PI)
            theta_ += 2.0 * M_PI;


        // =========================================================
        // PUBLICAR
        // =========================================================

        publish_odometry(
            delta_left_m,
            delta_right_m);
    }


    // =============================================================
    // PUBLICAR ODOM
    // =============================================================

    void publish_odometry(
        double delta_left_m,
        double delta_right_m)
    {
        auto stamp = now();


        // =========================================================
        // VELOCIDADES
        //
        // encoder_rate = 20 Hz
        // =========================================================

        double dt =
            1.0 / encoder_rate_;


        double linear_velocity =
            (delta_left_m +
             delta_right_m) /
            (2.0 * dt);


        double angular_velocity =
            (delta_right_m -
             delta_left_m) /
            (track_width_ * dt);


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


        odom.pose.pose.position.x =
            x_;

        odom.pose.pose.position.y =
            y_;

        odom.pose.pose.position.z =
            0.0;


        tf2::Quaternion q;

        q.setRPY(
            0.0,
            0.0,
            theta_);


        odom.pose.pose.orientation.x =
            q.x();

        odom.pose.pose.orientation.y =
            q.y();

        odom.pose.pose.orientation.z =
            q.z();

        odom.pose.pose.orientation.w =
            q.w();


        odom.twist.twist.linear.x =
            linear_velocity;

        odom.twist.twist.linear.y =
            0.0;

        odom.twist.twist.angular.z =
            angular_velocity;


        odom_pub_->publish(odom);


        // =========================================================
        // TF
        // =========================================================

        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped tf;


            tf.header.stamp =
                stamp;

            tf.header.frame_id =
                odom_frame_;

            tf.child_frame_id =
                base_frame_;


            tf.transform.translation.x =
                x_;

            tf.transform.translation.y =
                y_;

            tf.transform.translation.z =
                0.0;


            tf.transform.rotation.x =
                q.x();

            tf.transform.rotation.y =
                q.y();

            tf.transform.rotation.z =
                q.z();

            tf.transform.rotation.w =
                q.w();


            tf_broadcaster_->sendTransform(tf);
        }


        // =========================================================
        // DEBUG
        // =========================================================

        RCLCPP_DEBUG(
            get_logger(),
            "ENC L=%f R=%f | "
            "dL=%.5f dR=%.5f | "
            "x=%.3f y=%.3f theta=%.3f",
            delta_left_m,
            delta_right_m,
            delta_left_m,
            delta_right_m,
            x_,
            y_,
            theta_);
    }


    // =============================================================
    // STOP
    // =============================================================

    void stop_motors()
    {
        if (fd_ >= 0)
        {
            write_motor_speeds(
                0,
                0,
                0,
                0);
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
            std::make_shared<TankI2CDriver>();


        rclcpp::spin(node);
    }
    catch (
        const std::exception &e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger(
                "tank_i2c_driver"),
            "%s",
            e.what());
    }


    rclcpp::shutdown();

    return 0;
}
