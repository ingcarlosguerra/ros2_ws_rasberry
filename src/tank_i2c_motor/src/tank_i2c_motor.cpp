#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <chrono>
#include <cstdint>
#include <cmath>

class TankI2CMotor : public rclcpp::Node
{
public:

    TankI2CMotor()
        : Node("tank_i2c_motor")
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

        // Velocidad lineal maxima del robot
        declare_parameter<double>(
            "max_linear_speed",
            0.30);

        // Velocidad angular maxima
        declare_parameter<double>(
            "max_angular_speed",
            6.0);

        // Valor maximo aceptado por el controlador
        declare_parameter<int>(
            "max_motor_speed",
            50);

        // Distancia entre centros de los tracks
        // 10 cm
        declare_parameter<double>(
            "track_width",
            0.10);

        // Diametro de rueda
        // 50 mm
        declare_parameter<double>(
            "wheel_diameter",
            0.05);

        // ---------------------------------------------------------
        // SENTIDO DE LOS MOTORES
        //
        //  1  = sentido normal
        // -1  = invertir sentido
        //
        // Primero dejamos ambos en 1.
        // Si fisicamente un track queda invertido,
        // cambiamos solamente ese parametro.
        // ---------------------------------------------------------

        declare_parameter<int>(
            "left_motor_sign",
            1);

        declare_parameter<int>(
            "right_motor_sign",
            1);

        // =========================================================
        // LEER PARAMETROS
        // =========================================================

        i2c_device_ =
            get_parameter(
                "i2c_device").as_string();

        i2c_address_ =
            get_parameter(
                "i2c_address").as_int();

        max_linear_speed_ =
            get_parameter(
                "max_linear_speed").as_double();

        max_angular_speed_ =
            get_parameter(
                "max_angular_speed").as_double();

        max_motor_speed_ =
            get_parameter(
                "max_motor_speed").as_int();

        track_width_ =
            get_parameter(
                "track_width").as_double();

        wheel_diameter_ =
            get_parameter(
                "wheel_diameter").as_double();

        left_motor_sign_ =
            get_parameter(
                "left_motor_sign").as_int();

        right_motor_sign_ =
            get_parameter(
                "right_motor_sign").as_int();

        // =========================================================
        // VALIDACIONES
        // =========================================================

        if (max_linear_speed_ <= 0.0)
        {
            throw std::runtime_error(
                "max_linear_speed debe ser > 0");
        }

        if (max_motor_speed_ <= 0)
        {
            throw std::runtime_error(
                "max_motor_speed debe ser > 0");
        }

        if (track_width_ <= 0.0)
        {
            throw std::runtime_error(
                "track_width debe ser > 0");
        }

        if (wheel_diameter_ <= 0.0)
        {
            throw std::runtime_error(
                "wheel_diameter debe ser > 0");
        }

        if (left_motor_sign_ != 1 &&
            left_motor_sign_ != -1)
        {
            RCLCPP_WARN(
                get_logger(),
                "left_motor_sign invalido. "
                "Usando 1.");

            left_motor_sign_ = 1;
        }

        if (right_motor_sign_ != 1 &&
            right_motor_sign_ != -1)
        {
            RCLCPP_WARN(
                get_logger(),
                "right_motor_sign invalido. "
                "Usando 1.");

            right_motor_sign_ = 1;
        }

        // =========================================================
        // ABRIR I2C
        // =========================================================

        if (!open_i2c())
        {
            RCLCPP_FATAL(
                get_logger(),
                "No se pudo abrir I2C %s "
                "direccion 0x%02X",
                i2c_device_.c_str(),
                i2c_address_);

            throw std::runtime_error(
                "No se pudo abrir I2C");
        }

        // =========================================================
        // INFORMACION
        // =========================================================

        RCLCPP_INFO(
            get_logger(),
            "==========================================");

        RCLCPP_INFO(
            get_logger(),
            "       TANK I2C MOTOR DRIVER");

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
            "Max linear speed: %.3f m/s",
            max_linear_speed_);

        RCLCPP_INFO(
            get_logger(),
            "Max angular speed: %.3f rad/s",
            max_angular_speed_);

        RCLCPP_INFO(
            get_logger(),
            "Max motor value: %d",
            max_motor_speed_);

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
            "Left motor sign: %d",
            left_motor_sign_);

        RCLCPP_INFO(
            get_logger(),
            "Right motor sign: %d",
            right_motor_sign_);

        RCLCPP_INFO(
            get_logger(),
            "==========================================");

        // =========================================================
        // SUBSCRIBER /cmd_vel
        // =========================================================

        cmd_sub_ =
            create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &TankI2CMotor::cmd_vel_callback,
                    this,
                    std::placeholders::_1));

        // =========================================================
        // WATCHDOG
        // =========================================================

        watchdog_timer_ =
            create_wall_timer(
                std::chrono::milliseconds(10000),
                std::bind(
                    &TankI2CMotor::watchdog_callback,
                    this));

        last_cmd_time_ = now();

        // =========================================================
        // SEGURIDAD
        // =========================================================

        stop_motors();
    }

    ~TankI2CMotor()
    {
        stop_motors();

        if (fd_ >= 0)
        {
            close(fd_);
        }
    }

private:

    // =============================================================
    // REGISTROS Hiwonder
    // =============================================================

    static constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 51;

    // =============================================================
    // VARIABLES
    // =============================================================

    int fd_ = -1;

    std::string i2c_device_;

    int i2c_address_;

    double max_linear_speed_;

    double max_angular_speed_;

    int max_motor_speed_;

    double track_width_;

    double wheel_diameter_;

    int left_motor_sign_;

    int right_motor_sign_;

    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    rclcpp::Time last_cmd_time_;

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
    // ESCRIBIR VELOCIDADES
    //
    // El controlador espera:
    //
    // [51][M1][M2][M3][M4]
    //
    // M1 = track izquierdo
    // M2 = track derecho
    // M3 = 0
    // M4 = 0
    // =============================================================

    bool write_motor_speeds(
        int8_t m1,
        int8_t m2,
        int8_t m3,
        int8_t m4)
    {
        uint8_t data[5];

        data[0] =
            MOTOR_FIXED_SPEED_ADDR;

        data[1] =
            static_cast<uint8_t>(m1);

        data[2] =
            static_cast<uint8_t>(m2);

        data[3] =
            static_cast<uint8_t>(m3);

        data[4] =
            static_cast<uint8_t>(m4);

        ssize_t result =
            write(
                fd_,
                data,
                sizeof(data));

        if (result != sizeof(data))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Error escribiendo I2C: %s",
                strerror(errno));

            return false;
        }

        return true;
    }

    // =============================================================
    // CALLBACK /cmd_vel
    // =============================================================

    void cmd_vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Actualizar watchdog
        last_cmd_time_ = now();

        // =========================================================
        // VELOCIDAD LINEAL
        // =========================================================

        double linear =
            std::clamp(
                msg->linear.x,
                -max_linear_speed_,
                max_linear_speed_);

        // =========================================================
        // VELOCIDAD ANGULAR
        // =========================================================

        double angular =
            std::clamp(
                msg->angular.z,
                -max_angular_speed_,
                max_angular_speed_);

        // =========================================================
        // CINEMATICA DIFFERENTIAL / TANK
        //
        // v_left  = v + (w * L / 2)
        // v_right = v - (w * L / 2)
        //
        // L = distancia entre tracks
        //
        // En nuestro robot:
        //
        // L = 0.10 m
        // =========================================================

        double left =
            linear +
            (angular * track_width_ / 2.0);

        double right =
            linear -
            (angular * track_width_ / 2.0);

        // =========================================================
        // LIMITAR CADA TRACK
        // =========================================================

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
        // CONVERSION PROVISIONAL
        //
        // max_linear_speed = 0.30 m/s
        // max_motor_speed  = 50
        //
        // IMPORTANTE:
        // Esta es una escala provisional.
        //
        // Todavia NO significa que:
        //
        // 17 = exactamente 0.10 m/s
        //

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

        // =========================================================
        // LIMITAR VALORES
        // =========================================================

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
        // APLICAR POLARIDAD DEL HARDWARE
        // =========================================================

        left_motor *=
            left_motor_sign_;

        right_motor *=
            right_motor_sign_;

        // =========================================================
        // M1 = IZQUIERDA
        // M2 = DERECHA
        // M1 = left
        // M2 = right
        // =========================================================

        int8_t m1 =
            static_cast<int8_t>(
                left_motor);

        int8_t m2 =
            static_cast<int8_t>(
                right_motor);

        // Motores que no utilizamos
        int8_t m3 = 0;
        int8_t m4 = 0;

        // =========================================================
        // ENVIAR AL CONTROLADOR
        // =========================================================

        write_motor_speeds(
            m1,
            m2,
            m3,
            m4);

        // =========================================================
        // DEBUG
        // =========================================================

        RCLCPP_DEBUG(
            get_logger(),
            "cmd_vel: "
            "v=%.3f "
            "w=%.3f | "
            "L=%.3f "
            "R=%.3f | "
            "M1=%d "
            "M2=%d",
            linear,
            angular,
            left,
            right,
            m1,
            m2);
    }

    // =============================================================
    // DETENER MOTORES
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

    // =============================================================
    // WATCHDOG
    //
    // Si pasan mas de 0.3 segundos sin recibir /cmd_vel,
    // detenemos el robot.
    // =============================================================

    void watchdog_callback()
    {
        if (
            (now() - last_cmd_time_).seconds()
            > 2.0)
        {
            stop_motors();
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
            std::make_shared<TankI2CMotor>();

        rclcpp::spin(node);
    }
    catch (
        const std::exception &e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger(
                "tank_i2c_motor"),
            "%s",
            e.what());
    }

    rclcpp::shutdown();

    return 0;
}
