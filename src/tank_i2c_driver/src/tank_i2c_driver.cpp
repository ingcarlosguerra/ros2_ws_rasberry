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
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

class TankI2CDriver : public rclcpp::Node
{
public:
    TankI2CDriver() : Node("tank_i2c_driver")
    {
        // ---------------- PARAMETROS ----------------
        declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
        declare_parameter<int>("i2c_address", 0x34);
        declare_parameter<double>("max_linear_speed", 0.30);
        declare_parameter<double>("max_angular_speed", 6.0);
        declare_parameter<double>("track_width", 0.10);
        declare_parameter<double>("wheel_diameter", 0.05);
        declare_parameter<double>("ticks_per_revolution", 1980.0);
        declare_parameter<int>("max_motor_speed", 50);
        declare_parameter<int>("left_motor_sign", 1);
        declare_parameter<int>("right_motor_sign", 1);
        declare_parameter<int>("left_encoder_sign", 1);
        declare_parameter<int>("right_encoder_sign", 1);
        declare_parameter<int>("angular_sign", -1);          // antes hardcodeado
        declare_parameter<double>("motor_rate", 30.0);
        declare_parameter<double>("motor_keepalive", 0.10);  // reenviar aunque no cambie
        declare_parameter<double>("encoder_rate", 20.0);
        declare_parameter<bool>("publish_tf", true);
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<double>("watchdog_timeout", 0.25);
        declare_parameter<bool>("configure_board", true);
        declare_parameter<int>("motor_type", 3);             // VERIFICAR en SDK Hiwonder (JGB37_520_12V_110RPM)
        declare_parameter<int>("encoder_polarity", 0);
        declare_parameter<int>("max_delta_ticks", 2000);     // por ciclo de encoder
        declare_parameter<int>("read_retries", 3);
        declare_parameter<int>("reopen_after_failures", 5);

        i2c_device_ = get_parameter("i2c_device").as_string();
        i2c_address_ = get_parameter("i2c_address").as_int();
        max_linear_speed_ = get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = get_parameter("max_angular_speed").as_double();
        track_width_ = get_parameter("track_width").as_double();
        wheel_diameter_ = get_parameter("wheel_diameter").as_double();
        ticks_per_revolution_ = get_parameter("ticks_per_revolution").as_double();
        max_motor_speed_ = get_parameter("max_motor_speed").as_int();
        left_motor_sign_ = get_parameter("left_motor_sign").as_int();
        right_motor_sign_ = get_parameter("right_motor_sign").as_int();
        left_encoder_sign_ = get_parameter("left_encoder_sign").as_int();
        right_encoder_sign_ = get_parameter("right_encoder_sign").as_int();
        angular_sign_ = get_parameter("angular_sign").as_int();
        motor_rate_ = get_parameter("motor_rate").as_double();
        motor_keepalive_ = get_parameter("motor_keepalive").as_double();
        encoder_rate_ = get_parameter("encoder_rate").as_double();
        publish_tf_ = get_parameter("publish_tf").as_bool();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        watchdog_timeout_ = get_parameter("watchdog_timeout").as_double();
        configure_board_ = get_parameter("configure_board").as_bool();
        motor_type_ = get_parameter("motor_type").as_int();
        encoder_polarity_ = get_parameter("encoder_polarity").as_int();
        max_delta_ticks_ = get_parameter("max_delta_ticks").as_int();
        read_retries_ = get_parameter("read_retries").as_int();
        reopen_after_failures_ = get_parameter("reopen_after_failures").as_int();

        // ---------------- VALIDACIONES ----------------
        if (max_linear_speed_ <= 0.0) throw std::runtime_error("max_linear_speed debe ser > 0");
        if (track_width_ <= 0.0) throw std::runtime_error("track_width debe ser > 0");
        if (wheel_diameter_ <= 0.0) throw std::runtime_error("wheel_diameter debe ser > 0");
        if (ticks_per_revolution_ <= 0.0) throw std::runtime_error("ticks_per_revolution debe ser > 0");
        if (max_motor_speed_ <= 0 || max_motor_speed_ > 100)
            throw std::runtime_error("max_motor_speed debe estar en (0, 100]");
        if (motor_rate_ <= 0.0 || encoder_rate_ <= 0.0)
            throw std::runtime_error("motor_rate y encoder_rate deben ser > 0");

        meters_per_tick_ = M_PI * wheel_diameter_ / ticks_per_revolution_;

        // ---------------- I2C ----------------
        if (!open_i2c())
            throw std::runtime_error("No se pudo abrir el bus I2C");

        if (configure_board_ && !configure_board())
            RCLCPP_WARN(get_logger(), "No se pudo configurar la placa (motor type/polaridad)");

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(10),
            std::bind(&TankI2CDriver::cmd_vel_callback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));

        last_cmd_time_ = now();
        last_motor_send_ = now();
        last_encoder_time_ = now();

        motor_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / motor_rate_)),
            std::bind(&TankI2CDriver::motor_timer_callback, this));

        encoder_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / encoder_rate_)),
            std::bind(&TankI2CDriver::encoder_timer_callback, this));

        RCLCPP_INFO(get_logger(), "TANK I2C DRIVER | %s @0x%02X | %.9f m/tick | motores %.0f Hz | encoders %.0f Hz",
                    i2c_device_.c_str(), i2c_address_, meters_per_tick_, motor_rate_, encoder_rate_);

        stop_motors();
    }

    ~TankI2CDriver()
    {
        stop_motors();
        if (fd_ >= 0) close(fd_);
    }

private:
    // Registros Hiwonder EMM
    static constexpr uint8_t MOTOR_TYPE_ADDR = 0x14;
    static constexpr uint8_t MOTOR_ENCODER_POLARITY_ADDR = 0x15;
    static constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 0x33;
    static constexpr uint8_t MOTOR_ENCODER_TOTAL_ADDR = 0x3C;

    // I2C
    int fd_ = -1;
    std::string i2c_device_;
    int i2c_address_;
    std::mutex i2c_mutex_;

    // Parametros
    double max_linear_speed_, max_angular_speed_, track_width_, wheel_diameter_;
    double ticks_per_revolution_, meters_per_tick_;
    int max_motor_speed_, left_motor_sign_, right_motor_sign_;
    int left_encoder_sign_, right_encoder_sign_, angular_sign_;
    double motor_rate_, motor_keepalive_, encoder_rate_, watchdog_timeout_;
    bool publish_tf_, configure_board_;
    std::string odom_frame_, base_frame_;
    int motor_type_, encoder_polarity_, max_delta_ticks_;
    int read_retries_, reopen_after_failures_;

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr motor_timer_, encoder_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Comandos
    int target_left_motor_ = 0;
    int target_right_motor_ = 0;
    int sent_left_motor_ = 0;
    int sent_right_motor_ = 0;
    bool force_send_ = true;
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_motor_send_{0, 0, RCL_ROS_TIME};

    // Encoders
    int32_t previous_left_ticks_ = 0;
    int32_t previous_right_ticks_ = 0;
    bool encoder_initialized_ = false;
    int consecutive_failures_ = 0;
    int write_failures_ = 0;
    rclcpp::Time last_encoder_time_{0, 0, RCL_ROS_TIME};

    // Odometria
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

    // ---------------------------------------------------------
    bool open_i2c_locked()
    {
        fd_ = open(i2c_device_.c_str(), O_RDWR);
        if (fd_ < 0) return false;
        if (ioctl(fd_, I2C_SLAVE, i2c_address_) < 0)
        {
            close(fd_);
            fd_ = -1;
            return false;
        }
        return true;
    }

    bool open_i2c()
    {
        std::lock_guard<std::mutex> lock(i2c_mutex_);
        bool ok = open_i2c_locked();
        if (!ok)
            RCLCPP_ERROR(get_logger(), "No se pudo abrir %s: %s", i2c_device_.c_str(), strerror(errno));
        return ok;
    }

    bool reopen_i2c()
    {
        {
            std::lock_guard<std::mutex> lock(i2c_mutex_);
            if (fd_ >= 0) { close(fd_); fd_ = -1; }
            usleep(20000);
            if (!open_i2c_locked()) return false;
        }
        return true;
    }

    // Escritura de una sola transaccion: [reg, data...]
    bool write_register(uint8_t reg, const uint8_t *data, size_t length)
    {
        std::vector<uint8_t> buffer(length + 1);
        buffer[0] = reg;
        for (size_t i = 0; i < length; i++) buffer[i + 1] = data[i];

        ssize_t result;
        int err = 0;
        {
            std::lock_guard<std::mutex> lock(i2c_mutex_);
            if (fd_ < 0) return false;
            result = write(fd_, buffer.data(), buffer.size());
            err = errno;
        }
        if (result != static_cast<ssize_t>(buffer.size()))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Error I2C write 0x%02X: %s", reg, strerror(err));
            return false;
        }
        return true;
    }

    // Lectura con REPEATED START (equivale a transfer() de PX4)
    bool read_register(uint8_t reg, uint8_t *data, size_t length)
    {
        struct i2c_msg msgs[2];
        msgs[0].addr = static_cast<uint16_t>(i2c_address_);
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = &reg;

        msgs[1].addr = static_cast<uint16_t>(i2c_address_);
        msgs[1].flags = I2C_M_RD;
        msgs[1].len = static_cast<uint16_t>(length);
        msgs[1].buf = data;

        struct i2c_rdwr_ioctl_data xfer;
        xfer.msgs = msgs;
        xfer.nmsgs = 2;

        int rc, err = 0;
        {
            std::lock_guard<std::mutex> lock(i2c_mutex_);
            if (fd_ < 0) return false;
            rc = ioctl(fd_, I2C_RDWR, &xfer);
            err = errno;
        }
        if (rc < 0)
        {
            RCLCPP_DEBUG(get_logger(), "Error I2C read 0x%02X: %s", reg, strerror(err));
            return false;
        }
        return true;
    }

    // Igual que init() del driver PX4
    bool configure_board()
    {
        uint8_t type = static_cast<uint8_t>(motor_type_);
        uint8_t pol = static_cast<uint8_t>(encoder_polarity_);
        bool ok = write_register(MOTOR_TYPE_ADDR, &type, 1);
        usleep(5000);
        ok = write_register(MOTOR_ENCODER_POLARITY_ADDR, &pol, 1) && ok;
        usleep(5000);
        return ok;
    }

    bool write_motor_speeds(int8_t m1, int8_t m2, int8_t m3, int8_t m4)
    {
        uint8_t data[4] = {
            static_cast<uint8_t>(m1), static_cast<uint8_t>(m2),
            static_cast<uint8_t>(m3), static_cast<uint8_t>(m4)};
        return write_register(MOTOR_FIXED_SPEED_ADDR, data, 4);
    }

    bool read_encoders(std::array<int32_t, 4> &encoders)
    {
        uint8_t data[16];
        if (!read_register(MOTOR_ENCODER_TOTAL_ADDR, data, 16)) return false;

        for (int i = 0; i < 4; i++)
        {
            uint32_t value =
                static_cast<uint32_t>(data[i * 4]) |
                (static_cast<uint32_t>(data[i * 4 + 1]) << 8) |
                (static_cast<uint32_t>(data[i * 4 + 2]) << 16) |
                (static_cast<uint32_t>(data[i * 4 + 3]) << 24);
            encoders[i] = static_cast<int32_t>(value);
        }
        return true;
    }

    // ---------------------------------------------------------
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = now();

        double linear = std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
        double angular = angular_sign_ *
                         std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);

        double left = std::clamp(linear - angular * track_width_ / 2.0,
                                 -max_linear_speed_, max_linear_speed_);
        double right = std::clamp(linear + angular * track_width_ / 2.0,
                                  -max_linear_speed_, max_linear_speed_);

        int left_motor = static_cast<int>(std::lround(left / max_linear_speed_ * max_motor_speed_));
        int right_motor = static_cast<int>(std::lround(right / max_linear_speed_ * max_motor_speed_));

        left_motor = std::clamp(left_motor, -max_motor_speed_, max_motor_speed_) * left_motor_sign_;
        right_motor = std::clamp(right_motor, -max_motor_speed_, max_motor_speed_) * right_motor_sign_;

        target_left_motor_ = left_motor;
        target_right_motor_ = right_motor;
    }

    void motor_timer_callback()
    {
        auto t = now();

        if ((t - last_cmd_time_).seconds() > watchdog_timeout_)
        {
            target_left_motor_ = 0;
            target_right_motor_ = 0;
        }

        // Solo escribir si cambio, o como keep-alive
        bool changed = target_left_motor_ != sent_left_motor_ ||
                       target_right_motor_ != sent_right_motor_;
        bool keepalive = (t - last_motor_send_).seconds() >= motor_keepalive_;

        if (!(changed || keepalive || force_send_)) return;

        if (write_motor_speeds(static_cast<int8_t>(target_left_motor_),
                               static_cast<int8_t>(target_right_motor_), 0, 0))
        {
            sent_left_motor_ = target_left_motor_;
            sent_right_motor_ = target_right_motor_;
            last_motor_send_ = t;
            force_send_ = false;
            write_failures_ = 0;
        }
        else if (++write_failures_ >= reopen_after_failures_)
        {
            recover_bus();
            write_failures_ = 0;
        }
    }

    void recover_bus()
    {
        RCLCPP_WARN(get_logger(), "I2C caido, reabriendo bus...");
        if (!reopen_i2c())
        {
            RCLCPP_ERROR(get_logger(), "No se pudo reabrir el bus I2C");
            return;
        }
        if (configure_board_) configure_board();
        encoder_initialized_ = false;  // re-sincronizar deltas (la placa pudo reiniciarse)
        force_send_ = true;
    }

    void encoder_timer_callback()
    {
        std::array<int32_t, 4> encoders;
        bool ok = false;

        for (int attempt = 0; attempt < read_retries_ && !ok; attempt++)
        {
            ok = read_encoders(encoders);
            if (!ok) usleep(2000);
        }

        if (!ok)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No se pudieron leer los encoders");
            if (++consecutive_failures_ >= reopen_after_failures_)
            {
                recover_bus();
                consecutive_failures_ = 0;
            }
            return;
        }
        consecutive_failures_ = 0;

        auto stamp = now();

        int32_t left_ticks = encoders[0] * left_encoder_sign_;
        int32_t right_ticks = encoders[1] * right_encoder_sign_;

        if (!encoder_initialized_)
        {
            previous_left_ticks_ = left_ticks;
            previous_right_ticks_ = right_ticks;
            last_encoder_time_ = stamp;
            encoder_initialized_ = true;
            return;
        }

        int32_t delta_left = left_ticks - previous_left_ticks_;
        int32_t delta_right = right_ticks - previous_right_ticks_;

        previous_left_ticks_ = left_ticks;
        previous_right_ticks_ = right_ticks;

        double dt = (stamp - last_encoder_time_).seconds();
        last_encoder_time_ = stamp;

        // Rechazar lecturas corruptas / reinicio de la placa
        if (std::abs(static_cast<int64_t>(delta_left)) > max_delta_ticks_ ||
            std::abs(static_cast<int64_t>(delta_right)) > max_delta_ticks_ || dt <= 0.0)
        {
            RCLCPP_WARN(get_logger(), "Delta de encoder descartado (dL=%d dR=%d dt=%.3f)",
                        delta_left, delta_right, dt);
            return;
        }

        double delta_left_m = delta_left * meters_per_tick_;
        double delta_right_m = delta_right * meters_per_tick_;

        double delta_s = (delta_left_m + delta_right_m) / 2.0;
        double delta_theta = (delta_right_m - delta_left_m) / track_width_;

        double theta_mid = theta_ + delta_theta / 2.0;
        x_ += delta_s * std::cos(theta_mid);
        y_ += delta_s * std::sin(theta_mid);
        theta_ += delta_theta;

        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

        publish_odometry(stamp, delta_s, delta_theta, dt);
    }

    void publish_odometry(const rclcpp::Time &stamp, double delta_s, double delta_theta, double dt)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = delta_s / dt;
        odom.twist.twist.angular.z = delta_theta / dt;
        odom_pub_->publish(odom);

        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(tf);
        }
    }

    void stop_motors()
    {
        if (fd_ >= 0) write_motor_speeds(0, 0, 0, 0);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<TankI2CDriver>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("tank_i2c_driver"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}