#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include "tf2_ros/transform_broadcaster.h"

#include "mentorpi_i2c_motor/hiwonder_emm.hpp"


using namespace std::chrono_literals;


class MentorPiI2CMotorNode
    : public rclcpp::Node
{
public:

    MentorPiI2CMotorNode()
        : Node("mentorpi_i2c_motor_node")
    {

        // ==================================================
        // PARAMETROS
        // ==================================================

        declare_parameter(
            "wheel_diameter",
            0.050);

        declare_parameter(
            "wheel_separation",
            0.100);

        declare_parameter(
            "encoder_ticks_per_rev",
            1980.0);

        declare_parameter(
            "max_linear_speed",
            0.30);

        declare_parameter(
            "i2c_device",
            "/dev/i2c-1");

        declare_parameter(
            "left_motor_sign",
            -1);

        declare_parameter(
            "right_motor_sign",
            -1);

        declare_parameter(
            "cmd_timeout",
            0.5);


        get_parameter(
            "wheel_diameter",
            wheel_diameter_);

        get_parameter(
            "wheel_separation",
            wheel_separation_);

        get_parameter(
            "encoder_ticks_per_rev",
            encoder_ticks_per_rev_);

        get_parameter(
            "max_linear_speed",
            max_linear_speed_);

        get_parameter(
            "i2c_device",
            i2c_device_);

        get_parameter(
            "left_motor_sign",
            left_motor_sign_);

        get_parameter(
            "right_motor_sign",
            right_motor_sign_);

        get_parameter(
            "cmd_timeout",
            cmd_timeout_);


        // ==================================================
        // I2C DRIVER
        // ==================================================

        try {

            motor_ =
                std::make_unique<HiwonderEMM>(
                    i2c_device_);


            if (!motor_->configure()) {

                throw std::runtime_error(
                    "No se pudo configurar "
                    "el Hiwonder EMM");
            }


            RCLCPP_INFO(
                get_logger(),
                "Hiwonder EMM configurado");


        } catch (
            const std::exception &e) {

            RCLCPP_FATAL(
                get_logger(),
                "Error I2C: %s",
                e.what());

            throw;
        }


        // ==================================================
        // /cmd_vel
        // ==================================================

        cmd_vel_sub_ =
            create_subscription<
                geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &MentorPiI2CMotorNode::cmdVelCallback,
                    this,
                    std::placeholders::_1));


        // ==================================================
        // /odom
        // ==================================================

        odom_pub_ =
            create_publisher<
                nav_msgs::msg::Odometry>(
                "/odom",
                10);


        // ==================================================
        // TF
        // ==================================================

        tf_broadcaster_ =
            std::make_unique<
                tf2_ros::TransformBroadcaster>(
                *this);


        // ==================================================
        // TIMER
        // 50 Hz
        // ==================================================

        last_cmd_time_ =
            now();


        timer_ =
            create_wall_timer(
                20ms,
                std::bind(
                    &MentorPiI2CMotorNode::update,
                    this));


        RCLCPP_INFO(
            get_logger(),
            "================================");

        RCLCPP_INFO(
            get_logger(),
            "MentorPi I2C Motor Node");

        RCLCPP_INFO(
            get_logger(),
            "Wheel diameter: %.3f m",
            wheel_diameter_);

        RCLCPP_INFO(
            get_logger(),
            "Wheel separation: %.3f m",
            wheel_separation_);

        RCLCPP_INFO(
            get_logger(),
            "Encoder ticks/rev: %.1f",
            encoder_ticks_per_rev_);

        RCLCPP_INFO(
            get_logger(),
            "Max speed: %.2f m/s",
            max_linear_speed_);

        RCLCPP_INFO(
            get_logger(),
            "I2C: %s",
            i2c_device_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "================================");
    }


    ~MentorPiI2CMotorNode()
    {
        if (motor_) {

            motor_->stopMotors();
        }
    }


private:

    // ==================================================
    // CMD VEL
    // ==================================================

    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {

        double linear =
            msg->linear.x;


        double angular =
            msg->angular.z;


        // ----------------------------------------------
        // Diferencial
        // ----------------------------------------------

        double left =
            linear -
            angular *
            wheel_separation_ /
            2.0;


        double right =
            linear +
            angular *
            wheel_separation_ /
            2.0;


        // ----------------------------------------------
        // Limite de velocidad de las ruedas
        // ----------------------------------------------

        double maximum =
            std::max(
                std::abs(left),
                std::abs(right));


        if (
            maximum >
            max_linear_speed_
        ) {

            double scale =
                max_linear_speed_ /
                maximum;


            left *= scale;

            right *= scale;
        }


        // ----------------------------------------------
        // m/s → ticks/10 ms
        // ----------------------------------------------

        double left_ticks =
            velocityToTicks(left);


        double right_ticks =
            velocityToTicks(right);


        // ----------------------------------------------
        // Limite del EMM
        // ----------------------------------------------

        left_ticks =
            std::clamp(
                left_ticks,
                -127.0,
                127.0);


        right_ticks =
            std::clamp(
                right_ticks,
                -127.0,
                127.0);


        // ----------------------------------------------
        // Signo del motor
        // ----------------------------------------------

        int8_t motor_left =
            static_cast<int8_t>(
                std::round(
                    left_ticks *
                    left_motor_sign_));


        int8_t motor_right =
            static_cast<int8_t>(
                std::round(
                    right_ticks *
                    right_motor_sign_));


        // ----------------------------------------------
        // Enviar al EMM
        // ----------------------------------------------

        motor_->setMotorSpeeds(
            motor_left,
            motor_right,
            0,
            0);


        last_cmd_time_ =
            now();
    }


    // ==================================================
    // VELOCIDAD → TICKS
    // ==================================================

    double velocityToTicks(
        double velocity)
    {

        const double circumference =
            M_PI *
            wheel_diameter_;


        return
            velocity *
            encoder_ticks_per_rev_ *
            0.01 /
            circumference;
    }


    // ==================================================
    // UPDATE
    // ==================================================

    void update()
    {

        // ----------------------------------------------
        // WATCHDOG
        // ----------------------------------------------

        double age =
            (now() -
             last_cmd_time_).seconds();


        if (
            age >
            cmd_timeout_
        ) {

            motor_->stopMotors();
        }


        // ----------------------------------------------
        // Leer encoders
        // ----------------------------------------------

        int32_t encoders[4] =
        {
            0,
            0,
            0,
            0
        };


        if (
            !motor_->readEncoders(
                encoders)
        ) {

            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Error leyendo encoders");

            return;
        }


        // ----------------------------------------------
        // M1 = izquierda
        // M2 = derecha
        // ----------------------------------------------

        int32_t left =
            encoders[0];


        int32_t right =
            encoders[1];


        // ----------------------------------------------
        // Primera lectura
        // ----------------------------------------------

        if (!initialized_) {

            previous_left_encoder_ =
                left;

            previous_right_encoder_ =
                right;

            last_update_time_ =
                now();

            initialized_ =
                true;

            return;
        }


        // ----------------------------------------------
        // Delta
        // ----------------------------------------------

        int32_t delta_left =
            left -
            previous_left_encoder_;


        int32_t delta_right =
            right -
            previous_right_encoder_;


        previous_left_encoder_ =
            left;


        previous_right_encoder_ =
            right;


        // ----------------------------------------------
        // Distancia/tick
        // ----------------------------------------------

        const double meters_per_tick =
            M_PI *
            wheel_diameter_ /
            encoder_ticks_per_rev_;


        double distance_left =
            delta_left *
            meters_per_tick;


        double distance_right =
            delta_right *
            meters_per_tick;


        // ----------------------------------------------
        // Signo del encoder
        // ----------------------------------------------

        distance_left *=
            left_motor_sign_;


        distance_right *=
            right_motor_sign_;


        // ----------------------------------------------
        // Diferencial
        // ----------------------------------------------

        double distance_center =
            (distance_left +
             distance_right) /
            2.0;


        double delta_theta =
            (distance_right -
             distance_left) /
            wheel_separation_;


        // ----------------------------------------------
        // Tiempo
        // ----------------------------------------------

        rclcpp::Time current_time =
            now();


        double dt =
            (current_time -
             last_update_time_).seconds();


        last_update_time_ =
            current_time;


        if (
            dt <= 0.0 ||
            dt > 1.0
        ) {

            return;
        }


        // ----------------------------------------------
        // Pose
        // ----------------------------------------------

        double theta_mid =
            theta_ +
            delta_theta /
            2.0;


        x_ +=
            distance_center *
            std::cos(theta_mid);


        y_ +=
            distance_center *
            std::sin(theta_mid);


        theta_ +=
            delta_theta;


        // ----------------------------------------------
        // Normalizar theta
        // ----------------------------------------------

        theta_ =
            std::atan2(
                std::sin(theta_),
                std::cos(theta_));


        // ----------------------------------------------
        // Velocidades
        // ----------------------------------------------

        double linear_velocity =
            distance_center /
            dt;


        double angular_velocity =
            delta_theta /
            dt;


        // ----------------------------------------------
        // Publicar
        // ----------------------------------------------

        publishOdometry(
            current_time,
            linear_velocity,
            angular_velocity);
    }


    // ==================================================
    // ODOM + TF
    // ==================================================

    void publishOdometry(
        const rclcpp::Time &stamp,
        double linear_velocity,
        double angular_velocity)
    {

        tf2::Quaternion q;


        q.setRPY(
            0.0,
            0.0,
            theta_);


        // ----------------------------------------------
        // ODOM
        // ----------------------------------------------

        nav_msgs::msg::Odometry odom;


        odom.header.stamp =
            stamp;


        odom.header.frame_id =
            "odom";


        odom.child_frame_id =
            "base_link";


        odom.pose.pose.position.x =
            x_;


        odom.pose.pose.position.y =
            y_;


        odom.pose.pose.position.z =
            0.0;


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


        odom.twist.twist.angular.z =
            angular_velocity;


        odom_pub_->publish(
            odom);


        // ----------------------------------------------
        // TF odom → base_link
        // ----------------------------------------------

        geometry_msgs::msg::TransformStamped tf;


        tf.header.stamp =
            stamp;


        tf.header.frame_id =
            "odom";


        tf.child_frame_id =
            "base_link";


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


        tf_broadcaster_->sendTransform(
            tf);
    }


    // ==================================================
    // VARIABLES
    // ==================================================

    std::unique_ptr<
        HiwonderEMM> motor_;


    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_sub_;


    rclcpp::Publisher<
        nav_msgs::msg::Odometry>::SharedPtr
        odom_pub_;


    std::unique_ptr<
        tf2_ros::TransformBroadcaster>
        tf_broadcaster_;


    rclcpp::TimerBase::SharedPtr
        timer_;


    rclcpp::Time last_cmd_time_;

    rclcpp::Time last_update_time_;


    bool initialized_ =
        false;


    int32_t previous_left_encoder_ =
        0;


    int32_t previous_right_encoder_ =
        0;


    double x_ =
        0.0;


    double y_ =
        0.0;


    double theta_ =
        0.0;


    // ----------------------------------------------
    // Robot
    // ----------------------------------------------

    double wheel_diameter_ =
        0.050;


    double wheel_separation_ =
        0.100;


    double encoder_ticks_per_rev_ =
        1980.0;


    double max_linear_speed_ =
        0.30;


    // ----------------------------------------------
    // I2C
    // ----------------------------------------------

    std::string i2c_device_ =
        "/dev/i2c-1";


    // ----------------------------------------------
    // Motor direction
    // ----------------------------------------------

    int left_motor_sign_ =
        -1;


    int right_motor_sign_ =
        -1;


    // ----------------------------------------------
    // Watchdog
    // ----------------------------------------------

    double cmd_timeout_ =
        0.5;
};


// ======================================================

int main(
    int argc,
    char *argv[])
{
    rclcpp::init(
        argc,
        argv);


    auto node =
        std::make_shared<
            MentorPiI2CMotorNode>();


    rclcpp::spin(
        node);


    rclcpp::shutdown();


    return 0;
}
