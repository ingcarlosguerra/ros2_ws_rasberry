#ifndef MENTORPI_I2C_MOTOR__HIWONDER_EMM_HPP_
#define MENTORPI_I2C_MOTOR__HIWONDER_EMM_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

class HiwonderEMM
{
public:

    static constexpr uint8_t I2C_ADDR = 0x34;

    static constexpr uint8_t MOTOR_TYPE_ADDR = 20;

    static constexpr uint8_t MOTOR_ENCODER_POLARITY_ADDR = 21;

    static constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 51;

    static constexpr uint8_t MOTOR_ENCODER_TOTAL_ADDR = 60;

    static constexpr uint8_t MOTOR_TYPE_JGB37_520_12V_110RPM = 3;

    explicit HiwonderEMM(
        const std::string &device = "/dev/i2c-1");

    ~HiwonderEMM();

    bool configure();

    bool setMotorSpeeds(
        int8_t m1,
        int8_t m2,
        int8_t m3 = 0,
        int8_t m4 = 0);

    bool stopMotors();

    bool readEncoders(
        int32_t encoders[4]);

private:

    int fd_ = -1;

    bool transfer(
        const uint8_t *send,
        size_t send_length,
        uint8_t *receive,
        size_t receive_length);

    bool writeRegister(
        uint8_t reg,
        const uint8_t *data,
        size_t length);

    bool writeBytes(
        const uint8_t *data,
        size_t length);
};

#endif

