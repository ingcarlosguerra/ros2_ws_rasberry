#include "mentorpi_i2c_motor/hiwonder_emm.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <stdexcept>

#include <sys/ioctl.h>
#include <unistd.h>


HiwonderEMM::HiwonderEMM(
    const std::string &device)
{
    fd_ = open(
        device.c_str(),
        O_RDWR);

    if (fd_ < 0) {

        throw std::runtime_error(
            "No se pudo abrir " +
            device +
            ": " +
            std::string(strerror(errno)));
    }

    // No usamos I2C_SLAVE + read/write separados.
    // Las transferencias se realizan mediante I2C_RDWR.
}


HiwonderEMM::~HiwonderEMM()
{
    if (fd_ >= 0) {

        stopMotors();

        close(fd_);

        fd_ = -1;
    }
}


// ============================================================
// TRANSFERENCIA I2C
// ============================================================
//
// Equivalente conceptual al I2C::transfer() utilizado por PX4.
//
// Permite:
//
//   WRITE
//
// o:
//
//   WRITE + READ
//
// dentro de una única llamada I2C_RDWR.
// ============================================================

bool HiwonderEMM::transfer(
    const uint8_t *send,
    size_t send_length,
    uint8_t *receive,
    size_t receive_length)
{
    if (fd_ < 0) {

        return false;
    }

    if (send_length == 0 && receive_length == 0) {

        return false;
    }

    // PX4 utiliza hasta dos mensajes:
    //
    //   mensaje 0 = WRITE
    //   mensaje 1 = READ

    struct i2c_msg messages[2] {};

    int message_count = 0;


    // --------------------------------------------------------
    // WRITE
    // --------------------------------------------------------

    if (send_length > 0) {

        messages[message_count].addr =
            I2C_ADDR;

        messages[message_count].flags =
            0;

        messages[message_count].buf =
            const_cast<uint8_t *>(send);

        messages[message_count].len =
            static_cast<__u16>(send_length);

        message_count++;
    }


    // --------------------------------------------------------
    // READ
    // --------------------------------------------------------

    if (receive_length > 0) {

        messages[message_count].addr =
            I2C_ADDR;

        messages[message_count].flags =
            I2C_M_RD;

        messages[message_count].buf =
            receive;

        messages[message_count].len =
            static_cast<__u16>(receive_length);

        message_count++;
    }


    struct i2c_rdwr_ioctl_data packets {};

    packets.msgs =
        messages;

    packets.nmsgs =
        static_cast<__u32>(message_count);


    // --------------------------------------------------------
    // REINTENTOS
    // --------------------------------------------------------

    constexpr int MAX_RETRIES = 3;


    for (int attempt = 0;
         attempt < MAX_RETRIES;
         ++attempt) {

        int result =
            ioctl(
                fd_,
                I2C_RDWR,
                &packets);


        if (result == message_count) {

            return true;
        }


        if (result < 0) {

            // EINTR: la llamada fue interrumpida.
            // Podemos reintentarla inmediatamente.

            if (errno == EINTR) {

                continue;
            }
        }


        // Pequeña pausa antes del siguiente intento.

        usleep(1000);
    }


    return false;
}


// ============================================================
// CONFIGURACIÓN
// ============================================================

bool HiwonderEMM::configure()
{
    // --------------------------------------------------------
    // MOTOR TYPE
    // Registro 20
    // --------------------------------------------------------

    const uint8_t motor_type =
        MOTOR_TYPE_JGB37_520_12V_110RPM;


    if (!writeRegister(
            MOTOR_TYPE_ADDR,
            &motor_type,
            1)) {

        return false;
    }


    usleep(5000);


    // --------------------------------------------------------
    // ENCODER POLARITY
    // Registro 21
    // --------------------------------------------------------

    const uint8_t polarity = 0;


    if (!writeRegister(
            MOTOR_ENCODER_POLARITY_ADDR,
            &polarity,
            1)) {

        return false;
    }


    usleep(5000);


    return true;
}


// ============================================================
// VELOCIDADES
// ============================================================

bool HiwonderEMM::setMotorSpeeds(
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


    return writeBytes(
        data,
        sizeof(data));
}


// ============================================================
// STOP
// ============================================================

bool HiwonderEMM::stopMotors()
{
    return setMotorSpeeds(
        0,
        0,
        0,
        0);
}


// ============================================================
// ENCODERS
// ============================================================
//
// Esta es la parte más importante.
//
// En lugar de:
//
//     write(reg)
//     read(buffer)
//
// hacemos:
//
//     I2C_RDWR
//       WRITE: registro 60
//       READ : 16 bytes
//
// Esto corresponde al patrón usado por PX4.
// ============================================================

bool HiwonderEMM::readEncoders(
    int32_t encoders[4])
{
    const uint8_t reg =
        MOTOR_ENCODER_TOTAL_ADDR;


    uint8_t buffer[16] = {};


    if (!transfer(
            &reg,
            1,
            buffer,
            sizeof(buffer))) {

        return false;
    }


    for (int i = 0; i < 4; ++i) {

        uint32_t value =
            static_cast<uint32_t>(
                buffer[i * 4]) |

            (static_cast<uint32_t>(
                buffer[i * 4 + 1]) << 8) |

            (static_cast<uint32_t>(
                buffer[i * 4 + 2]) << 16) |

            (static_cast<uint32_t>(
                buffer[i * 4 + 3]) << 24);


        encoders[i] =
            static_cast<int32_t>(value);
    }


    return true;
}


// ============================================================
// WRITE REGISTER
// ============================================================

bool HiwonderEMM::writeRegister(
    uint8_t reg,
    const uint8_t *data,
    size_t length)
{
    uint8_t buffer[32];


    if (length + 1 > sizeof(buffer)) {

        return false;
    }


    buffer[0] = reg;


    std::memcpy(
        &buffer[1],
        data,
        length);


    return transfer(
        buffer,
        length + 1,
        nullptr,
        0);
}


// ============================================================
// WRITE BYTES
// ============================================================

bool HiwonderEMM::writeBytes(
    const uint8_t *data,
    size_t length)
{
    return transfer(
        data,
        length,
        nullptr,
        0);
}
