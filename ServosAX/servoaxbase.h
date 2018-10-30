#ifndef _SERVOAX_BASE_H
#define _SERVOAX_BASE_H

// ====================================================
//                  AX12 REGISTER DEFINITION
// ====================================================
// EEPROM AREA
#define AX_REG_MODEL_NUMBER_L           0
#define AX_REG_MODEL_NUMBER             AX_REG_MODEL_NUMBER_L
#define AX_REG_MODEL_NUMBER_H           1
#define AX_REG_VERSION                  2
#define AX_REG_ID                       3
#define AX_REG_BAUD_RATE                4
#define AX_REG_RETURN_DELAY_TIME        5
#define AX_REG_CW_ANGLE_LIMIT_L         6
#define AX_REG_CW_ANGLE_LIMIT           AX_REG_CW_ANGLE_LIMIT_L
#define AX_REG_CW_ANGLE_LIMIT_H         7
#define AX_REG_CCW_ANGLE_LIMIT_L        8
#define AX_REG_CCW_ANGLE_LIMIT          AX_REG_CCW_ANGLE_LIMIT_L
#define AX_REG_CCW_ANGLE_LIMIT_H        9
#define AX_REG_SYSTEM_DATA2             10
#define AX_REG_LIMIT_TEMPERATURE        11
#define AX_REG_LOW_LIMIT_VOLTAGE        12
#define AX_REG_HIGH_LIMIT_VOLTAGE       13
#define AX_REG_MAX_TORQUE_L             14
#define AX_REG_MAX_TORQUE               AX_REG_MAX_TORQUE_L
#define AX_REG_MAX_TORQUE_H             15
#define AX_REG_RETURN_LEVEL             16
#define AX_REG_ALARM_LED                17
#define AX_REG_ALARM_SHUTDOWN           18
#define AX_REG_OPERATING_MODE           19
#define AX_REG_DOWN_CALIBRATION_L       20
#define AX_REG_DOWN_CALIBRATION         AX_REG_DOWN_CALIBRATION_L
#define AX_REG_DOWN_CALIBRATION_H       21
#define AX_REG_UP_CALIBRATION_L         22
#define AX_REG_UP_CALIBRATION           AX_REG_UP_CALIBRATION_L
#define AX_REG_UP_CALIBRATION_H         23
// RAM AREA
#define AX_REG_TORQUE_ENABLE            24
#define AX_REG_LED                      25
#define AX_REG_CW_COMPLIANCE_MARGIN     26
#define AX_REG_CCW_COMPLIANCE_MARGIN    27
#define AX_REG_CW_COMPLIANCE_SLOPE      28
#define AX_REG_CCW_COMPLIANCE_SLOPE     29
#define AX_REG_GOAL_POSITION_L          30
#define AX_REG_GOAL_POSITION            AX_REG_GOAL_POSITION_L
#define AX_REG_GOAL_POSITION_H          31
#define AX_REG_GOAL_SPEED_L             32
#define AX_REG_GOAL_SPEED               AX_REG_GOAL_SPEED_L
#define AX_REG_GOAL_SPEED_H             33
#define AX_REG_TORQUE_LIMIT_L           34
#define AX_REG_TORQUE_LIMIT             AX_REG_TORQUE_LIMIT_L
#define AX_REG_TORQUE_LIMIT_H           35
#define AX_REG_PRESENT_POSITION_L       36
#define AX_REG_PRESENT_POSITION         AX_REG_PRESENT_POSITION_L
#define AX_REG_PRESENT_POSITION_H       37
#define AX_REG_PRESENT_SPEED_L          38
#define AX_REG_PRESENT_SPEED            AX_REG_PRESENT_SPEED_L
#define AX_REG_PRESENT_SPEED_H          39
#define AX_REG_PRESENT_LOAD_L           40
#define AX_REG_PRESENT_LOAD             AX_REG_PRESENT_LOAD_L
#define AX_REG_PRESENT_LOAD_H           41
#define AX_REG_PRESENT_VOLTAGE          42
#define AX_REG_PRESENT_TEMPERATURE      43
#define AX_REG_REGISTERED_INSTRUCTION   44
#define AX_REG_PAUSE_TIME               45
#define AX_REG_MOVING                   46
#define AX_REG_LOCK                     47
#define AX_REG_PUNCH_L                  48
#define AX_REG_PUNCH                    AX_REG_PUNCH_L
#define AX_REG_PUNCH_H                  49

// ====================================================
//            AX12 INSTRUCTIONS DEFINITION
// ====================================================
#define AX_INSTRUCTION_PING             0x1
#define AX_INSTRUCTION_READ_DATA        0x2
#define AX_INSTRUCTION_WRITE_DATA       0x3
#define AX_INSTRUCTION_REG_WRITE        0x4
#define AX_INSTRUCTION_ACTION           0x5
#define AX_INSTRUCTION_RESET            0x6
#define AX_INSTRUCTION_RESET_SYNC_WRITE 0x83

// ====================================================
//                    AX STATUS CODE
// ====================================================
// Bitfield
#define AX_STATUS_ERROR_INPUT_VOLTAGE   0x01
#define AX_STATUS_ERROR_ANGLE_LIMIT     0x02
#define AX_STATUS_ERROR_TEMPERATURE     0x04
#define AX_STATUS_ERROR_RANGE           0x08
#define AX_STATUS_ERROR_CHECKSUM        0x10
#define AX_STATUS_ERROR_TORQUE          0x20
#define AX_STATUS_ERROR_INSTRUCTION     0x40

// ====================================================
//                    AX BAUD RATE
// ====================================================
#define AX_BAUD_1MBPS       0x01
#define AX_BAUD_500KBPS     0x03
#define AX_BAUD_400KBPS     0x04
#define AX_BAUD_250KBPS     0x07
#define AX_BAUD_200KBPS     0x09
#define AX_BAUD_115200BPS   0x10
#define AX_BAUD_57600BPS    0x22
#define AX_BAUD_19200BPS    0x67
#define AX_BAUD_9600BPS     0xCF


// ====================================================
//               INTERNAL DRIVER ERROR CODE
// ====================================================
typedef enum {
    AX_OK = 0,
    AX_TIMEOUT,
    AX_INVALID_PACKET,
    AX_WRITE_ERROR,
    AX_NOT_FOUND,
    AX_INVALID_VALUE
}tAxErr;

// ====================================================
//               RUNNING MODE
// ====================================================
#define AX_MODE_ROTATIONAL      0
#define AX_MODE_FREE_RUNNING    1

// ====================================================
//
// ====================================================
#define AX_BROADCAST_ID 0xFE


// ====================================================
//          ABSTRACT CLASS FOR AX SERVOMOTORS
// ====================================================
class ServoAXBase
{
public:
    ServoAXBase();
    virtual ~ServoAXBase();

    // pure virtual methods for hardware abstraction.
    // to be implemented on specific hardware.
    virtual tAxErr read(unsigned char *buff_data, unsigned char size) = 0;
    virtual tAxErr write(unsigned char *buff_data, unsigned char size) = 0;
    virtual tAxErr flushSerialInput() = 0;
    virtual tAxErr waitTransmitComplete() = 0;
    virtual tAxErr setTxEnable(bool state) = 0;
    virtual void delay_us(unsigned long delay) = 0;

    // High level API
    virtual bool isPresent(unsigned char id);
    virtual tAxErr setPosition(unsigned char id, unsigned short position);
    virtual unsigned short getPosition(unsigned char id, tAxErr *err_status=nullptr);
    virtual tAxErr setSpeed(unsigned char id, unsigned short speed);
    virtual tAxErr setPositionSpeed(unsigned char id, unsigned short position, unsigned short speed);
    virtual bool isMoving(unsigned char id, tAxErr *err_status=nullptr);
    virtual tAxErr changeID(unsigned char old_id, unsigned char new_id);
    virtual tAxErr enableTorque(unsigned char id, bool on_off=true);
    virtual tAxErr setMode(unsigned char id, unsigned char mode);
    virtual tAxErr setLimitPositionMin(unsigned char id, unsigned short pos);
    virtual tAxErr setLimitPositionMax(unsigned char id, unsigned short pos);

    // Medium level API
    tAxErr read8bitsRegister(unsigned char id, unsigned char reg_addr, unsigned char *value, unsigned char *err_status=nullptr);
    tAxErr read16bitsRegister(unsigned char id, unsigned char reg_addr, unsigned short *value, unsigned char *err_status=nullptr);
    tAxErr write8bitsRegister(unsigned char id, unsigned char reg_addr, unsigned char value);
    tAxErr write16bitsRegister(unsigned char id, unsigned char reg_addr, unsigned short value);

private:
    //! Create an instruction packet based on useful parameters
    tAxErr createWriteDataPacket(unsigned char id, unsigned char *parameters, unsigned char size, bool send_packet=true);
    //! Create a read packet based on request parameters
    tAxErr createReadDataPacket(unsigned char id, unsigned char start_addr, unsigned char length);

    //! Compute checksum from the packet
    unsigned char getChecksum(unsigned char *packet);

    //! Check if packet is valid
    bool isPacketValid(unsigned char *packet);

    tAxErr _write(unsigned char *buff_data, unsigned char size);

    //! Communication is half-duplex so the same packet buffer can be used for both transmit or receive communication without conflict
    unsigned char m_packet[32];
};

#endif // _SERVOAX_BASE_H
