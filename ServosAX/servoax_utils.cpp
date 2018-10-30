#include <string.h>
#include "servoax_utils.h"


// ____________________________________________________________
const char * ServoAXUtils::ErrorCodeToText(tAxErr err)
{
    switch (err)
    {
        case AX_OK:                 return "AX_OK";
        case AX_TIMEOUT:            return "AX_TIMEOUT";
        case AX_INVALID_PACKET:     return "AX_INVALID_PACKET";
        case AX_WRITE_ERROR:        return "AX_WRITE_ERROR";
        case AX_NOT_FOUND:          return "AX_NOT_FOUND";
        case AX_INVALID_VALUE:      return "AX_INVALID_VALUE";
        default:                    return "";
    }
}

// ____________________________________________________________
const char * ServoAXUtils::RegisterAddressToText(unsigned char reg_addr)
{
    switch (reg_addr)
    {
        case AX_REG_MODEL_NUMBER_L:
        case AX_REG_MODEL_NUMBER_H:             return "AX_REG_MODEL_NUMBER";
        case AX_REG_VERSION:                    return "AX_REG_VERSION";
        case AX_REG_ID:                         return "AX_REG_ID";
        case AX_REG_BAUD_RATE:                  return "AX_REG_BAUD_RATE";
        case AX_REG_RETURN_DELAY_TIME:          return "AX_REG_RETURN_DELAY_TIME";
        case AX_REG_CW_ANGLE_LIMIT_L:
        case AX_REG_CW_ANGLE_LIMIT_H:           return "AX_REG_CW_ANGLE_LIMIT";
        case AX_REG_CCW_ANGLE_LIMIT_L:
        case AX_REG_CCW_ANGLE_LIMIT_H:          return "AX_REG_CCW_ANGLE_LIMIT";
        case AX_REG_SYSTEM_DATA2:               return "AX_REG_SYSTEM_DATA2";
        case AX_REG_LIMIT_TEMPERATURE:          return "AX_REG_LIMIT_TEMPERATURE";
        case AX_REG_LOW_LIMIT_VOLTAGE:         return "AX_REG_DOWN_LIMIT_VOLTAGE";
        case AX_REG_HIGH_LIMIT_VOLTAGE:           return "AX_REG_UP_LIMIT_VOLTAGE";
        case AX_REG_MAX_TORQUE_L:
        case AX_REG_MAX_TORQUE_H:               return "AX_REG_MAX_TORQUE";
        case AX_REG_RETURN_LEVEL:               return "AX_REG_RETURN_LEVEL";
        case AX_REG_ALARM_LED:                  return "AX_REG_ALARM_LED";
        case AX_REG_ALARM_SHUTDOWN:             return "AX_REG_ALARM_SHUTDOWN";
        case AX_REG_OPERATING_MODE:             return "AX_REG_OPERATING_MODE";
        case AX_REG_DOWN_CALIBRATION_L:
        case AX_REG_DOWN_CALIBRATION_H:         return "AX_REG_DOWN_CALIBRATION";
        case AX_REG_UP_CALIBRATION_L:
        case AX_REG_UP_CALIBRATION_H:           return "AX_REG_UP_CALIBRATION";
        case AX_REG_TORQUE_ENABLE:              return "AX_REG_TORQUE_ENABLE";
        case AX_REG_LED:                        return "AX_REG_LED";
        case AX_REG_CW_COMPLIANCE_MARGIN:       return "AX_REG_CW_COMPLIANCE_MARGIN";
        case AX_REG_CCW_COMPLIANCE_MARGIN:      return "AX_REG_CCW_COMPLIANCE_MARGIN";
        case AX_REG_CW_COMPLIANCE_SLOPE:        return "AX_REG_CW_COMPLIANCE_SLOPE";
        case AX_REG_CCW_COMPLIANCE_SLOPE:       return "AX_REG_CCW_COMPLIANCE_SLOPE";
        case AX_REG_GOAL_POSITION_L:
        case AX_REG_GOAL_POSITION_H:            return "AX_REG_GOAL_POSITION";
        case AX_REG_GOAL_SPEED_L:
        case AX_REG_GOAL_SPEED_H:               return "AX_REG_GOAL_SPEED";
        case AX_REG_TORQUE_LIMIT_L:
        case AX_REG_TORQUE_LIMIT_H:             return "AX_REG_TORQUE_LIMIT";
        case AX_REG_PRESENT_POSITION_L:
        case AX_REG_PRESENT_POSITION_H:         return "AX_REG_PRESENT_POSITION";
        case AX_REG_PRESENT_SPEED_L:
        case AX_REG_PRESENT_SPEED_H:            return "AX_REG_PRESENT_SPEED";
        case AX_REG_PRESENT_LOAD_L:
        case AX_REG_PRESENT_LOAD_H:             return "AX_REG_PRESENT_LOAD";
        case AX_REG_PRESENT_VOLTAGE:            return "AX_REG_PRESENT_VOLTAGE";
        case AX_REG_PRESENT_TEMPERATURE:        return "AX_REG_PRESENT_TEMPERATURE";
        case AX_REG_REGISTERED_INSTRUCTION:     return "AX_REG_REGISTERED_INSTRUCTION";
        case AX_REG_PAUSE_TIME:                 return "AX_REG_PAUSE_TIME";
        case AX_REG_MOVING:                     return "AX_REG_MOVING";
        case AX_REG_LOCK:                       return "AX_REG_LOCK";
        case AX_REG_PUNCH_L:
        case AX_REG_PUNCH_H:                    return "AX_REG_PUNCH";
        default:                                return "";
    }
}

// ____________________________________________________________
const char * ServoAXUtils::RegisterAddressToInfo(unsigned char reg_addr)
{
    switch (reg_addr)
    {
        case AX_REG_MODEL_NUMBER_L:
        case AX_REG_MODEL_NUMBER_H:             return "Model Number in dynamixel AX family";
        case AX_REG_VERSION:                    return "Firmware version";
        case AX_REG_ID:                         return "Actuator unique ID";
        case AX_REG_BAUD_RATE:                  return "Baudrate: Communication speed";
        case AX_REG_RETURN_DELAY_TIME:          return " The time it takes for the Status Packet to return after the Instruction Packet is sent. The delay time is given by 2uSec * Address5 value.";
        case AX_REG_CW_ANGLE_LIMIT_L:
        case AX_REG_CW_ANGLE_LIMIT_H:           return "Minimum angle limit";
        case AX_REG_CCW_ANGLE_LIMIT_L:
        case AX_REG_CCW_ANGLE_LIMIT_H:          return "Maximum angle limit";
        case AX_REG_SYSTEM_DATA2:               return "Reserved";
        case AX_REG_LIMIT_TEMPERATURE:          return "Limit temperature to consider overheating";
        case AX_REG_LOW_LIMIT_VOLTAGE:         return "Low limit voltage to consider voltage error";
        case AX_REG_HIGH_LIMIT_VOLTAGE:           return "High limit voltage to consider voltage error";
        case AX_REG_MAX_TORQUE_L:
        case AX_REG_MAX_TORQUE_H:               return "The maximum torque output for the Dynamixel actuator (EEPROM : value restored in RAM at startup)";
        case AX_REG_RETURN_LEVEL:               return "Determines whether the Dynamixel actuator will return a Status Packet after receiving an Instruction Packet.";
        case AX_REG_ALARM_LED:                  return "Bitfield : if the corresponding Bit is set to 1, the LED blinks when an Error occurs.";
        case AX_REG_ALARM_SHUTDOWN:             return "Bitfield : if the corresponding Bit is set to a 1, the Dynamixel actuator’s torque will be turned off when an error occurs.";
        case AX_REG_OPERATING_MODE:             return "Reserved";
        case AX_REG_DOWN_CALIBRATION_L:
        case AX_REG_DOWN_CALIBRATION_H:
        case AX_REG_UP_CALIBRATION_L:
        case AX_REG_UP_CALIBRATION_H:           return "Data used for compensating for the differences between potentiometers used in the Dynamixel units. The user cannot change this data.";
        case AX_REG_TORQUE_ENABLE:              return "Enable or disable the torque";
        case AX_REG_LED:                        return "Switch on/off the led";
        case AX_REG_CW_COMPLIANCE_MARGIN:       return "Compliance margin";
        case AX_REG_CCW_COMPLIANCE_MARGIN:      return "Compliance margin";
        case AX_REG_CW_COMPLIANCE_SLOPE:        return "Compliance slope";
        case AX_REG_CCW_COMPLIANCE_SLOPE:       return "Compliance slope";
        case AX_REG_GOAL_POSITION_L:
        case AX_REG_GOAL_POSITION_H:            return "Requested angular position [0; 0x3ff] for the Dynamixel actuator output to move to. Setting this value to 0x3ff moves the output shaft to the position at 300°.";
        case AX_REG_GOAL_SPEED_L:
        case AX_REG_GOAL_SPEED_H:               return "Sets the angular velocity [0; 0x3ff] of the output moving to the Goal Position.";
        case AX_REG_TORQUE_LIMIT_L:
        case AX_REG_TORQUE_LIMIT_H:             return "The maximum torque output for the Dynamixel actuator (RAM)";
        case AX_REG_PRESENT_POSITION_L:
        case AX_REG_PRESENT_POSITION_H:         return "Current angular position [0; 0x3ff]";
        case AX_REG_PRESENT_SPEED_L:
        case AX_REG_PRESENT_SPEED_H:            return "Current angular velocity [0; 0x3ff]";
        case AX_REG_PRESENT_LOAD_L:
        case AX_REG_PRESENT_LOAD_H:             return "Current load. Unit ?? - Range ??";
        case AX_REG_PRESENT_VOLTAGE:            return "Current power supply voltage (factor 10)";
        case AX_REG_PRESENT_TEMPERATURE:        return "Current internal temperature [degC]";
        case AX_REG_REGISTERED_INSTRUCTION:     return "Apply instruction pending by an REG_WRITE instruction. Used to synchronize serveral actuators";
        case AX_REG_PAUSE_TIME:                 return "Reserved";
        case AX_REG_MOVING:                     return "Set to 1 when the Dynamixel actuator is moving by its own power";
        case AX_REG_LOCK:                       return "If set to 1, only Address 0x18 to 0x23 can be written to and other areas cannot.";
        case AX_REG_PUNCH_L:
        case AX_REG_PUNCH_H:                    return "The minimum current supplied to the motor during operation.";
        default:                                return "";
    }
}

// ____________________________________________________________
const char * ServoAXUtils::StatusBitToText(unsigned char bitset)
{
    switch(bitset)
    {
        case AX_STATUS_ERROR_INPUT_VOLTAGE: return "Input Voltage";
        case AX_STATUS_ERROR_ANGLE_LIMIT:   return "Angle Limit";
        case AX_STATUS_ERROR_TEMPERATURE:   return "Temperature";
        case AX_STATUS_ERROR_RANGE:         return "Range";
        case AX_STATUS_ERROR_CHECKSUM:      return "Checksum";
        case AX_STATUS_ERROR_TORQUE:        return "Torque";
        case AX_STATUS_ERROR_INSTRUCTION:   return "Instruction";
        default : return "";
    }
}

// ____________________________________________________________
void ServoAXUtils::StatusCodeToText(unsigned char status, char *buff, unsigned long buff_size)
{
    memset(buff, 0, buff_size);

    if (status == 0) {
        strcpy(buff, "OK");
        return;
    }
    unsigned char error_count=0;
    for (unsigned char i=0; i<8; i++)
    {
        unsigned char bit = 1 << i;
        if (status & bit) {
            if (error_count > 0) strcat(buff, " / ");
            strcat(buff, StatusBitToText(bit));
            error_count++;
        }
    }
}


// ____________________________________________________________
const char *ServoAXUtils::BaudrateToText(unsigned char baudrate)
{
    switch(baudrate)
    {
        case AX_BAUD_1MBPS:     return "1MBPS";
        case AX_BAUD_500KBPS:   return "500KBPS";
        case AX_BAUD_400KBPS:   return "400KBPS";
        case AX_BAUD_250KBPS:   return "250KBPS";
        case AX_BAUD_200KBPS:   return "200KBPS";
        case AX_BAUD_115200BPS: return "115200BPS";
        case AX_BAUD_57600BPS:  return "57600BPS";
        case AX_BAUD_19200BPS:  return "19200BPS";
        case AX_BAUD_9600BPS:   return "9600BPS";
        default : return "";
    }
}

// ____________________________________________________________
bool ServoAXUtils::isRegisterInEEPROM(unsigned char reg_addr)
{
    return (reg_addr <= AX_REG_UP_CALIBRATION_H);
}

// ____________________________________________________________
bool ServoAXUtils::isRegisterInRAM(unsigned char reg_addr)
{
    return ( (reg_addr >= AX_REG_TORQUE_ENABLE) && (reg_addr <= AX_REG_PUNCH_H) );
}

// ____________________________________________________________
bool ServoAXUtils::isRegisterWritable(unsigned char reg_addr)
{
    switch(reg_addr)
    {
        case AX_REG_ID:
        case AX_REG_BAUD_RATE:
        case AX_REG_RETURN_DELAY_TIME:
        case AX_REG_CW_ANGLE_LIMIT_L:
        case AX_REG_CW_ANGLE_LIMIT_H:
        case AX_REG_CCW_ANGLE_LIMIT_L:
        case AX_REG_CCW_ANGLE_LIMIT_H:
        case AX_REG_LIMIT_TEMPERATURE:
        case AX_REG_LOW_LIMIT_VOLTAGE:
        case AX_REG_HIGH_LIMIT_VOLTAGE:
        case AX_REG_MAX_TORQUE_L:
        case AX_REG_MAX_TORQUE_H:
        case AX_REG_RETURN_LEVEL:
        case AX_REG_ALARM_LED:
        case AX_REG_ALARM_SHUTDOWN:
        case AX_REG_TORQUE_ENABLE:
        case AX_REG_LED:
        case AX_REG_CW_COMPLIANCE_MARGIN:
        case AX_REG_CCW_COMPLIANCE_MARGIN:
        case AX_REG_CW_COMPLIANCE_SLOPE:
        case AX_REG_CCW_COMPLIANCE_SLOPE:
        case AX_REG_GOAL_POSITION_L:
        case AX_REG_GOAL_POSITION_H:
        case AX_REG_GOAL_SPEED_L:
        case AX_REG_GOAL_SPEED_H:
        case AX_REG_TORQUE_LIMIT_L:
        case AX_REG_TORQUE_LIMIT_H:
        case AX_REG_REGISTERED_INSTRUCTION:
        case AX_REG_LOCK:
        case AX_REG_PUNCH_L:
        case AX_REG_PUNCH_H:
            return true;
        default :
            return false;
    }
}

// ____________________________________________________________
bool ServoAXUtils::isRegisterReserved(unsigned char reg_addr)
{
    switch(reg_addr)
    {
        case AX_REG_SYSTEM_DATA2:
        case AX_REG_OPERATING_MODE:
        case AX_REG_PAUSE_TIME:
            return true;

        default :
            return false;
    }
}

// ____________________________________________________________
ServoAXUtils::tRegDataType ServoAXUtils::RegisterToDataType(unsigned char reg_addr)
{
    switch(reg_addr)
    {
        case AX_REG_MODEL_NUMBER:
        case AX_REG_CW_ANGLE_LIMIT:
        case AX_REG_CCW_ANGLE_LIMIT:
        case AX_REG_MAX_TORQUE:
        case AX_REG_DOWN_CALIBRATION:
        case AX_REG_UP_CALIBRATION:
        case AX_REG_GOAL_POSITION:
        case AX_REG_GOAL_SPEED:
        case AX_REG_TORQUE_LIMIT:
        case AX_REG_PRESENT_POSITION:
        case AX_REG_PRESENT_SPEED:
        case AX_REG_PRESENT_LOAD:
        case AX_REG_PUNCH:
            return REGTYPE_UINT16;

        case AX_REG_ALARM_LED:
        case AX_REG_TORQUE_ENABLE:
        case AX_REG_LED:
        case AX_REG_MOVING:
        case AX_REG_LOCK:
            return REGTYPE_BOOL;

        default :
            return REGTYPE_UINT8;
    }
}

// ____________________________________________________________
unsigned char ServoAXUtils::getLastRegisterAddress()
{
    return AX_REG_PUNCH_H;
}

