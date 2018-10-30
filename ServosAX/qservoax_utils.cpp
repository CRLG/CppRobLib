#include <string.h>
#include "qservoax_utils.h"

// ____________________________________________________________
QList<unsigned char> QServoAXUtils::getRegistersList()
{
    QList<unsigned char> list;
    list << getEepromRegistersList()
         << getRamRegistersList();
    return list;
}

// ____________________________________________________________
QList<unsigned char> QServoAXUtils::getEepromRegistersList()
{
    QList<unsigned char> list;
    list << AX_REG_MODEL_NUMBER
         << AX_REG_VERSION
         << AX_REG_ID
         << AX_REG_BAUD_RATE
         << AX_REG_RETURN_DELAY_TIME
         << AX_REG_CW_ANGLE_LIMIT
         << AX_REG_CCW_ANGLE_LIMIT
         << AX_REG_LIMIT_TEMPERATURE
         << AX_REG_LOW_LIMIT_VOLTAGE
         << AX_REG_HIGH_LIMIT_VOLTAGE
         << AX_REG_MAX_TORQUE
         << AX_REG_RETURN_LEVEL
         << AX_REG_ALARM_LED
         << AX_REG_ALARM_SHUTDOWN
         << AX_REG_OPERATING_MODE
         << AX_REG_DOWN_CALIBRATION
         << AX_REG_UP_CALIBRATION;
    return list;
}

// ____________________________________________________________
QList<unsigned char> QServoAXUtils::getRamRegistersList()
{
    QList<unsigned char> list;
    list << AX_REG_TORQUE_ENABLE
         << AX_REG_LED
         << AX_REG_CW_COMPLIANCE_MARGIN
         << AX_REG_CCW_COMPLIANCE_MARGIN
         << AX_REG_CW_COMPLIANCE_SLOPE
         << AX_REG_CCW_COMPLIANCE_SLOPE
         << AX_REG_GOAL_POSITION
         << AX_REG_GOAL_SPEED
         << AX_REG_TORQUE_LIMIT
         << AX_REG_PRESENT_POSITION
         << AX_REG_PRESENT_SPEED
         << AX_REG_PRESENT_LOAD
         << AX_REG_PRESENT_VOLTAGE
         << AX_REG_PRESENT_TEMPERATURE
         << AX_REG_REGISTERED_INSTRUCTION
         << AX_REG_MOVING
         << AX_REG_LOCK
         << AX_REG_PUNCH;
    return list;
}
