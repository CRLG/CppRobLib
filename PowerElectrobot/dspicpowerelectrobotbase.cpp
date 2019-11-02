#include "dspicpowerelectrobotbase.h"

dsPicPowerElectrobotBase::dsPicPowerElectrobotBase()
    : m_outputs_port(0),
      m_compteurErrCom(0)
{

}

dsPicPowerElectrobotBase::~dsPicPowerElectrobotBase()
{

}

// ______________________________________________
void dsPicPowerElectrobotBase::init(unsigned char _8bits_i2c_addr)
{
    m_address = _8bits_i2c_addr;
}

// ______________________________________________
void dsPicPowerElectrobotBase::periodicCall()
{
    readRegisters();
}

// ______________________________________________
void dsPicPowerElectrobotBase::readRegisters()
{
    unsigned char checksum=0;
    unsigned char i;
    unsigned char buff[16];

    buff[8] = 0xFF; // Pour être certain de ne pas conserver un bon checksum du coup d'avant
    readI2C(buff, 9);
    // Calcul le checksum à partir des données reçues
    for (i=0; i<8; i++) {
      checksum+= buff[i];
    }

    // Ne prend en compte les données reçues que si le checksum est bon
    if (checksum == buff[8]) {
        unsigned short val;
        val = (((unsigned short)buff[0])<<8) + buff[1];
        m_raw_battery_voltage = val;
        m_battery_voltage = RegisterToBatteryVoltage(val);

        val = (((unsigned short)buff[2])<<8) + buff[3];
        m_raw_global_current = val;
        m_global_current = RegisterToGlobalCurrent(val);

        val = (((unsigned short)buff[4])<<8) + buff[5];
        m_raw_current_out1 = val;
        m_current_out1 = RegisterToCurrentOut1(val);

        val = (((unsigned short)buff[6])<<8) + buff[7];
        m_raw_current_out2 = val;
        m_current_out2 = RegisterToCurrentOut2(val);
    }
    else {
      m_compteurErrCom++;
    }
}

// ______________________________________________
void dsPicPowerElectrobotBase::writeRegister(unsigned char reg, unsigned char val)
{
    unsigned char buff[4];
    buff[0] = 3;
    buff[1] = reg;
    buff[2] = val;
    buff[3] =  buff[0] + buff[1] + buff[2];
    writeI2C(buff, 4);
}

// ______________________________________________
void dsPicPowerElectrobotBase::writeWordRegister(unsigned char reg, unsigned short val)
{
    unsigned char buff[5];
    buff[0] = 4;
    buff[1] = reg;
    buff[2] = val>>8;   // MSB
    buff[3] = val;      // LSB
    buff[4] =  buff[0] + buff[1] + buff[2] + buff[3];
    writeI2C(buff, 5);
}


// ______________________________________________
float dsPicPowerElectrobotBase::RegisterToBatteryVoltage(unsigned short val)
{
    return (float)val / 1000.0f;    // [mV] -> [V]
}

// ______________________________________________
float dsPicPowerElectrobotBase::RegisterToGlobalCurrent(unsigned short val)
{
    return (float)val / 1000.0f;    // [mA] -> [A]
}

// ______________________________________________
float dsPicPowerElectrobotBase::RegisterToCurrentOut1(unsigned short val)
{
    return (float)val / 1000.0f;    // [mA] -> [A]
}

// ______________________________________________
float dsPicPowerElectrobotBase::RegisterToCurrentOut2(unsigned short val)
{
    return (float)val / 1000.0f;
}

// ______________________________________________
float dsPicPowerElectrobotBase::getBatteryVoltage()
{
    return m_battery_voltage;
}

// ______________________________________________
float dsPicPowerElectrobotBase::getGlobalCurrent()
{
    return m_global_current;
}

// ______________________________________________
float dsPicPowerElectrobotBase::getCurrentOut1()
{
    return m_current_out1;
}

// ______________________________________________
float dsPicPowerElectrobotBase::getCurrentOut2()
{
    return m_current_out2;
}

// ______________________________________________
unsigned short dsPicPowerElectrobotBase::getRawBatteryVoltage()
{
    return m_raw_battery_voltage;
}
// ______________________________________________
unsigned short dsPicPowerElectrobotBase::getRawGlobalCurrent()
{
    return m_raw_global_current;
}
// ______________________________________________
unsigned short dsPicPowerElectrobotBase::getRawCurrentOut1()
{
    return m_raw_current_out1;
}
// ______________________________________________
unsigned short dsPicPowerElectrobotBase::getRawCurrentOut2()
{
    return m_raw_current_out2;
}

// ______________________________________________
void dsPicPowerElectrobotBase::setOutput(tSwitchOutput output, bool state)
{
    if (output > OUTPUT_STOR8) return;
    switch(output) {
        case OUTPUT_STOR1 : writeRegister(REG_STOR_1, state); setBitPort(0, state); break;
        case OUTPUT_STOR2 : writeRegister(REG_STOR_2, state); setBitPort(1, state); break;
        case OUTPUT_STOR3 : writeRegister(REG_STOR_3, state); setBitPort(2, state); break;
        case OUTPUT_STOR4 : writeRegister(REG_STOR_4, state); setBitPort(3, state); break;
        case OUTPUT_STOR5 : writeRegister(REG_STOR_5, state); setBitPort(4, state); break;
        case OUTPUT_STOR6 : writeRegister(REG_STOR_6, state); setBitPort(5, state); break;
        case OUTPUT_STOR7 : writeRegister(REG_STOR_7, state); setBitPort(6, state); break;
        case OUTPUT_STOR8 : writeRegister(REG_STOR_8, state); setBitPort(7, state); break;
        default : break;
    }
}

// ______________________________________________
void dsPicPowerElectrobotBase::setOutputPort(unsigned char val)
{
    writeRegister(REG_PORT_STOR_1to8, val);
    m_outputs_port = val;
}

// ______________________________________________
void dsPicPowerElectrobotBase::setBitPort(unsigned char bit, bool val)
{
    if (val) {
        m_outputs_port |= (1<<bit);
    }
    else {
        m_outputs_port &= ~(1<<bit);
    }
}

// ______________________________________________
void dsPicPowerElectrobotBase::setAllOutputs(bool state)
{
    for (unsigned int i=OUTPUT_STOR1; i<=OUTPUT_STOR8; i++) {
        setOutput((tSwitchOutput)i, state);
    }
}

// ______________________________________________
void dsPicPowerElectrobotBase::refreshOuptuts()
{
    writeRegister(REG_PORT_STOR_1to8, m_outputs_port);
}

// ______________________________________________
unsigned long dsPicPowerElectrobotBase::getCountError()
{
    return m_compteurErrCom;
}

// ______________________________________________
void dsPicPowerElectrobotBase::unlockWriteEEPROM()
{
    writeRegister(REG_EEPROM_WRITE_UNPROTECT, EEPROM_WRITE_UNPROTECT_CODE);
}

// ______________________________________________
void dsPicPowerElectrobotBase::resetFactoryEEPROM()
{
    writeRegister(REG_EEPROM_RESET_FACTORY, EEPROM_RESET_FACTORY_CODE);
}

// ______________________________________________
void dsPicPowerElectrobotBase::changeI2CAddress(unsigned char _8bits_i2c_addr)
{
    writeRegister(REG_I2C_8BITS_ADDRESS, _8bits_i2c_addr);
}


// ==============================================
//                      CALIBRATION
// ==============================================

void dsPicPowerElectrobotBase::setCalibrationBatteryVoltagePoint1(unsigned short voltage_mV)
{
    writeWordRegister(REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_1_H, voltage_mV);
}
void dsPicPowerElectrobotBase::setCalibrationBatteryVoltagePoint2(unsigned short voltage_mV)
{
    writeWordRegister(REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_2_H, voltage_mV);
}

void dsPicPowerElectrobotBase::setCalibrationGlobalCurrentPoint1(unsigned short current_mA)
{
    writeWordRegister(REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_1_H, current_mA);
}

void dsPicPowerElectrobotBase::setCalibrationGlobalCurrentPoint2(unsigned short current_mA)
{
    writeWordRegister(REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_2_H, current_mA);
}

void dsPicPowerElectrobotBase::setCalibrationCurrentOut1Point1(unsigned short current_mA)
{
    writeWordRegister(REG_CALIB_CURRENT_OUT1_PHYS_POINT_1_H, current_mA);
}

void dsPicPowerElectrobotBase::setCalibrationCurrentOut1Point2(unsigned short current_mA)
{
    writeWordRegister(REG_CALIB_CURRENT_OUT1_PHYS_POINT_2_H, current_mA);
}

void dsPicPowerElectrobotBase::setCalibrationCurrentOut2Point1(unsigned short current_mA)
{
    writeWordRegister(REG_CALIB_CURRENT_OUT2_PHYS_POINT_1_H, current_mA);
}

void dsPicPowerElectrobotBase::setCalibrationCurrentOut2Point2(unsigned short current_mA)
{
    writeWordRegister(REG_CALIB_CURRENT_OUT2_PHYS_POINT_2_H, current_mA);
}
