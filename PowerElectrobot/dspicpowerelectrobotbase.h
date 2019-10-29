#ifndef DSPIC_POWER_ELECTROBOT_BASE_H
#define DSPIC_POWER_ELECTROBOT_BASE_H

class dsPicPowerElectrobotBase
{
    // Enum commun avec le SW dsPIC
    typedef enum {
        // Read Only registers
        REG_VERSION_SOFT_MAJ = 0,
        REG_VERSION_SOFT_MIN,
        REG_PTR_REG_LECTURE_I2C,
        REG_NBRE_REGISTRES_LECTURE_I2C,
        REG_EANA_VBAT_H,
        REG_EANA_VBAT_L,
        REG_EANA_MEAS_CURR_VBAT_H,
        REG_EANA_MEAS_CURR_VBAT_L,
        REG_EANA_MEAS_CURR_1_H,
        REG_EANA_MEAS_CURR_1_L,
        REG_EANA_MEAS_CURR_2_H,
        REG_EANA_MEAS_CURR_2_L,
        // Read write registers
        REG_STOR_1,
        REG_STOR_2,
        REG_STOR_3,
        REG_STOR_4,
        REG_STOR_5,
        REG_STOR_6,
        REG_STOR_7,
        REG_STOR_8,
        REG_STOR_PGED,
        REG_STOR_PGEC,
        REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_1_H,
        REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_1_L,
        REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_2_H,
        REG_CALIB_BATTERY_VOLTAGE_PHYS_POINT_2_L,
        REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_1_H,
        REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_1_L,
        REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_2_H,
        REG_CALIB_GLOBAL_CURRENT_PHYS_POINT_2_L,
        REG_CALIB_CURRENT_OUT1_PHYS_POINT_1_H,
        REG_CALIB_CURRENT_OUT1_PHYS_POINT_1_L,
        REG_CALIB_CURRENT_OUT1_PHYS_POINT_2_H,
        REG_CALIB_CURRENT_OUT1_PHYS_POINT_2_L,
        REG_CALIB_CURRENT_OUT2_PHYS_POINT_1_H,
        REG_CALIB_CURRENT_OUT2_PHYS_POINT_1_L,
        REG_CALIB_CURRENT_OUT2_PHYS_POINT_2_H,
        REG_CALIB_CURRENT_OUT2_PHYS_POINT_2_L,

        REG_I2C_8BITS_ADDRESS,
        REG_EEPROM_WRITE_UNPROTECT,
        REG_EEPROM_RESET_FACTORY,
        // _____________________________
        MAX_REGISTRES_NUMBER

    }T_REG_ADDRESS;
    #define EEPROM_WRITE_UNPROTECT_CODE (0x5A)
    #define EEPROM_RESET_FACTORY_CODE (0x69)

public:
    dsPicPowerElectrobotBase();
    virtual ~dsPicPowerElectrobotBase();

    virtual void writeI2C(unsigned char *buff, unsigned short size) = 0;
    virtual void readI2C(unsigned char *dest_buff, unsigned short size) = 0;

    void init(unsigned char _8bits_i2c_addr);

    void periodicCall();

    float getBatteryVoltage();
    float getGlobalCurrent();
    float getCurrentOut1();
    float getCurrentOut2();

    unsigned short getRawBatteryVoltage();
    unsigned short getRawGlobalCurrent();
    unsigned short getRawCurrentOut1();
    unsigned short getRawCurrentOut2();

    unsigned long getCountError();

    void unlockWriteEEPROM();
    void resetFactoryEEPROM();
    void changeI2CAddress(unsigned char _8bits_i2c_addr);

    void setCalibrationBatteryVoltagePoint1(unsigned short voltage_mV);
    void setCalibrationBatteryVoltagePoint2(unsigned short voltage_mV);
    void setCalibrationGlobalCurrentPoint1(unsigned short current_mA);
    void setCalibrationGlobalCurrentPoint2(unsigned short current_mA);
    void setCalibrationCurrentOut1Point1(unsigned short current_mA);
    void setCalibrationCurrentOut1Point2(unsigned short current_mA);
    void setCalibrationCurrentOut2Point1(unsigned short current_mA);
    void setCalibrationCurrentOut2Point2(unsigned short current_mA);

    typedef enum {
        OUTPUT_STOR1 = 0,
        OUTPUT_STOR2,
        OUTPUT_STOR3,
        OUTPUT_STOR4,
        OUTPUT_STOR5,
        OUTPUT_STOR6,
        OUTPUT_STOR7,
        OUTPUT_STOR8
    }tSwitchOutput;
#define OUTPUT_J11 OUTPUT_STOR1
#define OUTPUT_J13 OUTPUT_STOR2
#define OUTPUT_J16 OUTPUT_STOR3
#define OUTPUT_J18 OUTPUT_STOR4
#define OUTPUT_J12 OUTPUT_STOR5
#define OUTPUT_J14 OUTPUT_STOR6
#define OUTPUT_J17 OUTPUT_STOR7
#define OUTPUT_J19 OUTPUT_STOR8

    void setOutput(tSwitchOutput output, bool state);
    void setAllOutputs(bool state);

    unsigned short m_raw_battery_voltage;
    unsigned short m_raw_global_current;
    unsigned short m_raw_current_out1;
    unsigned short m_raw_current_out2;

private :
    void readRegisters();
    void writeRegister(unsigned char reg, unsigned char val);
    void writeWordRegister(unsigned char reg, unsigned short val);
    unsigned long m_compteurErrCom;

    float m_battery_voltage;
    float m_global_current;
    float m_current_out1;
    float m_current_out2;

protected :
    unsigned char m_address;

    float RegisterToBatteryVoltage(unsigned short val);
    float RegisterToGlobalCurrent(unsigned short val);
    float RegisterToCurrentOut1(unsigned short val);
    float RegisterToCurrentOut2(unsigned short val);
};

#endif // DSPIC_POWER_ELECTROBOT_BASE_H
