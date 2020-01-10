#ifndef DSPIC_POWER_ELECTROBOT_BASE_H
#define DSPIC_POWER_ELECTROBOT_BASE_H

/*  =============================
 *    DOCUMENTATION DU DRIVER
    =============================
 * Cette classe est le driver de communication avec le dsPIC de la carte PowerElectrobot.
 * Elle est indépendante de toute plateforme hardware et peut être utilisée sur toute
 *  plateforme C++ standard.
 * C'est une classe abstraite pure (ne peut être instanciée directement) et certaines méthodes
 *  doivent être ré-implémentées par l'applicatif pour faire le lien avec le hardware :
 *     > writeI2C : envoie d'un buffer de donnée sur le bus I2C de la plateforme.
 *     > readI2C : lecture d'un certain nombre de données sur le bus I2C de la plateforme.
 *
 * Avec cette classe, il est possible d'avoir accès aux mesures effectuées par la carte
 *     > Mesures de tension batterie : battery_voltage
 *     > Mesure de courant global : global_current
 *     > Mesure de courant sur la sortie mesurée n°1 : current_out1
 *     > Mesure de courant sur la sortie mesurée n°2 : current_out2
 * Les données "raw" sont les valeurs brutes renvoyées par la carte en [mV] pour la tension et [mA] pour les courants.
 * L'API est également fournie pour renvoyer directement les valeurs en Volts ou Ampères
 * La méthode periodicCall() doit être appelée périodiquement par l'applicatif.
 * Cette méthode va gérer automatiquement les requêtes de lectures des différentes mesures et les convertir dans une unité exploitable.

 * Avec cette classe, il est également possible de piloter les 8 sorties commutables
 * Soit par l'API sortie par sortie :
 *      > setOutput(numéro_de_sortie, valeur);
 * Soit par une valeur globale sur tout le port :
 *      > setOutputPort(valeur_a_positionner_sur_le_port);

 * La méthode refreshOutputs() peut être appelée périodiquement par l'applicatif pour renvoyer les valeurs attendues sur sorties.
 * L'objectif est la sécurisation du fonctionnement du système au cas où un transfert se serait mal passé.

 * Attention :
 * - La liste des registres "T_REG_ADDRESS" est commune avec le code du dsPIC.
 * Lorsque la définition des registres est changé sur le dsPIC, elle doit également l'être dans ce driver.
 * Dans le cas contraire, les 2 ne se comprendraient plus.
 *
 * - Par défaut, l'EEPROM est protégée en écriture.
 * Avant toute action ayant pour objectif de changer des paramètres en EEPROM (adresse I2C, calibration, ...),
 *  il sera nécessaire d'envoyer la requête de dévérouillage de l'EEPROM.
 * Ce mécanisme étant mis en place pour éviter les écritures involotaires ou par erreur.

* Utilisation :
 *  1. Créer une nouvelle classe héritant de cette classe et réimplémenter les méthodes virtuelles pures.

 *  2. Mise en oeuvre :
   PowerElectrobot m_power_electrobot;
    main()
    {
       ...
       m_power_electrobot.init(0x60); // Configure l'adresse I2C où est vue la carte PowerElectrobot

       m_power_electrobot.setOutput(dsPicPowerElectrobotBase::OUTPUT_STOR1, 1);  // Active la sortie STOR_1
       m_power_electrobot.setOutputPort(0xAA); Active 1 sortie sur 2 (8, 6, 4, 2)
       Dans une tâche à 50msec :
            m_power_electrobot.periodicCall();

       Dans une tâche à 500msec : (facultatif)
            m_power_electrobot.refreshOutputs();

       unsigned short battery_voltage_mV = getRawBatteryVoltage();
       float battery_voltage_V = getBatteryVoltage();
    }
 *
*/

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
        REG_PORT_STOR_1to8,
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
// Constantes utilisant les numéros de connecteurs indiqués sur la sérigraphie de la carte PowerElectrobot.
#define OUTPUT_J11 OUTPUT_STOR1
#define OUTPUT_J13 OUTPUT_STOR2
#define OUTPUT_J16 OUTPUT_STOR3
#define OUTPUT_J18 OUTPUT_STOR4
#define OUTPUT_J12 OUTPUT_STOR5
#define OUTPUT_J14 OUTPUT_STOR6
#define OUTPUT_J17 OUTPUT_STOR7
#define OUTPUT_J19 OUTPUT_STOR8

    virtual void setOutput(tSwitchOutput output, bool state);
    virtual void setOutputPort(unsigned char val);
    virtual void setAllOutputs(bool state);
    virtual unsigned char getOutputPort();
    void refreshOuptuts();

private :
    void readRegisters();
    void writeRegister(unsigned char reg, unsigned char val);
    void writeWordRegister(unsigned char reg, unsigned short val);
    void setBitPort(unsigned char bit, bool val);
    unsigned char m_outputs_port;
    unsigned long m_compteurErrCom;


protected :
    unsigned char m_address;

    unsigned short m_raw_battery_voltage;
    unsigned short m_raw_global_current;
    unsigned short m_raw_current_out1;
    unsigned short m_raw_current_out2;

    float m_battery_voltage;
    float m_global_current;
    float m_current_out1;
    float m_current_out2;

    float RegisterToBatteryVoltage(unsigned short val);
    float RegisterToGlobalCurrent(unsigned short val);
    float RegisterToCurrentOut1(unsigned short val);
    float RegisterToCurrentOut2(unsigned short val);
};

#endif // DSPIC_POWER_ELECTROBOT_BASE_H
