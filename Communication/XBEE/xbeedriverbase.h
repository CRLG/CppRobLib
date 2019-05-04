#ifndef _XBEE_DRIVER_BASE_H_
#define _XBEE_DRIVER_BASE_H_

// ====================================================
//               INTERNAL DRIVER ERROR CODE
// ====================================================
typedef enum {
    XBEE_OK = 0,
    XBEE_TIMEOUT,
    XBEE_CHECKSUM_ERROR,
    XBEE_WRITE_ERROR,
    XBEE_INIT_ERROR,
}tXbeeErr;

// ====================================================
//               XBEE SETTINGS
// ====================================================
typedef struct {
    char PANID[4];          // XBEE NETWORK ID      (4 char : 0 à F en ASCII)
    char CHANNEL[2];        // Channel du reseau    (2 char : 0 à F en ASCII)
    unsigned char ID;             //XBEE ID               (4 char : 0 à F en ASCII)
    unsigned char APIMODE;           //Definit le mode de transmission (0 ou 1 en ASCII)
    unsigned char SECURITY;          //Active la clé reseau  (0 ou 1 en ASCII)
    char KEY[32];            //clé reseau            (32 char : 0 à F en ASCII)
    unsigned char COORDINATOR;       //Coordinateur du reseau ou routeur (0 ou 1 en ASCII)
    unsigned char COORDINATOR_OPTION;      //Option du coordinateur (0x04 = autorise les Xbee à rejoindre son reseau)
}tXbeeSettings;


// ====================================================
//               XBEE RAW MESSAGE
// ====================================================
#define XBEE_MAX_DATA_LEN    80
typedef struct {
    unsigned char FrameType;        //Type de message
    unsigned int SourceID;          //Source du message envoyé
    unsigned short DestinationID;   //! TODO : peut Ãªtre qu'on en a pas besoin Ã  ce niveau lÃ  ?? A supprimer sinon
    unsigned int DLC; // Data Lenght
    unsigned char Data[XBEE_MAX_DATA_LEN];
    unsigned char Checksum;
}tXbeeMessage;

// ====================================================
//               XBEE RECEPTION STATE
// ====================================================
typedef enum {
    XBEE_HEADER = 0,
    XBEE_LENGTH_MSB,
    XBEE_LENGTH_LSB,
    XBEE_FRAME,
    XBEE_SOURCE_MSB,
    XBEE_SOURCE_LSB,
    XBEE_RSSI,
    XBEE_OPTION,
    XBEE_DATA,
    XBEE_CHECKSUM,
}xmessage_state;

// ====================================================
//
// ====================================================
#define XBEE_BROADCAST_ID 0xFFFF  //valeur de la doc XBEE qui permet d'adresser tous les XBEE

// ====================================================
//        BASE CLASS FOR XBEE DRIVER
// ====================================================
class XbeeDriverBase
{
public:
    XbeeDriverBase();
    virtual ~XbeeDriverBase();

    // pure virtual methods.
    // this method is called by driver to inform a valid buffer ws receinved and now ready to be used by application
    virtual void readyBytes(unsigned char *buff_data, unsigned char buff_size, unsigned short source_id=0)=0;
    // this method is called by driver to write a buffer to physical serial port on specific hardware
    virtual void write(unsigned char *buff_data, unsigned char buff_size)=0;
    // this method is called by driver to request a delay on specific hardware
    virtual void delay_us(unsigned long delay)=0;

    // call this method to inform driver a new data was received on serial port
    virtual void decode(unsigned char newData);
    virtual void decode(unsigned char *buff_data, unsigned char buff_size);

    // call this method to encode and send data in a Xbee format packet
    virtual void encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address=0);

    //call this method to initialize Xbee register
    virtual void setRegister(char *parameter, unsigned char *value, unsigned short valueLength=1);
    virtual void setRegister(char *parameter, unsigned char value);

    //call this method to get the value of a register or apply the parameters
    virtual void getRegister(char *parameter);

    //call this method to check the setting of a register is ok
    virtual tXbeeErr decodeInit(unsigned char *buff_data, unsigned char buff_size);

    // High level API
    virtual tXbeeErr init(const tXbeeSettings& settings);
    virtual tXbeeErr connect();
    virtual bool isPresent(unsigned char xbee_id);
    virtual bool isConnected();
    virtual tXbeeErr reset();

    void setID(unsigned short id);
    unsigned char getID();

private :
    //Parametres xbee
    unsigned char m_panid[4];
    unsigned char m_channel[2];
    unsigned char m_id;
    unsigned char m_apimode;
    unsigned char m_security;
    unsigned char m_key[32];
    unsigned char m_coordinator;
    unsigned char m_coordinator_option;

    //! Compute checksum from the packet
    unsigned char getChecksum(unsigned char *xpacket);

    //! Current packet
    tXbeeMessage m_current_xmessage;   //"xmessage" = Xbee message
    //unsigned char m_xmessage_state;
    unsigned char m_xmessage_index;    //! TO DO : a initialiser a 0 a l'init
    xmessage_state m_xmessage_state;
};

#endif // _XBEE_DRIVER_BASE_H_
