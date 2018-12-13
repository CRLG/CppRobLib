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
}tXbeeErr;

// ====================================================
//               XBEE SETTINGS
// ====================================================
typedef struct {
    unsigned short ID;  //! XBEE ID
    // baud rate ?
    // ...
}tXbeeSettings;


// ====================================================
//               XBEE RAW MESSAGE
// ====================================================
#define XBEE_MAX_DATA_LEN    32
typedef struct {
    unsigned short SourceID;      //! TODO : peut Ãªtre qu'on en a pas besoin Ã  ce niveau lÃ  ?? A supprimer sinon
    unsigned short DestinationID;   //! TODO : peut Ãªtre qu'on en a pas besoin Ã  ce niveau lÃ  ?? A supprimer sinon
    unsigned char DLC; // Data Lenght
    unsigned char Data[XBEE_MAX_DATA_LEN];
}tXbeeMessage;

// ====================================================
//
// ====================================================
#define XBEE_BROADCAST_ID 0x1234  //! TODO : valeur mise au pif. Mettre la valeur de la doc XBEE qui permet d'adresser tous les XBEE

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

    // High level API
    virtual tXbeeErr init(const tXbeeSettings& settings);
    virtual tXbeeErr connect();
    virtual bool isPresent(unsigned char xbee_id);
    virtual bool isConnected();
    virtual tXbeeErr reset();

    void setID(unsigned short id);
    unsigned short getID();

private :
    unsigned short m_id;
    //! Compute checksum from the packet
    unsigned char getChecksum(unsigned char *packet);
    bool isXMessageValid(tXbeeMessage *msg);

    //! Current packet
    tXbeeMessage m_current_xmessage;   //"xmessage" = Xbee message
    unsigned char m_xmessage_state;
    unsigned char m_xmessage_index;
};

#endif // _XBEE_DRIVER_BASE_H_
