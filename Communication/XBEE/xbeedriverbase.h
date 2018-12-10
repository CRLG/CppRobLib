#ifndef _XBEE_DRIVER_BASE_H
#define _XBEE_DRIVER_BASE_H

#include "comdriverbase.h"

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
//        ABSTRACT DRIVER CLASS FOR XBEE
// ====================================================
class XbeeDriverBase : public ComDriverBase
{
public:
    XbeeDriverBase();
    virtual ~XbeeDriverBase();

    // pure virtual methods for hardware abstraction.
    // to be implemented on specific hardware.
    virtual tXbeeErr write(unsigned char *buff_data, unsigned short buff_size) = 0;
    //virtual void newMessage(unsigned short id_sender, char *buff, int buff_size) = 0;
    virtual void newMessage(const tXbeeMessage& msg) = 0;
    virtual void delay_us(unsigned long delay) = 0;

    // High level API
    virtual tXbeeErr init(const tXbeeSettings& settings);
    virtual tXbeeErr connect();
    virtual bool isPresent(unsigned char xbee_id);
    virtual bool isConnected();
    virtual tXbeeErr reset();

    unsigned short getID();

    // call this method to inform driver a new data was received on serial port
    virtual void decode(unsigned char newData);

    // call this method to encode and send data in a Xbee format packet
    virtual void encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address=0);

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

#endif // _XBEE_DRIVER_BASE_H
