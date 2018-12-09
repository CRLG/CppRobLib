#include "xbeedriverbase.h"
#include "transporterbase.h"

XbeeDriverBase::XbeeDriverBase()
    : m_xmessage_index(0)
{
}

XbeeDriverBase::~XbeeDriverBase()
{
}

// ____________________________________________________________
unsigned short XbeeDriverBase::getID()
{
    return m_id;
}

// ____________________________________________________________
bool XbeeDriverBase::isPresent(unsigned char id)
{
    bool dummy = true;
    return (dummy);
}

// ____________________________________________________________
tXbeeErr XbeeDriverBase::init(const tXbeeSettings& settings)
{
    tXbeeErr dummy=XBEE_OK;

    m_id = settings.ID;

    m_xmessage_index = 0;

    return dummy;
}

// ____________________________________________________________
tXbeeErr XbeeDriverBase::connect()
{
    tXbeeErr dummy=XBEE_OK;
    return dummy;
}

// ____________________________________________________________
bool XbeeDriverBase::isConnected()
{
    bool dummy = true;
    return (dummy);
}

// ____________________________________________________________
tXbeeErr XbeeDriverBase::reset()
{
    tXbeeErr dummy=XBEE_OK;
    return dummy;
}

// ____________________________________________________________
/*! \brief Entry point to encode and send a buffer in Xbee format.
 *
 * \param buff_data: the buffer to encode.
 * \param buff_size: buffer size.
 * \param dest_address: Xbee destination ID.
 */
void XbeeDriverBase::encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address)
{
/*
    unsigned char xbuff_size = buff_size + 5;   // "5", c'est au pif, juste pour essayer
    unsigned char xbuff[xbuff_size];
    xbuff[0] = dest_address;  (ou ailleur...)
    ...
    xbuff[n] = getChecksum(xbuff);

    write(xbuff, xbuff_size);
  */
  //! TODO Dans le cas d'une Ã©mission, le champ ProducerID n'a pas d'utilitÃ© puisque l'ID du producteur, c'est "m_id"
    //! En revanche, il faut peut certainement encapsuler "dest_address" quelquepart dans le message comme l'attend le XBEE
    write(buff_data, buff_size);   // renvoie directement le buffer d'entrée, c'est juste essayer
}


// ____________________________________________________________
/*! \brief Called each time a byte is received from Xbee.
 *
 * \param newData: the byte received.
 * \remarks : build a XMessage step by step
  */
void XbeeDriverBase::decode(unsigned char newData)
{
    //! TODO : reconstituer le message XBEE
    m_xmessage_index++;
/*
    switch(m_xmessage_state) {
        case ....

    }
*/
    //! Le bout de code ici, c'est juste pour les essais
    //! A supprimer une fois la machine d'état de reconstitution créée
    if (m_xmessage_index == 10) {  // 10 pour le test
        m_current_xmessage.SourceID = 0x4;
        unsigned int i=0;
        // m_current_xmessage.Data[i++] = ...;        // Xbee header ??
        m_current_xmessage.Data[i++] = 'T';        // synchro   <- first index app data
        m_current_xmessage.Data[i++] = 0x12;       // ID MSB
        m_current_xmessage.Data[i++] = 0x34;       // ID LSB
        m_current_xmessage.Data[i++] = 4;          // DLC
        m_current_xmessage.Data[i++] = 0x01;       // Data[0]
        m_current_xmessage.Data[i++] = 0x23;
        m_current_xmessage.Data[i++] = 0x45;
        m_current_xmessage.Data[i++] = 0x67;       // Data[DLC-1]
        m_current_xmessage.Data[i++] = 0x1A;       // Messenger Checksum   <- last index app data
        //m_current_xmessage.Data[i++] = ...;    // Xbee packet checksum
        m_current_xmessage.DLC = i;
        if (isXMessageValid(&m_current_xmessage)) {
            unsigned char first_index_app_data = 0;  // index in the buffer of the synchro data
            unsigned char app_data_size= i;
            if (m_transporter) m_transporter->decode(&m_current_xmessage.Data[first_index_app_data], app_data_size, m_current_xmessage.SourceID);
            //newMessage(m_current_xmessage);
        }
        m_xmessage_index = 0;
    };
}


// ____________________________________________________________
/*! \brief Compute checksum from the packet.
 *
 * \param packet : the packet to compute
 * \return the checksum
 * \remarks :
  */
unsigned char XbeeDriverBase::getChecksum(unsigned char *packet)
{
    unsigned char sum=0;
//! TODO implement the correct checksum algorithm
/*    for (int i=2; i<=2+packet[3]; i++) {
        sum+= packet[i];
    }
*/
    return ~sum;
}

// ____________________________________________________________
/*! \brief Check if packet is valid.
 *   A valid packet is :
 *      header is xxxx
 *      ...
 *      checksum is OK
 *
 * \param packet : the packet to compute
 * \return the checksum
 */
//!
bool XbeeDriverBase::isXMessageValid(tXbeeMessage *msg)
{
   return true;
}
