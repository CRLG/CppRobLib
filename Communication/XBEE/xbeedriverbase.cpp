#include <stdio.h>
#include <string.h>
#include "xbeedriverbase.h"

XbeeDriverBase::XbeeDriverBase()
    : m_id(0),
      m_xmessage_index(0)

{
}

XbeeDriverBase::~XbeeDriverBase()
{
}

// ____________________________________________________________
unsigned char XbeeDriverBase::getID()
{
    return m_id;
}

// ____________________________________________________________
void XbeeDriverBase::setID(unsigned short id)
{
    m_id = id;
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
    for (int i=0; i< 4; i++){
        m_panid[i] = settings.PANID[i];
    }
    for (int i=0; i< 2; i++){
        m_channel[i] = settings.CHANNEL[i];
    }
    m_apimode = settings.APIMODE;
    m_security = settings.SECURITY;
    for (int i=0; i< 32; i++){
        m_key[i] = settings.KEY[i];
    }
    m_coordinator = settings.COORDINATOR;
    m_coordinator_option = settings.COORDINATOR_OPTION;
    m_xmessage_index = 0;

    //! Mettre un delay d'une sec
    unsigned long delay = 100000;
    unsigned long delay_1sec = 1e6;

    delay_us(delay_1sec);
    unsigned char Plus[] = "+++";
    write(Plus, 3);
    delay_us(delay_1sec);
    setRegister("ID", m_panid, 4);
    delay_us(delay);
    setRegister("CH", m_channel, 2);
    delay_us(delay);
    setRegister("MY", m_id);
    delay_us(delay);
    setRegister("CE", m_coordinator);
    delay_us(delay);
    setRegister("EE", m_security);
    delay_us(delay);
    setRegister("KY", m_key, 32);
    delay_us(delay);
    setRegister("A2", m_coordinator_option);
    delay_us(delay);
    setRegister("AP", m_apimode);
    delay_us(delay);
    getRegister("CN");      //apply parameters
    delay_us(delay_1sec);

    return dummy;
}


// ____________________________________________________________
void XbeeDriverBase::setRegister(char* parameter, unsigned char *value, unsigned short valueLength){
    unsigned char param_size = strlen(parameter);
    unsigned short dataLength = param_size + valueLength + 3;
    unsigned char data[dataLength];
    data[0] = 'A';
    data[1] = 'T';
    for (int i=0; i<param_size; i++){
        data[i + 2] = parameter[i];
    }
    for (int i=0; i<valueLength; i++){
        data[i + param_size + 2] = value[i];
    }
    data[dataLength - 1] = 0x0D;
    write(data, dataLength);

    //attendre de recevoir la réponse du Xbee : OK\r
    //appeler decodeInit
}

// ____________________________________________________________
void XbeeDriverBase::setRegister(char *parameter, unsigned char value){
    unsigned char param_size = strlen(parameter);
    unsigned short dataLength = strlen(parameter) + 4;
    unsigned char data[dataLength];
    data[0] = 'A';
    data[1] = 'T';
    for (int i=0; i<param_size; i++){
        data[i + 2] = parameter[i];
    }
    data[param_size + 2] = value;
    data[dataLength - 1] = 0x0D;
    write(data, dataLength);

    //attendre de recevoir la réponse du Xbee : OK\r
    //appeler decodeInit
}

// ____________________________________________________________
void XbeeDriverBase::getRegister(char *parameter){
    unsigned char param_size = strlen(parameter);
    unsigned short dataLength = param_size + 3;
    unsigned char data[dataLength];
    data[0] = 'A';
    data[1] = 'T';
    for (int i=0; i<param_size; i++){
        data[i + 2] = parameter[i];
    }
    data[dataLength - 1] = 0x0D;
    write(data, dataLength);

    //attendre de recevoir la réponse du Xbee : OK\r
    //appeler decodeInit
}

// ____________________________________________________________
/*! \brief Called each time a packet of data is received from Xbee during init.
 *
 * \param buff_data: the buffer of bytes received.
 * \param buff_size: buffer size
  */
tXbeeErr XbeeDriverBase::decodeInit(unsigned char *buff_data, unsigned char buff_size)
{
    tXbeeErr dummy;
    if(buff_data[0] == 'O' && buff_data[1] == 'K' && buff_data[2] == '\r'){
        dummy = XBEE_OK;
    }
    else{
        dummy = XBEE_INIT_ERROR;
    }
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

    unsigned char xbuff_size = buff_size + 9;
    unsigned char xbuff[xbuff_size];
    unsigned short checksum = 0;
    xbuff[0] = 0x7E;                            //entete Xbee en mode API
    xbuff[1] = 0x00;                            //MSB longueur message : donnee utile + des parametres d'envoi du message
    xbuff[2] = buff_size + 5;                   //LSB longueur message : donnee utile + des parametres d'envoi du message
    xbuff[3] = 0x01;                            //definit fonction du message, ici envoi vers un autre module
    xbuff[4] = 0x00;                            //ACK : 0 = no ACK (pas tout compris)
    xbuff[5] = 0x00;                            //MSB : dest_adress
    xbuff[6] = dest_address;                    //LSB : dest_adress
    xbuff[7] = 0x01;                            //OPTION : 1 = disable ACK
    for(int i =0;i<buff_size;i++){              //donnee utile
        xbuff[8+i] = buff_data[i];
    }
    for(int i =3;i<buff_size+8;i++){            //calcul le checksum sur les parametres du message et sur la donnee utile
          checksum += xbuff[i];
    }
    xbuff[buff_size+8] = 0xFF - checksum;       //checksum message Xbee

    write(xbuff, xbuff_size);                   //envoi du message
}


// ____________________________________________________________
/*! \brief Called each time a packet of data is received from Xbee.
 *
 * \param buff_data: the buffer of bytes received.
 * \param buff_size: buffer size
  */
void XbeeDriverBase::decode(unsigned char *buff_data, unsigned char buff_size)
{
    for (int i=0; i<buff_size; i++)
    {
        decode(buff_data[i]);
    }
}

// ____________________________________________________________
/*! \brief Called each time a byte is received from Xbee.
 *
 * \param newData: the byte received.
 * \remarks : build a XMessage step by step
  */
void XbeeDriverBase::decode(unsigned char newData)
{
    switch(m_xmessage_state) {
        //Réception d'une trame en mode API
        case XBEE_HEADER :      //Header du message Xbee
            if (newData == 0x7E) {
                m_xmessage_state = XBEE_LENGTH_MSB;
            }
            break;
        case XBEE_LENGTH_MSB :      //Taille de la donnée + les options
            m_current_xmessage.DLC = newData << 8;
            m_xmessage_state = XBEE_LENGTH_LSB;
            break;
        case XBEE_LENGTH_LSB :      //Taille de la donnée + les options
            m_current_xmessage.DLC += newData;
            m_xmessage_state = XBEE_FRAME;
            break;
        case XBEE_FRAME :       //Type de message : réception d'un message = 0x81
            m_current_xmessage.FrameType = newData;
            if (m_current_xmessage.FrameType == 0x81){      //Reception d'un message Xbee
                m_current_xmessage.DLC -= 5;                //Taille de la donnée utile
                m_xmessage_state = XBEE_SOURCE_MSB;
            }
            break;
        case XBEE_SOURCE_MSB :      //Source du message
            m_current_xmessage.SourceID = newData << 8;
            m_xmessage_state = XBEE_SOURCE_LSB;
            break;
        case XBEE_SOURCE_LSB :      //Source du message
            m_current_xmessage.SourceID += newData;
            m_xmessage_state = XBEE_RSSI;
            break;
        case XBEE_RSSI :        //Puissance du signal reçu
            m_xmessage_state = XBEE_OPTION;
            break;
        case XBEE_OPTION :
            m_xmessage_state = XBEE_DATA;
            break;
        case XBEE_DATA :
            m_current_xmessage.Data[m_xmessage_index] = newData;
            m_xmessage_index++;
            if (m_xmessage_index == m_current_xmessage.DLC){    //La donnée utile est récupérée
                m_xmessage_index = 0;
                m_xmessage_state = XBEE_CHECKSUM;
            }
            break;
        case XBEE_CHECKSUM :                                    //Checksum du message Xbee : Vérifie la transmission UART
                m_current_xmessage.Checksum = newData;
                m_xmessage_state = XBEE_HEADER;
            break;
    }
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
