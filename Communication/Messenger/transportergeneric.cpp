#include "transportergeneric.h"
#include "databasebase.h"
#include "messengereventbase.h"
#include "comdriverbase.h"

TransporterGeneric::TransporterGeneric() :
      m_decoder_state(DECODER_STATE_INIT),
      m_transfert_with_checksum(true)
{
    _initDecoder();
}

TransporterGeneric::~TransporterGeneric()
{
}

// ____________________________________________________________
/*! \brief Decode an incoming buffer.
 *
 * \param buff_data the buffer of data to decode
 * \param buff_size the buffer size
 * \param source_address the address of the sender
 * \remarks Call this method when a packet of serveral data is coming
 */
void TransporterGeneric::decode(unsigned char *buff_data, unsigned char buff_size, unsigned short source_address)
{
    m_current_frame.SourceAddress = source_address;
    m_decoder_state = DECODER_STATE_INIT;
    for (unsigned int i=0; i<buff_size; i++) {
        decode(buff_data[i], source_address);
    }
}

// ____________________________________________________________
/*! \brief Decode the current frame.
 *
 * \param newData the new byte to compute
 * \remarks Call this method when a simple data is coming
 */
void TransporterGeneric::decode(unsigned char newData, unsigned short source_address)
{
    if (source_address != m_current_frame.SourceAddress) {  // reset state machin if the source of the message change (avoid message corruption)
        m_current_frame.SourceAddress = source_address;
        m_decoder_state = DECODER_STATE_INIT;
    }
    switch(m_decoder_state)
    {
        // ----------------------------------------- ETATS PRIMAIRE D'AIGUILLAGE DU TYPE D'INFO RECUES
        case DECODER_STATE_INIT :
            // Initialise les champs d'une précédente réception
            _initDecoder();

            // Le message est une trame
            if (newData == 'T') {
                m_decoder_state = DECODER_STATE_ID_MSB;
            }
        break;
        // ----------------------------------------- ETATS LIES A LA RECEPTION DE TRAMES
        case DECODER_STATE_ID_MSB :
            m_current_frame.ID = (newData << 8);
            m_decoder_state = DECODER_STATE_ID_LSB;
        break;
        // -----------------------------------------
        case DECODER_STATE_ID_LSB :
            m_current_frame.ID += (newData&0xFF);
            m_decoder_state = DECODER_STATE_DLC;
        break;
        // -----------------------------------------
        case DECODER_STATE_DLC :
            m_current_frame.DLC = newData;
            if (newData > MESSENGER_MAX_DATA_LEN) {
                m_decoder_state = DECODER_STATE_INIT;
            }
            else if (newData > 0) {
                m_decoder_state = DECODER_STATE_DATA_i;
            }
            else { // Aucune donnée
                if (m_transfert_with_checksum)  { m_decoder_state = DECODER_STATE_CHECKSUM; }
                else                            { readyFrame(&m_current_frame);  m_decoder_state = DECODER_STATE_INIT; }
            }
        break;
        // ----------------------------------------- Les DLC données
        case DECODER_STATE_DATA_i :
            m_current_frame.Data[m_data_number] = newData;
            m_data_number++;
            if (m_current_frame.DLC > m_data_number)
            {  /* ne rien faire : il reste des données à recevoir */ }
            else {
                if (m_transfert_with_checksum)  { m_decoder_state = DECODER_STATE_CHECKSUM; }
                else                            { readyFrame(&m_current_frame);  m_decoder_state = DECODER_STATE_INIT;}
            }
        break;
        // ----------------------------------------- Le CHECKSUM
        case DECODER_STATE_CHECKSUM :
            if (newData == _getChecksum(&m_current_frame)) {
                readyFrame(&m_current_frame);
            }
            m_decoder_state = DECODER_STATE_INIT;
        break;
        // -----------------------------------------
        default :
            m_decoder_state = DECODER_STATE_INIT;
        break;
    }
}

// ____________________________________________________________
/*! \brief Serialize a frame.
 *
 * \param frame : the frame to serialize
 */
void TransporterGeneric::encode(tMessengerFrame *frame)
{
    unsigned short buff_size = 5 + frame->DLC;
    unsigned char data[buff_size];
    unsigned char i=0;
    data[i++] = 'T';
    data[i++] = frame->ID>>8;
    data[i++] = frame->ID;
    data[i++] = frame->DLC;;

    for(unsigned char j=0; j<frame->DLC; j++) {
      data[i++] = frame->Data[j];
    }
    if (m_transfert_with_checksum) {
        data[i++] = _getChecksum(frame);
    }

    if (m_com_driver) m_com_driver->encode(data, i, frame->DestinationAddress);
    if (m_event_mgr) m_event_mgr->frameTransmited(frame);
}


// ____________________________________________________________
/*! \brief Compute checksum from the packet.
 *
 * \param packet : the packet to compute
 * \return the checksum
 * \remarks :
  */
unsigned char TransporterGeneric::_getChecksum(tMessengerFrame *frame)
{
    unsigned char checksum = 0;
    unsigned char i=0;

    checksum += ((frame->ID)>>8)&0xFF;
    checksum += (frame->ID)&0xFF;
    checksum += frame->DLC;
    for(i=0; i<frame->DLC; i++) {
       checksum += frame->Data[i];
    }
    return(checksum);
}

// ____________________________________________________________
/*! \brief Initialise the message to build.
  */
void TransporterGeneric::_initDecoder()
{
    m_data_number = 0;
    // Initialise les champs de la trame courante
    m_current_frame.ID = 0xFFFF;
    m_current_frame.DLC = 0xFF;
    for (unsigned int i=0; i<MESSENGER_MAX_DATA_LEN; i++) {
        m_current_frame.Data[i] = 0xFF;
    }
}

