#include "stdio.h"
#include "string.h"
#include "message_commande_mouvement_xy.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"
#include "data_encoder_decoder.h"
#include "asservissementdeporte_facteursconversion.h"

// ==============================================================
// ==============================================================
Message_COMMANDE_MOUVEMENT_XY::Message_COMMANDE_MOUVEMENT_XY()
{
    m_id = 0x20;
    m_dlc = 6;
}

// ____________________________________________________________
const char* Message_COMMANDE_MOUVEMENT_XY::getName()
{
    return "COMMANDE_MOUVEMENT_XY";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_COMMANDE_MOUVEMENT_XY::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    short old_id = id;
    float old_x = x;
    float old_y = y;
#endif // MESSENGER_FULL

    id = CDataEncoderDecoder::decode_uint16((unsigned char*)buff_data, 0);
    x = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 2) / CONV_DISTANCE2MESSAGE;
    y = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 4) / CONV_DISTANCE2MESSAGE;

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];

        strcpy(name, "id");
        sprintf(val_str, "%d", id);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (id != old_id) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "x");
        sprintf(val_str, "%f", x);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (x != old_x) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "y");
        sprintf(val_str, "%f", y);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (y != old_y) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_COMMANDE_MOUVEMENT_XY::encode(unsigned char *buff_data)
{
    CDataEncoderDecoder::encode_uint16(buff_data, 0, id);
    CDataEncoderDecoder::encode_int16(buff_data, 2, x * CONV_DISTANCE2MESSAGE);
    CDataEncoderDecoder::encode_int16(buff_data, 4, y * CONV_DISTANCE2MESSAGE);

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];
        strcpy(name, "id");
        sprintf(val_str, "%d", id);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "x");
        sprintf(val_str, "%f", x);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "y");
        sprintf(val_str, "%f", y);
        m_messenger_interface->dataSent(this, name, val_str);
    }
#endif // MESSENGER_FULL
}
