#include "stdio.h"
#include "string.h"
#include "message_reinit_position_xy_teta.h"
#include "messengerinterfacebase.h"
#include "databasebase.h"
#include "data_encoder_decoder.h"
#include "asservissementdeporte_facteursconversion.h"

// ==============================================================
// ==============================================================
Message_REINIT_POSITION_XY_TETA::Message_REINIT_POSITION_XY_TETA()
{
    m_id = 0x25;
    m_dlc = 8;
}

// ____________________________________________________________
const char* Message_REINIT_POSITION_XY_TETA::getName()
{
    return "REINIT_POSITION_XY_TETA";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_REINIT_POSITION_XY_TETA::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    short old_id = id;
    float old_x = x;
    float old_y = y;
    float old_teta = teta;
#endif // MESSENGER_FULL

    id = CDataEncoderDecoder::decode_uint16((unsigned char*)buff_data, 0);
    x = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 2) / CONV_DISTANCE2MESSAGE;
    y = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 4) / CONV_DISTANCE2MESSAGE;
    teta = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 6) / CONV_ANGLE2MESSAGE;

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

        strcpy(name, "teta");
        sprintf(val_str, "%f", teta);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (teta != old_teta) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_REINIT_POSITION_XY_TETA::encode(unsigned char *buff_data)
{
    CDataEncoderDecoder::encode_uint16(buff_data, 0, id);
    CDataEncoderDecoder::encode_int16(buff_data, 2, x * CONV_DISTANCE2MESSAGE);
    CDataEncoderDecoder::encode_int16(buff_data, 4, y * CONV_DISTANCE2MESSAGE);
    // TODO : ramener l'angle en -Pi;+Pi avant l'envoie
    CDataEncoderDecoder::encode_int16(buff_data, 6, teta * CONV_ANGLE2MESSAGE);

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

        strcpy(name, "teta");
        sprintf(val_str, "%f", teta);
        m_messenger_interface->dataSent(this, name, val_str);
    }
#endif // MESSENGER_FULL
}
