#include "stdio.h"
#include "string.h"
#include "message_commande_mouvement_distance_angle.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"
#include "data_encoder_decoder.h"
#include "asservissementdeporte_facteursconversion.h"

// ==============================================================
// ==============================================================
Message_COMMANDE_MOUVEMENT_DISTANCE_ANGLE::Message_COMMANDE_MOUVEMENT_DISTANCE_ANGLE()
{
    m_id = 0x24;
    m_dlc = 6;
}

// ____________________________________________________________
const char* Message_COMMANDE_MOUVEMENT_DISTANCE_ANGLE::getName()
{
    return "COMMANDE_MOUVEMENT_DISTANCE_ANGLE";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_COMMANDE_MOUVEMENT_DISTANCE_ANGLE::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    short old_id = id;
    float old_distance = distance;
    float old_angle = angle;
#endif // MESSENGER_FULL

    id = CDataEncoderDecoder::decode_uint16((unsigned char*)buff_data, 0);
    distance = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 2) / CONV_DISTANCE2MESSAGE;
    angle = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 4) / CONV_ANGLE2MESSAGE;

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];

        strcpy(name, "id");
        sprintf(val_str, "%d", id);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (id != old_id) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "distance");
        sprintf(val_str, "%f", distance);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (distance != old_distance) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "angle");
        sprintf(val_str, "%f", angle);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (angle != old_angle) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_COMMANDE_MOUVEMENT_DISTANCE_ANGLE::encode(unsigned char *buff_data)
{
    CDataEncoderDecoder::encode_uint16(buff_data, 0, id);
    CDataEncoderDecoder::encode_int16(buff_data, 2, distance * CONV_DISTANCE2MESSAGE);
    CDataEncoderDecoder::encode_int16(buff_data, 4, angle * CONV_ANGLE2MESSAGE);
    // TODO : ramener l'angle en -Pi;+Pi avant l'envoie

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];
        strcpy(name, "id");
        sprintf(val_str, "%d", id);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "distance");
        sprintf(val_str, "%f", distance);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "angle");
        sprintf(val_str, "%f", angle);
        m_messenger_interface->dataSent(this, name, val_str);
    }
#endif // MESSENGER_FULL
}
