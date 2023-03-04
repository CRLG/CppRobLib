#include "stdio.h"
#include "string.h"
#include "messengerinterfacebase.h"
#include "message_commande_vitesse_mouvement.h"
#include "databasebase.h"
#include "data_encoder_decoder.h"
#include "asservissementdeporte_facteursconversion.h"

// ==============================================================
// ==============================================================
Message_COMMANDE_VITESSE_MOUVEMENT::Message_COMMANDE_VITESSE_MOUVEMENT()
{
    m_id = 0x30;
    m_dlc = 6;
}

// ____________________________________________________________
const char* Message_COMMANDE_VITESSE_MOUVEMENT::getName()
{
    return "COMMANDE_VITESSE_MOUVEMENT";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_COMMANDE_VITESSE_MOUVEMENT::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    short old_id = id;
    float old_vitesse_avance = vitesse_avance;
    float old_vitesse_angle = vitesse_angle;
#endif // MESSENGER_FULL

    id = CDataEncoderDecoder::decode_uint16((unsigned char*)buff_data, 0);
    vitesse_avance = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 2) / CONV_DISTANCE2MESSAGE;
    vitesse_angle = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 4) / CONV_ANGLE2MESSAGE;

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];

        strcpy(name, "id");
        sprintf(val_str, "%d", id);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (id != old_id) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "vitesse_avance");
        sprintf(val_str, "%f", vitesse_avance);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (vitesse_avance != old_vitesse_avance) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "vitesse_angle");
        sprintf(val_str, "%f", vitesse_angle);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (vitesse_angle != old_vitesse_angle) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_COMMANDE_VITESSE_MOUVEMENT::encode(unsigned char *buff_data)
{
    CDataEncoderDecoder::encode_uint16(buff_data, 0, id);
    CDataEncoderDecoder::encode_int16(buff_data, 2, vitesse_avance * CONV_DISTANCE2MESSAGE);
    CDataEncoderDecoder::encode_int16(buff_data, 4, vitesse_angle * CONV_ANGLE2MESSAGE);

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];
        strcpy(name, "id");
        sprintf(val_str, "%d", id);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "vitesse_avance");
        sprintf(val_str, "%f", vitesse_avance);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "vitesse_angle");
        sprintf(val_str, "%f", vitesse_angle);
        m_messenger_interface->dataSent(this, name, val_str);
    }
#endif // MESSENGER_FULL
}
