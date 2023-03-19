#include "stdio.h"
#include "string.h"
#include "messengerinterfacebase.h"
#include "message_commande_indice_sportivite.h"
#include "databasebase.h"
#include "data_encoder_decoder.h"
#include "asservissementdeporte_facteursconversion.h"

// ==============================================================
// ==============================================================
Message_COMMANDE_INDICE_SPORTIVITE::Message_COMMANDE_INDICE_SPORTIVITE()
{
    m_id = 0x31;
    m_dlc = 4;
}

// ____________________________________________________________
const char* Message_COMMANDE_INDICE_SPORTIVITE::getName()
{
    return "COMMANDE_INDICE_SPORTIVITE";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_COMMANDE_INDICE_SPORTIVITE::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    short old_id = id;
    float old_indice_sportivite = indice_sportivite;
#endif // MESSENGER_FULL

    id = CDataEncoderDecoder::decode_uint16((unsigned char*)buff_data, 0);
    indice_sportivite = CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 2) / CONV_INDICE_SPORTIVIE;

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];

        strcpy(name, "id");
        sprintf(val_str, "%d", id);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (id != old_id) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "indice_sportivite");
        sprintf(val_str, "%f", indice_sportivite);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (indice_sportivite != old_indice_sportivite) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_COMMANDE_INDICE_SPORTIVITE::encode(unsigned char *buff_data)
{
    CDataEncoderDecoder::encode_uint16(buff_data, 0, id);
    CDataEncoderDecoder::encode_int16(buff_data, 2, indice_sportivite * CONV_INDICE_SPORTIVIE);

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];
        strcpy(name, "id");
        sprintf(val_str, "%d", id);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "indice_sportivite");
        sprintf(val_str, "%f", indice_sportivite);
        m_messenger_interface->dataSent(this, name, val_str);
    }
#endif // MESSENGER_FULL
}
