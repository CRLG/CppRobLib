#include "stdio.h"
#include "string.h"
#include "message_etat_asservissement.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"
#include "data_encoder_decoder.h"
#include "asservissementdeporte_facteursconversion.h"

// ==============================================================
// ==============================================================
Message_ETAT_ASSERVISSEMENT::Message_ETAT_ASSERVISSEMENT()
{
    m_id = 0x39;
    m_dlc = 13;
}

// ____________________________________________________________
const char* Message_ETAT_ASSERVISSEMENT::getName()
{
    return "ETAT_ASSERVISSEMENT";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_ETAT_ASSERVISSEMENT::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    float old_X_robot = X_robot;
    float old_Y_robot = Y_robot;
    float old_angle_robot = angle_robot;
    char old_convergence_rapide = convergence_rapide;
    char old_convergence_conf = convergence_conf;
    char old_diag_blocage = diag_blocage;
    unsigned char old_ModeAsservissement = ModeAsservissement;
    float old_cde_moteur_D = cde_moteur_D;
    float old_cde_moteur_G = cde_moteur_G;
    unsigned short old_last_cde_id = last_cde_id;
#endif // MESSENGER_FULL

    X_robot =               CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 0) / CONV_DISTANCE2MESSAGE;
    Y_robot =               CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 2) / CONV_DISTANCE2MESSAGE;
    angle_robot =           CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 4) / CONV_ANGLE2MESSAGE;
    convergence_rapide =    CDataEncoderDecoder::decode_bit((unsigned char*)buff_data, 5, 0);
    convergence_conf =      CDataEncoderDecoder::decode_bit((unsigned char*)buff_data, 5, 1);
    diag_blocage =          CDataEncoderDecoder::decode_bit((unsigned char*)buff_data, 5, 2);
    ModeAsservissement =    CDataEncoderDecoder::decode_int8((unsigned char*)buff_data, 6);
    cde_moteur_D =          CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 7) / CONV_CDEMOTEUR2MESSAGE;
    cde_moteur_G =          CDataEncoderDecoder::decode_int16((unsigned char*)buff_data, 9) / CONV_CDEMOTEUR2MESSAGE;
    last_cde_id =           CDataEncoderDecoder::decode_uint16((unsigned char*)buff_data, 11);

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];

        strcpy(name, "X_robot");
        sprintf(val_str, "%f", X_robot);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (X_robot != old_X_robot) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "Y_robot");
        sprintf(val_str, "%f", Y_robot);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (Y_robot != old_Y_robot) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "angle_robot");
        sprintf(val_str, "%f", angle_robot);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (angle_robot != old_angle_robot) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "convergence_rapide");
        sprintf(val_str, "%d", convergence_rapide);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (convergence_rapide != old_convergence_rapide) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "convergence_conf");
        sprintf(val_str, "%d", convergence_conf);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (convergence_conf != old_convergence_conf) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "diag_blocage");
        sprintf(val_str, "%d", diag_blocage);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (diag_blocage != old_diag_blocage) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "ModeAsservissement");
        sprintf(val_str, "%d", ModeAsservissement);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (ModeAsservissement != old_ModeAsservissement) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "cde_moteur_D");
        sprintf(val_str, "%f", cde_moteur_D);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (cde_moteur_D != old_cde_moteur_D) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "cde_moteur_G");
        sprintf(val_str, "%f", cde_moteur_G);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (cde_moteur_G != old_cde_moteur_G) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "last_cde_id");
        sprintf(val_str, "%d", last_cde_id);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (last_cde_id != old_last_cde_id) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_ETAT_ASSERVISSEMENT::encode(unsigned char *buff_data)
{
    CDataEncoderDecoder::encode_int16(buff_data, 0, X_robot*CONV_DISTANCE2MESSAGE);
    CDataEncoderDecoder::encode_int16(buff_data, 2, Y_robot*CONV_DISTANCE2MESSAGE);
    CDataEncoderDecoder::encode_int16(buff_data, 4, angle_robot*CONV_ANGLE2MESSAGE);
    CDataEncoderDecoder::encode_bit(buff_data, 5, 0, convergence_rapide);
    CDataEncoderDecoder::encode_bit(buff_data, 5, 1, convergence_conf);
    CDataEncoderDecoder::encode_bit(buff_data, 5, 2, diag_blocage);
    CDataEncoderDecoder::encode_int8(buff_data, 6, ModeAsservissement);
    CDataEncoderDecoder::encode_int16(buff_data, 7, cde_moteur_D*CONV_CDEMOTEUR2MESSAGE);
    CDataEncoderDecoder::encode_int16(buff_data, 9, cde_moteur_G*CONV_CDEMOTEUR2MESSAGE);
    CDataEncoderDecoder::encode_uint16(buff_data, 11, last_cde_id);

#ifdef MESSENGER_FULL
    char name[20];
    char val_str[10];

    strcpy(name, "X_robot");
    sprintf(val_str, "%f", X_robot);
    m_messenger_interface->dataSent(this, name, val_str);

    strcpy(name, "Y_robot");
    sprintf(val_str, "%f", Y_robot);
    m_messenger_interface->dataSent(this, name, val_str);

    strcpy(name, "angle_robot");
    sprintf(val_str, "%f", angle_robot);
    m_messenger_interface->dataSent(this, name, val_str);

    strcpy(name, "convergence_rapide");
    sprintf(val_str, "%d", convergence_rapide);
    m_messenger_interface->dataSent(this, name, val_str);

    strcpy(name, "convergence_conf");
    sprintf(val_str, "%d", convergence_conf);
    m_messenger_interface->dataSent(this, name, val_str);

    strcpy(name, "diag_blocage");
    sprintf(val_str, "%d", diag_blocage);
    m_messenger_interface->dataSent(this, name, val_str);

    strcpy(name, "ModeAsservissement");
    sprintf(val_str, "%d", ModeAsservissement);
    m_messenger_interface->dataSent(this, name, val_str);

    strcpy(name, "cde_moteur_D");
    sprintf(val_str, "%f", cde_moteur_D);
    m_messenger_interface->dataSent(this, name, val_str);

    strcpy(name, "cde_moteur_G");
    sprintf(val_str, "%f", cde_moteur_G);
    m_messenger_interface->dataSent(this, name, val_str);

    strcpy(name, "last_cde_id");
    sprintf(val_str, "%d", last_cde_id);
    m_messenger_interface->dataSent(this, name, val_str);

#endif // MESSENGER_FULL
}
