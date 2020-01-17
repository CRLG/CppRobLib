#include "stdio.h"
#include "string.h"
#include "message_robot_lego_2019.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"

// ==============================================================
// ==============================================================
Message_ROBOT_LEGO_2019::Message_ROBOT_LEGO_2019()
{
    m_id = 0x62;
    m_dlc = 3;
}

// ____________________________________________________________
const char* Message_ROBOT_LEGO_2019::getName()
{
    return "ROBOT_LEGO_2019";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_ROBOT_LEGO_2019::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    unsigned char old_Nb_points_lego = Nb_points_lego;
    unsigned char old_Atomes_distr_removed = Atomes_distr_removed;
    unsigned char old_Nb_atomes_balance_lego = Nb_atomes_balance_lego;
#endif // MESSENGER_FULL

    Nb_points_lego = buff_data[0];
    Atomes_distr_removed = buff_data[1];
    Nb_atomes_balance_lego = buff_data[2];;

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[25];
        char val_str[10];
        strcpy(name, "Nb_points_lego");
        sprintf(val_str, "%d", Nb_points_lego);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (Nb_points_lego != old_Nb_points_lego) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "Atomes_distr_removed");
        sprintf(val_str, "%d", Atomes_distr_removed);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (Atomes_distr_removed != old_Atomes_distr_removed) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "Nb_atomes_balance_lego");
        sprintf(val_str, "%d", Nb_atomes_balance_lego);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (Nb_atomes_balance_lego != old_Nb_atomes_balance_lego) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_ROBOT_LEGO_2019::encode(unsigned char *buff_data)
{
    buff_data[0] = Nb_points_lego;
    buff_data[1] = Atomes_distr_removed;
    buff_data[2] = Nb_atomes_balance_lego;

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[25];
        char val_str[10];
        strcpy(name, "Nb_points_lego");
        sprintf(val_str, "%d", Nb_points_lego);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "Atomes_distr_removed");
        sprintf(val_str, "%d", Atomes_distr_removed);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "Nb_atomes_balance_lego");
        sprintf(val_str, "%d", Nb_atomes_balance_lego);
        m_messenger_interface->dataSent(this, name, val_str);
    }
#endif // MESSENGER_FULL
}
