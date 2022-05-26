#include "stdio.h"
#include "string.h"
#include "message_commande_experience.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"

// ==============================================================
// ==============================================================
Message_COMMANDE_EXPERIENCE::Message_COMMANDE_EXPERIENCE()
{
    m_id = 0x39;
    m_dlc = 2;
    ExperienceCmd = EXPERIENCE_CMD_NO_ACTION;
}

// ____________________________________________________________
const char* Message_COMMANDE_EXPERIENCE::getName()
{
    return "COMMANDE_EXPERIENCE";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_COMMANDE_EXPERIENCE::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    short old_ExperienceCmd = ExperienceCmd;
#endif // MESSENGER_FULL

    ExperienceCmd= ((short)buff_data[0] << 8) + buff_data[1];

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];
        strcpy(name, "ExperienceCmd");
        sprintf(val_str, "%d", ExperienceCmd);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (ExperienceCmd != old_ExperienceCmd) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_COMMANDE_EXPERIENCE::encode(unsigned char *buff_data)
{
    buff_data[0] = ExperienceCmd >> 8;
    buff_data[1] = ExperienceCmd & 0xFF;

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        char val_str[10];
        strcpy(name, "ExperienceCmd");
        sprintf(val_str, "%d", ExperienceCmd);
        m_messenger_interface->dataSent(this, name, val_str);
    }
#endif // MESSENGER_FULL
}
