#include "stdio.h"
#include "string.h"
#include "message_experience_status.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"

// ==============================================================
// ==============================================================
Message_EXPERIENCE_STATUS::Message_EXPERIENCE_STATUS()
{
    m_id = 0x30;
    m_dlc = 2;
}

// ____________________________________________________________
const char* Message_EXPERIENCE_STATUS::getName()
{
    return "EXPERIENCE_STATUS";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_EXPERIENCE_STATUS::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    short old_ExperienceStatus = ExperienceStatus;
#endif // MESSENGER_FULL

    ExperienceStatus= ((short)buff_data[0] << 8) + buff_data[1];

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[40];
        char val_str[10];
        sprintf(name, "%s.ExperienceStatus", getName());
        sprintf(val_str, "%d", ExperienceStatus);
        m_messenger_interface->dataUpdated(name, val_str);
        if (ExperienceStatus != old_ExperienceStatus) m_messenger_interface->dataChanged(name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_EXPERIENCE_STATUS::encode(unsigned char *buff_data)
{
    buff_data[0] = ExperienceStatus >> 8;
    buff_data[1] = ExperienceStatus & 0xFF;
}
