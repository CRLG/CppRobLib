#include "stdio.h"
#include "string.h"
#include "message_free_string.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"

// ==============================================================
// ==============================================================
Message_FREE_STRING::Message_FREE_STRING()
{
    m_id = 0x02;
    m_dlc = STR_BUFFER_SIZE;
}

// ____________________________________________________________
const char* Message_FREE_STRING::getName()
{
    return "FREE_STRING";
}

// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_FREE_STRING::decode(const unsigned char *buff_data)
{
    memcpy(str, buff_data, STR_BUFFER_SIZE);
    str[STR_BUFFER_SIZE-1] = '\0'; // ensure string terminaison is null

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        strcpy(name, "str");
        m_messenger_interface->dataUpdated(this, name, str);
        m_messenger_interface->dataChanged(this, name, str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_FREE_STRING::encode(unsigned char *buff_data)
{
    memcpy(buff_data, str, STR_BUFFER_SIZE);
    buff_data[STR_BUFFER_SIZE-1] = '\0';

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[20];
        strcpy(name, "str");
        m_messenger_interface->dataSent(this, name, str);
    }
#endif // MESSENGER_FULL
}
