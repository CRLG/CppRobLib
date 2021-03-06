#include "stdio.h"
#include "string.h"
#include "message_timestamp_match.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"

// ==============================================================
// ==============================================================
Message_TIMESTAMP_MATCH::Message_TIMESTAMP_MATCH()
{
    m_id = 0x1;
    m_dlc = 2;
}

// ____________________________________________________________
const char* Message_TIMESTAMP_MATCH::getName()
{
    return "TIMESTAMP_MATCH";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_TIMESTAMP_MATCH::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    short old_Timestamp = Timestamp;
#endif // MESSENGER_FULL

    Timestamp= ((short)buff_data[0] << 8) + buff_data[1];

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[16];
        char val_str[10];
        strcpy(name, "Timestamp");
        sprintf(val_str, "%d", Timestamp);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (Timestamp != old_Timestamp) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_TIMESTAMP_MATCH::encode(unsigned char *buff_data)
{
    buff_data[0] = Timestamp >> 8;
    buff_data[1] = Timestamp & 0xFF;

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[16];
        char val_str[10];
        strcpy(name, "Timestamp");
        sprintf(val_str, "%d", Timestamp);
        m_messenger_interface->dataSent(this, name, val_str);
    }
#endif // MESSENGER_FULL
}
