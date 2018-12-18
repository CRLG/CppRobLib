#include <stdio.h>
#include "messagebase.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"

MessageBase::MessageBase() :
    m_id(0),
    m_dlc(0),
    m_direction(MSG_RXTX),
    m_destination_address(0),
    m_source_address(0),
    m_updated(0),
    m_database(NULL),
    m_messenger_interface(NULL)
{
}

MessageBase::~MessageBase()
{
}

// ____________________________________________________________
/*! \brief Set message direction.
 *
 * \param direction : the direction MSG_RX / MSG_TX / MSG_RXTX
 */
void MessageBase::setDirection(tMessageDirection direction)
{
    m_direction = direction;
}

// ____________________________________________________________
/*! \brief Set the message destination address.
 *
 * \param address: the destination address of the message
 */
void MessageBase::setDestinationAddress(unsigned short address)
{
    m_destination_address = address;
}

// ____________________________________________________________
/*! \brief Set the message source address.
 *
 * \param address: the source of the message (where the message come from)
 */
void MessageBase::setSourceAddress(unsigned short address)
{
    m_source_address = address;
}

// ____________________________________________________________
/*! \brief Set the database the message belong to.
 *
 * \param database : the database the message belong to
 */
void MessageBase::setDatabase(DatabaseBase *database)
{
    m_database = database;
}

// ____________________________________________________________
/*! \brief Set the messenger interface.
 *
 * \param messenger_interface : the messenger interface
 */
void MessageBase::setMessengerInterface(MessengerInterfaceBase* messenger_interface)
{
    m_messenger_interface = messenger_interface;
}

// ____________________________________________________________
/*! \brief Get the message name.
 *  \remarks default message name is none
 */
const char* MessageBase::getName()
{
    return "";
}

// ____________________________________________________________
/*! \brief Return true if a new message is available and data are updated.
 *
 * \param frame : the frame to decode
 */
bool MessageBase::isNewMessage()
{
    if (m_updated) {
        m_updated = false;
        return true;
    }
    return false;
}

// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void MessageBase::decode(const unsigned char *buff_data)
{
    // no action if message is a TX only message
    (void)buff_data; // avoid warning
}

// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void MessageBase::encode(unsigned char *buff_data)
{
    // no action if message is a RX only message
    (void)buff_data; // avoid warning
}
// ____________________________________________________________
/*! \brief Send the message.
 *
 */
void MessageBase::send()
{
    if (m_database) m_database->encode(this);
}
