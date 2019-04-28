#include "nodebase.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"

NodeBase::NodeBase() :
    m_lost_time_duration(3000),
    m_last_time_receive_message(0),
    m_present(false),
    m_database(NULL),
    m_messenger_interface(NULL)
{
}

NodeBase::~NodeBase()
{
}

// ____________________________________________________________
/*! \brief Set the database the message belong to.
 *
 * \param database : the database the node belong to
 */
void NodeBase::setDatabase(DatabaseBase *database)
{
    m_database = database;
}

// ____________________________________________________________
/*! \brief Set the messenger interface.
 *
 * \param messenger_interface : the messenger interface
 */
void NodeBase::setMessengerInterface(MessengerInterfaceBase* messenger_interface)
{
    m_messenger_interface = messenger_interface;
}

// ____________________________________________________________
bool NodeBase::isPresent()
{
    return m_present;
}

// ____________________________________________________________
void NodeBase::setLostComDuration(long lost_time)
{
    m_lost_time_duration = lost_time;
}

// ____________________________________________________________
void NodeBase::newMessageReceived(long current_time)
{
    m_last_time_receive_message = current_time;
}

// ____________________________________________________________
void NodeBase::diagPresence(long current_time)
{
    bool old_present = m_present;
    if ( (current_time - m_last_time_receive_message) > m_lost_time_duration ) {
        m_present = false;
    }
    else {
        m_present = true;
    }

    if (m_present != old_present) {
        if (m_messenger_interface) {
            m_messenger_interface->nodeCommunicationStatusChanged(this);
        }
    }

}

