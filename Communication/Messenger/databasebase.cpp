#include <stdio.h>
#include "databasebase.h"
#include "messagebase.h"
#include "transporterbase.h"
#include "messengerinterfacebase.h"
#include "nodebase.h"

DatabaseBase::DatabaseBase() :
    m_p_messages_list(NULL),
    m_p_nodes_list(NULL),
    m_transporter(NULL)
{
}

DatabaseBase::~DatabaseBase()
{
}

// ____________________________________________________________
void DatabaseBase::setTransporter(TransporterBase *transporter)
{
    if (transporter) {
       m_transporter = transporter;
    }
}

// ____________________________________________________________
/*! \brief Set the mesenger interface.
 *
 * \param messenger_interface : the interface
 */
void DatabaseBase::setMessengerInterface(MessengerInterfaceBase *messenger_interface)
{
    m_messenger_interface = messenger_interface;
    for (int i=0; i<getMessageCount(); i++) {
        if (m_p_messages_list[i]) {
            m_p_messages_list[i]->setMessengerInterface(messenger_interface);
        }
    }
    for (int i=0; i<getNodeCount(); i++) {
        if (m_p_nodes_list[i]) {
            m_p_nodes_list[i]->setMessengerInterface(messenger_interface);
        }
    }
}

// ____________________________________________________________
const char* DatabaseBase::getName()
{
    return "";
}

// ___________________________________________________________
/*! \brief Link each message with database.
 *
 */
void DatabaseBase::initMessages()
{
    for (int i=0; i<getMessageCount(); i++) {
        if (m_p_messages_list[i]) {
            m_p_messages_list[i]->setDatabase(this);
        }
    }
}

// ___________________________________________________________
/*! \brief Link each message with database.
 *
 */
void DatabaseBase::initNodes()
{
    for (int i=0; i<getNodeCount(); i++) {
        if (m_p_nodes_list[i]) {
            m_p_nodes_list[i]->setDatabase(this);
        }
    }
}

// ___________________________________________________________
/*! \brief Clean messages.
 *
 */
void DatabaseBase::restart()
{
    for (int i=0; i<getMessageCount(); i++) {
        if (m_p_messages_list[i]) {
            m_p_messages_list[i]->isNewMessage();
        }
    }
}

// ___________________________________________________________
void DatabaseBase::decode(const tMessengerFrame* frame)
{
    for (unsigned short int i=0; i<getMessageCount(); i++) {
        MessageBase *msg = m_p_messages_list[i];
        if (msg) {
            if ( (msg->getID() == frame->ID) && (msg->getDirection() & MSG_RX) ){
                msg->setSourceAddress(frame->SourceAddress);
                msg->decode(frame->Data);
                if (m_messenger_interface) {
                    long current_time = m_messenger_interface->getTime();
                    msg->setLastTransfertTime(current_time);
                    m_messenger_interface->newMessageReceived(msg);
                    // inform the node a message was received
                    NodeBase *node = nodeIdToNode(frame->SourceAddress);
                    if (node) node->newMessageReceived(current_time);
                }
            }
        }
    }
}

// ____________________________________________________________
/*! \brief Encode the message in a frame.
 *
 * \param frame : the frame to encode
 */
void DatabaseBase::encode(MessageBase* msg)
{
    tMessengerFrame frame;
    frame.ID = msg->getID();
    frame.DLC = msg->getDLC();
    frame.DestinationAddress = msg->getDestinationAddress();
    msg->encode(frame.Data);
    if (m_transporter) m_transporter->encode(&frame);
    if (m_messenger_interface) {
        msg->setLastTransfertTime(m_messenger_interface->getTime());
        m_messenger_interface->messageTransmited(msg);
    }
}


// ____________________________________________________________
/*! \brief Check for each TX message if it's time to send
 *
 */
void DatabaseBase::checkAndSendPeriodicMessages()
{
    long current_time = 0;
    if (m_messenger_interface) current_time = m_messenger_interface->getTime();

    for (int i=0; i<getMessageCount(); i++) {
        if (m_p_messages_list[i] && (m_p_messages_list[i]->getDirection() & MSG_TX)) {
            if (m_p_messages_list[i]->isTimeToTransfert(current_time)) {
                m_p_messages_list[i]->send();
            }
        }
    }
}

// ____________________________________________________________
/*! \brief Check for each node the communication
 *
 */
void DatabaseBase::checkNodeCommunication()
{
    long current_time = 0;
    if (m_messenger_interface) current_time = m_messenger_interface->getTime();

    for (int i=0; i<getNodeCount(); i++) {
        if (m_p_nodes_list[i]) {
                m_p_nodes_list[i]->diagPresence(current_time);
        }
    }
}

// ____________________________________________________________
/*! \brief Return the node pointer from ID
 *
 */
NodeBase* DatabaseBase::nodeIdToNode(unsigned short id)
{
    for (int i=0; i<getNodeCount(); i++) {
        if (m_p_nodes_list[i]) {
            if (m_p_nodes_list[i]->getID() == id) return m_p_nodes_list[i];
        }
    }
    return NULL;
}

// ____________________________________________________________
const char* DatabaseBase::nodeIdToName(unsigned short id)
{
    NodeBase *node = nodeIdToNode(id);
    if (node) return node->getName();
    return "";
}

// ____________________________________________________________
MessageBase** DatabaseBase::getMessagesList() const
{
    return m_p_messages_list;
}

// ____________________________________________________________
NodeBase** DatabaseBase::getNodesList() const
{
    return m_p_nodes_list;
}
