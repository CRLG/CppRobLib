#include <stdio.h>
#include "databasebase.h"
#include "messagebase.h"
#include "transporterbase.h"
#include "messengerinterfacebase.h"

DatabaseBase::DatabaseBase() :
    m_p_messages_list(NULL),
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
        for (unsigned int i=0; i<getMessageCount(); i++) {
            if (m_p_messages_list[i]) m_p_messages_list[i]->setDatabase(this);
        }
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
                msg->decode(frame->Data);
                msg->setSourceAddress(frame->SourceAddress);
                if (m_messenger_interface) m_messenger_interface->newMessageReceived(msg);
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
    if (m_messenger_interface) m_messenger_interface->messageTransmited(msg);
}
