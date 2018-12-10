#include <stdio.h>
#include "databasebase.h"
#include "messagebase.h"
#include "transporterbase.h"
#include "messengereventbase.h"

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
/*! \brief Set the event manager.
 *
 * \param event_mgr : the event manager
 */
void DatabaseBase::setEventManager(MessengerEventBase *event_mgr)
{
    m_event_mgr = event_mgr;
    for (int i=0; i<getMessageCount(); i++) {
        if (m_p_messages_list[i]) {
            m_p_messages_list[i]->setEventManager(event_mgr);
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
void DatabaseBase::decode(const tMessengerFrame* frame)
{
    for (unsigned short int i=0; i<getMessageCount(); i++) {
        MessageBase *msg = m_p_messages_list[i];
        if (msg) {
            if ( (msg->getID() == frame->ID) && (msg->getDirection() & MSG_RX) ){
                msg->decode(frame->Data);
                msg->setSourceAddress(frame->SourceAddress);
                if (m_event_mgr) m_event_mgr->newMessageReceived(msg);
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
    if (m_event_mgr) m_event_mgr->messageTransmited(msg);
}