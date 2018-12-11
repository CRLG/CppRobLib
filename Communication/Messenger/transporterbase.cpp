#include <stdio.h>
#include "databasebase.h"
#include "transporterbase.h"
#include "messengereventbase.h"
#include "messengerinterfacebase.h"

TransporterBase::TransporterBase()
    : m_database(NULL),
      m_messenger_interface(NULL),
      m_event_mgr(NULL)
{
}

TransporterBase::~TransporterBase()
{
}

// ____________________________________________________________
void TransporterBase::setMessengerInterface(MessengerInterfaceBase *messenger_interface )
{
    m_messenger_interface = messenger_interface;
}

// ____________________________________________________________
void TransporterBase::setDatabase(DatabaseBase *database)
{
    m_database = database;
}

// ____________________________________________________________
/*! \brief Set the event manager.
 *
 * \param event_mgr : the event manager
 */
void TransporterBase::setEventManager(MessengerEventBase *event_mgr)
{
    m_event_mgr = event_mgr;
}

// ____________________________________________________________
/*! \brief Inform the database a new frame was received.
 *
 * \param packet : the packet to compute
 * \return the checksum
 * \remarks :
  */
void TransporterBase::readyFrame(tMessengerFrame* frame)
{
    if (m_event_mgr) m_event_mgr->newFrameReceived(frame);
    if (m_database) m_database->decode(frame);
}
