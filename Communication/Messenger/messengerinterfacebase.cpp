#include <stdio.h>
#include "messengerinterfacebase.h"
#include "databasebase.h"

MessengerInterfaceBase::MessengerInterfaceBase() :
    m_transporter(NULL),
    m_database(NULL)
{
}

MessengerInterfaceBase::~MessengerInterfaceBase()
{
}

// ____________________________________________________________
void MessengerInterfaceBase::init(TransporterBase *transporter, DatabaseBase *database)
{
    m_transporter = transporter;
    if (m_transporter) {
        m_transporter->setDatabase(database);
        m_transporter->setMessengerInterface(this);
    }

    m_database = database;
    if (m_database) {
        m_database->setTransporter(m_transporter);
        m_database->setMessengerInterface(this);
    }
}

// ____________________________________________________________
void MessengerInterfaceBase::decode(unsigned char newData, unsigned short source_address)
{
    if (m_transporter) m_transporter->decode(newData, source_address);
}

// ____________________________________________________________
void MessengerInterfaceBase::decode(unsigned char *buff_data, unsigned char buff_size, unsigned short source_address)
{
    if (m_transporter) m_transporter->decode(buff_data, buff_size, source_address);
}


// ============================================================
//                    EVENTS TO APPLICATION
// ============================================================

// ____________________________________________________________
void MessengerInterfaceBase::newFrameReceived(tMessengerFrame *frame)
{
    (void)frame;
}

// ____________________________________________________________
void MessengerInterfaceBase::frameTransmited(tMessengerFrame *frame)
{
    (void)frame;
}

// ____________________________________________________________
void MessengerInterfaceBase::newMessageReceived(MessageBase *msg)
{
    (void)msg;
}

// ____________________________________________________________
void MessengerInterfaceBase::messageTransmited(MessageBase *msg)
{
    (void)msg;
}

// ____________________________________________________________
void MessengerInterfaceBase::dataUpdated(char *name, char *val_str)
{
    (void)name;
    (void)val_str;
}

// ____________________________________________________________
void MessengerInterfaceBase::dataChanged(char *name, char *val_str)
{
    (void)name;
    (void)val_str;
}
