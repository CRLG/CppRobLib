#include <stdio.h>
#include "messengerinterfacebase.h"

MessengerInterfaceBase::MessengerInterfaceBase() :
    m_transporter(NULL)
{
}

MessengerInterfaceBase::~MessengerInterfaceBase()
{
}

// ____________________________________________________________
void MessengerInterfaceBase::setTransporter(TransporterBase *transporter)
{
    m_transporter = transporter;
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
