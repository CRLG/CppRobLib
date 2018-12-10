#include "databasesystemcup2019.h"

DatabaseSystemCup2019::DatabaseSystemCup2019()
{
    initMessages();
}

DatabaseSystemCup2019::~DatabaseSystemCup2019()
{
}

// ____________________________________________________________
unsigned short DatabaseSystemCup2019::getMessageCount()
{
    return MESSAGES_COUNT;
}

// ____________________________________________________________
const char* DatabaseSystemCup2019::getName()
{
    return "DATABASE_SYSTEM_CUP2019";
}

// ___________________________________________________________
void DatabaseSystemCup2019::getVersion(unsigned char *maj, unsigned char *min)
{
    *maj=1;
    *min=0;
}

// ___________________________________________________________
void DatabaseSystemCup2019::initMessages()
{
    unsigned char i;

    for (i=0; i<getMessageCount(); i++)
    {
        m_messages_list[i] = 0;
    }
    i=0;
    m_messages_list[i++] = &m_TimestampMatch;
    m_messages_list[i++] = &m_ExperienceStatus;

    m_p_messages_list = m_messages_list;

    DatabaseBase::initMessages();
}

