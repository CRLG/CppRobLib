#include "databasexbeenetwork2019.h"

DatabaseXbeeNetwork2019::DatabaseXbeeNetwork2019()
{
    initMessages();
    initNodes();
}

DatabaseXbeeNetwork2019::~DatabaseXbeeNetwork2019()
{
}

// ____________________________________________________________
unsigned short DatabaseXbeeNetwork2019::getMessageCount()
{
    return MESSAGES_COUNT;
}

// ____________________________________________________________
unsigned short DatabaseXbeeNetwork2019::getNodeCount()
{
    return NODES_COUNT;
}

// ____________________________________________________________
const char* DatabaseXbeeNetwork2019::getName()
{
    return "DATABASE_SYSTEM_CUP2019";
}

// ___________________________________________________________
void DatabaseXbeeNetwork2019::getVersion(unsigned char *maj, unsigned char *min)
{
    *maj=1;
    *min=0;
}

// ___________________________________________________________
void DatabaseXbeeNetwork2019::initNodes()
{
    unsigned char i;

    for (i=0; i<getNodeCount(); i++)
    {
        m_nodes_list[i] = 0;
    }
    i=0;
    m_nodes_list[i++] = &m_node_grosbot;
    m_nodes_list[i++] = &m_node_legobot;
    m_nodes_list[i++] = &m_node_balise;
    m_nodes_list[i++] = &m_node_experience;
    m_nodes_list[i++] = &m_node_diag_tool;

    m_p_nodes_list = m_nodes_list;

    DatabaseBase::initNodes();
}


// ___________________________________________________________
void DatabaseXbeeNetwork2019::initMessages()
{
    unsigned char i;

    for (i=0; i<getMessageCount(); i++)
    {
        m_messages_list[i] = 0;
    }
    i=0;
    m_messages_list[i++] = &m_TimestampMatch;
    m_messages_list[i++] = &m_ExperienceStatus;
    m_messages_list[i++] = &m_RobotLego2019;
    m_messages_list[i++] = &m_RobotLegoStatus;
    m_messages_list[i++] = &m_BalisePositions;
    m_messages_list[i++] = &m_FreeString;

    m_p_messages_list = m_messages_list;

    DatabaseBase::initMessages();
}


