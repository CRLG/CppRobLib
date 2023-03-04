#include "databaseasservissementdeporte.h"

DatabaseAsservissementDeporte::DatabaseAsservissementDeporte()
{
    initMessages();
    initNodes();
    m_msg_id = 0;
}

DatabaseAsservissementDeporte::~DatabaseAsservissementDeporte()
{
}

// ____________________________________________________________
unsigned short DatabaseAsservissementDeporte::getMessageCount()
{
    return MESSAGES_COUNT;
}

// ____________________________________________________________
unsigned short DatabaseAsservissementDeporte::getNodeCount()
{
    return NODES_COUNT;
}

// ____________________________________________________________
const char* DatabaseAsservissementDeporte::getName()
{
    return "DATABASE_ASSERV_DEPORTE";
}

// ___________________________________________________________
void DatabaseAsservissementDeporte::getVersion(unsigned char *maj, unsigned char *min)
{
    *maj=1;
    *min=0;
}

// ___________________________________________________________
void DatabaseAsservissementDeporte::initNodes()
{
    unsigned char i;

    for (i=0; i<getNodeCount(); i++)
    {
        m_nodes_list[i] = 0;
    }
    i=0;
    m_nodes_list[i++] = &m_node_grosbot;
    m_nodes_list[i++] = &m_node_asserv_deporte;

    m_p_nodes_list = m_nodes_list;

    DatabaseBase::initNodes();
}


// ___________________________________________________________
void DatabaseAsservissementDeporte::initMessages()
{
    unsigned char i;

    for (i=0; i<getMessageCount(); i++)
    {
        m_messages_list[i] = 0;
    }
    i=0;
    m_messages_list[i++] = &m_EtatAsservissement;
    m_messages_list[i++] = &m_CommandeMouvementXY;
    m_messages_list[i++] = &m_CommandeMouvementXYTeta;
    m_messages_list[i++] = &m_CommandeMouvementDistanceAngle;
    m_messages_list[i++] = &m_CommandeMouvementXY_A;
    m_messages_list[i++] = &m_CommandeMouvementXY_B;
    m_messages_list[i++] = &m_CommandeVitesseMouvement;

    m_p_messages_list = m_messages_list;

    DatabaseBase::initMessages();
}


