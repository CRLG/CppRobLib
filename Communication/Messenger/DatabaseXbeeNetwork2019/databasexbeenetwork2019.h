#ifndef _DATABASE_XBEE_NETWORK_2019_H
#define _DATABASE_XBEE_NETWORK_2019_H

#include "message_timestamp_match.h"
#include "message_grosbot_position.h"
#include "message_balise_positions.h"
#include "message_experience_status.h"
#include "message_commande_experience.h"
#include "message_robot_lego_2019.h"
#include "message_robotlego_status.h"
#include "message_free_string.h"

#include "node_grosbot.h"
#include "node_legobot.h"
#include "node_balise.h"
#include "node_experience.h"
#include "node_diag_tool.h"

#include "databasebase.h"

// ====================================================
//        DATABASE
// ====================================================
class DatabaseXbeeNetwork2019 : public DatabaseBase
{
public:
    DatabaseXbeeNetwork2019();
    ~DatabaseXbeeNetwork2019();

    virtual const char *getName();
    virtual void getVersion(unsigned char *maj, unsigned char *min);
    virtual unsigned short getMessageCount();
    virtual unsigned short getNodeCount();

    static const unsigned short NODES_COUNT = 5;
    NodeBase *m_nodes_list[NODES_COUNT];

#define NODE_ID_TEST 5

    NodeGrosbot m_node_grosbot;
    NodeLegobot m_node_legobot;
    NodeBalise m_node_balise;
    NodeExperience m_node_experience;
    NodeDiagTool m_node_diag_tool;


    static const unsigned short MESSAGES_COUNT = 8;
    MessageBase *m_messages_list[MESSAGES_COUNT];

    Message_TIMESTAMP_MATCH m_TimestampMatch;
    Message_GROSBOT_POSITION m_GrosbotPosition;
    Message_EXPERIENCE_STATUS m_ExperienceStatus;
    Message_COMMANDE_EXPERIENCE m_CommandeExperience;
    Message_ROBOT_LEGO_2019 m_RobotLego2019;
    Message_ROBOTLEGO_STATUS m_RobotLegoStatus;
    Message_BALISE_POSITIONS m_BalisePositions;
    Message_FREE_STRING m_FreeString;

private :
    void initMessages();
    void initNodes();
};

#endif // _DATABASE_XBEE_NETWORK_2019_H
