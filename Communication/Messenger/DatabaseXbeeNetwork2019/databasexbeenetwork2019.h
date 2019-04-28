#ifndef _DATABASE_XBEE_NETWORK_2019_H
#define _DATABASE_XBEE_NETWORK_2019_H

#include "message_timestamp_match.h"
#include "message_balise_positions.h"
#include "message_experience_status.h"
#include "message_robot_lego_2019.h"

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
    virtual const char *NodeIdToName(unsigned short id);

    typedef enum {
        NODE_ID_ROBOT_MECA   = 0x1,
        NODE_ID_ROBOT_LEGO   = 0x2,
        NODE_ID_BALISE       = 0x3,
        NODE_ID_EXPERIENCE   = 0x4,
        NODE_ID_DIAG_TOOL    = 0x5
    }tDatabase2019_XbeeID;

#define MESSAGES_COUNT 4
    MessageBase *m_messages_list[MESSAGES_COUNT];

    Message_TIMESTAMP_MATCH m_TimestampMatch;
    Message_EXPERIENCE_STATUS m_ExperienceStatus;
    Message_ROBOT_LEGO_2019 m_RobotLego2019;
    Message_BALISE_POSITIONS m_BalisePositions;

private :
    void initMessages();
};

#endif // _DATABASE_XBEE_NETWORK_2019_H
