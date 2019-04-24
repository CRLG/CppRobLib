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
