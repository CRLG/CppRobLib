#ifndef _DATABASE_SYSTEM_CUP_2019_H
#define _DATABASE_SYSTEM_CUP_2019_H

#include "message_timestamp_match.h"
#include "message_experience_status.h"

#include "databasebase.h"

// ====================================================
//        DATABASE
// ====================================================
class DatabaseSystemCup2019 : public DatabaseBase
{
public:
    DatabaseSystemCup2019();
    ~DatabaseSystemCup2019();

    virtual const char *getName();
    virtual void getVersion(unsigned char *maj, unsigned char *min);
    virtual unsigned short getMessageCount();

#define MESSAGES_COUNT 2
    MessageBase *m_messages_list[MESSAGES_COUNT];

    Message_TIMESTAMP_MATCH m_TimestampMatch;
    Message_EXPERIENCE_STATUS m_ExperienceStatus;
private :
    void initMessages();
};

#endif // _DATABASE_SYSTEM_CUP_2019_H
