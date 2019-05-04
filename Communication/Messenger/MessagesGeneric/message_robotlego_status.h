#ifndef _MESSAGES_ROBOLEGO_STATUS
#define _MESSAGES_ROBOLEGO_STATUS

#include "messagebase.h"

class Message_ROBOTLEGO_STATUS : public MessageBase
{
public:
    Message_ROBOTLEGO_STATUS();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    typedef enum {
        ROBOTLEGO_EN_PREPARATION = 0,
        ROBOTLEGO_EN_ATTENTE,
        ROBOTLEGO_MATCH_EN_COURS,
        ROBOTLEGO_FIN_DU_MATCH = 10,
        ROBOTLEGO_EN_DEFAUT = -1
    }RobotLegoStatusEnum;

    short Status;
    short Position_X;
    short Position_Y;
    short Angle;
};

#endif // _MESSAGES_ROBOLEGO_STATUS
