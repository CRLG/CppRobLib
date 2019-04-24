#ifndef _MESSAGES_ROBOT_LEGO_2019
#define _MESSAGES_ROBOT_LEGO_2019

#include "messagebase.h"

class Message_ROBOT_LEGO_2019 : public MessageBase
{
public:
    Message_ROBOT_LEGO_2019();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    unsigned char Nb_points_lego;
    unsigned char Atomes_distr_removed;
    unsigned char Nb_atomes_balance_lego;
};

#endif // _MESSAGES_ROBOT_LEGO_2019
