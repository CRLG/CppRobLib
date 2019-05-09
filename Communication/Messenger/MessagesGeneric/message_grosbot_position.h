#ifndef _MESSAGES_GROSBOT_POSITION
#define _MESSAGES_GROSBOT_POSITION

#include "messagebase.h"

class Message_GROSBOT_POSITION : public MessageBase
{
public:
    Message_GROSBOT_POSITION();

    virtual const char* getName();
    virtual void decode(const unsigned char *buff_data);
    virtual void encode(unsigned char *buff_data);

    short Position_X;   // 1LSB = 10mm (0.1cm) -> -3000mm;+3000mm (-3.000m;+3.000m)
    short Position_Y;   // 1LSB = 10mm (0.1cm) -> -3000mm;+3000mm (-3.000m;+3.000m)
    short Angle;        // 1LSB = 1° -> -180°;+180°
};

#endif // _MESSAGES_GROSBOT_POSITION
