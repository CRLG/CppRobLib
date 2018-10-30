#ifndef _SERVOAX_UTILS_H
#define _SERVOAX_UTILS_H

#include "servoaxbase.h"

// ====================================================
//         UTILITIES FOR AX SERVOMOTORS
// ====================================================
class ServoAXUtils
{
public:
    typedef enum {
        REGTYPE_BOOL    = 1,
        REGTYPE_UINT8   = 8,
        REGTYPE_UINT16  = 16
    }tRegDataType;

    static const char *ErrorCodeToText(tAxErr err);
    static const char *RegisterAddressToText(unsigned char reg_addr);
    static const char *RegisterAddressToInfo(unsigned char reg_addr);
    static const char *StatusBitToText(unsigned char bitfield);
    static void StatusCodeToText(unsigned char status, char *buff, unsigned long buff_size);
    static const char *BaudrateToText(unsigned char baudrate);
    static bool isRegisterInEEPROM(unsigned char reg_addr);
    static bool isRegisterInRAM(unsigned char reg_addr);
    static bool isRegisterWritable(unsigned char reg_addr);
    static bool isRegisterReserved(unsigned char reg_addr);
    static tRegDataType RegisterToDataType(unsigned char reg_addr);
    static unsigned char getLastRegisterAddress();
};

#endif // _SERVOAX_UTILS_H
