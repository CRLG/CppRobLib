#ifndef _Q_SERVOAX_UTILS_H
#define _Q_SERVOAX_UTILS_H

#include <QList>

#include "servoaxbase.h"
#include "servoax_utils.h"

// ====================================================
//         QT UTILITIES FOR AX SERVOMOTORS
// ====================================================
class QServoAXUtils
{
public:
    static QList<unsigned char> getRegistersList();
    static QList<unsigned char> getEepromRegistersList();
    static QList<unsigned char> getRamRegistersList();
};

#endif // _Q_SERVOAX_UTILS_H
