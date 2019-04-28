#ifndef _NODE_LEGOBOT_H_
#define _NODE_LEGOBOT_H_

#include "nodebase.h"

// ====================================================
//        NODE
// ====================================================
class NodeLegobot: public NodeBase
{
public:
    inline const char *getName() { return "ROBOT_LEGO"; }
    inline unsigned short getID() { return 2; }
};

#endif // _NODE_LEGOBOT_H_
