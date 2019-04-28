#ifndef _NODE_EXPERIENCE_H_
#define _NODE_EXPERIENCE_H_

#include "nodebase.h"

// ====================================================
//        NODE
// ====================================================
class NodeExperience: public NodeBase
{
public:
    inline const char *getName() { return "EXPERIENCE"; }
    inline unsigned short getID() { return 4; }
};

#endif // _NODE_EXPERIENCE_H_
