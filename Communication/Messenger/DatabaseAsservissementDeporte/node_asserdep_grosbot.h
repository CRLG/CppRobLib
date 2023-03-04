#ifndef _NODE_ASSERDEP_GROSBOT_H_
#define _NODE_ASSERDEP_GROSBOT_H_

#include "nodebase.h"

// ====================================================
//        NODE
// ====================================================
class NodeAsserdepGrosbot: public NodeBase
{
public:
    inline const char *getName() { return "GROSBOT"; }
    inline unsigned short getID() { return 1; }
};

#endif // _NODE_ASSERDEP_GROSBOT_H_
