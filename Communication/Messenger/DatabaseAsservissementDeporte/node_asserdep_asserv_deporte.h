#ifndef _NODE_ASSERDEP_ASSERV_H_
#define _NODE_ASSERDEP_ASSERV_H_

#include "nodebase.h"

// ====================================================
//        NODE
// ====================================================
class NodeAsserdepAsservissement: public NodeBase
{
public:
    inline const char *getName() { return "ASSERVISSEMENT"; }
    inline unsigned short getID() { return 4; }
};

#endif // _NODE_ASSERDEP_ASSERV_H_
