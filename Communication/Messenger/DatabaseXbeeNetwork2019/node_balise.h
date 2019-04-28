#ifndef _NODE_BALISE_H_
#define _NODE_BALISE_H_

#include "nodebase.h"

// ====================================================
//        NODE
// ====================================================
class NodeBalise: public NodeBase
{
public:
    inline const char *getName() { return "BALISE"; }
    inline unsigned short getID() { return 3; }
};

#endif // _NODE_BALISE_H_
