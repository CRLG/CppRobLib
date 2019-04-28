#ifndef _NODE_DIAG_TOOL_H_
#define _NODE_DIAG_TOOL_H_

#include "nodebase.h"

// ====================================================
//        NODE
// ====================================================
class NodeDiagTool: public NodeBase
{
public:
    inline const char *getName() { return "DIAG_TOOL"; }
    inline unsigned short getID() { return 5; }
};

#endif // _NODE_DIAG_TOOL_H_
