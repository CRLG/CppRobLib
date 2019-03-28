#ifndef _MCP23017_BASE_H_
#define _MCP23017_BASE_H_


// ================================================================
//      ABSTRACT DRIVER CLASS FOR MICROCHIP MCP23017 IO EXPANDER
// ================================================================
class MCP23017Base
{
public:

typedef enum {
     MCP23017_IODIRA      = 0x00,
     MCP23017_IODIRB      = 0x01,
     MCP23017_IPOLA       = 0x02,
     MCP23017_IPOLB       = 0x03,
     MCP23017_GPINTENA    = 0x04, 
     MCP23017_GPINTENB    = 0x05,
     MCP23017_GPPUA       = 0x0C,
     MCP23017_GPPUB       = 0x0D,
     MCP23017_GPIOA       = 0x12,
     MCP23017_GPIOB       = 0x13,
     MCP23017_OLATA       = 0x14,
     MCP23017_OLATB       = 0x15
}tRegMCP23017;

    MCP23017Base();
    virtual ~MCP23017Base();

    void setAddress(unsigned char address_8bits);

    unsigned char readRegister(unsigned char reg_addr);
    void writeRegister(unsigned char reg_addr, unsigned char data);

    void configDirectionPortA(unsigned char direction);
    void configDirectionPortB(unsigned char direction);
    void configDirections(unsigned char directionA, unsigned char directionB);

    void configPullUpPortA(unsigned char pullup);
    void configPullUpPortB(unsigned char pullup);
    void configPullUp(unsigned char pullupA, unsigned char pullupB);

    void writePortA(unsigned char port_val);
    void writePortB(unsigned char port_val);
    void writePorts(unsigned char portA, unsigned char portB);
    void writeBitPortA(unsigned char bit_pos, bool val);
    void writeBitPortB(unsigned char bit_pos, bool val);

    unsigned char readPortA(void);
    unsigned char readPortB(void);

    void refreshOutputs(void);

private : 
    virtual void writeI2C(unsigned char addr, unsigned char *data, unsigned char len) = 0;
    virtual void readI2C(unsigned char addr, unsigned char *dest_data, unsigned char len) = 0;

    unsigned char m_mcp_address_8bits;
    unsigned char m_port_A;
    unsigned char m_port_B;
 };

#endif // _MCP23017_BASE_H_
