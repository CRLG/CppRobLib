#include "mcp23017base.h"

MCP23017Base::MCP23017Base()
    : m_mcp_address_8bits(0X40),
      m_port_A(0),
      m_port_B(0)
{
}

MCP23017Base::~MCP23017Base()
{
}

// ____________________________________________________________
void MCP23017Base::setAddress(unsigned char address_8bits)
{
  m_mcp_address_8bits = address_8bits;
}


// ____________________________________________________________
unsigned char MCP23017Base::readRegister(unsigned char reg_addr)
{
    unsigned char buff[1];
    buff[0] = reg_addr;
    writeI2C(m_mcp_address_8bits, buff, 1);
    readI2C(m_mcp_address_8bits, buff, 1);
    return (buff[0]);
}

// ____________________________________________________________
void MCP23017Base::writeRegister(unsigned char reg_addr, unsigned char data)
{
    unsigned char buff[2];
    buff[0] = reg_addr;
    buff[1] = data;
    writeI2C(m_mcp_address_8bits, buff, 2);
}

// ____________________________________________________________
void MCP23017Base::configDirectionPortA(unsigned char direction)
{
    writeRegister(MCP23017_IODIRA, direction);
}

// ____________________________________________________________
void MCP23017Base::configDirectionPortB(unsigned char direction)
{
    writeRegister(MCP23017_IODIRB, direction);
}

// ____________________________________________________________
void MCP23017Base::configDirections(unsigned char directionA, unsigned char directionB)
{
    configDirectionPortA(directionA);
    configDirectionPortB(directionB);
}

// ____________________________________________________________
void MCP23017Base::configPullUpPortA(unsigned char pullup)
{
    writeRegister(MCP23017_GPPUA, pullup);
}

// ____________________________________________________________
void MCP23017Base::configPullUpPortB(unsigned char pullup)
{
    writeRegister(MCP23017_GPPUB, pullup);
}

// ____________________________________________________________
void MCP23017Base::configPullUp(unsigned char pullupA, unsigned char pullupB)
{
    configPullUpPortA(pullupA);
    configPullUpPortB(pullupB);
}

// ____________________________________________________________
void MCP23017Base::writePortA(unsigned char port_val)
{
    m_port_A = port_val;
    writeRegister(MCP23017_OLATA, port_val);
}

// ____________________________________________________________
void MCP23017Base::writePortB(unsigned char port_val)
{
    m_port_B = port_val;
    writeRegister(MCP23017_OLATB, port_val);
}

// ____________________________________________________________
void MCP23017Base::writePorts(unsigned char portA, unsigned char portB)
{
    writePortA(portA);
    writePortB(portB);
}

// ____________________________________________________________
void MCP23017Base::writeBitPortA(unsigned char bit_pos, bool val)
{
    if (val) {
        m_port_A |= (1<<bit_pos);
    }
    else {
        m_port_A &= ~(1<<bit_pos);
    }
    writePortA(m_port_A);
}

// ____________________________________________________________
void MCP23017Base::writeBitPortB(unsigned char bit_pos, bool val)
{
    if (val) {
        m_port_B |= (1<<bit_pos);
    }
    else {
        m_port_B &= ~(1<<bit_pos);
    }
    writePortB(m_port_B);
}


// ____________________________________________________________
unsigned char MCP23017Base::readPortA(void)
{
    return readRegister(MCP23017_GPIOA);
}

// ____________________________________________________________
unsigned char MCP23017Base::readPortB(void)
{
    return readRegister(MCP23017_GPIOB);
}


