#include "servoaxbase.h"

ServoAXBase::ServoAXBase()
    : m_rx_err_count(0),
      m_tx_err_count(0)
{
}

ServoAXBase::~ServoAXBase()
{
}

// ____________________________________________________________
bool ServoAXBase::isPresent(unsigned char id)
{
    unsigned char val;
    tAxErr err = read8bitsRegister(id, AX_REG_VERSION, &val);
    return (err == AX_OK);
}

// ____________________________________________________________
tAxErr ServoAXBase::setPosition(unsigned char id, unsigned short position)
{
    return write16bitsRegister(id, AX_REG_GOAL_POSITION, position);
}

// ____________________________________________________________
tAxErr ServoAXBase::setSpeed(unsigned char id, unsigned short speed)
{
    return write16bitsRegister(id, AX_REG_GOAL_SPEED, speed);
}

// ____________________________________________________________
tAxErr ServoAXBase::setPositionSpeed(unsigned char id, unsigned short position, unsigned short speed)
{
    unsigned char data[5];
    data[0] = AX_REG_GOAL_POSITION;
    data[1] = position;
    data[2] = (position>>8);
    data[3] = speed;
    data[4] = (speed>>8);
    return sendWriteDataPacket(id, data, 5);
}

// ____________________________________________________________
unsigned short ServoAXBase::getPosition(unsigned char id, tAxErr *err_status)
{
    unsigned short position=0;
    tAxErr err;
    err = read16bitsRegister(id, AX_REG_PRESENT_POSITION, &position);
    if (err_status != nullptr) *err_status = err;
    return position;
}

// ____________________________________________________________
bool ServoAXBase::isMoving(unsigned char id, tAxErr *err_status)
{
    unsigned char moving=0;
    tAxErr err;
    err = read8bitsRegister(id, AX_REG_MOVING, &moving);
    if (err_status != nullptr) *err_status = err;
    return moving;
}

// ____________________________________________________________
tAxErr ServoAXBase::changeID(unsigned char old_id, unsigned char new_id)
{
    tAxErr err;
    err = write8bitsRegister(old_id, AX_REG_ID, new_id);
    if (err != AX_OK) return AX_NOT_FOUND;

    return AX_OK;
}

// ____________________________________________________________
tAxErr ServoAXBase::enableTorque(unsigned char id, bool on_off)
{
    return write8bitsRegister(id, AX_REG_TORQUE_ENABLE, on_off);
}

// ____________________________________________________________
tAxErr ServoAXBase::setLimitPositionMin(unsigned char id, unsigned short pos)
{
    return write16bitsRegister(id, AX_REG_CW_ANGLE_LIMIT, pos);
}

// ____________________________________________________________
tAxErr ServoAXBase::setLimitPositionMax(unsigned char id, unsigned short pos)
{
    return write16bitsRegister(id, AX_REG_CCW_ANGLE_LIMIT, pos);
}

// ____________________________________________________________
tAxErr ServoAXBase::setMode(unsigned char id, unsigned char mode)
{
    tAxErr err;
    unsigned short cw_limit, ccw_limit;

    switch (mode)
    {
        case AX_MODE_ROTATIONAL:
            cw_limit = 0;
            ccw_limit = 0x3FF;
        break;

        case AX_MODE_FREE_RUNNING:
            cw_limit = 0;
            ccw_limit = 0;
        break;

        default :
            return AX_INVALID_VALUE;
    }
    err = setSpeed(id, 0);  // ensure motor won't move after mode is changing
    if (err != AX_OK) return err;
    err = setLimitPositionMin(id, cw_limit);
    if (err != AX_OK) return err;
    err = setLimitPositionMax(id, ccw_limit);
    if (err != AX_OK) return err;
    return AX_OK;
}

// ____________________________________________________________
/*! \brief Switch on/Off the LED.
 *
 * \param id : the identifer of the servo
 * \param state : the state of the led
 * \return error code
 */
tAxErr ServoAXBase::setLed(unsigned char id, bool state)
{
    return write8bitsRegister(id, AX_REG_LED, state);
}

// ____________________________________________________________
/*! \brief Write a value in a 16 bits register.
 *
 * \param id : the identifer of the servo
 * \param reg_addr : the start address of the register
 * \param value : the value to write
 * \return error code
 */
tAxErr ServoAXBase::write16bitsRegister(unsigned char id, unsigned char reg_addr, unsigned short value)
{
    unsigned char data[3];
    data[0] = reg_addr;
    data[1] = value;
    data[2] = (value>>8);
    return sendWriteDataPacket(id, data, 3);
}

// ____________________________________________________________
/*! \brief Read the value in a 8 bits register.
 *
 * \param id : the identifer of the servo
 * \param reg_addr : the start address of the register
 * \param value : the return value read in the register
 * \return error code
 */
tAxErr ServoAXBase::_read8bitsRegister(unsigned char id, unsigned char reg_addr, unsigned char *value, unsigned char *err_status)
{
    tAxErr err;

    err = sendReadDataRequestPacket(id, reg_addr, 1);
    if (err != AX_OK) return err;

    err = read(m_packet, 7);
    if (err != AX_OK) return err;

    if (!isPacketValid(m_packet))
        return AX_INVALID_PACKET;

    if (err_status) *err_status = m_packet[4];
    if (value) *value = m_packet[5];

    return AX_OK;
}

// ____________________________________________________________
/*! \brief Read the value in a 8 bits register with retry if error.
 *
 * \param id : the identifer of the servo
 * \param reg_addr : the start address of the register
 * \param value : the return value read in the register
 * \return error code
 */
tAxErr ServoAXBase::read8bitsRegister(unsigned char id, unsigned char reg_addr, unsigned char *value, unsigned char *err_status)
{
    tAxErr err;
    unsigned char attempt=0;

    do {
        err = _read8bitsRegister(id, reg_addr, value, err_status);
        attempt++;
    }while ( (err != AX_OK) && (attempt < AX_RETRY_ON_ERROR) );

    if (err != AX_OK) m_rx_err_count++;
    return err;
}

// ____________________________________________________________
/*! \brief Read the value in a 16 bits register.
 *
 * \param id : the identifer of the servo
 * \param reg_addr : the start address of the register
 * \param value : the return value read in the register
 * \return error code
 */
tAxErr ServoAXBase::_read16bitsRegister(unsigned char id, unsigned char reg_addr, unsigned short *value, unsigned char *err_status)
{
    tAxErr err;

    err = sendReadDataRequestPacket(id, reg_addr, 2);
    if (err != AX_OK) return err;

    err = read(m_packet, 8);
    if (err != AX_OK) return err;

    if (!isPacketValid(m_packet)) return AX_INVALID_PACKET;

    if (err_status) *err_status = m_packet[4];
    if (value) *value = ((unsigned short)m_packet[6]<<8) + m_packet[5];

    return AX_OK;
}

// ____________________________________________________________
/*! \brief Read the value in a 16 bits register with retry if error.
 *
 * \param id : the identifer of the servo
 * \param reg_addr : the start address of the register
 * \param value : the return value read in the register
 * \return error code
 */
tAxErr ServoAXBase::read16bitsRegister(unsigned char id, unsigned char reg_addr, unsigned short *value, unsigned char *err_status)
{
    tAxErr err;
    unsigned char attempt=0;

    do {
        err = _read16bitsRegister(id, reg_addr, value, err_status);
        attempt++;
    }while ( (err != AX_OK) && (attempt < AX_RETRY_ON_ERROR) );

    if (err != AX_OK) m_rx_err_count++;
    return err;
}
// ____________________________________________________________
/*! \brief Write a value in a 8 bits register.
 *
 * \param id : the identifer of the servo
 * \param reg_addr : the start address of the register
 * \param value : the value to write
 * \return error code
 */
tAxErr ServoAXBase::write8bitsRegister(unsigned char id, unsigned char reg_addr, unsigned char value)
{
    unsigned char data[2];
    data[0] = reg_addr;
    data[1] = value;
    return sendWriteDataPacket(id, data, 2);
}

// ____________________________________________________________
/*! \brief Create and send a Write Data packet.
 *
 * \param id : the identifer of the servo
 * \param parameters : the array of useful parameters values
 * \param size : the number of parameters
// * \param packet : the destination buffer
 * \return error code
 * \remarks : the buffer is supposed to be allocated to receive all data
 *      Instruction Packet : 0XFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 ...PARAMETER_N CHECKSUM
 */
tAxErr ServoAXBase::sendWriteDataPacket(unsigned char id, unsigned char *parameters, unsigned char size, bool send_packet)
{
    m_packet[0] = 0xFF;
    m_packet[1] = 0xFF;
    m_packet[2] = id;
    m_packet[3] = size + 2;
    m_packet[4] = AX_INSTRUCTION_WRITE_DATA;
    unsigned int i;
    for (i=0; i<size; i++) {
        m_packet[i+5] = parameters[i];
    }
    m_packet[i+5] = getChecksum(m_packet);

    if (send_packet) {
        return _write(m_packet, i+6);
    }
    return AX_OK;
}

// ____________________________________________________________
/*! \brief Create a Read Data packet.
 *
 * \param id : the identifer of the servo
 * \param start_addr : the first address to read
 * \param length: the number of bytes to read
 * \return error code
 * \remarks :
 *      Read Packet : 0xFF 0xFF ID 0x04 0x02 start_addr length CHECKSUM
 */
tAxErr ServoAXBase::sendReadDataRequestPacket(unsigned char id, unsigned char start_addr, unsigned char length)
{
    m_packet[0] = 0xFF;
    m_packet[1] = 0xFF;
    m_packet[2] = id;
    m_packet[3] = 4; // packet length
    m_packet[4] = AX_INSTRUCTION_READ_DATA;
    m_packet[5] = start_addr;
    m_packet[6] = length;
    m_packet[7] = getChecksum(m_packet);

    return _write(m_packet, 8);
}

// ____________________________________________________________
/*! \brief Write packet.
 *
 * \param buff_data: the buffer data to be written
 * \param size: the size of the buffer
 * \return error code
 */
tAxErr ServoAXBase::_write(unsigned char *buff_data, unsigned char size)
{
    tAxErr err = AX_OK;
    setTxEnable(true);
    delay_us(1);
    err = write(buff_data, size);
    waitTransmitComplete();  // ensure transmission is complete before enable line is reset
    delay_us(1);
    setTxEnable(false);
    flushSerialInput();      // ensure no data is pending in uart input buffer (in case of a write start a read request)

    if (err != AX_OK) m_tx_err_count++;
    return err;
}

// ____________________________________________________________
/*! \brief Compute checksum from the packet.
 *
 * \param packet : the packet to compute
 * \return the checksum
 * \remarks :
 *      Instruction Packet : 0XFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 ...PARAMETER_N CHECKSUM
 *          Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N) for write packet
 *          Check Sum = ~ (ID + Length + Error + Parameter1 + ... Parameter N) for status packet
 */
unsigned char ServoAXBase::getChecksum(unsigned char *packet)
{
    unsigned char sum=0;
    for (int i=2; i<=2+packet[3]; i++) {
        sum+= packet[i];
    }
    return ~sum;
}

// ____________________________________________________________
/*! \brief Check if packet is valid.
 *   A valid packet is :
 *      first two bytes are 0xff
 *      checksum is OK
 *
 * \param packet : the packet to compute
 * \return the checksum
 */
//!
bool ServoAXBase::isPacketValid(unsigned char *packet)
{
    if (packet[0] != 0xFF) return false;
    if (packet[1] != 0xFF) return false;
    unsigned char cs_index = 3 + packet[3];
    if (getChecksum(packet) != packet[cs_index]) return false;
    return true;
}
