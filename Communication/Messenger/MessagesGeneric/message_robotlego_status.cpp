#include "stdio.h"
#include "string.h"
#include "message_robotlego_status.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"

// ==============================================================
// ==============================================================
Message_ROBOTLEGO_STATUS::Message_ROBOTLEGO_STATUS()
{
    m_id = 0x60;
    m_dlc = 8;
}

// ____________________________________________________________
const char* Message_ROBOTLEGO_STATUS::getName()
{
    return "ROBOTLEGO_STATUS";
}

// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_ROBOTLEGO_STATUS::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    short old_Status = Status;
    short old_Position_X = Position_X;
    short old_Position_Y = Position_Y;
    short old_Angle = Angle;
#endif // MESSENGER_FULL

    Status= ((short)buff_data[0] << 8) + buff_data[1];
    Position_X= ((short)buff_data[2] << 8) + buff_data[3];
    Position_Y= ((short)buff_data[4] << 8) + buff_data[5];
    Angle= ((short)buff_data[6] << 8) + buff_data[7];

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[30];
        char val_str[10];
        strcpy(name, "Status");
        sprintf(val_str, "%d", Status);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (Status != old_Status) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "Position_X");
        sprintf(val_str, "%d", Position_X);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (Position_X != old_Position_X) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "Position_Y");
        sprintf(val_str, "%d", Position_Y);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (Position_Y != old_Position_Y) m_messenger_interface->dataChanged(this, name, val_str);

        strcpy(name, "Angle");
        sprintf(val_str, "%d", Angle);
        m_messenger_interface->dataUpdated(this, name, val_str);
        if (Angle != old_Angle) m_messenger_interface->dataChanged(this, name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_ROBOTLEGO_STATUS::encode(unsigned char *buff_data)
{
    buff_data[0] = Status >> 8;
    buff_data[1] = Status & 0xFF;

    buff_data[2] = Position_X >> 8;
    buff_data[3] = Position_X & 0xFF;

    buff_data[4] = Position_Y >> 8;
    buff_data[5] = Position_Y & 0xFF;

    buff_data[6] = Angle >> 8;
    buff_data[7] = Angle & 0xFF;

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[30];
        char val_str[10];
        strcpy(name, "Status");
        sprintf(val_str, "%d", Status);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "Position_X");
        sprintf(val_str, "%d", Position_X);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "Position_Y");
        sprintf(val_str, "%d", Position_Y);
        m_messenger_interface->dataSent(this, name, val_str);

        strcpy(name, "Angle");
        sprintf(val_str, "%d", Angle);
        m_messenger_interface->dataSent(this, name, val_str);
    }
#endif // MESSENGER_FULL
}
