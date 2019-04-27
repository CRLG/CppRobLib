#include "stdio.h"
#include "string.h"
#include "message_balise_positions.h"
#include "databasebase.h"
#include "messengerinterfacebase.h"

// ==============================================================
// ==============================================================
Message_BALISE_POSITIONS::Message_BALISE_POSITIONS()
{
    m_id = 0x40;
    m_dlc = 16;
}

// ____________________________________________________________
const char* Message_BALISE_POSITIONS::getName()
{
    return "BALISE_POSITIONS";
}


// ____________________________________________________________
/*! \brief Decode the message data in frame.
 *
 * \param frame : the frame to decode
 */
void Message_BALISE_POSITIONS::decode(const unsigned char *buff_data)
{
#ifdef MESSENGER_FULL
    unsigned char old_PosX_Grosbot = PosX_Grosbot;
    unsigned char old_PosY_Grosbot = PosY_Grosbot;
    unsigned char old_PosX_Minibot = PosX_Minibot;
    unsigned char old_PosY_Minibot = PosY_Minibot;
    unsigned char old_PosX_Adversaire1 = PosX_Adversaire1;
    unsigned char old_PosY_Adversaire1 = PosY_Adversaire1;
    unsigned char old_PosX_Adversaire2 = PosX_Adversaire2;
    unsigned char old_PosY_Adversaire2 = PosY_Adversaire2;
#endif // MESSENGER_FULL

    PosX_Grosbot= ((short)buff_data[0] << 8) + buff_data[1];
    PosY_Grosbot= ((short)buff_data[2] << 8) + buff_data[3];
    PosX_Minibot= ((short)buff_data[4] << 8) + buff_data[5];
    PosY_Minibot= ((short)buff_data[6] << 8) + buff_data[7];

    PosX_Adversaire1= ((short)buff_data[8] << 8) + buff_data[9];
    PosY_Adversaire1= ((short)buff_data[10] << 8) + buff_data[11];

    PosX_Adversaire2= ((short)buff_data[12] << 8) + buff_data[13];
    PosY_Adversaire2= ((short)buff_data[14] << 8) + buff_data[15];

#ifdef MESSENGER_FULL
    if (m_messenger_interface) {
        char name[45];
        char val_str[10];
        sprintf(name, "%s.PosX_Grosbot", getName());
        sprintf(val_str, "%d", PosX_Grosbot);
        m_messenger_interface->dataUpdated(name, val_str);
        if (PosX_Grosbot != old_PosX_Grosbot) m_messenger_interface->dataChanged(name, val_str);

        sprintf(name, "%s.PosY_Grosbot", getName());
        sprintf(val_str, "%d", PosY_Grosbot);
        m_messenger_interface->dataUpdated(name, val_str);
        if (PosY_Grosbot != old_PosY_Grosbot) m_messenger_interface->dataChanged(name, val_str);

        sprintf(name, "%s.PosX_Minibot", getName());
        sprintf(val_str, "%d", PosX_Minibot);
        m_messenger_interface->dataUpdated(name, val_str);
        if (PosX_Minibot != old_PosX_Minibot) m_messenger_interface->dataChanged(name, val_str);

        sprintf(name, "%s.PosY_Minibot", getName());
        sprintf(val_str, "%d", PosY_Minibot);
        m_messenger_interface->dataUpdated(name, val_str);
        if (PosY_Minibot != old_PosY_Minibot) m_messenger_interface->dataChanged(name, val_str);

        sprintf(name, "%s.PosX_Adversaire1", getName());
        sprintf(val_str, "%d", PosX_Adversaire1);
        m_messenger_interface->dataUpdated(name, val_str);
        if (PosX_Adversaire1 != old_PosX_Adversaire1) m_messenger_interface->dataChanged(name, val_str);

        sprintf(name, "%s.PosY_Adversaire1", getName());
        sprintf(val_str, "%d", PosY_Adversaire1);
        m_messenger_interface->dataUpdated(name, val_str);
        if (PosY_Adversaire1 != old_PosY_Adversaire1) m_messenger_interface->dataChanged(name, val_str);

        sprintf(name, "%s.PosX_Adversaire2", getName());
        sprintf(val_str, "%d", PosX_Adversaire2);
        m_messenger_interface->dataUpdated(name, val_str);
        if (PosX_Adversaire2 != old_PosX_Adversaire2) m_messenger_interface->dataChanged(name, val_str);

        sprintf(name, "%s.PosY_Adversaire2", getName());
        sprintf(val_str, "%d", PosY_Adversaire2);
        m_messenger_interface->dataUpdated(name, val_str);
        if (PosY_Adversaire2 != old_PosY_Adversaire2) m_messenger_interface->dataChanged(name, val_str);
    }
#endif // MESSENGER_FULL
    m_updated = true;
}


// ____________________________________________________________
/*! \brief Encode the message data into frame.
 *
 * \param frame : the frame to encode
 */
void Message_BALISE_POSITIONS::encode(unsigned char *buff_data)
{
    buff_data[0] = PosX_Grosbot >> 8;
    buff_data[1] = PosX_Grosbot & 0xFF;

    buff_data[2] = PosY_Grosbot >> 8;
    buff_data[3] = PosY_Grosbot & 0xFF;

    buff_data[4] = PosX_Minibot >> 8;
    buff_data[5] = PosX_Minibot & 0xFF;

    buff_data[6] = PosY_Minibot >> 8;
    buff_data[7] = PosY_Minibot & 0xFF;

    buff_data[8] = PosX_Adversaire1 >> 8;
    buff_data[9] = PosX_Adversaire1 & 0xFF;

    buff_data[10] = PosY_Adversaire1 >> 8;
    buff_data[11] = PosY_Adversaire1 & 0xFF;

    buff_data[12] = PosX_Adversaire2 >> 8;
    buff_data[13] = PosX_Adversaire2 & 0xFF;

    buff_data[14] = PosY_Adversaire2 >> 8;
    buff_data[15] = PosY_Adversaire2 & 0xFF;
}
