#include "ydlidar_tminiplus_base.h"

YDLIDAR_TminiPlusBase::YDLIDAR_TminiPlusBase()
	: m_cycles_count(0)
{
    init_reconstitution();
}


// ____________________________________________________________
/*! Retourne le nombre de cycles complets reçus
 *
 */
unsigned long YDLIDAR_TminiPlusBase::get_cycles_count()
{
	return m_cycles_count;
}

// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlusBase::init_reconstitution
 */
void YDLIDAR_TminiPlusBase::init_reconstitution()
{
    m_packet_reconstitution_state = RS_HEADER_LSB;
    m_sample_data_index = 0;
    m_packet_error_count = 0;
    m_data_count_in_cycle = 0;
}

// ____________________________________________________________
/*! \brief Reconstitue un paquet à partir d'un buffer d'entrée
 *  \param le buffer de données reçues
 *  \param la longueur du buffer reçu
 */
void YDLIDAR_TminiPlusBase::reconstitution(unsigned char data[], unsigned short len)
{
	for (int i=0; i<len; i++) {
		reconstitution(data[i]);
	}
}


// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlus::reconstitution
 * \param data
 */
void YDLIDAR_TminiPlusBase::reconstitution(unsigned char data)
{
    switch(m_packet_reconstitution_state) {
    // ____________________________
    case RS_HEADER_LSB :
        if (data == 0xAA) m_packet_reconstitution_state++;
        else m_packet_error_count++;
        //m_packet_error_count = 0;
        break;
    case RS_HEADER_MSB :
        if (data == 0x55) m_packet_reconstitution_state++;
        else {
        	m_packet_reconstitution_state = RS_HEADER_LSB;
        	m_packet_error_count++;
        }
        break;
    // ____________________________
    case RS_CT :
        m_packet.package_type = data;
        m_packet_reconstitution_state++;
        break;
    // ____________________________
    case RS_LS :
        m_packet.packet_len = data;
        m_packet_reconstitution_state++;
        break;
    // ____________________________
    case RS_FSA_LSB :
        m_packet.first_sample_angle = data;
        m_packet_reconstitution_state++;
        break;
    case RS_FSA_MSB :
        m_packet.first_sample_angle |= (unsigned short)data << 8;
        m_packet_reconstitution_state++;
        break;
    // ____________________________
    case RS_LSA_LSB :
        m_packet.last_sample_angle = data;
        m_packet_reconstitution_state++;
        break;
    case RS_LSA_MSB :
        m_packet.last_sample_angle |= (unsigned short)data << 8;
        m_packet_reconstitution_state++;
        break;
    // ____________________________
    case RS_CS_LSB :
        m_packet.checksum = data;
        m_packet_reconstitution_state++;
        break;
    case RS_CS_MSB :
        m_packet.checksum |= (unsigned short)data << 8;
        m_packet_reconstitution_state++;
        m_sample_data_index = 0;
        break;
    // ____________________________
    case RS_DATA_i :
        m_packet.data[m_sample_data_index++] = data;
        if (m_sample_data_index >= (3*m_packet.packet_len)) {
            if ( compute_checksum(&m_packet) == m_packet.checksum ) {
                m_data_count_in_cycle += m_packet.packet_len;
                new_packet();

                if (isLastPacketOfCycle(&m_packet))  {
                    new_cycle();
                    m_cycles_count++;
                    m_data_count_in_cycle = 0;
                }
            }
            else {
                m_packet_error_count++;
                packet_error();
            }
            m_packet_reconstitution_state = RS_HEADER_LSB;
        }
        break;
    // ____________________________
    default :
        m_packet_reconstitution_state = RS_HEADER_LSB;
        break;
    }
}

// ____________________________________________________________
/*!
 * \brief Indique si le paquet est le premier d'un cycle
 * \param package_type l'octet CT
 * \return vrai si c'est le premier paquet du cycle. Faux sinon
 */
bool YDLIDAR_TminiPlusBase::isFirstPacket(tPacket *packet)
{
    return ((packet->package_type&0x01));
}

// ____________________________________________________________
/*!
 * \brief Indique si le paquet est le dernier du cycle
 * \param packet le paquet
 * \remark principe : l'angle de fin du paquet est plus petit que l'angle de départ -> on a fait un tour
 * \return vrai si le paquet est le dernier du cycle / faux sinon
 */
bool YDLIDAR_TminiPlusBase::isLastPacketOfCycle(tPacket *packet)
{
    return (packet->last_sample_angle < packet->first_sample_angle);
}

// ____________________________________________________________
//!
//! \brief Retrouve la valeur de l'angle à partir de l'index de donnée
//! \param paquet le paquet de donnée reçu
//! \param data_index l'index de la donnée (0, 1, 2, ...)
//! \return l'angle
//!
float YDLIDAR_TminiPlusBase::dataindex2Angle(tPacket *packet, unsigned short data_index)
{
    float _first_angle = firstAngle(packet);
    float _last_angle = lastAngle(packet);
    if(packet->packet_len<=1) return _first_angle; //protection contre les division par 0
    float angle = diffAngles(_first_angle, _last_angle)*data_index/(packet->packet_len-1) + firstAngle(packet);
    return angle;
}

// ____________________________________________________________
//!
//! \brief Retrouve la valeur de la distance à partir de l'index de donnée
//! \param data_index l'index de la donnée
//! \return la distance en [mm]
//!
unsigned int YDLIDAR_TminiPlusBase::dataindex2Distance(tPacket *packet, unsigned short data_index)
{
    unsigned int __index=3*data_index;
    float distance = ((unsigned short)(packet->data[__index+2]) << 6) | (packet->data[__index+1]>>2);
    return distance;
}

// ____________________________________________________________
/*!
* \brief Indique si la mesure de distance pour la data est valide ou non
* \param data_index l'index de la donnée
* \return true si la donnée est valide. False sinon
*/
bool YDLIDAR_TminiPlusBase::isDistanceValid(tPacket *packet, unsigned short data_index)
{
    unsigned int __index=3*data_index;
    return ((packet->data[__index+1] & 0x03) == 0);
}

// ____________________________________________________________
/*!
 * \brief Calcul le checksum du paquet courant
 * \param paquet le paquet de donnée
 * \return le checksum calculé selon la datasheet
 */
unsigned short YDLIDAR_TminiPlusBase::compute_checksum(tPacket *packet)
{
    unsigned short cs = PACKET_HEADER;
    cs ^= ((unsigned short)(packet->packet_len)<<8) | packet->package_type;
    cs ^= packet->first_sample_angle;
    cs ^= packet->last_sample_angle;

    for (unsigned int i=0; i<packet->packet_len; i++) {
        unsigned int index=3*i;
        cs ^= packet->data[index];
        cs ^= ((unsigned short)(packet->data[index+2]) << 8) | packet->data[index+1];
    }
    return cs;
}

// ____________________________________________________________
float YDLIDAR_TminiPlusBase::firstAngle(tPacket *packet)
{
    return (packet->first_sample_angle>>1)/64.;
}
// ____________________________________________________________
float YDLIDAR_TminiPlusBase::lastAngle(tPacket *packet)
{
    return (packet->last_sample_angle>>1)/64.;
}

// ____________________________________________________________
float YDLIDAR_TminiPlusBase::diffAngles(float first_angle, float last_angle)
{
   if (last_angle < first_angle) last_angle = 360 + last_angle;
   return (last_angle - first_angle);
}

// ____________________________________________________________
/*!
 * \brief
 */
void YDLIDAR_TminiPlusBase::new_packet()
{
    // Comportement par défaut : à ré_implémenter dans la classe fille si besoin
}

// ____________________________________________________________
/*!
 * \brief
 */
void YDLIDAR_TminiPlusBase::packet_error()
{
    // Comportement par défaut : à ré_implémenter dans la classe fille si besoin
}

// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlusBase::new_frame
 */
void YDLIDAR_TminiPlusBase::new_cycle()
{
    // Comportement par défaut : à ré_implémenter dans la classe fille si besoin
}

// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlusBase::start_measures
 */
void YDLIDAR_TminiPlusBase::start_measures()
{
    send_command(0x60);
    init_reconstitution();  // recommence le cycle courant
}

// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlusBase::stop_measures
 */
void YDLIDAR_TminiPlusBase::stop_measures()
{
    send_command(0x65);
    init_reconstitution();  // recommence le cycle courant
}

// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlusBase::stop_measures
 */

void YDLIDAR_TminiPlusBase::restart()
{
    send_command(0x40);
    init_reconstitution();  // recommence le cycle courant
}
// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlusBase::scan_M1
 */
void YDLIDAR_TminiPlusBase::scan_M1()
{
    send_command(0x0C);
    init_reconstitution();  // recommence le cycle courant
}

// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlusBase::scan_P1
 */
void YDLIDAR_TminiPlusBase::scan_P1()
{
    send_command(0x0B);
    init_reconstitution();  // recommence le cycle courant
}

// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlusBase::send_command
 * \param cmd
 */
void YDLIDAR_TminiPlusBase::send_command(unsigned char cmd)
{
    char _buff[2];
    _buff[0] = 0xA5;
    _buff[1]= cmd;
    write_serial(_buff, 2);
}


// ____________________________________________________________
//! TODO : continuer ici
// Le message reçu est composé :
//		- 7 octets pour le header
//		- 20 octets de données utiles
bool YDLIDAR_TminiPlusBase::read_device_information(tDeviceInformation *device_info)
{
	const unsigned char MSG_LEN = 27;
	const unsigned char header[7] = { 0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04};
	char _buff[MSG_LEN];

	send_command(0x90);
	//delay_ms(20); // laisse le temps de répondre
	if (!read_serial(_buff, MSG_LEN)) return false;

	// Vérifie que le header est bien celui attendu pour cette réponse
	for (int i=0; i<7; i++) {
		if (_buff[i] != header[i]) return false;
	}

	// interprète les 20 octets de données utiles
	device_info->model_number = _buff[7];
	device_info->firmware_version_maj = _buff[8];
	device_info->firmware_version_min = _buff[9];
	device_info->hardware_version = _buff[10];
	for (int i=0; i<16; i++) device_info->serial_number[i] = _buff[i+11];
	return true;
}

// ____________________________________________________________
//! TODO : continuer ici
bool YDLIDAR_TminiPlusBase::read_status(tHealthStatus *status)
{
	const unsigned char MSG_LEN = 10;
	const unsigned char header[7] = { 0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06};
	char _buff[MSG_LEN];

	send_command(0x92);
	//delay_ms(20); // laisse le temps de répondre
	if (!read_serial(_buff, MSG_LEN)) return false;

	// Vérifie que le header est bien celui attendu pour cette réponse
	for (int i=0; i<7; i++) {
		if (_buff[i] != header[i]) return false;
	}

	// interprète les 20 octets de données utiles
	status->health_status = _buff[7];
	status->error_code = (unsigned short)_buff[8] | _buff[9];
	return true;
}

// ____________________________________________________________
//! TODO : continuer ici
bool YDLIDAR_TminiPlusBase::read_scan_frequency(tScanFrequency *frequency)
{
	const unsigned char MSG_LEN = 10;
	const unsigned char header[7] = { 0xA5, 0x5A, 0x04, 0x00, 0x00, 0x00, 0x04};
	char _buff[MSG_LEN];

	send_command(0x0D);
	//delay_ms(20); // laisse le temps de répondre
	if (!read_serial(_buff, MSG_LEN)) return false;

	// Vérifie que le header est bien celui attendu pour cette réponse
	for (int i=0; i<7; i++) {
		if (_buff[i] != header[i]) return false;
	}

	// interprète les 20 octets de données utiles
	frequency->scan_frequency = ((unsigned long)_buff[10] << 24) | ((unsigned long)_buff[9] << 16) | ((unsigned long)_buff[8] << 8) | ((unsigned long)_buff[7]);
	return true;
}
