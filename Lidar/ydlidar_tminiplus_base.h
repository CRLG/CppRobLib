#ifndef _YDLIDAR_TMINIPLUS_BASE_H_
#define _YDLIDAR_TMINIPLUS_BASE_H_


class YDLIDAR_TminiPlusBase
{
public:
    YDLIDAR_TminiPlusBase();

    void reconstitution(unsigned char data);

    virtual void start_measures();
    virtual void stop_measures();
    virtual void scan_M1();
    virtual void scan_P1();

private :

    void init_reconstitution();

    static const unsigned int MAX_DATA_PER_PACKET = 40;
    static const unsigned int MAX_DATA_BYTE_PER_PACKET = (3 * MAX_DATA_PER_PACKET);
    static const unsigned short PACKET_HEADER = 0x55AA;
    typedef struct {
        unsigned char package_type;
        unsigned char packet_len;
        unsigned short first_sample_angle;
        unsigned short last_sample_angle;
        unsigned short checksum;
        unsigned char data[MAX_DATA_BYTE_PER_PACKET];
    }tPacket;

    unsigned char m_packet_reconstitution_state;
    unsigned int m_sample_data_index;
    unsigned int m_packet_error_count;

    typedef enum {
        RS_HEADER_LSB = 0,
        RS_HEADER_MSB,
        RS_CT,
        RS_LS,
        RS_FSA_LSB,
        RS_FSA_MSB,
        RS_LSA_LSB,
        RS_LSA_MSB,
        RS_CS_LSB,
        RS_CS_MSB,
        RS_DATA_i
    }tReconstitutionState;

protected :
    tPacket m_packet;
    long m_data_count_in_cycle;

    bool isFirstPacket(tPacket *packet);
    bool isLastPacketOfCycle(tPacket *packet);
    float dataindex2Angle(tPacket *packet, unsigned short data_index);
    unsigned int dataindex2Distance(tPacket *packet, unsigned short data_index);
    bool isDistanceValid(tPacket *packet, unsigned short data_index);

    // opérations sur le paquet reçu
    unsigned short compute_checksum(tPacket *packet);
    float firstAngle(tPacket *packet);
    float lastAngle(tPacket *packet);
    float diffAngles(float first_angle, float last_angle);

    virtual void new_packet();
    virtual void packet_error();
    virtual void new_cycle();

    virtual void send_command(unsigned char cmd);
    virtual void write_serial(const char buff[], unsigned long len) = 0;

};

#endif // _YDLIDAR_TMINIPLUS_BASE_H_
