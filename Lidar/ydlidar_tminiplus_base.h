#ifndef _YDLIDAR_TMINIPLUS_BASE_H_
#define _YDLIDAR_TMINIPLUS_BASE_H_


class YDLIDAR_TminiPlusBase
{
public:
    YDLIDAR_TminiPlusBase();

    void reconstitution(unsigned char data);
    void reconstitution(unsigned char data[], unsigned short len);
    unsigned long get_cycles_count();

    typedef struct {
    	unsigned char model_number;
    	unsigned char firmware_version_maj;
    	unsigned char firmware_version_min;
    	unsigned char hardware_version;
    	unsigned char serial_number[16];
    }tDeviceInformation;

    typedef struct {
    	unsigned char health_status; // TODO : décortiquer chaque bit
    	unsigned short error_code;
    }tHealthStatus;

    typedef struct {
    	unsigned long scan_frequency;	// *100 Hz
    }tScanFrequency;

    virtual void start_measures();
    virtual void stop_measures();
    virtual void restart();
    virtual void scan_M1();
    virtual void scan_P1();
    virtual bool read_device_information(tDeviceInformation *device_info);
    virtual bool read_status(tHealthStatus *status);
    virtual bool read_scan_frequency(tScanFrequency *frequency);

private :
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
    unsigned long m_cycles_count; // nombre total de cycles complets reçus

    bool m_continus_response_mode;

    void init_reconstitution();
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
    virtual bool write_serial(const char buff[], unsigned long len) = 0;
    virtual bool read_serial(const char buff[], unsigned long len) = 0;
};

#endif // _YDLIDAR_TMINIPLUS_BASE_H_
