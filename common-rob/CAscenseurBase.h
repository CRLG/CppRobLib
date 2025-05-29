#ifndef _ASCENSEUR_BASE_H_
#define _ASCENSEUR_BASE_H_


// ====================================================
//
// ====================================================
class CAscenseurBase
{
public:
    CAscenseurBase();
    virtual ~CAscenseurBase();

    typedef enum {
        POSITION_UNKNOWN = 0,
        POSITION_HIGH,
        POSITION_LOW
    }tPositionAscenseur;

    typedef enum {
        TARGET_STOP,
        TARGET_UP,
        TARGET_DOWN
    }tTargetAscenseur;

    virtual void command_motor(signed char consigne_pourcent) = 0;  // consigne_pource positif pour le sens monter / négatif pour le sens descente
    virtual bool is_sensor_high() = 0;  // capteur de butée haute
    virtual bool is_sensor_low() = 0;   // capteur de butée basse

    virtual void periodicCall();

    void up();
    void down();
    void set_position(unsigned char target);
    void stop();
    unsigned int get_position();
    void set_speeds(int speed_up, int speed_down);

protected :
    unsigned char m_target;
    int m_speed_up;     // Vitesse en montée [%]
    int m_speed_down;   // Vitesse en descente [%]
};

#endif // _ASCENSEUR_BASE_H_
