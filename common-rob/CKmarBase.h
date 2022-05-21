#ifndef CKMARBASE_H
#define CKMARBASE_H

class CKmarBase;

// ===========================================================
class CKmarMouvement
{
public :
    CKmarMouvement(CKmarBase *kmar) :
        m_kmar(kmar),
        m_state(MOVEMENT_FINISHED)
    { }
    virtual ~CKmarMouvement() { }

    virtual void start();
    virtual void stop();
    virtual bool isFinished();

    virtual void step()=0;

    static const short MOVEMENT_START = 0;
    static const short MOVEMENT_FINISHED = -1;
protected :
    CKmarBase *m_kmar;
    short           m_state;                // Etat courant
    short           m_next_state;           // Prochain état
    short           m_old_state;            // Etat précédent
    long            m_timeout;              // Donnée interne pour la gestion des timeouts

    static const long  NO_TIMEOUT = -1;
    static const unsigned short REFRESH_PERIOD = 50;   // Période d'appel du Kmar [msec]

    // Méthodes d'aides au changement d'état
    void gotoState(unsigned short next_state);
    void gotoStateAfter(unsigned short next_state, long timeout);
    void gotoStateIfTrue(unsigned short next_state, bool condition, long timeout=NO_TIMEOUT);
    void gotoStateIfConvergence(unsigned short next_state, long timeout=NO_TIMEOUT);
    void gotoStateIfNear(unsigned short next_state, int axis, int target, unsigned char percent, long timeout=NO_TIMEOUT);
    void gotoNextState();
    void gotoFinish();

    bool onEntry();
    bool onExit();
};

// ===========================================================
class CKmarBase
{
public:
    CKmarBase();
    virtual ~CKmarBase();

    typedef enum {
        KMAR_STATUS_OK = 0,
    }tKmarStatus;

    static const int NO_MOUVEMENT = 0;

    virtual bool setAxisPosition(int axis, int pos, int speed=-1)=0;
    virtual bool setAxisSpeed(int axis, int speed)=0;
    virtual int getAxisCount()=0;
    virtual bool isMoving(int axis)=0;
    virtual int getPosition(int axis)=0;
    virtual void arm(int axis)=0;
    virtual void disarm(int axis)=0;
    virtual int getTime()=0;
    virtual void delay_ms(int delay)=0;
    virtual void start(int mouvement)=0;

    virtual void getPosition(int *pos_axis1, int *pos_axis2, int *pos_axis3, int *pos_axis4);
    virtual bool autotest();
    virtual void arm();
    virtual void disarm();
    virtual bool isMoveInProgress();
    virtual bool isFinished();
    virtual void setSpeedFactor(float factor);
    virtual int getNumMouvementInProgress();

    virtual void stop();
    virtual void emergencyStop();
    virtual void compute();

    virtual void catchObject()=0;
    virtual void releaseObject()=0;
    virtual bool isObjectCatched()=0;

    float m_speed_factor;

    CKmarMouvement *m_mouvement_en_cours;
    int             m_num_mouvement_en_cours;
};

#endif // CKMARBASE_H
