#ifndef SM_STATEMACHINEBASE_H
#define SM_STATEMACHINEBASE_H

#include "ConfigSpecifiqueCoupe.h"
//#include "CGlobale.h"

class IA;
class SM_DatasInterface;
class SM_OutputsInterface;
class SM_InputsInterface;

class SM_StateMachineBase
{
public:
    SM_StateMachineBase();
    virtual ~SM_StateMachineBase();

    IA *m_ia;

    // API
    virtual void step() = 0;

    virtual const char* getName();
    virtual const char* stateToName(unsigned short state);

    virtual void init(IA *ia);
    virtual void start();
    virtual void stop();
    virtual void interruptForEvitement();
    virtual unsigned short getScore();
    virtual unsigned short getScoreMax();
    virtual unsigned char getPrioriteExecution();
    void setPrioriteExecution(unsigned char priorite);
    virtual bool isEnabled();
    void setEnabled(bool state);

    virtual bool isSucces();

    bool isMainMission();

    int getCurrentState();
    bool isActive();

    // Méthodes d'aides au changement d'état
    void gotoState(unsigned short next_state);
    void gotoStateAfter(unsigned short next_state, long timeout);
    void gotoStateIfTrue(unsigned short next_state, bool condition, long timeout=-1);
    void gotoStateIfConvergence(unsigned short next_state, long timeout=-1);
    void gotoStateIfConvergenceRapide(unsigned short next_state, long timeout=-1);
    void gotoStateIfConvergenceRack(unsigned short next_state, long timeout=-1);

    // Méthodes d'accès simplifiées
    SM_DatasInterface       *internals();
    SM_OutputsInterface     *outputs();
    SM_InputsInterface      *inputs();

    bool onEntry();
    bool onExit();

protected :
    const long  NO_TIMEOUT = -1;

    typedef enum {
        SM_NOT_INITIALIZED = 0,
        SM_START,
        SM_WAITING_FOR_START,
        SM_ACTIVE,
        SM_PAUSE,
        SM_INTERRUPTED,
        SM_STATE_END,
        // _________________
        SM_FIRST_STATE,
    }tStateMachineBaseState;

    bool            m_main_mission_type;    // Indique si la SM est une mission de haut niveau ou une sous-séquence
    bool            m_enabled;              // Permet d'interdire l'exécution d'une mission pour qu'elle ne soit pas exécutée en match
    short           m_state;                // Etat de la machine d'état
    short           m_next_state;           // Prochain état
    short           m_old_state;            // Etat précédent
    bool            m_active;               // Indique si la machine d'état est en cours d'exécution
    bool            m_succes;               // Indique si le job est terminé avec succès et que la SM a terminé ce pour quoi elle était conçue
    long            m_timeout;              // Donnée interne pour la gestion des timeouts
    unsigned short  m_score;                // Calcul des points marqués par la SM
    unsigned short  m_max_score;            // Maximum de point que peut rapporter la SM
    unsigned char   m_priorite_execution;   // Choix d'une priorité d'exécution (utilisé dans le cas d'une mission) : 0=Le plus prioritaire / 255=Le moins prioritaire

    int             m_goback_to_state;      // Etat dans lequel revenir après une interruption (évitement d'obstacle)

    float           m_x_target;             // Coordonnée X du point d'intérêt de la mission
    float           m_y_target;             // Coordonnée Y du point d'intérêt de la mission
};

#endif // SM_STATEMACHINEBASE_H
