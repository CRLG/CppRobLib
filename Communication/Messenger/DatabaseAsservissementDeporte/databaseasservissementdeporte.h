#ifndef _DATABASE_ASSERVISSEMENT_DEPORTE_H_
#define _DATABASE_ASSERVISSEMENT_DEPORTE_H_

#include "message_etat_asservissement.h"
#include "message_commande_mouvement_xy.h"
#include "message_commande_mouvement_xy_teta.h"
#include "message_commande_mouvement_distance_angle.h"
#include "message_commande_mouvement_xy_a.h"
#include "message_commande_mouvement_xy_b.h"
#include "message_commande_vitesse_mouvement.h"
#include "message_reinit_position_xy_teta.h"
#include "message_commande_indice_sportivite.h"

#include "node_asserdep_grosbot.h"
#include "node_asserdep_asserv_deporte.h"

#include "databasebase.h"

// ====================================================
//        DATABASE
// ====================================================
class DatabaseAsservissementDeporte : public DatabaseBase
{
public:
    DatabaseAsservissementDeporte();
    ~DatabaseAsservissementDeporte();

    virtual const char *getName();
    virtual void getVersion(unsigned char *maj, unsigned char *min);
    virtual unsigned short getMessageCount();
    virtual unsigned short getNodeCount();

    static const unsigned short NODES_COUNT = 5;
    NodeBase *m_nodes_list[NODES_COUNT];

    NodeAsserdepGrosbot         m_node_grosbot;
    NodeAsserdepAsservissement  m_node_asserv_deporte;

    static const unsigned short MESSAGES_COUNT = 9;
    MessageBase *m_messages_list[MESSAGES_COUNT];

    Message_ETAT_ASSERVISSEMENT                 m_EtatAsservissement;
    Message_COMMANDE_MOUVEMENT_XY               m_CommandeMouvementXY;
    Message_COMMANDE_MOUVEMENT_XY_TETA          m_CommandeMouvementXYTeta;
    Message_COMMANDE_MOUVEMENT_DISTANCE_ANGLE   m_CommandeMouvementDistanceAngle;
    Message_COMMANDE_MOUVEMENT_XY_A             m_CommandeMouvementXY_A;
    Message_COMMANDE_MOUVEMENT_XY_B             m_CommandeMouvementXY_B;
    Message_COMMANDE_VITESSE_MOUVEMENT          m_CommandeVitesseMouvement;
    Message_REINIT_POSITION_XY_TETA             m_ReinitPositionXYTeta;
    Message_COMMANDE_INDICE_SPORTIVITE          m_CommandeIndiceSportivite;

    unsigned short m_msg_id;
private :
    void initMessages();
    void initNodes();
};

#endif // _DATABASE_ASSERVISSEMENT_DEPORTE_H_
