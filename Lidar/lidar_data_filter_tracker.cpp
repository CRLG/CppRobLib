#include <math.h>
#include "lidar_data_filter_tracker.h"
#include "lidar_data.h"
#include "Lidar_utils.h"

CLidarDataFilterTracker::CLidarDataFilterTracker()
{
    //Paramétrage par défaut
    m_dot_size = 20;
    m_d_dist_offset = 43.6;
    m_i_MAX_BLANK = 3;
    m_i_MIN_COUNT = 2;
    m_i_MAX_COUNT = 60;
    m_d_MAX_dist = 3600.;
    m_d_MIN_dist = 150.;
    m_d_seuil_filtrage_dist = 150.;
    m_i_seuil_filtrage_angle = 3;
    m_d_seuil_gradient = 150.;
    // Reglement : enveloppe convexe du support de balise entre un cercle de 70 mm de diametre et un
    // carre de 100 mm de cote (demi-diagonale 70,7 mm). Les courbes enveloppes sont elargies de part
    // et d'autre : diagonale de 3 cm (R=15 mm) et de 18 cm (R=90 mm).
    m_d_R_mini = 15.;
    m_d_R_maxi = 90.;
}

/*!
  \brief filtre des données brutes du lidar en scannant les données sérialisées
  Le filtre traite les données les unes après les autres sur toute la zone angulaire de détection du Lidar
  Le but du filtre est de découper des nuages de points caractéristiques. Comme on les parcourt les uns après
  les autres, en représentation linéaire, ça formme des sortes de créneaux ==> CREC (CRéneau REception).
  Le filtre va utiliser différents traitements pour former ces CREC et s'affranchir de différents cas de parasitage des données.

  Traitements du filtre:

  * vue à l'infini: naturellement on fixe un horizon de détection (vision à l'infini: on ne distingue plus ce qui est trop loin).
  C'est le cas idéal=rien n'est détecté autour de l'objet détecté et une zone "blanche" se forme autour de lui.
  réglages à faire:
  m_d_MAX_dist qui est l'horizon de l'infini, toute mesure au dealà ne sera pas prise en compte
  m_i_MAX_BLANK qui une sorte de contraste angulaire, la largeur de la zone "blanche" qui permet de distinguer chaque objet

  * problème de masquage:
  Rien ne ressemble plus à une mesure qu'une autre mesure, cependant on peut considérer quand l'écart de mesure
  est significatif et durable qu'on ne mesure plus le même objet
  On peut donc filtrer les forts gradients de distance pour distinguer le fond ou les objets lointains

  * filtrage de la taille de l'objet:
  Le support balise des robots est normé et ne peut donc excéder une certaine taille. De même les CREC trop petits sont des mesures parasites.

  * fusion des objets trop proches:
  Les robots ont une taille minimum donc deux objets détectés dans un rayons inférieur à cette taille minimum sont sûrement le même objet

  * facteur de forme:
  pour filtrer les gros objet apparents donc normalement proches mais pourtant lointains
  il y a une relation entre taille de l'objet et sa distance (obtenu par apprentissage pour un objet comme le support balise) c'est le facteur de forme;
  si le couple taille/distance s'éloigne trop du facteur de forme, il ne correspond pas à une balise
  Le facteur de forme est calcule : D = R / tan(theta/2), avec R la demi-diagonale du mat balise et
  theta sa largeur angulaire apparente. Il est borne des DEUX cotes par les courbes enveloppes
  m_d_R_mini / m_d_R_maxi : une seule borne laissait passer tout objet plus proche que la courbe,
  donc un mur vu de loin. La largeur theta est calculee a partir de la resolution angulaire reelle
  du balayage, et non du nombre d'echantillons : le driver recalcule cette resolution a chaque tour
  (360 / nombre de points), un facteur de forme exprime en echantillons se decalibre des que la
  frequence de rotation ou le nombre de points change.
*/
// _______________________________________________________________
void CLidarDataFilterTracker::filter(const CLidarData *data_in, CLidarData *data_out)
{
    if (!data_in)   return;
    if (!data_out)  return;

    //le balayage de sortie reprend l'entete du balayage d'entree AVANT d'etre rince : sans cela
    //data_out->m_measures_count reste a sa valeur initiale (zero) et le balayage filtre parait vide
    //a tous ses consommateurs, alors qu'il contient des objets. Le filtre "example" ne montrait pas
    //le probleme : il recopie l'integralite du balayage d'entree, entete comprise.
    data_out->m_timestamp = data_in->m_timestamp;
    data_out->m_start_angle = data_in->m_start_angle;
    data_out->m_angle_step_resolution = data_in->m_angle_step_resolution;
    data_out->m_scan_frequency = data_in->m_scan_frequency;
    data_out->m_scale_factor = data_in->m_scale_factor;
    data_out->m_measures_count = (data_in->m_measures_count <= CLidarData::MAX_MEASURES_COUNT) ?
                                  data_in->m_measures_count : CLidarData::MAX_MEASURES_COUNT;

    //on s'assure que les données sont rincées
    for(int i=0;i<data_out->m_measures_count;i++)
        data_out->m_dist_measures[i]=LidarUtils::NO_OBSTACLE;

    m_blobs.clear();

    //Variables internes algo    
    //pour gérer le CREC
    int i_min_CREC=0;
    int i_max_CREC=0;
    bool b_CREC=false;
    bool b_RESET=false;
    //pourgérer le blob de points
    int i_COUNT=0;
    double d_somme=0.;
    //pour détecter les découpes de points
    int i_COUNT_BLANK=0;
    //pour fusionner les blobs trop proches
    double old_dist=0.;
    int old_angle=0;
    bool toMerge=false;
    //pour gérer les forts gradients
    bool b_THRESHOLD=false;
    double d_Samples_Threshold[m_i_MAX_SAMPLES_THRESHOLD];
    for(int j=0;j<m_i_MAX_SAMPLES_THRESHOLD;j++)
        d_Samples_Threshold[j]=0.;
    int i_COUNT_THRESHOLD=0;
    double buffer_average=0.;
    double buffer_std_deviation=0.;

    //ALGO de détection
    for(int i=0;i<data_in->m_measures_count;i++)
    {
        if (i==data_in->m_measures_count-1)
        {
            //Fin du balayage, fermeture automatique du CREC
            b_RESET=true;
        }
        else
        {
            //Données enregistrables
            if((data_in->m_dist_measures[i]>m_d_MIN_dist)&&(data_in->m_dist_measures[i]<m_d_MAX_dist))
            {
                //ENREGISTREMENT PENDANT CREC
                if (b_CREC)
                {
                    i_COUNT_BLANK=0;
                    i_COUNT++;
                    d_somme=d_somme+data_in->m_dist_measures[i];
                }
                else
                {
                    //OUVERTURE DU CREC
                    b_CREC=true;
                    i_min_CREC=i;
                    i_max_CREC=i;
                    i_COUNT=1;
                    d_somme=data_in->m_dist_measures[i];
                }

                //on remplit le buffer
                if(i_COUNT_THRESHOLD<m_i_MAX_SAMPLES_THRESHOLD-1)
                {
                    d_Samples_Threshold[i_COUNT_THRESHOLD]=data_in->m_dist_measures[i];
                    i_COUNT_THRESHOLD++;
                }
                //on peut vérifier le gradient
                else
                {
                    d_Samples_Threshold[i_COUNT_THRESHOLD]=data_in->m_dist_measures[i];
                    //le buffer est rempli on fait le traitement
                    buffer_average=0.;
                    buffer_std_deviation=0.;
                    for(int j=0;j<m_i_MAX_SAMPLES_THRESHOLD;j++)
                    {
                        buffer_average=buffer_average+d_Samples_Threshold[j];
                        buffer_std_deviation=buffer_std_deviation+d_Samples_Threshold[j]*d_Samples_Threshold[j];
                    }
                    buffer_average=buffer_average/m_i_MAX_SAMPLES_THRESHOLD;
                    buffer_std_deviation=sqrt((buffer_std_deviation/m_i_MAX_SAMPLES_THRESHOLD)-(buffer_average*buffer_average));

                    //décalage du buffer
                    for(int j=0;j<m_i_MAX_SAMPLES_THRESHOLD-1;j++)
                        d_Samples_Threshold[j]=d_Samples_Threshold[j+1];

                    //DETECTION FIN CREC: pb de masquage
                    if (buffer_std_deviation > m_d_seuil_gradient)
                    {
                        b_RESET=true;
                        b_THRESHOLD=true;
                    }
                }
            }
            else if (b_CREC)
            {
                //DETECTION FIN CREC: vue à l'infini
                i_COUNT_BLANK++;
                if(i_COUNT_BLANK>=m_i_MAX_BLANK)
                    b_RESET=true;
            }

            if(b_CREC && b_RESET)
            {
                //MARQUAGE FIN CREC
                i_max_CREC=i;

                //MOYENNE DU CREC
                //Moyenne des données enregistrées lors d'un CREC
                //On ne fait la moyenne que s'il y a assez de points, et on ne la fait pas quand il y en a trop
                if((i_COUNT>=m_i_MIN_COUNT) && (i_COUNT<=m_i_MAX_COUNT))
                {
                    //on a détecté un fort gradient, on peut enlever au moins la dernière mesure
                    if(b_THRESHOLD)
                    {
                        d_somme=d_somme-data_in->m_dist_measures[i];
                        i_COUNT--;
                    }
                    double d_moyenne = d_somme/i_COUNT;
                    int i_moyenne = i_max_CREC - ((i_max_CREC-i_min_CREC)/2);


                    if((old_angle>0.) && (old_dist>0.))
                    {
                        //les deux points sont très proches en angle
                        if((fabs(i_moyenne-old_angle)<m_i_seuil_filtrage_angle))
                        {
                            //si ils sont également très proche en distance on fusionne
                            if(fabs(d_moyenne-old_dist)<m_d_seuil_filtrage_dist)
                                toMerge=true;
                            else
                                toMerge=false;

                        }
                        else
                            toMerge=false;
                    }


                    int i_recorded=0;
                    double dist_recorded=0.;
                    //qDebug() << "à fusionner "<<toMerge;
                    if(toMerge)
                    {
                        i_recorded=abs((i_moyenne+old_angle)/2);
                        dist_recorded=fabs((d_moyenne+old_dist)/2);
                    }
                    else
                    {
                        i_recorded=i_moyenne;
                        dist_recorded=d_moyenne;
                    }
                    const double distance_mesuree = dist_recorded + m_d_dist_offset;

                    //vérification du facteur de forme : D = R / tan(theta/2), borne des deux cotés
                    //par les courbes enveloppes. theta est la largeur angulaire réelle du créneau.
                    const double theta_deg = i_COUNT * data_in->m_angle_step_resolution;
                    const double demi_theta_rad = 0.5 * theta_deg * M_PI / 180.;
                    const double tan_demi_theta = tan(demi_theta_rad);
                    bool forme_douteuse = true;
                    if (tan_demi_theta > 0.) {
                        const double distance_mini = m_d_R_mini / tan_demi_theta;
                        const double distance_maxi = m_d_R_maxi / tan_demi_theta;
                        forme_douteuse = (distance_mesuree < distance_mini) || (distance_mesuree > distance_maxi);
                    }

                    //l'objet est publié dans tous les cas, marqué s'il est douteux : un objet est
                    //déclassé, jamais supprimé. Seuls les objets retenus alimentent data_out.
                    m_blobs.append((float)(data_in->m_start_angle + i_recorded*data_in->m_angle_step_resolution),
                                   (float)distance_mesuree, (float)theta_deg, forme_douteuse);

                    if(!forme_douteuse)
                    {
                        data_out->m_dist_measures[i_recorded]=distance_mesuree;

                       // data_out->m_dist_measures[i_moyenne]=d_moyenne-m_d_dist_offset;
                        //mémorisation pour filtrage
                        old_angle=i_moyenne;
                        old_dist=d_moyenne;
                    }
                }

                //FERMETURE ET RESET DU CREC
                b_CREC=false;
                b_RESET=false;
                i_COUNT_BLANK=0;
                i_COUNT=0;
                d_somme=0.;
                i_COUNT_THRESHOLD=0;
                b_THRESHOLD=false;
            }
        }
    }
}

