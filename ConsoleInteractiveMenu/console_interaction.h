#ifndef _CONSOLE_INTERACTION_H_
#define _CONSOLE_INTERACTION_H_
/*
Cette classe permet de mettre en place un menu interractif composé de pages pour les applications en console.
Elle est indépdendante de la plateforme matérielle sur laquelle est est exécutée.

Il existe 2 types de pages :
    - Les pages "classiques" permettant d'accèder à une autre page du menu ou déclencher une action simple
    - Les pages "saisies" dans lesquelles une valeur est attendue (string, int, double) pour déclencher une action (type changement de valeur d'un paramètre, ...)
      Les pages "saisies" ont les mêmes possiblités que les pages "classiques", à la différence qu'elles attendent la saisie d'une valeur pour déclencher une action spécifique.

Dans chaque page, il est possible :
    - d'attribuer une touche pour changer de page dans le menu
    - d'attribuer une touche pour déclencher une action

Le mécanisme de changement de page est automatisé dans la classe de base ConsoleInteraction.
Les actions sont des fonctions callback appelées (avec ou sans paramètre suivant le cas).
Les prototypes des callbacks doivent impérativement être respectées car la classe utilise la notion de "pointeurs de fonctions".

Il existe 2 types d'actions :
    - Les actions déclenchées par l'appui sur une touche de la page (callback sans paramètre d'entrée)
    - Les actions déclenchées par une valeur attendue (callback avec paramètre d'entrée et paramètre de sortie de type succès/échec)

La classe ConsoleInteraction est virtuelle pure.
Une seule fonction doit être ré-implémentée (send_to_console(char msg[])) pour faire le lien avec le hardware
pour envoyer la chaine de caractère sur le port de communication.

L'utilisateur de la classe doit déclarer ses différentes pages et les actions possibles de chaque page.
Une page est matérialisée par une fonction.
Dans ces fonctions sont déclarées le contenu de la page et actions à effecuter par des macros DECLARE_PAGE, DECLARE_OPTION, DECLARE_ACTION, ...
Ces macros sont présentes pour faciliter et simplifier l'écriture des pages.

Parmi toutes les pages, une sera la page de démarrage.
L'utilisateur doit le préciser dans la classe héritée par la macro DECLARE_START_PAGE

L'utilisateur de la classe instanciée a accès aux méthodes suivantes :
    set_prompt_symbol(char prompt)                      : Change le symbol du prompt
    void enable_echo(bool on_off);                      : Active ou inhibe l'écho (renvoie de chaque caractère reçu)
    void force_start_page_on_char(char car);            : Sélectionne le caractère (=la touche) qui revient à la page de démarrage (par défaut "ESCAPE"). Mettre la valeur numérique 0 pour inhiber la fonction.
    void force_restart_current_page_on_char(char car);  : Sélectionne le caractère (=la touche) qui ré-affiche la page courante (par défaut "TABULATION"). Mettre la valeur numérique 0 pour inhiber la fonction.

Ce qui n'est pas pris en charge :
    - Si dans une page, la même touche est affectée à plusieurs actions, la dernière déclarée sera retenue

===========================================================
Exemple d'utilisation :
  - Menu à 6 pages avec changement de pages
  - 2 pages d'écriture de paramètres
  - Actions de lecture des paramètres
===========================================================
   1. Définition du menu par héritage de la classe ConsoleInteraction
   ------------------------------------------------------------------

class CMenuApp : public ConsoleInteraction
{
public:
    CMenuApp();

    void send_to_console(char msg[]);  // méthode virtuelle pure ré-implémentée

    void page1();
    void page1_1();
    void page1_2();
    void page1_3();
    void page_set_param_1();
    void page_set_param_2();

    bool action_set_param1(double val);
    bool action_read_param1();

    bool action_set_param2(int val);
    bool action_read_param2();

    bool action_read_params();

    double m_param1;
    int m_param2;
};


CMenuApp::CMenuApp()
{
    enable_echo(false);
    DECLARE_START_PAGE(CMenuApp::page1)
}

// ______________________________________________
void CMenuApp::send_to_console(char msg[])
{
    printf("%s", msg);
}

// ============================================================================
//                             LES MENUS
// ============================================================================
void CMenuApp::page1()
{
    DECLARE_PAGE("Menu Page 1", CMenuApp::page1)
    DECLARE_OPTION('a', "Page 1.1", CMenuApp::page1_1)
    DECLARE_OPTION('z', "Page 1.2", CMenuApp::page1_2)
    DECLARE_OPTION('e', "Page 1.3", CMenuApp::page1_3)
}

void CMenuApp::page1_1()
{
    DECLARE_PAGE("Page 1_1", CMenuApp::page1_1)
    DECLARE_OPTION('q', "Retour en page principale", CMenuApp::page1)
    DECLARE_OPTION('r', "Vers page 1_2", CMenuApp::page1_2);
}

void CMenuApp::page1_2()
{
    DECLARE_PAGE("Page 1_2", CMenuApp::page1_2)
    DECLARE_OPTION('s', "Retour en Page1_1", CMenuApp::page1_1)
    DECLARE_OPTION('p', "Changement du parametre Param1", CMenuApp::page_set_param_1)
    DECLARE_OPTION('m', "Changement du parametre Param2", CMenuApp::page_set_param_2)
    DECLARE_ACTION('n', "Lecture Params", CMenuApp::action_read_params)
    DECLARE_OPTION('q', "Retour en page principale", CMenuApp::page1)
}

void CMenuApp::page1_3()
{
    DECLARE_PAGE("Page 1_3", CMenuApp::page1_3)
    DECLARE_OPTION('u', "Retour en Page1_1", CMenuApp::page1_1)
    DECLARE_OPTION('i', "Retour en Page1_2", CMenuApp::page1_2)
    DECLARE_OPTION('q', "Retour au menu principal", CMenuApp::page1)
}

void CMenuApp::page_set_param_1()
{
    DECLARE_PAGE("Forçage paramètre Param1", CMenuApp::page_set_param_1)
    DECLARE_ACTION('r', "Lecture du paramètre", CMenuApp::action_read_param1)
    DECLARE_OPTION('q', "Retour en page principale", CMenuApp::page1)
    DECLARE_ACTION_DOUBLE("Entrez une valeur pour Param1", CMenuApp::action_set_param1)
}

void CMenuApp::page_set_param_2()
{
    DECLARE_PAGE("Forçage paramètre Param2", CMenuApp::page_set_param_2)
    DECLARE_ACTION('r', "Lecture du paramètre", CMenuApp::action_read_param2)
    DECLARE_OPTION('q', "Retour en page principale", CMenuApp::page_set_param_1)
    DECLARE_ACTION_INT("Entrez une valeur pour Param2", CMenuApp::action_set_param2)
}


// ============================================================================
//                             LES ACTIONS
// ============================================================================
bool CMenuApp::action_set_param1(double val)
{
    _printf("Changement de la valeur du paramètre Param1: %f\n\r", val);
    m_param1 = val;
    return true;
}

bool CMenuApp::action_read_param1()
{
    _printf("Valeur du Param1: %f\n\r", m_param1);
    return true;
}

bool CMenuApp::action_set_param2(int val)
{
    _printf("Changement de la valeur du paramètre Param2: %d\n\r", val);
    m_param2 = val;
    return true;
}

bool CMenuApp::action_read_param2()
{
    _printf("Valeur du Param2: %d\n\r", m_param2);
    return true;
}

bool CMenuApp::action_read_params()
{
    _printf("Valeur du Param1: %f\n\r", m_param1);
    _printf("Valeur du Param2: %d\n\r", m_param2);
    return true;
}

   2. Instantiation du menu
   -------------------------
CMenuApp menu;

int main()
{
    menu.start();

    while(1) {
        fflush(stdin);
        char car = fgetc(stdin);
        menu.receive_car(car);
    }

    return 0;
}

*/


/*!
 * \brief The ConsoleInteraction class
 *
 */
class ConsoleInteraction
{
public:
    ConsoleInteraction();
    virtual ~ConsoleInteraction();

    // API
    virtual void receive_car(char car);
    virtual void start();

    void set_prompt_symbol(char prompt);
    void enable_echo(bool on_off);
    void force_start_page_on_char(char car);
    void force_restart_current_page_on_char(char car);

private :
    virtual bool receive_str(char str[]);
    virtual void display_title(const char title[]);
    virtual void display_unexpected_value();
    virtual void display_prompt();
    virtual void display_option(char car, const char msg[]);
    virtual void display_action(const char msg[]);
    virtual void clear_screen();

    virtual void send_to_console(char msg[]) = 0;   // Fonction à implémenter par la classe héritée pour faire le lien avec le hardware (envoie d'une chaine de caractèrevers la console)

protected :
    typedef bool (ConsoleInteraction::* funcPtrExecute)();
    typedef bool (ConsoleInteraction::* funcPtrExecuteString)(char tab[]);
    typedef bool (ConsoleInteraction::* funcPtrExecuteDouble)(double);
    typedef bool (ConsoleInteraction::* funcPtrExecuteInt)(int);
    typedef void (ConsoleInteraction::* funcPtrDisplay)();

private :
    static const int STRING_MAX_SIZE = 64;          // Taille max d'une chaine de caractère
    static const int MAX_ITEM_NB_PER_PAGE = 35;     // Nombre max d'item dans une page

    typedef struct {
        char car;
        funcPtrDisplay page;
        funcPtrExecute action;
    }tCarToPageOrAction;

    typedef struct {
        funcPtrDisplay          display_page;                       // Fonction d'affichage du menu
        int                     item_nb;                            // Nombre d'items (changement de page ou action) dans la page courante
        tCarToPageOrAction      carToPageOrAction[MAX_ITEM_NB_PER_PAGE];  // Correspondance entre touche et un changement de page ou une action
        funcPtrExecuteString    actionFuncString;                   // Callback avec string en entrée sur saisie utilisateur sur cette page
        funcPtrExecuteDouble    actionFuncDouble;                   // Callback avec double en entrée sur saisie utilisateur sur cette page
        funcPtrExecuteInt       actionFuncInt;                      // Callback avec int en entrée sur saisie utilisateur sur cette page
    }tCurrentPage;

    funcPtrDisplay m_start_page;
    tCurrentPage m_current_page;

    char m_prompt;
    bool m_echo;
    char m_start_page_on_char;
    char m_restart_current_page_on_char;

    void goto_page(funcPtrDisplay menu);
    void init_page();

    bool todouble(char str[], double *out_value);
    bool toint(char str[], int *out_value);

protected :
    virtual void _printf(const char *format, ...);
    virtual void display_error(const char msg[]);
    void declare_page(const char title[], funcPtrDisplay fct_display);
    void declare_start_page(funcPtrDisplay fct_display);
    void declare_option(char car, const char msg[], funcPtrDisplay fct_display);
    void declare_action(char car, const char msg[], funcPtrExecute fct_action);
    void declare_action_string(const char msg[], funcPtrExecuteString fct_action);
    void declare_action_double(const char msg[], funcPtrExecuteDouble fct_action);
    void declare_action_int(const char msg[],funcPtrExecuteInt fct_action);

    // Macro d'aide à la déclaration des pages et des actions
    #define DECLARE_PAGE(a, b) declare_page(a, (funcPtrDisplay)&b);
    #define DECLARE_START_PAGE(a) declare_start_page((funcPtrDisplay)&a);
    #define DECLARE_OPTION(a, b, c) declare_option(a, b, (funcPtrDisplay)&c);
    #define DECLARE_ACTION(a, b, c) declare_action(a, b, (funcPtrExecute)&c);
    #define DECLARE_ACTION_STRING(a, b) declare_action_string(a, (funcPtrExecuteString)&b);
    #define DECLARE_ACTION_DOUBLE(a, b) declare_action_double(a, (funcPtrExecuteDouble)&b);
    #define DECLARE_ACTION_INT(a, b) declare_action_int(a, (funcPtrExecuteInt)&b);
};

#endif // _CONSOLE_INTERACTION_H_
