#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <cstdlib>

#include "console_interaction.h"

//ctor
ConsoleInteraction::ConsoleInteraction()
{
    init_page();
    m_start_page = nullptr;
    set_prompt_symbol('>');
    enable_echo(true);
    force_restart_current_page_on_char('\t');   //TAB
    force_start_page_on_char(0x1B);             //ESC
}

//dtor
ConsoleInteraction::~ConsoleInteraction()
{
}

// ______________________________________________
/*!
 * \brief Fonction à appeler sur réception de chaque caractère saisie en console
 * \param car le caractère reçu
 * Lorsque la chaine est reçue, elle est passée à receive_str() pour traitement
 */
void ConsoleInteraction::receive_car(char car)
{
    static char str[STRING_MAX_SIZE];
    static int i=0;

    if (m_echo) _printf("%c", car); // renvoie le caractère reçu en echo

    if (car=='\n' || car=='\r') {
        str[i] = '\0';  // fin de chaine
        receive_str(str);
        i=0;
    }
    else {
        if (i<STRING_MAX_SIZE) str[i++] = car;
        else display_error("Internal error : too long line for buffer size");
    }
}

// ______________________________________________
/*!
 * \brief Traitement à effectuer lorsqu'une chaine complète est reçue (après ENTREE)
 * \param str la chaine (sans la touche ENTREE)
 * \return true si le traitement s'est bien passé / false si la chaine n'est pas reconnue pour la page courante
 */
bool ConsoleInteraction::receive_str(char str[])
{
    bool status = false;

    // Cas d'un touche entrée seule -> ne rien faire
    if (strlen(str) == 0) {
        status = true;
    }
    // Cas de la touche ESCAPE : revient au menu de démarrage
    if ( (str[0] == m_start_page_on_char) && (m_start_page_on_char != 0)) {
        goto_page(m_start_page);
        status = true;
    }
    // Cas "tabulation" -> ré-affiche le menu
    if ( (str[0] == m_restart_current_page_on_char) && (m_restart_current_page_on_char!=0)) {
        goto_page(m_current_page.display_page);
        status = true;
    }
    // Cas d'un menu action qui attend une string
    if (m_current_page.actionFuncString) {
        status = ((*this).*m_current_page.actionFuncString)(str);
    }
    // Cas d'un menu action qui attend un double
    if (m_current_page.actionFuncDouble) {
        double val;
        if (todouble(str, &val)) {
            status = ((*this).*m_current_page.actionFuncDouble)(val);
        }
    }
    // Cas d'un menu action qui attend un entier
    if (m_current_page.actionFuncInt) {
        int val;
        if (toint(str, &val)) {
            status = ((*this).*m_current_page.actionFuncInt)(val);
        }
    }

    if (!status) {
        for (int i=0; i<m_current_page.item_nb; i++) {
            if (str[0] == m_current_page.carToPageOrAction[i].car) {
                if (m_current_page.carToPageOrAction[i].page)      goto_page(m_current_page.carToPageOrAction[i].page);               // traite les touches qui changent de page
                if (m_current_page.carToPageOrAction[i].action)    status = ((*this).*m_current_page.carToPageOrAction[i].action)();  // traite les touches qui déclenchent une action immédiate
                display_prompt();
                return true;
            }
        }
    }

    // si on arrive ici avec status=false, c'est que la saisie n'a pas été reconnue valide
    if (!status) display_unexpected_value();
    display_prompt();
    return true;
}

// ______________________________________________
/*!
 * \brief Démarre le menu
 */
void ConsoleInteraction::start()
{
    if (m_start_page)   goto_page(m_start_page);
    else                display_error("Start page is not defined !");
    display_prompt();
}

// ______________________________________________
/*!
 * \brief Affiche le titre de la page
 * \param title le titre à afficher
 */
void ConsoleInteraction::display_title(const char title[])
{
    _printf("*****************************************\n\r");
    _printf("\t%s\n\r", title);
    _printf("*****************************************\n\r");
}

// ______________________________________________
/*!
 * \brief Affiche le message d'erreur de valeur innatendue
 */
void ConsoleInteraction::display_unexpected_value()
{
    _printf("Unexpected value\n\r");
}

// ______________________________________________
/*!
 * \brief Affiche un message d'erreur
 * \param msg le message à afficher
 */
void ConsoleInteraction::display_error(const char msg[])
{
    _printf("Error: %s\n\r", msg);
}

// ______________________________________________
/*!
 * \brief Définit le symbol du prompt
 */
void ConsoleInteraction::display_prompt()
{
    _printf("%c ", m_prompt);
}

// ______________________________________________
/*!
 * \brief Affiche un message de type "la touche x déclenche l'action"
 * \param car le caractère déclenchant l'action
 * \param msg le message à afficher
 */
void ConsoleInteraction::display_option(char car, const char msg[])
{
    _printf("   %c........ %s\n\r", car, msg);
}

// ______________________________________________
/*!
 * \brief Affiche une action dans la page
 * \param msg le message à afficher (ex: "Saisir une valeur décimale pour le coef Kp");
 */
void ConsoleInteraction::display_action(const char msg[])
{
    _printf("   %s\n\r", msg);
}

// ______________________________________________
/*!
 * \brief Envoie la séquence clear screen "cls" par défaut
 * Réimplémentation possible par la classe héritée
 */
void ConsoleInteraction::clear_screen()
{
    _printf("\033[2J");         // clean
    _printf("\033[1;1H");       // cursor top left
}

// ______________________________________________
/*!
 * \brief Ré-implémentation du printf independant de la plateforme
 * \param format paramètre idem à la fonction printf
 */
void ConsoleInteraction::_printf(const char *format, ...)
{
    const int BUFF_SIZE = 256; // à ajuster si les lignes font plus de 256 caractère
    char buff[BUFF_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf (buff, BUFF_SIZE, format, args);
    send_to_console(buff);  // appel de la méthode virtuelle pure d'interaction avec le hardware de la plateforme pour envoyer une chaine sur la console
    va_end(args);
}

// ______________________________________________
/*!
 * \brief Initialise les information de la page courante
 */
void ConsoleInteraction::init_page()
{
    m_current_page.item_nb = 0;
    m_current_page.actionFuncString = nullptr;
    m_current_page.actionFuncDouble = nullptr;
    m_current_page.actionFuncInt = nullptr;
}

// ______________________________________________
/*!
 * \brief Déclare une page du menu
 * \param title le titre de la page
 * \param fct_display la fonction elle même (à mémoriser pour le mécanisme de ré-affichage de la page)
 */
void ConsoleInteraction::declare_page(const char title[], ConsoleInteraction::funcPtrDisplay fct_display)
{
    m_current_page.display_page = fct_display;
    display_title(title);
    init_page();
}
// ______________________________________________
/*!
 * \brief Déclare une page comme étant la page de démarrage
 * \param fct_display la fonction décrivant la page
 */
void ConsoleInteraction::declare_start_page(ConsoleInteraction::funcPtrDisplay fct_display)
{
    m_start_page = fct_display;
}

// ______________________________________________
/*!
 * \brief Déclare un caractère qui déclenche un changement de page du menu
 * \param car le caractère déclenchant l'action
 * \param msg le message à présenter à l'utilisateur dans le menu (Ex: "1.......Menu 1.3.4")
 * \param fct_display la fonction callback à appeler contenant la desciption de la page à pointer si le caractère car est saisit
 */
void ConsoleInteraction::declare_option(char car, const char msg[], ConsoleInteraction::funcPtrDisplay fct_display)
{
    tCarToPageOrAction cartopage;
    cartopage.car = car;
    cartopage.page = fct_display;
    cartopage.action = nullptr;
    m_current_page.carToPageOrAction[m_current_page.item_nb++] = cartopage;

    display_option(car, msg);
}

// ______________________________________________
/*!
 * \brief Déclare une action à exécuter lorsqu'une touche particulière est saisie. Ajoute une ligne dans le menu
 * \param car le caractère déclenchant l'action
 * \param msg le message à présenter à l'utilisateur dans le menu (Ex: "a.......Lire les paramètres")
 * \param fct_action la fonction callback à appeler lorsque le caractère car sera saisit
 */
void ConsoleInteraction::declare_action(char car, const char msg[], funcPtrExecute fct_action)
{
    tCarToPageOrAction cartoaction;
    cartoaction.car = car;
    cartoaction.page = nullptr;
    cartoaction.action = fct_action;
    m_current_page.carToPageOrAction[m_current_page.item_nb++] = cartoaction;

    display_option(car, msg);
}

// ______________________________________________
/*!
 * \brief Déclare une action où la valeur attendue est une chaine de caractère
 * \param msg le message à afficher dans le menu
 * \param fct_action la fonction à appeler avec le paramètre d'entrée string lorsque la valeur chaine est saisie
 */
void ConsoleInteraction::declare_action_string(const char msg[], ConsoleInteraction::funcPtrExecuteString fct_action)
{
    display_action(msg);
    m_current_page.actionFuncString = fct_action;
}

// ______________________________________________
/*!
 * \brief Déclare une action où la valeur attendue est un double
 * \param msg le message à afficher dans le menu
 * \param fct_action la fonction à appeler avec le paramètre d'entrée double lorsque la valeur double est saisie
 */
void ConsoleInteraction::declare_action_double(const char msg[], ConsoleInteraction::funcPtrExecuteDouble fct_action)
{
    display_action(msg);
    m_current_page.actionFuncDouble = fct_action;
}

// ______________________________________________
/*!
 * \brief Déclare une action où la valeur attendue est un entirer
 * \param msg le message à afficher dans le menu
 * \param fct_action la fonction à appeler avec le paramètre d'entrée int lorsque la valeur entière est saisie
 */
void ConsoleInteraction::declare_action_int(const char msg[], ConsoleInteraction::funcPtrExecuteInt fct_action)
{
    display_action(msg);
    m_current_page.actionFuncInt = fct_action;
}

// ______________________________________________
/*!
 * \brief Va au menu pointé
 * \param menu la fonction contenant la description du menu
 */
void ConsoleInteraction::goto_page(funcPtrDisplay menu)
{
    clear_screen();
    if (menu)               ((*this).*menu)();
     // tente d'afficher la page de démarrage
    else if (m_start_page)  ((*this).*m_start_page)();
    else {
        _printf("Page is not defined !\n\r");
    }
}

// ______________________________________________
/*!
 * \brief Vérifie et convertit une chaine en valeur double
 * \param str la chaine d'entrée
 * \param out_value la valeur double retournée si la conversion a été possible
 * \return true si la conversion a été possible / false sinon
 */
bool ConsoleInteraction::todouble(char str[], double *out_value)
{
    if (strlen(str) == 0) return false;
    if (!out_value) return false;
    char *pEnd;
    double val = strtod(str, &pEnd);
    if (*pEnd != '\0') return false;

    *out_value = val;
    return true;
}

// ______________________________________________
/*!
 * \brief Vérifie et convertit une chaine en valeur entière
 * \param str la chaine d'entrée
 * \param out_value la valeur entière retournée si la conversion a été possible
 * \return true si la conversion a été possible / false sinon
 */
bool ConsoleInteraction::toint(char str[], int *out_value)
{
    if (strlen(str) == 0) return false;
    if (!out_value) return false;
    char *pEnd;
    int val = strtol(str, &pEnd, 10);
    if (*pEnd != '\0') return false;

    *out_value = val;
    return true;
}

// ______________________________________________
/*!
 * \brief Change le caractère de prompt
 * \param prompt le caractère de prompt souhaité
 */
void ConsoleInteraction::set_prompt_symbol(char prompt)
{
    m_prompt = prompt;
}

// ______________________________________________
/*!
 * \brief Active ou inhibe l'écho de chaque appui réalisé
 * \param on_off active ou inhibe l'écho
 */
void ConsoleInteraction::enable_echo(bool on_off)
{
    m_echo = on_off;
}

// ______________________________________________
/*!
 * \brief Force l'affichage de la page de démarrage
 * \param car caractère qui déclenche la page de démarrage (valeur spéciale 0 pour désactiver la fonction)
 */
void ConsoleInteraction::force_start_page_on_char(char car)
{
    m_start_page_on_char = car;
}

// ______________________________________________
/*!
 * \brief Force le ré-affichage de la page courante
 * \param car caractère qui déclenche l'action (valeur spéciale 0 pour désactiver la fonction)
 */
void ConsoleInteraction::force_restart_current_page_on_char(char car)
{
    m_restart_current_page_on_char = car;
}
