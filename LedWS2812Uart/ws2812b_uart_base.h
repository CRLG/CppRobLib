#ifndef _LEDS_WS2812B_UART_BASE_H_
#define	_LEDS_WS2812B_UART_BASE_H_

/*
Classe de base virtuelle pure pour les LED RGB WS2812 gérées avec une UART à 8Mhz dont le signal électrique est inversé
L'envoie des données se fait en DMA avec une UART configurée avec un baudrate de 8Mhz / 1 bit de stop / Pas de parité
Le signal électrique doit être inversé (niveau 0V au repos / niveau +5V l'état actif)
C'est la classe fille qui déclare les buffers de données nécessaires en fonction du nombre de LED à piloter qui peut
  être très variable d'une utilisation à l'autre
Ce module consomme de la RAM (24 octets par LED + la structure de donnée) mais ne consomme que très peu de ressource CPU pour le transfert qui se fait en DMA

Exemple de classe fille :
_________________________________________________ Début exemple header
#ifndef _LEDS_WS2812B_H_
#define	_LEDS_WS2812B_H_

#include "ws2812b_uart_base.h"

class WS2812b : public WS2812UartBase
{
public :
    WS2812b();

protected :
    static const unsigned int NB_OF_LEDS = 16;     // 16 LED à contrôler
    static const unsigned int BITS_PER_LED = 24;   // 1 LED = 24 bits (1 octet UART utilisé pour coder 1 bit)
    static const unsigned int LED_DMA_BUFFER_SIZE = (NB_OF_LEDS*BITS_PER_LED);

    unsigned char m_led_ws2812b_dma_buffer[LED_DMA_BUFFER_SIZE];
    tWS2812BPattern m_led_ws2812b[NB_OF_LEDS];

    unsigned int getNumberOfLed();
    unsigned char *getDmaBuffer();
    tWS2812BPattern *getPatternBuffer();
    void send_buffer_dma();

};
#endif	// _LEDS_WS2812B_H_
_________________________________________________  Fin exemple header

_________________________________________________  Début exemple .cpp
#include "ws2812b.h"
#include "RessourcesHardware.h"

WS2812b::WS2812b()
{
}

void WS2812b::send_buffer_dma()
{
    HAL_UART_Transmit_DMA(&huart1, m_led_ws2812b_dma_buffer, LED_DMA_BUFFER_SIZE);
}

unsigned int WS2812b::getNumberOfLed()
{
    return NB_OF_LEDS;
}

unsigned char *WS2812b::getDmaBuffer()
{
    return m_led_ws2812b_dma_buffer;
}

WS2812UartBase::tWS2812BPattern *WS2812b::getPatternBuffer()
{
    return m_led_ws2812b;
}
_________________________________________________  Fin exemple .cpp
*/

class RGBColor {
public :
    static const unsigned long OFF_BLACK   = (0x000000);
    static const unsigned long RED         = (0xFF0000);
    static const unsigned long GREEN       = (0x00FF00);
    static const unsigned long BLUE        = (0x0000FF);
    static const unsigned long PURPLE      = (0x990066);
    static const unsigned long YELLOW      = (0xFF9900);
    static const unsigned long WHITE       = (0xFFFFFF);
    static const unsigned long CHARTREUSE  = (0x7FFF00);
    static const unsigned long TURQUOISE   = (0x40E0D0);
    static const unsigned long OLIVE       = (0x808000);
};

class WS2812UartBase
{
public :
    WS2812UartBase();
    virtual ~WS2812UartBase();

    virtual void init();

    void setState(unsigned short index, unsigned char state);
    void setIntensity(unsigned short index, unsigned char intensity);
    void setColor(unsigned short index, unsigned long rgb, unsigned char intensity=100);
    void configOnOffColor(unsigned short index, unsigned long on_rgb, unsigned long off_rgb, unsigned char intensity=100);
    void setPattern(unsigned short index, unsigned char ton, unsigned char toff);
    void periodicTask();

protected :
    typedef struct {
        unsigned char Toff;             // Durée à l'état OFF
        unsigned char Ton;              // Durée à l'état ON
        unsigned long ColorOff;         // Couleur de l'état OFF
        unsigned long ColorOn;          // Couleur de l'état ON
        unsigned char Intensity;        // Puissance (entre 0 et 100%)
        unsigned char Timer;            // Compteur de temps
        unsigned char CurrentState;     // Mémorise l'état courant OFF ou ON
    }tWS2812BPattern;

    const unsigned char WS2812B_CODE_0 = 0xFC;  // Séquence de 0 et de 1 à 8MHz posté sur l'UART pour représenter un "0" pour la WS2812
    const unsigned char WS2812B_CODE_1 = 0xE0;  // Code pour représenter un "1" pour la WS2812

    // Les buffers sont alloués dans les classes filles en fonction du nombre de LED utilisées sur le projet
    virtual unsigned int getNumberOfLed() = 0;
    virtual unsigned char *getDmaBuffer() = 0;          // à définir dans la classe fille : buffer RAM pour envoi des données en DMA
    virtual tWS2812BPattern *getPatternBuffer() = 0;    // à définir dans la classe fille : buffer de la structure Pattern
    virtual void send_buffer_dma() = 0;  // envoie le buffer de donnée vers l'UART en DMA

private :
    void compute_led_state(unsigned short index);
    void setLED_dma(unsigned short index, unsigned char R, unsigned char G, unsigned char B);
    void setLED_dma(unsigned short index, unsigned long rgb);
};
#endif	/* _LEDS_WS2812B_UART_BASE_H_ */

