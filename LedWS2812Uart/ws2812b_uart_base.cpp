#include "ws2812b_uart_base.h"

WS2812UartBase::WS2812UartBase()
{
}

WS2812UartBase::~WS2812UartBase()
{
}

// _______________________________________________
void WS2812UartBase::init()
{
    tWS2812BPattern *leds = getPatternBuffer();
    if (!leds) return;

    for (unsigned int i=0; i<getNumberOfLed(); i++) {
        leds[i].Ton          = 0;
        leds[i].Toff         = 0;
        leds[i].Timer        = 0;
        leds[i].ColorOn      = RGBColor::BLUE;
        leds[i].ColorOff     = RGBColor::OFF_BLACK;
        leds[i].Intensity    = 100;
    }
    periodicTask();
}

// =======================================================
//                      API
// =======================================================
void WS2812UartBase::setState(unsigned short index, unsigned char state)
{
    if (index >= getNumberOfLed()) return;
    tWS2812BPattern *leds = getPatternBuffer();
    if (!leds) return;

    if (state == 0) {   // Force l'etat OFF
        leds[index].Ton  = 0;
        leds[index].Toff = 1;
    }
    else {              // Force l'etat ON
        leds[index].Ton  = 1;
        leds[index].Toff = 0;
    }
}

// _______________________________________________
void WS2812UartBase::setIntensity(unsigned short index, unsigned char intensity)
{
    if (index >= getNumberOfLed()) return;
    tWS2812BPattern *leds = getPatternBuffer();
    if (!leds) return;
    leds[index].Intensity = intensity;
}

// _______________________________________________
void WS2812UartBase::setColor(unsigned short index, unsigned long rgb, unsigned char intensity)
{
    if (index >= getNumberOfLed()) return;
    tWS2812BPattern *leds = getPatternBuffer();
    if (!leds) return;

    leds[index].ColorOn = rgb;
    leds[index].Intensity = intensity;
    setState(index, 1);
}

// _______________________________________________
void WS2812UartBase::configOnOffColor(unsigned short index, unsigned long on_rgb, unsigned long off_rgb, unsigned char intensity)
{
    if (index >= getNumberOfLed()) return;
    tWS2812BPattern *leds = getPatternBuffer();
    if (!leds) return;

    leds[index].ColorOn  = on_rgb;
    leds[index].ColorOff = off_rgb;
    leds[index].Intensity = intensity;
}

// _______________________________________________
void WS2812UartBase::setPattern(unsigned short index, unsigned char ton, unsigned char toff)
{
    if (index >= getNumberOfLed()) return;
    tWS2812BPattern *leds = getPatternBuffer();
    if (!leds) return;

    if ( (leds[index].Ton == ton) && (leds[index].Toff== toff) ) {
        // Ne rien faire
        return;
    }
    leds[index].Ton  = ton;
    leds[index].Toff = toff;
    leds[index].Timer = 0;

}

// =======================================================
//                     DMA BUFFER
// =======================================================
// _______________________________________________
//                 LED0                |                LED1 ...
// [     G;         R;        B]       |[     G;         R;        B]
// [ [G7...G0] [R7....R0] [B7...B0] ]  |[ [G7...G0] [R7....R0] [B7...B0] ]
//     8bits     8bits      8bits      |    8bits     8bits      8bits
void WS2812UartBase::setLED_dma(unsigned short index, unsigned char R, unsigned char G, unsigned char B)
{
    if (index >= getNumberOfLed()) return;
    tWS2812BPattern *leds = getPatternBuffer();
    unsigned char *buff_dma = getDmaBuffer();
    if ((!leds) || (!buff_dma)) return;

    unsigned short i = index * 24;
    int j;

    float div_factor = leds[index].Intensity/100.;
    R = R * div_factor;
    G = G * div_factor;
    B = B * div_factor;

    // Green
    for (j=7; j>=0; j--) {
        buff_dma[i++] = ((G>>j)&0x01)?WS2812B_CODE_1:WS2812B_CODE_0;
    }
    // Red
    for (j=7; j>=0; j--) {
        buff_dma[i++] = ((R>>j)&0x01)?WS2812B_CODE_1:WS2812B_CODE_0;
    }
    // Blue
    for (j=7; j>=0; j--) {
        buff_dma[i++] = ((B>>j)&0x01)?WS2812B_CODE_1:WS2812B_CODE_0;
    }
}

// _______________________________________________
void WS2812UartBase::setLED_dma(unsigned short index, unsigned long rgb)
{
    unsigned char r = (rgb>>16)&0xFF;
    unsigned char g = (rgb>> 8)&0xFF;
    unsigned char b = (rgb>> 0)&0xFF;
    setLED_dma(index, r, g, b);
}


// _______________________________________________
void WS2812UartBase::compute_led_state(unsigned short index)
{
    if (index >= getNumberOfLed()) return;
    tWS2812BPattern *leds = getPatternBuffer();
    if (!leds) return;

    if (leds[index].Ton == 0) {
        leds[index].CurrentState = 0;
        return;
    }
    if (leds[index].Toff == 0) {
        leds[index].CurrentState = 1;
        return;
    }

    leds[index].CurrentState = (leds[index].Timer < leds[index].Ton);

    unsigned short period = leds[index].Ton + leds[index].Toff;
    if (++leds[index].Timer > period) {
        leds[index].Timer = 0;
    }
}

// _______________________________________________
void WS2812UartBase::periodicTask()
{
    tWS2812BPattern *leds = getPatternBuffer();
    if (!leds) return;

    unsigned short i;
    for (i=0; i<getNumberOfLed(); i++) {
       compute_led_state(i);
       unsigned long _color = leds[i].CurrentState==0?leds[i].ColorOff:leds[i].ColorOn;
       // met a jour le buffer DMA avant le transfert
       setLED_dma(i, _color);
    }
    send_buffer_dma();
}

