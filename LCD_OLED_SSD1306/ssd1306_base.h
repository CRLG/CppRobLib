/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>

   ----------------------------------------------------------------------
    Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#ifndef _SSD1306_BASE_H_
#define _SSD1306_BASE_H_

/**
 * This SSD1306 LCD uses I2C for communication
 *
 * Library features functions for drawing lines, rectangles and circles.
 *
 * It also allows you to draw texts and characters using appropriate functions provided in library.
 *
 * Default pinout
 *
 */
#include "fonts.h"
#include <stdlib.h>
#include <string.h>

class SSD1306Base
{
public :
    SSD1306Base();
    virtual ~SSD1306Base();

    // Méthode virtuelle pure à réimplémenter dans l'applicatif
    virtual bool I2C_Write(uint8_t i2c_addr, uint8_t *pData, uint16_t Size, uint32_t Timeout)=0;
    virtual bool isPresent(uint8_t i2c_addr)=0;

    /**
     * @brief  SSD1306 color enumeration
     */
    typedef enum {
        SSD1306_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
        SSD1306_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
    } SSD1306_COLOR_t;

    /**
     * @brief  Initializes SSD1306 LCD
     * @param  None
     * @retval Initialization status:
     *           - 0: LCD was not detected on I2C port
     *           - > 0: LCD initialized OK and ready to use
     */
    uint8_t Init(uint8_t i2c_addr=SSD1306_I2C_DEFAULT_ADDR);

    /**
     * @brief  Updates buffer from internal RAM to LCD
     * @note   This function must be called each time you do some changes to LCD, to update buffer from RAM to LCD
     * @param  None
     * @retval None
     */
    void UpdateScreen(void);

    /**
     * @brief  Toggles pixels invertion inside internal RAM
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  None
     * @retval None
     */
    void ToggleInvert(void);

    /**
     * @brief  Fills entire LCD with desired color
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  Color: Color to be used for screen fill. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval None
     */
    void Fill(SSD1306_COLOR_t Color);

    /**
     * @brief  Draws pixel at desired location
     * @note   @ref SSD1306_UpdateScreen() must called after that in order to see updated LCD screen
     * @param  x: X location. This parameter can be a value between 0 and SSD1306_WIDTH - 1
     * @param  y: Y location. This parameter can be a value between 0 and SSD1306_HEIGHT - 1
     * @param  color: Color to be used for screen fill. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval None
     */
    void DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color);

    /**
     * @brief  Sets cursor pointer to desired location for strings
     * @param  x: X location. This parameter can be a value between 0 and SSD1306_WIDTH - 1
     * @param  y: Y location. This parameter can be a value between 0 and SSD1306_HEIGHT - 1
     * @retval None
     */
    void GotoXY(uint16_t x, uint16_t y);

    /**
     * @brief  Puts character to internal RAM
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  ch: Character to be written
     * @param  *Font: Pointer to @ref FontDef_t structure with used font
     * @param  color: Color used for drawing. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval Character written
     */
    char Putc(char ch, FontDef_t* Font, SSD1306_COLOR_t color);

    /**
     * @brief  Puts string to internal RAM
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  *str: String to be written
     * @param  *Font: Pointer to @ref FontDef_t structure with used font
     * @param  color: Color used for drawing. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval Zero on success or character value when function failed
     */
    char Puts(char* str, FontDef_t* Font, SSD1306_COLOR_t color);

    /**
     * @brief  Draws line on LCD
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  x0: Line X start point. Valid input is 0 to SSD1306_WIDTH - 1
     * @param  y0: Line Y start point. Valid input is 0 to SSD1306_HEIGHT - 1
     * @param  x1: Line X end point. Valid input is 0 to SSD1306_WIDTH - 1
     * @param  y1: Line Y end point. Valid input is 0 to SSD1306_HEIGHT - 1
     * @param  c: Color to be used. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval None
     */
    void DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c);

    /**
     * @brief  Draws rectangle on LCD
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  x: Top left X start point. Valid input is 0 to SSD1306_WIDTH - 1
     * @param  y: Top left Y start point. Valid input is 0 to SSD1306_HEIGHT - 1
     * @param  w: Rectangle width in units of pixels
     * @param  h: Rectangle height in units of pixels
     * @param  c: Color to be used. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval None
     */
    void DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);

    /**
     * @brief  Draws filled rectangle on LCD
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  x: Top left X start point. Valid input is 0 to SSD1306_WIDTH - 1
     * @param  y: Top left Y start point. Valid input is 0 to SSD1306_HEIGHT - 1
     * @param  w: Rectangle width in units of pixels
     * @param  h: Rectangle height in units of pixels
     * @param  c: Color to be used. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval None
     */
    void DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);

    /**
     * @brief  Draws triangle on LCD
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  x1: First coordinate X location. Valid input is 0 to SSD1306_WIDTH - 1
     * @param  y1: First coordinate Y location. Valid input is 0 to SSD1306_HEIGHT - 1
     * @param  x2: Second coordinate X location. Valid input is 0 to SSD1306_WIDTH - 1
     * @param  y2: Second coordinate Y location. Valid input is 0 to SSD1306_HEIGHT - 1
     * @param  x3: Third coordinate X location. Valid input is 0 to SSD1306_WIDTH - 1
     * @param  y3: Third coordinate Y location. Valid input is 0 to SSD1306_HEIGHT - 1
     * @param  c: Color to be used. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval None
     */
    void DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);

    void DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);

    /**
     * @brief  Draws circle to STM buffer
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  x: X location for center of circle. Valid input is 0 to SSD1306_WIDTH - 1
     * @param  y: Y location for center of circle. Valid input is 0 to SSD1306_HEIGHT - 1
     * @param  r: Circle radius in units of pixels
     * @param  c: Color to be used. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval None
     */
    void DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);

    /**
     * @brief  Draws filled circle to STM buffer
     * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
     * @param  x: X location for center of circle. Valid input is 0 to SSD1306_WIDTH - 1
     * @param  y: Y location for center of circle. Valid input is 0 to SSD1306_HEIGHT - 1
     * @param  r: Circle radius in units of pixels
     * @param  c: Color to be used. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
     * @retval None
     */
    void DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);

    void _ON(void);
    void _OFF(void);

    /**
     * @brief  Writes single byte to slave
     * @param  reg: register to write to
     * @param  data: data to be written
     * @retval None
     */
    void _I2C_Write(uint8_t reg, uint8_t data);

    /**
     * @brief  Writes multi bytes to slave
     * @param  reg: register to write to
     * @param  *data: pointer to data array to write it to slave
     * @param  count: how many bytes will be written
     * @retval None
     */
    void _I2C_WriteMulti(uint8_t reg, uint8_t *data, uint16_t count);

protected :
    static const unsigned int SSD1306_WIDTH = 128;
    static const unsigned int SSD1306_HEIGHT = 64;
    static const unsigned int I2C_TIMEOUT = 20000;
    static const unsigned char SSD1306_I2C_DEFAULT_ADDR = 0x78;

    unsigned char m_i2c_addr;

    /* SSD1306 data buffer */
    uint8_t m_buff[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

    /* Private SSD1306 structure */
    typedef struct {
        uint16_t CurrentX;
        uint16_t CurrentY;
        uint8_t Inverted;
        uint8_t Initialized;
    } SSD1306_t;

    SSD1306_t m_SSD1306;
};

#endif // _SSD1306_BASE_H_
