/*! \file CDataEncoderDecoder.cpp
*/
#include "data_encoder_decoder.h"

//___________________________________________________________________________
 /*!
   \brief Encode une donnée 1 bit dans le buffer
   \param buff : le buffer de destination
   \param byte_position : numéro d'octet dans le buffer (0 = 1er octet du buffer)
   \param bit_position : numéro du bit dans l'octet (0 à 7)
   \param data : la valeur à encoder (booléen)
   \return --
*/
void CDataEncoderDecoder::encode_bit(unsigned char *buff, unsigned char byte_position, unsigned char bit_position, bool data)
{
    if (bit_position > 7) return;  // protection
    if (data)   buff[byte_position] |= (1 << bit_position);
    else        buff[byte_position] &= ~(1 << bit_position);
}

//___________________________________________________________________________
 /*!
   \brief Encode une donnée 8 bits dans le buffer
   \param buff : le buffer de destination
   \param position : position de la data dans le buffer
   \param data : la valeur à encoder
   \return --
*/
void CDataEncoderDecoder::encode_int8(unsigned char *buff, unsigned char position, unsigned char data)
{
    buff[position] = data;
}

//___________________________________________________________________________
 /*!
   \brief Encode une donnée 16 bits dans le buffer (MSB first)
   \param buff : le buffer de destination
   \param position : position de la data dans le buffer
   \param data : la valeur à encoder
   \return --
*/
void CDataEncoderDecoder::encode_int16(unsigned char *buff, unsigned char position, unsigned short data)
{
    buff[position]      = (data >> 8)&0xFF;
    buff[position+1]    = (data&0xFF);
}
//___________________________________________________________________________
 /*!
   \brief Encode une donnée 32 bits dans le buffer (MSB first)
   \param buff : le buffer de destination
   \param position : position de la data dans le buffer
   \param data : la valeur à encoder
   \return --
*/
void CDataEncoderDecoder::encode_int32(unsigned char *buff, unsigned char position, unsigned long data)
{
    buff[position]      = (data >> 24)&0xFF;
    buff[position+1]    = (data >> 16)&0xFF;
    buff[position+2]    = (data >> 8)&0xFF;
    buff[position+3]    = (data&0xFF);
}

//___________________________________________________________________________
 /*!
   \brief Décode une donnée 1 bit du buffer
   \param buff : le buffer source
   \param byte_position : numéro d'octet dans le buffer (0 = 1er octet du buffer)
   \param bit_position : numéro du bit dans l'octet (0 à 7)
   \return la data sur 1 bit
*/
bool CDataEncoderDecoder::decode_bit(unsigned char *buff, unsigned char byte_position, unsigned char bit_position)
{
    if (bit_position > 7) return false;  // protection
    return ((buff[byte_position] & (1 << bit_position)) != 0);
}

//___________________________________________________________________________
 /*!
   \brief Décode une donnée 8 bits du buffer
   \param buff : le buffer de destination
   \param position : position de la data dans le buffer
   \return la data sur 8 bits
*/
unsigned char CDataEncoderDecoder::decode_int8(unsigned char *buff, unsigned char position)
{
    return buff[position];
}

//___________________________________________________________________________
 /*!
   \brief Décode une donnée 16 bits du buffer (MSB first)
   \param buff : le buffer de destination
   \param position : position de la data dans le buffer
   \return la data sur 16 bits
*/
unsigned short CDataEncoderDecoder::decode_int16(unsigned char *buff, unsigned char position)
{
    return ( ( ((unsigned short)(buff[position+1])) & 0xFF) )  |  ( ( ((unsigned short)(buff[position])) & 0xFF) << 8 ) ;
}

//___________________________________________________________________________
 /*!
   \brief Décode une donnée 32 bits du buffer (MSB first)
   \param buff : le buffer de destination
   \param position : position de la data dans le buffer
   \return la data sur 32 bits
*/
unsigned long CDataEncoderDecoder::decode_int32(unsigned char *buff, unsigned char position)
{
    return ( ( ((unsigned long)(buff[position+3])) & 0xFF) )  |
            ( ( ((unsigned long)(buff[position+2])) & 0xFF) << 8 ) |
            ( ( ((unsigned long)(buff[position+1])) & 0xFF) << 16 ) |
            ( ( ((unsigned long)(buff[position])) & 0xFF) << 24 );
}


