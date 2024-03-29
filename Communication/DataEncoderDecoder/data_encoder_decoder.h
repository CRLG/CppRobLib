/*! \file CDataEncoderDecoder.h
	\brief Classe qui contient les méthodes génériques de décodage/encodage des données dans un buffer
*/

#ifndef _DATA_ENCODER_DECODER_H_
#define _DATA_ENCODER_DECODER_H_

//!
//! \brief Cette classe ne contient que des méthodes statiques
//! Utilitaire d'aide à l'encodage/décodage des données dans un buffer
//!
class CDataEncoderDecoder 
{
public :
    static void encode_bit(unsigned char *buff, unsigned char byte_position, unsigned char bit_position, bool data);
    static void encode_uint8(unsigned char *buff, unsigned char position, unsigned char data);
    static void encode_uint16(unsigned char *buff, unsigned char position, unsigned short data);
    static void encode_uint24(unsigned char *buff, unsigned char position, unsigned long data);
    static void encode_uint32(unsigned char *buff, unsigned char position, unsigned long data);
    static void encode_int8(unsigned char *buff, unsigned char position, signed char data);
    static void encode_int16(unsigned char *buff, unsigned char position, signed short data);
    static void encode_int32(unsigned char *buff, unsigned char position, signed long data);

    static bool decode_bit(unsigned char *buff, unsigned char byte_position, unsigned char bit_position);
    static unsigned char decode_uint8(unsigned char *buff, unsigned char position);
    static unsigned short decode_uint16(unsigned char *buff, unsigned char position);
    static unsigned long  decode_uint24(unsigned char *buff, unsigned char position);
    static unsigned long  decode_uint32(unsigned char *buff, unsigned char position);
    static signed char  decode_int8(unsigned char *buff, unsigned char position);
    static signed short decode_int16(unsigned char *buff, unsigned char position);
    static signed long  decode_int32(unsigned char *buff, unsigned char position);
};

#endif


