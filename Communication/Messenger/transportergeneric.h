#ifndef _TRANSPORTER_GENERIC_H
#define _TRANSPORTER_GENERIC_H

#include "transporterbase.h"

// ====================================================
// ====================================================
class TransporterGeneric : public TransporterBase
{
public:
    TransporterGeneric();
    virtual ~TransporterGeneric();

    // this method must be called each time a data or a buffer is received
    virtual void decode(unsigned char newData, unsigned short source_address=0);
    virtual void decode(unsigned char *buff_data, unsigned char buff_size, unsigned short source_address=0);

    // entry point to encapsulate and send message
    virtual void encode(tMessengerFrame *frame);

private :
    //! Enumere les differents etats de la machine d'etat de l'automate.
    //! Cet enumere contient toutes les valeurs prises par la machine d'etat de reconstitution des donnees
    typedef enum {
      DECODER_STATE_INIT = 0,
      DECODER_STATE_ERREUR,
      DECODER_STATE_ID_MSB,
      DECODER_STATE_ID_LSB,
      DECODER_STATE_DLC,
      DECODER_STATE_DATA_i,
      DECODER_STATE_CHECKSUM
    } tRECONST_STATE;

    unsigned char m_decoder_state;
    unsigned char m_data_number;
    bool m_transfert_with_checksum;

    unsigned char _getChecksum(tMessengerFrame *frame);
    void _initDecoder();
};

#endif // _TRANSPORTER_GENERIC_H
