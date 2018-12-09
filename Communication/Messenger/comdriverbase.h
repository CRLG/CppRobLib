#ifndef _COM_DRIVER_BASE_H
#define _COM_DRIVER_BASE_H

// ====================================================
//       ABSTRACT COMMUNICATION DRIVER CLASS
// ====================================================

typedef enum {
    COMDRV_OK = 0,
    COMDRV_WRITE_ERROR
}tComDriverErr;

class TransporterBase;

class ComDriverBase
{
public:
    ComDriverBase() { }
    virtual ~ComDriverBase() { }

    inline void setTransporter(TransporterBase *transporter) { m_transporter = transporter; }

    // pure virtual methods for hardware abstraction.
    // to be implemented on specific hardware.
    virtual void encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address=0) = 0;

protected :
    TransporterBase *m_transporter;
};

#endif // _COM_DRIVER_BASE_H
