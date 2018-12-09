#ifndef _MESSENGER_DEFINITIONS_H
#define _MESSENGER_DEFINITIONS_H

#define MESSENGER_MAX_DATA_LEN    8
typedef struct {
    unsigned short ID; // Message ID
    unsigned char DLC; // Data Lenght
    unsigned char Data[MESSENGER_MAX_DATA_LEN];
    unsigned short SourceAddress;
    unsigned short DestinationAddress;
}tMessengerFrame;

#endif // _MESSENGER_DEFINITIONS_H
