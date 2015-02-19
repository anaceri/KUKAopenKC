#ifndef COMINTERFACE_H
#define COMINTERFACE_H

#include "RebaType.h"
//#define OKC_HOST "129.70.129.23"
#define OKC_HOST "192.168.0.100"
#define OKC_PORT "49938"
#define LEFTARM_IP "192.168.0.10"
#define RIGHTARM_IP "192.168.0.20"

class ComInterface
{
public:
    ComInterface();
    virtual void connect() = 0;
    virtual bool isConnected() = 0;
    virtual void waitForFinished() = 0;
    float cycle_time;
};

#endif // COMINTERFACE_H
