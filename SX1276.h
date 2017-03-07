#ifndef __SX1276_H__
#define __SX1276_H__

#include <driverlib.h>
#include <SPI.h>

#include <sx1276Regs-Fsk.h>
#include <sx1276Regs-LoRa.h>
#include <sx1276Enums.h>

#define SX1276_OK      0
#define SX1276_ERROR   1

// 2s timeout (in ms)
#define TIMEOUT 2000

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ               32000000
#define FREQ_STEP               61.03515625d

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;

class SX1276
{
protected:

public:
    SX1276();
    virtual ~SX1276( ) {};
    unsigned char init();
    void reset();
    bool ping();
    void sleep();
    void standby();
    void setFrequency(unsigned long);
    unsigned long getFrequency();
    signed short getFrequencyError();
    void setOpMode(unsigned char);
    unsigned char getOpMode();
    void setModem(RadioModems_t modem);
    void setTxConfig( TxConfig_t* );
    void setRxConfig( RxConfig_t* );
    bool send(unsigned char *, unsigned char);
    void RxChainCalibration( void );
    unsigned char GetFskBandwidthRegValue( unsigned long );
    unsigned char startReceiver( );
    signed short GetRssi( RadioModems_t  );
	unsigned char getRXData(unsigned char *, unsigned char );

private:
	bool pktFixLen;
	static volatile bool DIO0event;
	unsigned char readRegister(unsigned char);
    void writeRegister(unsigned char, unsigned char);
    void write(unsigned char, unsigned char *, unsigned char);
    void read(unsigned char, unsigned char *, unsigned char);
    void writeFifo(unsigned char *, unsigned char);
    void readFifo(unsigned char *, unsigned char);
	static void GPIO_IRQHandler( void );
    void delayms(unsigned short );
    void delay100us(unsigned short );
 	static const FskBandwidth_t FskBandwidths[] ;
};


#endif 