#ifndef __SX1276_H__
#define __SX1276_H__

#include <driverlib.h>
#include <DSPI.h>

#include <sx1276Regs-Fsk.h>
#include <sx1276Regs-LoRa.h>
#include <sx1276Enums.h>

#define SX1276_OK	   0
#define SX1276_ERROR   1

// 2s timeout (in ms)
#define TIMEOUT 2000

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ				32000000
#define FREQ_STEP				61.03515625f

/*!
 * FSK bandwidth definition
 */
typedef struct
{
	uint32_t bandwidth;
	uint8_t	 RegValue;
}FskBandwidth_t;

typedef struct
{
    unsigned long CSPort;
    unsigned long CSPin;
    unsigned long RESETPort;
    unsigned long RESETPin;
    unsigned long DIO0Port;
    unsigned long DIO0Pin;
    void(*callback)(  );

} SX1276Pins;

class SX1276
{
protected:
	DSPI &line;
	DSPI *bitModeSPI;
    const SX1276Pins *pins;
    SX1276 *instance;

public:
	SX1276( DSPI &spi, const SX1276Pins *pinsDefinition );
	virtual ~SX1276( ) {};
	unsigned char init();
	void reset();
	bool ping();
	void setFrequency(unsigned long);
	unsigned long getFrequency();
	signed short getFrequencyError();
	unsigned char setIdleMode( bool );
	void setTxConfig( TxConfig_t* );
	void setRxConfig( RxConfig_t* );
	bool send(unsigned char *, unsigned char);
	void RxChainCalibration( void );
	unsigned char startReceiver( );
	signed short GetRssi( RadioModems_t	 );
	unsigned char getRXData(unsigned char *, unsigned char );
	void enableBitMode(DSPI& bitspi, void(*rxHandler)( uint8_t ), uint8_t(*txHandler)( void ));
	void disableBitMode();
    void GPIO_IRQHandler( void );
    void RxLockPll();

private:
	bool pktFixLen;
	static volatile bool DIO0event;
	void setModem(RadioModems_t modem);
	unsigned char GetFskBandwidthRegValue( unsigned long );
	unsigned char readRegister(unsigned char);
	unsigned char setOpMode(unsigned char);
	unsigned char getOpMode();
	void sleep();
	void standby();
	void writeRegister(unsigned char, unsigned char);
	void write(unsigned char, unsigned char *, unsigned char);
	void read(unsigned char, unsigned char *, unsigned char);
	void writeFifo(unsigned char *, unsigned char);
	void readFifo(unsigned char *, unsigned char);
	void delayms(unsigned short );
	void delay100us(unsigned short );
	static const FskBandwidth_t FskBandwidths[] ;
};


#endif 
