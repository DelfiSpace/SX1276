#include "SX1276.h"



#define RESET_PIN 2 // changed from pin 13!
#define CS_PIN 18
#define DIO0 17		// changed from pin 24!
#define DIO1 12		//5
//#define DIO2 33		//25
//#define DIO3 6

/**** static data ****/

const FskBandwidth_t SX1276::FskBandwidths[] =
{		
	{ 2600	, 0x17 },	
	{ 3100	, 0x0F },
	{ 3900	, 0x07 },
	{ 5200	, 0x16 },
	{ 6300	, 0x0E },
	{ 7800	, 0x06 },
	{ 10400 , 0x15 },
	{ 12500 , 0x0D },
	{ 15600 , 0x05 },
	{ 20800 , 0x14 },
	{ 25000 , 0x0C },
	{ 31300 , 0x04 },
	{ 41700 , 0x13 },
	{ 50000 , 0x0B },
	{ 62500 , 0x03 },
	{ 83333 , 0x12 },
	{ 100000, 0x0A },
	{ 125000, 0x02 },
	{ 166700, 0x11 },
	{ 200000, 0x09 },
	{ 250000, 0x01 },
	{ 300000, 0x00 }, // Invalid Badwidth
};

bool volatile SX1276::DIO0event = false;

/**** ISR/IRQ Handles ****/
void SX1276::GPIO_IRQHandler( void )
{
    // cleanup the interrupt flag
    uint_fast16_t status = ROM_GPIO_getEnabledInterruptStatus(pins->DIO0Port);
    MAP_GPIO_clearInterruptFlag(pins->DIO0Port, pins->DIO0Pin);

    // if RF_OPMODE_TRANSMITTER: packet transmitted
    // if RF_OPMODE_RECEIVER: packet received
    if ( status & pins->DIO0Pin)
    {
        // set the data received flag
        DIO0event = true;
    }
}

/**** Functions ****/
SX1276::SX1276(DSPI &spi, const SX1276Pins *pinsDefinition):
        line(spi), pins(pinsDefinition)
{
	bitModeSPI = 0;
}

void SX1276::enableBitMode(DSPI &bitspi, void(*rxHandler)( uint8_t ), uint8_t(*txHandler)( void ))
{
	bitModeSPI = &bitspi;
	bitModeSPI->initSlave(DSPI::MODE0, DSPI::MSBFirst);
	bitModeSPI->onTransmit(txHandler);
	bitModeSPI->onReceive(rxHandler);
}

void SX1276::disableBitMode()
{
	bitModeSPI = 0;
	bitModeSPI->onTransmit(0);
	bitModeSPI->onReceive(0);
}
	
unsigned char SX1276::init()
{
	// monitor the PacketDone pin
	MAP_GPIO_setAsInputPin( pins->DIO0Port, pins->DIO0Pin );
	
	// set the CS_PIN as disabled and then as output (to avoid a glitch during init)
    MAP_GPIO_setOutputHighOnPin( pins->CSPort, pins->CSPin );
    MAP_GPIO_setAsOutputPin( pins->CSPort, pins->CSPin );
  
  	// set the RESET_PIN as disabled and then as output (to avoid a glitch during init)
    MAP_GPIO_setOutputHighOnPin( pins->RESETPort, pins->RESETPin );
    MAP_GPIO_setAsOutputPin( pins->RESETPort, pins->RESETPin );
	
	// initialize SPI:
	line.initMaster(DSPI::MODE0, DSPI::MSBFirst, 1000000);

	// ensure the radio got a fresh start
	reset();
	
	// ensure the radio is in sleep mode
	sleep();
}

void SX1276::reset()
{
    MAP_GPIO_setOutputLowOnPin( pins->RESETPort, pins->RESETPin );
	delayms(10);
	MAP_GPIO_setOutputHighOnPin( pins->RESETPort, pins->RESETPin );
}

bool SX1276::ping()
{
	return readRegister(REG_VERSION) == SAMTEC_ID;
}

void SX1276::setFrequency(unsigned long freq)
{
	freq = (unsigned long)((double)freq / FREQ_STEP );

	writeRegister(REG_FRFMSB, (unsigned char)((freq >> 16) & 0xFF ) );
	writeRegister(REG_FRFMID, (unsigned char)((freq >>	8) & 0xFF ) );
	writeRegister(REG_FRFLSB, (unsigned char)( freq		   & 0xFF ) );
}

unsigned long SX1276::getFrequency()
{
	return (unsigned long)((double)(((unsigned long)readRegister(REG_FRFMSB) << 16 ) |
									((unsigned long)readRegister(REG_FRFMID) <<	 8 ) |
									((unsigned long)readRegister(REG_FRFLSB)	   )) * FREQ_STEP);
}

void SX1276::sleep( void )
{
	setOpMode(RF_OPMODE_SLEEP);
}

void SX1276::standby( void )
{
	setOpMode(RF_OPMODE_STANDBY);
}

void SX1276::RxChainCalibration( void )
{
	uint8_t regPaConfigInitVal;
	uint32_t initialFreq;

	// Save context
	regPaConfigInitVal = readRegister( REG_PACONFIG );
	initialFreq = getFrequency();
	
	// Cut the PA just in case, RFO output, power = -1 dBm
	writeRegister( REG_PACONFIG, 0x00 );

	if (initialFreq < 700000000)
	{
		// Launch Rx chain calibration for LF band
		writeRegister ( REG_IMAGECAL, ( readRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
		while( ( readRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
		{
		}
	}
	else
	{
		// Launch Rx chain calibration for HF band 
		writeRegister ( REG_IMAGECAL, ( readRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
		while( ( readRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
		{
		}
	}
}

unsigned char SX1276::setOpMode( unsigned char opMode )
{
	// only enable the interrupts if we are in packet mode
	if (!bitModeSPI)
	{
		// if we are switching to transmit or receive mode, enable the interrupts
		// otherwise, disable them
		if ((opMode == RF_OPMODE_TRANSMITTER) || (opMode == RF_OPMODE_RECEIVER))
		{
			MAP_GPIO_clearInterruptFlag( pins->DIO0Port, pins->DIO0Pin );
			MAP_GPIO_interruptEdgeSelect( pins->DIO0Port, pins->DIO0Pin, GPIO_LOW_TO_HIGH_TRANSITION );
			MAP_GPIO_registerInterrupt(pins->DIO0Port, pins->callback);
			MAP_GPIO_enableInterrupt( pins->DIO0Port, pins->DIO0Pin );
		}
	}
	
	// now we can change the operating mode...
	writeRegister(REG_OPMODE, (readRegister(REG_OPMODE) & RF_OPMODE_MASK) | opMode);
	
	unsigned short time = 0;
	while ((getOpMode() != opMode) && (time < 10 * TIMEOUT))
	{
		delay100us(1);
		time++;
	}
	
	// only enable the interrupts if we are in packet mode
	if (!bitModeSPI)
	{
		// if we are not switching to transmit or receive mode, disable the interrupts
		if ((opMode != RF_OPMODE_TRANSMITTER) && (opMode != RF_OPMODE_RECEIVER))
		{
			MAP_GPIO_disableInterrupt( pins->RESETPort, pins->RESETPin );
			MAP_GPIO_unregisterInterrupt( pins->RESETPort );
		}
	}
	return time < TIMEOUT;
}

unsigned char SX1276::getOpMode( )
{
	return readRegister(REG_OPMODE) & ~RF_OPMODE_MASK;
}

void SX1276::setModem(RadioModems_t modem)
{
	switch(modem)
	{
	default:

	case MODEM_FSK:
		setOpMode( RF_OPMODE_SLEEP );
		writeRegister(REG_OPMODE, (readRegister(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_OFF);
	
		writeRegister(REG_DIOMAPPING1, 0x00);
		writeRegister(REG_DIOMAPPING2, 0x30); // DIO5=ModeReady
		break;

	case MODEM_LORA:
		setOpMode( RF_OPMODE_SLEEP );
		writeRegister(REG_OPMODE, (readRegister(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);

		writeRegister(REG_DIOMAPPING1, 0x00);
		writeRegister(REG_DIOMAPPING2, 0x00);
		break;
	}
}

unsigned char SX1276::GetFskBandwidthRegValue( unsigned long bandwidth )
{
	uint8_t i;

	for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
	{
		if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
		{
			return FskBandwidths[i].RegValue;
		}
	}
	// ERROR: Value not found
	while( 1 );
}

void SX1276::setRxConfig( RxConfig_t* config )
{
	setModem( config->modem );

	switch( config->modem )
	{
	case MODEM_FSK:
		{
			// pulse shaping
			switch( config->filtertype )
			{
			case BT_1_0:
				// gaussian shaping, BT = 1.0
				writeRegister(REG_PARAMP, RF_PARAMP_SHAPING_BT_1_0);
				break;
				
			case BT_0_5:
				// gaussian shaping, BT = 0.5
				writeRegister(REG_PARAMP, RF_PARAMP_SHAPING_BT_0_5);
				break;
				
			case BT_0_3:
				// gaussian shaping, BT = 0.3
				writeRegister(REG_PARAMP, RF_PARAMP_SHAPING_BT_0_3);
				break;
				
			default:
			case NONE:
				// gaussian shaping, BT = 1.0
				writeRegister(REG_PARAMP, RF_PARAMP_SHAPING_NONE);
				break;
			}
			
			config->datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )config->datarate );
			writeRegister( REG_BITRATEMSB, ( uint8_t )( config->datarate >> 8 ) );
			writeRegister( REG_BITRATELSB, ( uint8_t )( config->datarate & 0xFF ) );

			writeRegister( REG_RXBW, GetFskBandwidthRegValue( config->bandwidth ) );
			writeRegister( REG_AFCBW, GetFskBandwidthRegValue( config->bandwidthAfc ) );

			writeRegister( REG_PREAMBLEMSB, ( uint8_t )( ( config->preambleLen >> 8 ) & 0xFF ) );
			writeRegister( REG_PREAMBLELSB, ( uint8_t )( config->preambleLen & 0xFF ) );
			
			pktFixLen = config->fixLen;
			
			if( pktFixLen )
			{
				writeRegister( REG_PAYLOADLENGTH, config->payloadLen );
			}
			else
			{
				writeRegister( REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum 
			}
			
			// configure the receiver to automatically restart after reception
			writeRegister( REG_RXCONFIG, readRegister( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
	
			writeRegister(REG_PACKETCONFIG1, (( config->whitening ) ? RF_PACKETCONFIG1_DCFREE_WHITENING : RF_PACKETCONFIG1_DCFREE_OFF ) |
											(( config->fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
											(( config->crcOn == 1 ) ? RF_PACKETCONFIG1_CRC_ON : RF_PACKETCONFIG1_CRC_OFF));
											
			if (!bitModeSPI)
			{
				writeRegister(REG_PACKETCONFIG2, 0x40); 			// enable packet mode
				writeRegister( REG_OOKPEAK, 0x28 ); 				// force synchroniser on
			}
			else
			{
				writeRegister( REG_PACKETCONFIG2, 0x00 );			// enable continuous mode
				writeRegister( REG_OOKPEAK, 0x28 ); 				// force synchroniser on
			}
			
			// set the synch word
			writeRegister(REG_SYNCCONFIG, RF_SYNCCONFIG_SYNCSIZE_4 | RF_SYNCCONFIG_SYNC_ON | 
											RF_SYNCCONFIG_PREAMBLEPOLARITY_AA | 
											RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON);
			writeRegister(REG_SYNCVALUE1, 0xD3);
			writeRegister(REG_SYNCVALUE2, 0x91);
			writeRegister(REG_SYNCVALUE3, 0xDA);
			writeRegister(REG_SYNCVALUE4, 0x26);

			// set RSSI config
			writeRegister(REG_RSSICONFIG, RF_RSSICONFIG_OFFSET_P_00_DB | RF_RSSICONFIG_SMOOTHING_256);

		}
		break;
	/*case MODEM_LORA:
		{
			if( config->bandwidth > 2 )
			{
				// Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
				while( 1 );
			}
			config->bandwidth += 7;

			if( config->datarate > 12 )
			{
				config->datarate = 12;
			}
			else if( config->datarate < 6 )
			{
				config->datarate = 6;
			}
		
			if( ( ( config->bandwidth == 7 ) && ( ( config->datarate == 11 ) || ( config->datarate == 12 ) ) ) ||
				( ( config->bandwidth == 8 ) && ( config->datarate == 12 ) ) )
			{
				this->settings.LoRa.LowDatarateOptimize = 0x01;
			}
			else
			{
				this->settings.LoRa.LowDatarateOptimize = 0x00;
			}

			writeRegister( REG_LR_MODEMCONFIG1, 
						 ( readRegister( REG_LR_MODEMCONFIG1 ) &
						   RFLR_MODEMCONFIG1_BW_MASK &
						   RFLR_MODEMCONFIG1_CODINGRATE_MASK &
						   RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
						   ( config->bandwidth << 4 ) | ( config->coderate << 1 ) | 
						   config->fixLen );
						
			writeRegister( REG_LR_MODEMCONFIG2,
						 ( readRegister( REG_LR_MODEMCONFIG2 ) &
						   RFLR_MODEMCONFIG2_SF_MASK &
						   RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
						   RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
						   ( config->datarate << 4 ) | ( config->crcOn << 2 ) |
						   ( ( config->symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

			writeRegister( REG_LR_MODEMCONFIG3, 
						 ( readRegister( REG_LR_MODEMCONFIG3 ) &
						   RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
						   ( this->settings.LoRa.LowDatarateOptimize << 3 ) );

			writeRegister( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( config->symbTimeout & 0xFF ) );
			
			writeRegister( REG_LR_PREAMBLEMSB, ( uint8_t )( ( config->preambleLen >> 8 ) & 0xFF ) );
			writeRegister( REG_LR_PREAMBLELSB, ( uint8_t )( config->preambleLen & 0xFF ) );

			if( config->fixLen == 1 )
			{
				writeRegister( REG_LR_PAYLOADLENGTH, config->payloadLen );
			}

			if( this->settings.LoRa.FreqHopOn == true )
			{
				writeRegister( REG_LR_PLLHOP, ( readRegister( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
				writeRegister( REG_LR_HOPPERIOD, this->settings.LoRa.HopPeriod );
			}

			if( ( config->bandwidth == 9 ) && ( this->settings.Channel > RF_MID_BAND_THRESH ) )
			{
				// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth 
				writeRegister( REG_LR_TEST36, 0x02 );
				writeRegister( REG_LR_TEST3A, 0x64 );
			}
			else if( config->bandwidth == 9 )
			{
				// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
				writeRegister( REG_LR_TEST36, 0x02 );
				writeRegister( REG_LR_TEST3A, 0x7F );
			}
			else
			{
				// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
				writeRegister( REG_LR_TEST36, 0x03 );
			}
			
			if( config->datarate == 6 )
			{
				writeRegister( REG_LR_DETECTOPTIMIZE, 
							 ( readRegister( REG_LR_DETECTOPTIMIZE ) &
							   RFLR_DETECTIONOPTIMIZE_MASK ) |
							   RFLR_DETECTIONOPTIMIZE_SF6 );
				writeRegister( REG_LR_DETECTIONTHRESHOLD, 
							 RFLR_DETECTIONTHRESH_SF6 );
			}
			else
			{
				writeRegister( REG_LR_DETECTOPTIMIZE,
							 ( readRegister( REG_LR_DETECTOPTIMIZE ) &
							 RFLR_DETECTIONOPTIMIZE_MASK ) |
							 RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
				writeRegister( REG_LR_DETECTIONTHRESHOLD, 
							 RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
			}
		}
		break;*/
	}
}

void SX1276::setTxConfig( TxConfig_t* config )
{
	setModem( config->modem );
	
	unsigned char paConfig = readRegister(REG_PACONFIG);
	unsigned char paDac = readRegister(REG_PADAC);
//
//	paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK );
//	paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

//	if( config->power < -1 )
//	{
//		config->power = -1;
//	}
//	if( config->power > 14 )
//	{
//		config->power = 14;
//	}
//	paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( unsigned char )( ( unsigned short )( 1 ) & 0x0F );
 
    paConfig = 0x00 | (config->power & 0x0F);
	writeRegister(REG_PACONFIG, paConfig);
//	writeRegister(REG_PACONFIG, config->power );
	writeRegister(REG_PADAC, paDac);

	switch( config->modem )
	{
	case MODEM_FSK:
		{
			// pulse shaping and internal PA ramp up
			switch( config->filtertype )
			{
			case BT_1_0:
				// gaussian shaping, BT = 1.0
				writeRegister(REG_PARAMP, RF_PARAMP_SHAPING_BT_1_0 | RF_PARAMP_3400_US);
				break;
				
			case BT_0_5:
				// gaussian shaping, BT = 0.5
				writeRegister(REG_PARAMP, RF_PARAMP_SHAPING_BT_0_5 | RF_PARAMP_3400_US);
				break;
				
			case BT_0_3:
				// gaussian shaping, BT = 0.3
				writeRegister(REG_PARAMP, RF_PARAMP_SHAPING_BT_0_3 | RF_PARAMP_3400_US);
				break;
				
			default:
			case NONE:
				// gaussian shaping, BT = 1.0
				writeRegister(REG_PARAMP, RF_PARAMP_SHAPING_NONE | RF_PARAMP_3400_US);
				break;
			}
			
			config->fdev = (unsigned short)((double)config->fdev / FREQ_STEP);
			writeRegister(REG_FDEVMSB, (unsigned char)(config->fdev >> 8));
			writeRegister(REG_FDEVLSB, (unsigned char)(config->fdev & 0xFF));
			// read back the frequency deviation
			config->fdev = (unsigned int)((double)(((unsigned int)readRegister(REG_FDEVMSB) << 8 ) |
												   ((unsigned int)readRegister(REG_FDEVLSB))) * FREQ_STEP);

			// set packet length to be fixed or variable
			pktFixLen = config->fixLen;
			
			config->datarate = (unsigned short)((double)XTAL_FREQ / (double)config->datarate);
			writeRegister(REG_BITRATEMSB, (unsigned char)(config->datarate >> 8 ));
			writeRegister(REG_BITRATELSB, (unsigned char)(config->datarate & 0xFF));
			// read back the datarate
			config->datarate = (unsigned int)((double)(((unsigned int)readRegister(REG_BITRATEMSB) << 8 ) |
												   ((unsigned int)readRegister(REG_BITRATELSB))) * (double)XTAL_FREQ);

			writeRegister(REG_PREAMBLEMSB, (config->preambleLen >> 8) & 0x00FF);
			writeRegister(REG_PREAMBLELSB, config->preambleLen & 0xFF);

			writeRegister(REG_PACKETCONFIG1, (( config->whitening ) ? RF_PACKETCONFIG1_DCFREE_WHITENING : RF_PACKETCONFIG1_DCFREE_OFF ) |
											(( config->fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
											(( config->crcOn == 1 ) ? RF_PACKETCONFIG1_CRC_ON : RF_PACKETCONFIG1_CRC_OFF));
											
			if (!bitModeSPI)
			{
				writeRegister(REG_PACKETCONFIG2, 0x40); 		//enable packet mode
				writeRegister( REG_OOKPEAK, 0x28); 				// force synchroniser on
			}
			else
			{
				writeRegister( REG_PACKETCONFIG2, 0x00 );		// enable continuous mode
				writeRegister( REG_OOKPEAK, 0x28); 				// force synchroniser on
			}
			
			writeRegister(REG_SYNCCONFIG, RF_SYNCCONFIG_SYNCSIZE_4 | RF_SYNCCONFIG_SYNC_ON | RF_SYNCCONFIG_PREAMBLEPOLARITY_AA | RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON);
			writeRegister(REG_SYNCVALUE1, 0xD3);
			writeRegister(REG_SYNCVALUE2, 0x91);
			writeRegister(REG_SYNCVALUE3, 0xDA);
			writeRegister(REG_SYNCVALUE4, 0x26);
		}
		break;
	case MODEM_LORA:
		{
			if( config->bandwidth > 2 )
			{
				// When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
				config->bandwidth = 2;
			}
			//bandwidth += 7;
			/*this->settings.LoRa.Bandwidth = bandwidth;
			this->settings.LoRa.Datarate = datarate;
			this->settings.LoRa.Coderate = coderate;
			this->settings.LoRa.PreambleLen = preambleLen;
			this->settings.LoRa.FixLen = fixLen;
			this->settings.LoRa.FreqHopOn = freqHopOn;
			this->settings.LoRa.HopPeriod = hopPeriod;
			this->settings.LoRa.CrcOn = crcOn;
			this->settings.LoRa.IqInverted = iqInverted;
			this->settings.LoRa.TxTimeout = timeout;*/

			if( config->datarate > 12 )
			{
				config->datarate = 12;
			}
			else if( config->datarate < 6 )
			{
				config->datarate = 6;
			}

			unsigned char LowDatarateOptimize = 0x00;
			if( ( ( config->bandwidth == 0 ) && ( ( config->datarate == 11 ) || ( config->datarate == 12 ) ) ) ||
				( ( config->bandwidth == 1 ) && ( config->datarate == 12 ) ) )
			{
				LowDatarateOptimize = 0x01;
			}

			if( config->freqHopOn == true )
			{
				writeRegister(REG_LR_PLLHOP, (readRegister(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
				writeRegister(REG_LR_HOPPERIOD, config->hopPeriod );
			}

			writeRegister(REG_LR_MODEMCONFIG1, 
						 (readRegister(REG_LR_MODEMCONFIG1) &
						  RFLR_MODEMCONFIG1_BW_MASK &
						  RFLR_MODEMCONFIG1_CODINGRATE_MASK &
						  RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
						  ((config->bandwidth + 7) << 4) | (config->coderate << 1 ) | 
						  config->fixLen);

			writeRegister(REG_LR_MODEMCONFIG2,
						 (readRegister(REG_LR_MODEMCONFIG2) &
						  RFLR_MODEMCONFIG2_SF_MASK &
						  RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
						  ( config->datarate << 4 ) | ( config->crcOn << 2 ) );

			writeRegister(REG_LR_MODEMCONFIG3, 
						 (readRegister(REG_LR_MODEMCONFIG3) &
						 RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
						 (LowDatarateOptimize << 3));
		
			writeRegister(REG_LR_PREAMBLEMSB, (config->preambleLen >> 8) & 0x00FF);
			writeRegister(REG_LR_PREAMBLELSB, config->preambleLen & 0xFF);
			
			if( config->datarate == 6 )
			{
				writeRegister(REG_LR_DETECTOPTIMIZE, 
							 (readRegister(REG_LR_DETECTOPTIMIZE) &
							   RFLR_DETECTIONOPTIMIZE_MASK) |
							   RFLR_DETECTIONOPTIMIZE_SF6);
				writeRegister(REG_LR_DETECTIONTHRESHOLD, 
							 RFLR_DETECTIONTHRESH_SF6);
			}
			else
			{
				writeRegister(REG_LR_DETECTOPTIMIZE,
							 (readRegister(REG_LR_DETECTOPTIMIZE) &
							 RFLR_DETECTIONOPTIMIZE_MASK) |
							 RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
				writeRegister(REG_LR_DETECTIONTHRESHOLD, 
							 RFLR_DETECTIONTHRESH_SF7_TO_SF12);
			}
		}
		break;
	}
}

void SX1276::writeRegister(unsigned char address, unsigned char value) 
{
	// take the CS pin low to select the chip:
    MAP_GPIO_setOutputLowOnPin( pins->CSPort, pins->CSPin );

	// send in the address and value via SPI:
	line.transfer(address | 0x80);

	// write the value
	line.transfer(value);
  
	// take the CS pin high to de-select the chip:
	MAP_GPIO_setOutputHighOnPin( pins->CSPort, pins->CSPin );
}

unsigned char SX1276::readRegister(unsigned char address) 
{
	// take the CS pin low to select the chip:
    MAP_GPIO_setOutputLowOnPin( pins->CSPort, pins->CSPin );

	// send in the address
	line.transfer(address & 0x7F);

	// read the value
	unsigned char val = line.transfer(0);
  
	// take the CS pin high to de-select the chip:
	MAP_GPIO_setOutputHighOnPin( pins->CSPort, pins->CSPin );

	return val;
}

void SX1276::read(unsigned char address, unsigned char *data, unsigned char size) 
{
	// take the CS pin low to select the chip:
    MAP_GPIO_setOutputLowOnPin( pins->CSPort, pins->CSPin );

	// send in the address
	line.transfer(address & 0x7F);

	// read the value
	for (unsigned short i = 0; i < (unsigned short)size; i++)
	{
		data[i] = line.transfer(0);
	}
	
	// take the CS pin high to de-select the chip:
	MAP_GPIO_setOutputHighOnPin( pins->CSPort, pins->CSPin );
}

void SX1276::write(unsigned char address, unsigned char *data, unsigned char size) 
{
	// take the CS pin low to select the chip:
    MAP_GPIO_setOutputLowOnPin( pins->CSPort, pins->CSPin );

	// send in the address
	line.transfer(address | 0x80);

	// read the value
	for (unsigned short i = 0; i < (unsigned short)size; i++)
	{
		 line.transfer(data[i]);
	}
	
	// take the CS pin high to de-select the chip:
	MAP_GPIO_setOutputHighOnPin( pins->CSPort, pins->CSPin );
}

void SX1276::writeFifo(unsigned char *buffer, unsigned char size)
{
	write( 0, buffer, size );
}

void SX1276::readFifo(unsigned char *buffer, unsigned char size)
{
	read( 0, buffer, size );
}

unsigned char SX1276::setIdleMode( bool idle )
{
	if (idle)
	{
		return setOpMode( RF_OPMODE_TRANSMITTER );
	}
	else
	{
		return setOpMode( RF_OPMODE_SLEEP );
	}
}

bool SX1276::send(unsigned char *buffer, unsigned char size)
{
	// send is only working in packet mode
	if (bitModeSPI)
	{
		return false;
	}
	
	// set the FIFO threshold to its default value
	writeRegister(REG_FIFOTHRESH, RF_FIFOTHRESH_FIFOTHRESHOLD_THRESHOLD);
	
	DIO0event = false;
	
	if (pktFixLen)
	{
		// write the amount of bytes to send
		writeRegister( REG_PAYLOADLENGTH, size );
	
		// write the buffer to the FIFO: FIFO size is only 64 bytes so the chunk must fit!
		writeFifo( buffer, size );
	}
	else
	{
		// write the amount of bytes to send
		writeRegister( REG_PAYLOADLENGTH, 0xFF );
		
		writeFifo(&size, 1);
		// write the buffer to the FIFO: FIFO size is only 64 bytes so the chunk must fit!
		writeFifo( buffer, size );
	}
	
	// if the number of bytes to send is lower than the FIFO threshold for transmission, 
	// ensure the FIFO will be flushed
	if (size <= RF_FIFOTHRESH_FIFOTHRESHOLD_THRESHOLD)
	{
		writeRegister(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY | 
									  RF_FIFOTHRESH_FIFOTHRESHOLD_THRESHOLD );
	}
	
	// DIO0=PacketSent
	// DIO1=FifoEmpty
	// DIO2=FifoFull
	// DIO3=FifoEmpty
	// DIO4=LowBat
	// DIO5=ModeReady
	writeRegister( REG_DIOMAPPING1, ( readRegister( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
														RF_DIOMAPPING1_DIO2_MASK ) |
														RF_DIOMAPPING1_DIO1_01 );

	writeRegister( REG_DIOMAPPING2, ( readRegister( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
														RF_DIOMAPPING2_MAP_MASK ) );
	
	unsigned char previousMode = getOpMode();
	
	// switch to transmitter mode if we are not there yet
	if ( previousMode != RF_OPMODE_TRANSMITTER)
	{
		// turn the transmitter ON
		setOpMode( RF_OPMODE_TRANSMITTER );
	}
	unsigned short time = 0;
	while (!DIO0event && (time < 10 * TIMEOUT))
	{
		delay100us(1);
		time++;
	}
	
	// set the radio in the old operational mode, if we were not it transmit mode
	if (previousMode != RF_OPMODE_TRANSMITTER)
	{
		// turn the transmitter ON
		setOpMode( previousMode );
	}
	// return true if timeout did not elapse
	return time < TIMEOUT;
}

unsigned char SX1276::startReceiver( )
{
	// DIO0=PayloadReady
	// DIO1=FifoLevel
	// DIO2=SyncAddr
	// DIO3=FifoEmpty
	// DIO4=Preamble
	// DIO5=ModeReady
	writeRegister( REG_DIOMAPPING1, ( readRegister( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
																	RF_DIOMAPPING1_DIO1_MASK &
																	RF_DIOMAPPING1_DIO2_MASK ) |
																	RF_DIOMAPPING1_DIO0_00 |
																	RF_DIOMAPPING1_DIO2_11 );
	
	writeRegister( REG_DIOMAPPING2, ( readRegister( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
																	RF_DIOMAPPING2_MAP_MASK ) | 
																	RF_DIOMAPPING2_DIO4_11 |
																	RF_DIOMAPPING2_MAP_PREAMBLEDETECT );
																	
	// clear the receiver flag																
	DIO0event = false;
	
	return setOpMode( RF_OPMODE_RECEIVER );
}

unsigned char SX1276::getRXData(unsigned char *data, unsigned char sz)
{
	// getRXData is only working in packet mode
	if (bitModeSPI)
	{
		return 0;
	}
	
	if (DIO0event)
	{
		unsigned char size;
		if (pktFixLen)
		{
			// FIXME: handle errors reading from FIFO: packet too long, packet too short
			// re-initialize FIFO, etc
			size = readRegister( REG_PAYLOADLENGTH);
			// fixed length packet: read the data straight away
			readFifo( data, size );
		}
		else
		{
			// variable length packet: first read the size and then read the data
			readFifo(&size, 1);
			readFifo(data, size);
		}

		// configure the receiver to automatically restart after reception
		writeRegister( REG_RXCONFIG, readRegister( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
		
		// clear the flag
		DIO0event = false;
		
		return size;
	}
	
	return 0;
}

signed short SX1276::GetRssi( RadioModems_t modem )
{
	signed short rssi = 0;

	switch( modem )
	{
	case MODEM_FSK:
		rssi = -( readRegister( REG_RSSIVALUE ) >> 1 );
		break;
	/*case MODEM_LORA:
		if( this->settings.Channel > RF_MID_BAND_THRESH )
		{
			rssi = RSSI_OFFSET_HF + readRegister( REG_LR_RSSIVALUE );
		}
		else
		{
			rssi = RSSI_OFFSET_LF + readRegister( REG_LR_RSSIVALUE );
		}
		break;*/
	default:
		rssi = -1;
		break;
	}
	return rssi;
}

signed short SX1276::getFrequencyError()
{
	return (signed short)((double)(((unsigned short)readRegister(REG_FEIMSB) << 8 ) |
								 ((unsigned short)readRegister(REG_FEILSB)		 )) * FREQ_STEP);
}

void SX1276::delayms( unsigned short ms ) 
{
	unsigned int delayCycles = MAP_CS_getMCLK( ) / 9100;
	
	for (unsigned short j = 0; j < ms; j++)
	{
		for (int i = 0; i < delayCycles; i++) 
		{
			__no_operation();
		}
	}
}

void SX1276::delay100us( unsigned short iterations ) 
{
	unsigned int delayCycles = MAP_CS_getMCLK( ) / 91000;
	
	for (unsigned short j = 0; j < iterations; j++)
	{
		for (int i = 0; i < delayCycles; i++) 
		{
			__no_operation();
		}
	}
}
