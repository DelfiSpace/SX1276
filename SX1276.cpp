#include "Energia.h"
#include "SX1276.h"
#include <SPI.h>

#define RESET_PIN 13
#define CS_PIN 18
#define ANTSWITCH 26
#define DIO0 24
#define DIO1 5
#define DIO2 25
#define DIO3 6

const FskBandwidth_t SX1276::FskBandwidths[] =
{       
    { 2600  , 0x17 },   
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
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

SX1276::SX1276()
{
}

unsigned char SX1276::init()
{
    // set the antenna switch as disabled and then as output (to avoid a glitch during init)
    digitalWrite(ANTSWITCH, HIGH);
    pinMode(ANTSWITCH, OUTPUT);

    // monitor the PacketDone pin
    pinMode(DIO0, INPUT_PULLUP);
  
    // set the CS_PIN as disabled and then as output (to avoid a glitch during init)
    digitalWrite(CS_PIN, HIGH);
    pinMode(CS_PIN, OUTPUT);
  
    // initialise SPI:
    SPI.begin();

    reset();
}

void SX1276::reset()
{
    digitalWrite(RESET_PIN, LOW);
    pinMode(RESET_PIN, OUTPUT);
    delay(50);
    digitalWrite(RESET_PIN, HIGH);
}

bool SX1276::ping()
{
	if (readRegister(REG_VERSION) == 0x12)
        return true;
	return false;
}

void SX1276::setFrequency(unsigned long freq)
{
    freq = (unsigned long)((double)freq / FREQ_STEP );

    writeRegister(REG_FRFMSB, (unsigned char)((freq >> 16) & 0xFF ) );
    writeRegister(REG_FRFMID, (unsigned char)((freq >>  8) & 0xFF ) );
    writeRegister(REG_FRFLSB, (unsigned char)( freq        & 0xFF ) );
}

unsigned long SX1276::getFrequency()
{
    return (unsigned long)((double)(((unsigned long)readRegister(REG_FRFMSB) << 16 ) |
                                    ((unsigned long)readRegister(REG_FRFMID) <<  8 ) |
                                    ((unsigned long)readRegister(REG_FRFLSB)       )) * FREQ_STEP);
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

    // Launch Rx chain calibration for LF band
    writeRegister ( REG_IMAGECAL, ( readRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( readRegister( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    //SetChannel( 868000000 );

    // Launch Rx chain calibration for HF band 
    //Write ( REG_IMAGECAL, ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    //while( ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    //{
    //}

    // Restore context
    //writeRegister( REG_PACONFIG, regPaConfigInitVal );
    //setFrequency( initialFreq );
}

void SX1276::setOpMode( unsigned char opMode )
{
    // control the antenna switch
    if (opMode == RF_OPMODE_TRANSMITTER)
    {
         digitalWrite(ANTSWITCH, HIGH);
    }
    else
    {
         digitalWrite(ANTSWITCH, LOW);
    }

    writeRegister(REG_OPMODE, (readRegister(REG_OPMODE) & RF_OPMODE_MASK) | opMode);
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
            // gaussian shaping, BT = 0.3
            //writeRegister(REG_PARAMP, 0x60);
            
            config->datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )config->datarate );
            writeRegister( REG_BITRATEMSB, ( uint8_t )( config->datarate >> 8 ) );
            writeRegister( REG_BITRATELSB, ( uint8_t )( config->datarate & 0xFF ) );

            writeRegister( REG_RXBW, GetFskBandwidthRegValue( config->bandwidth ) );
            writeRegister( REG_AFCBW, GetFskBandwidthRegValue( config->bandwidthAfc ) );

            writeRegister( REG_PREAMBLEMSB, ( uint8_t )( ( config->preambleLen >> 8 ) & 0xFF ) );
            writeRegister( REG_PREAMBLELSB, ( uint8_t )( config->preambleLen & 0xFF ) );
            
            if( config->fixLen == 1 )
            {
                writeRegister( REG_PAYLOADLENGTH, config->payloadLen );
            }
            else
            {
                writeRegister( REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum 
            }
            
            writeRegister(REG_PACKETCONFIG1, RF_PACKETCONFIG1_DCFREE_WHITENING |
            								(( config->fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
            								(( config->crcOn == 1 ) ? RF_PACKETCONFIG1_CRC_ON : RF_PACKETCONFIG1_CRC_OFF));
            								
            writeRegister(REG_PACKETCONFIG2, 0x40); //enable packet mode
            
            writeRegister(REG_SYNCCONFIG, RF_SYNCCONFIG_SYNCSIZE_4 | RF_SYNCCONFIG_SYNC_ON | RF_SYNCCONFIG_PREAMBLEPOLARITY_AA | RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON);
            writeRegister(REG_SYNCVALUE1, 0xD3);
            writeRegister(REG_SYNCVALUE2, 0x91);
            writeRegister(REG_SYNCVALUE3, 0xDA);
            writeRegister(REG_SYNCVALUE4, 0x26);

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

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( config->power < -1 )
    {
        config->power = -1;
    }
    if( config->power > 14 )
    {
        config->power = 14;
    }
    paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( unsigned char )( ( unsigned short )( config->power + 1 ) & 0x0F );
 
    writeRegister(REG_PACONFIG, paConfig);
    writeRegister(REG_PADAC, paDac);

    switch( config->modem )
    {
    case MODEM_FSK:
        {
            // gaussian shaping, BT = 0.3
            //writeRegister(REG_PARAMP, 0x60);
            config->fdev = (unsigned short)((double)config->fdev / FREQ_STEP);
            writeRegister(REG_FDEVMSB, (unsigned char)(config->fdev >> 8));
            writeRegister(REG_FDEVLSB, (unsigned char)(config->fdev & 0xFF));
            // read back the frequency deviation
            config->fdev = (unsigned int)((double)(((unsigned int)readRegister(REG_FDEVMSB) << 8 ) |
                                                   ((unsigned int)readRegister(REG_FDEVLSB))) * FREQ_STEP);

            config->datarate = (unsigned short)((double)XTAL_FREQ / (double)config->datarate);
            writeRegister(REG_BITRATEMSB, (unsigned char)(config->datarate >> 8 ));
            writeRegister(REG_BITRATELSB, (unsigned char)(config->datarate & 0xFF));
            // read back the datarate
            config->datarate = (unsigned int)((double)(((unsigned int)readRegister(REG_BITRATEMSB) << 8 ) |
                                                   ((unsigned int)readRegister(REG_BITRATELSB))) * (double)XTAL_FREQ);

            writeRegister(REG_PREAMBLEMSB, (config->preambleLen >> 8) & 0x00FF);
            writeRegister(REG_PREAMBLELSB, config->preambleLen & 0xFF);

            writeRegister(REG_PACKETCONFIG1, RF_PACKETCONFIG1_DCFREE_WHITENING |
            								(( config->fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
            								(( config->crcOn == 1 ) ? RF_PACKETCONFIG1_CRC_ON : RF_PACKETCONFIG1_CRC_OFF));
            								
            writeRegister(REG_PACKETCONFIG2, 0x40); //enable packet mode
            
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
    digitalWrite(CS_PIN, LOW);

    // send in the address and value via SPI:
    SPI.transfer(address | 0x80);

    // write the value
    SPI.transfer(value);
  
    // take the CS pin high to de-select the chip:
    digitalWrite(CS_PIN, HIGH);
}

unsigned char SX1276::readRegister(unsigned char address) 
{
    // take the CS pin low to select the chip:
    digitalWrite(CS_PIN, LOW);

    // send in the address
    SPI.transfer(address & 0x7F);

    // read the value
    unsigned char val = SPI.transfer(0);
  
    // take the CS pin high to de-select the chip:
    digitalWrite(CS_PIN, HIGH);

    return val;
}

void SX1276::read(unsigned char address, unsigned char *data, unsigned char size) 
{
    // take the CS pin low to select the chip:
    digitalWrite(CS_PIN, LOW);

    // send in the address
    SPI.transfer(address & 0x7F);

    // read the value
    for (unsigned char i = 0; i < size; i++)
    {
    	data[i] = SPI.transfer(0);
    }
    
    // take the CS pin high to de-select the chip:
    digitalWrite(CS_PIN, HIGH);
}

void SX1276::write(unsigned char address, unsigned char *data, unsigned char size) 
{
    // take the CS pin low to select the chip:
    digitalWrite(CS_PIN, LOW);

    // send in the address
    SPI.transfer(address | 0x80);

    // read the value
    for (unsigned char i = 0; i < size; i++)
    {
    	 SPI.transfer(data[i]);
    }
    
    // take the CS pin high to de-select the chip:
    digitalWrite(CS_PIN, HIGH);
}

void SX1276::writeFifo(unsigned char *buffer, unsigned char size)
{
    write( 0, buffer, size );
}

void SX1276::readFifo(unsigned char *buffer, unsigned char size)
{
    read( 0, buffer, size );
}

void SX1276::send(unsigned char *buffer, unsigned char size)
{
	unsigned long previous = millis();
	
	// write the amount of bytes to send
	//writeFifo( ( uint8_t* )&size, 1 );
	//writeRegister( REG_PAYLOADLENGTH, size );
	
	writeRegister( REG_PAYLOADLENGTH, size );
	
	// write the buffer to the FIFO: FIFO size is only 64 bytes so the chunk must fit!
	writeFifo( buffer, size );
	
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
    
    // turn the transmitter ON
    setOpMode( RF_OPMODE_TRANSMITTER );
        
    while ((digitalRead(DIO0) != HIGH) && (millis() - previous < TIMEOUT))
    {
		// Condition to avoid an overflow (DO NOT REMOVE)
        if( millis() < previous )
        {
            previous = millis();
        }
    }

    sleep();
}

unsigned char SX1276::receive( uint32_t timeout )
{
    bool rxContinuous = false;
    
    //switch( this->settings.Modem )
    //{
    //case MODEM_FSK:
        //{
            //rxContinuous = this->settings.Fsk.RxContinuous;
            
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
            
           /* this->settings.FskPacketHandler.FifoThresh = Read( REG_FIFOTHRESH ) & 0x3F;
            
            this->settings.FskPacketHandler.PreambleDetected = false;
            this->settings.FskPacketHandler.SyncWordDetected = false;
            this->settings.FskPacketHandler.NbBytes = 0;
            this->settings.FskPacketHandler.Size = 0;*/
        //}
    /*    break;
    case MODEM_LORA:
        {
            if( this->settings.LoRa.IqInverted == true )
            {
                Write( REG_LR_INVERTIQ, ( ( Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                Write( REG_LR_INVERTIQ, ( ( Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }         
        

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( this->settings.LoRa.Bandwidth < 9 )
            {
                Write( REG_LR_DETECTOPTIMIZE, Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
                Write( REG_LR_TEST30, 0x00 );
                switch( this->settings.LoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    Write( REG_LR_TEST2F, 0x48 );
                    SetChannel(this->settings.Channel + 7.81e3 );
                    break;
                case 1: // 10.4 kHz
                    Write( REG_LR_TEST2F, 0x44 );
                    SetChannel(this->settings.Channel + 10.42e3 );
                    break;
                case 2: // 15.6 kHz
                    Write( REG_LR_TEST2F, 0x44 );
                    SetChannel(this->settings.Channel + 15.62e3 );
                    break;
                case 3: // 20.8 kHz
                    Write( REG_LR_TEST2F, 0x44 );
                    SetChannel(this->settings.Channel + 20.83e3 );
                    break;
                case 4: // 31.2 kHz
                    Write( REG_LR_TEST2F, 0x44 );
                    SetChannel(this->settings.Channel + 31.25e3 );
                    break;
                case 5: // 41.4 kHz
                    Write( REG_LR_TEST2F, 0x44 );
                    SetChannel(this->settings.Channel + 41.67e3 );
                    break;
                case 6: // 62.5 kHz
                    Write( REG_LR_TEST2F, 0x40 );
                    break;
                case 7: // 125 kHz
                    Write( REG_LR_TEST2F, 0x40 );
                    break;
                case 8: // 250 kHz
                    Write( REG_LR_TEST2F, 0x40 );
                    break;
                }
            }
            else
            {
                Write( REG_LR_DETECTOPTIMIZE, Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
            }

            rxContinuous = this->settings.LoRa.RxContinuous;
            
            if( this->settings.LoRa.FreqHopOn == true )
            {
                Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
                                              
                // DIO0=RxDone, DIO2=FhssChangeChannel
                Write( REG_DIOMAPPING1, ( Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
                                              
                // DIO0=RxDone
                Write( REG_DIOMAPPING1, ( Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            Write( REG_LR_FIFORXBASEADDR, 0 );
            Write( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }
*/
    //memset( rxtxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    //this->settings.State = RF_RX_RUNNING;
    //if( timeout != 0 )
    //{
    //    rxTimeoutTimer.attach_us( this, &SX1276::OnTimeoutIrq, timeout );
    //}

    //if( this->settings.Modem == MODEM_FSK )
    //{
        setOpMode( RF_OPMODE_RECEIVER );
        
        /*if( rxContinuous == false )
        {
            rxTimeoutSyncWord.attach_us( this, &SX1276::OnTimeoutIrq, ( 8.0 * ( this->settings.Fsk.PreambleLen +
                                                         ( ( Read( REG_SYNCCONFIG ) &
                                                            ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
                                                         1.0 ) + 10.0 ) /
                                                        ( double )this->settings.Fsk.Datarate ) * 1e6 );
        }
    }
    else
    {
        if( rxContinuous == true )
        {
            SetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }*/
    //while (digitalRead(DIO0) != HIGH) ;
    
    //while(!(readRegister(REG_IRQFLAGS1) & 0x01));
    //radio.writeRegister(0, f | 0x02);
    return 0;//readRegister(0);
 
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
                                 ((unsigned short)readRegister(REG_FEILSB)       )) * FREQ_STEP);
}