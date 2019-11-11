#include <DSPI.h>
#include <SX1276.h>
#include <CRC16CCITT.h>

#define PA_ENABLE 23

DSPI line;
SX1276 radio(line);
TxConfig_t fsk;
// CRC16-CCITT used by the SX1278
CRC16CCITT crc(0x1D0F, true);

// slave SPI for data
DSPI line2(2);

unsigned char msg[80];

volatile unsigned char msgCounter = -1;

uint8_t onTransmit()
{
  if (msgCounter < 42)
  {
    return msg[msgCounter++];
  }
  return 0x55;
}

void setup()
{
  // set pin as input to avoid clashes
  MAP_GPIO_setAsInputPin( GPIO_PORT_P5, GPIO_PIN1 );
  MAP_GPIO_setAsInputPin( GPIO_PORT_P5, GPIO_PIN2 );

  // preamble
  msg[0] = 0x55;
  msg[1] = 0x55;
  msg[2] = 0x55;
  msg[3] = 0x55;
  msg[4] = 0x55;
  msg[5] = 0x55;
  msg[6] = 0x55;
  msg[7] = 0x55;
  msg[8] = 0x55;
  msg[9] = 0x55;
  msg[10] = 0x55;
  msg[11] = 0x55;
  msg[12] = 0x55;
  msg[13] = 0x55;
  msg[14] = 0x55;
  // synch word
  msg[15] = 0xD3;
  msg[16] = 0x91;
  msg[17] = 0xDA;
  msg[18] = 0x26;
  // packet size
  msg[19] = 20;
  // packet number
  msg[20] = 0x55;
  // data
  msg[21] = 0xFF;
  msg[22] = 0xAA;
  msg[23] = 0x55;
  msg[24] = 0x33;
  msg[25] = 0x55;
  msg[26] = 0x33;
  msg[27] = 0x55;
  msg[28] = 0x33;
  msg[29] = 0x55;
  msg[30] = 0x33;
  msg[31] = 0x55;
  msg[32] = 0x33;
  msg[33] = 0x55;
  msg[34] = 0x33;
  msg[35] = 0x55;
  msg[36] = 0x33;
  msg[37] = 0x55;
  msg[38] = 0x33;
  msg[39] = 0x55;
  msg[40] = 0x5D;
  msg[41] = 0xF2;
  
  Serial.begin(115200);

  // GMSK:
  // Modem set to FSK, deviation set to 1/2 datarate, gaussian filter enabled
  fsk.modem = MODEM_FSK;
  fsk.filtertype = BT_0_5;
  fsk.power = 14;
  fsk.fdev = 1200;
  fsk.datarate = 2400;
  fsk.preambleLen = 5;
  fsk.payloadLen = 24;
  fsk.whitening = false;
  fsk.fixLen = false;
  fsk.crcOn = true;

  radio.init();
 
  bool found = radio.ping();

  if (found)
  {
    Serial.println("Found!");
  }
  else
  {
    Serial.println("Not Found");
  }

  radio.setFrequency(145900000);
  //radio.setFrequency(436000000);
  radio.enableBitMode(line2, 0, onTransmit);
  radio.setTxConfig(&fsk);

  pinMode(PA_ENABLE, OUTPUT);
}

void loop()
{        
  digitalWrite(PA_ENABLE, HIGH);

  // set radio ON
  radio.setIdleMode( true );
  
  // 5ms delay for PA to settle
  delay(5);

  for (unsigned char k = 0; k < 5; k++)
  {
    msg[20] = k;
    crc.init();
    for(unsigned char i = 19; i < 40; i++)
    {
      crc.newChar(msg[i]);
    }
    unsigned short valculatedCRC = crc.getCRC();
    msg[40] = (valculatedCRC >> 8) & 0xFF;
    msg[41] = valculatedCRC & 0xFF;
    msgCounter = 0;
    while(msgCounter < 42);
  }

  // 10ms delay to allow the signal to be flushed
  // this gives about 1.6ms of margin after the last
  // packet is over
  delay(10);
  
  // set radio OFF
  radio.setIdleMode( false );

  digitalWrite(PA_ENABLE, LOW);

  delay(1000);
}

