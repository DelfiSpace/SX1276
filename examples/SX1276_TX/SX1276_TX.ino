#include <DSPI.h>
#include <SX1276.h>

#define PA_ENABLE 23

DSPI line;
SX1276 radio(line);
TxConfig_t fsk;

unsigned char msg[80];

void setup()
{
  msg[0] = 0x55;
  msg[1] = 0xFF;
  msg[2] = 0xAA;
  msg[3] = 0x55;
  msg[4] = 0x33;
  msg[5] = 0x55;
  msg[6] = 0x33;
  msg[7] = 0x55;
  msg[8] = 0x33;
  msg[9] = 0x55;
  msg[10] = 0x33;
  msg[11] = 0x55;
  msg[12] = 0x33;
  msg[13] = 0x55;
  msg[14] = 0x33;
  msg[15] = 0x55;
  msg[16] = 0x33;
  msg[17] = 0x55;
  msg[18] = 0x33;
  msg[19] = 0x55;

  Serial.begin(115200);

  // GMSK:
  // Modem set to FSK, deviation set to 1/2 datarate, gaussian filter enabled
  fsk.modem = MODEM_FSK;
  fsk.filtertype = BT_0_5;
  fsk.power = 14;
  fsk.fdev = 1200;
  fsk.datarate = 2400;
  fsk.preambleLen = 5;
  fsk.payloadLen = 20;
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

  for(unsigned char k = 0; k < 20; k++)
  {
    //msg[0] = k;
    if ( !radio.send(msg, fsk.payloadLen) )
    {
          Serial.println("TX error!");
    }
  }

  // 5ms delay to allow the signal to be flushed
  delay(5);
  
  // set radio OFF
  radio.setIdleMode( false );

  digitalWrite(PA_ENABLE, LOW);

  delay(1000);
}

