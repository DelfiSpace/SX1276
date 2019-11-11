#include <DSPI.h>
#include <SX1276.h>

DSPI line;
// slave SPI for data
DSPI line2(2);

SX1276 radio(line);
RxConfig_t fsk;

#define RX_BUFFER_SIZE 255
unsigned char rxtxBuffer[RX_BUFFER_SIZE];

unsigned char count = 0;
void onReceive(uint8_t data)
{
  Serial.print((data & 0x80) != 0);
  Serial.print((data & 0x40) != 0);
  Serial.print((data & 0x20) != 0);
  Serial.print((data & 0x10) != 0);
  Serial.print((data & 0x08) != 0);
  Serial.print((data & 0x04) != 0);
  Serial.print((data & 0x02) != 0);
  Serial.print((data & 0x01) != 0);

  count++;
  if (count == 8)
  {
    count = 0;
    Serial.println();
  }
}

void setup()
{

// set pin as input to avoid clashes
  MAP_GPIO_setAsInputPin( GPIO_PORT_P5, GPIO_PIN1 );
  MAP_GPIO_setAsInputPin( GPIO_PORT_P5, GPIO_PIN2 );
  
  Serial.begin(115200);

  // GMSK:
  // Modem set to FSK, deviation set to 1/2 datarate, gaussian filter enabled
  fsk.modem = MODEM_FSK;
  fsk.filtertype = BT_0_5;
  fsk.bandwidth = 15000;
  fsk.bandwidthAfc = 83333;
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
  radio.RxChainCalibration();
  radio.enableBitMode(line2, onReceive, 0);
  radio.setRxConfig(&fsk);

  pinMode(GREEN_LED, OUTPUT);
  
  Serial.println("RX ready...");

   unsigned char a = radio.startReceiver();
}

void loop()
{       

}

