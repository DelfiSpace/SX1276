#include <DSPI.h>
#include <SX1276.h>

DSPI line;

SX1276 radio(line);
RxConfig_t fsk;

#define RX_BUFFER_SIZE 255
unsigned char rxtxBuffer[RX_BUFFER_SIZE];
unsigned short counter;

void dataReceived(unsigned char size)
{
  // Data received!
  digitalWrite(GREEN_LED, HIGH);
  Serial.print(counter, DEC);
  counter++;
  Serial.print(": ");

  for(int i = 0; i < size; i++)
  {
    Serial.print(rxtxBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  signed short fe = radio.getFrequencyError();
  signed short r = radio.GetRssi(MODEM_FSK);
  
  Serial.print("RSSI: ");
  Serial.print(r, DEC);
  Serial.println(" dBm");
  Serial.print("Frequency: ");
  Serial.print(fe, DEC);
  Serial.println(" Hz");
  Serial.println();
  digitalWrite(GREEN_LED, LOW);
}

void setup()
{
  // set pin as input to avoid clashes
  MAP_GPIO_setAsInputPin( GPIO_PORT_P5, GPIO_PIN1 );
  MAP_GPIO_setAsInputPin( GPIO_PORT_P5, GPIO_PIN2 );
  
  counter = 0;
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
  radio.setRxConfig(&fsk);

pinMode(GREEN_LED, OUTPUT);
  Serial.println("RX ready...");
  
  unsigned char a = radio.startReceiver();
}

void loop()
{       
  unsigned char rx = radio.getRXData(rxtxBuffer, RX_BUFFER_SIZE);
  if (rx)
  {
    dataReceived(rx);
  }

}

