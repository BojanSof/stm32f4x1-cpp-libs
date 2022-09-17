#include <SPI.h>

static const uint8_t masterRequestData = 0xAB;
static volatile uint8_t slaveData = 1;
static volatile uint8_t data = 0;
static volatile bool dataReceived = false;

void setup()
{
  Serial.begin(115200);   // set baud rate to 115200 for usart
  Serial.println("Arduino UNO as SPI slave. Connect to STM32F401 master.");
  pinMode(MISO, OUTPUT); // have to send on Master in so it set as output
  SPCR |= _BV(SPE); // turn on SPI in slave mode
  SPI.attachInterrupt();     // turn on interrupt
}

void loop(){
  if (dataReceived)
  {
    Serial.print("Received data from master:");
    Serial.println(data, HEX);
    Serial.print("Data sent to the master:");
    Serial.println(slaveData, HEX);
    dataReceived = false;
  }
}

// Interrupt function
ISR (SPI_STC_vect) 
{
  data = SPDR;        // read byte from SPI Data Register
  if(data == masterRequestData)
  {
    SPDR = slaveData++;
  }
  dataReceived = true;
}
