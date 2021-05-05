#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

#define MEASUREMENT_PERIOD 5000
#define SENDING_THRESHOLD 50

// Since the lowest number to accessible for the client is 2,
// client_address 2 corresponds with tap 1
#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1

#define PIN_SENSOR 10
#define PIN_3V3_ENABLE 8

//modem
// Singleton instance of the radio driver
RH_RF95 driver(6, 2);
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// flow rate
volatile uint16_t pulseCount;
float calibrationFactor = 98;
uint16_t totalMilliLitres;
unsigned long oldTime = 0;

//ISR
bool prevPulsState = true;

// Making an interrupt pin from pin 10
ISR(PCINT0_vect)
{                                                 //  pin change interrupt for D8 to D13
  PCIFR |= bit(digitalPinToPCICRbit(PIN_SENSOR)); // clear any outstanding interrupt
  bool currentState = (bool)digitalRead(PIN_SENSOR);
  if (prevPulsState && !currentState)
  {
    pulseCount++;
  }
  prevPulsState = currentState;
}

bool sendData(uint8_t *data, uint8_t len);
void calculatingFlow(void);

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ; // Wait for serial port to be available

  Serial.println(F("I/O init."));
  // Dramco uno - enable 3v3 voltage regulator
  pinMode(PIN_3V3_ENABLE, OUTPUT);
  digitalWrite(PIN_3V3_ENABLE, HIGH);

  //Setup Interuptpin 10
  //pinMode(PIN_SENSOR, INPUT_PULLUP);
  pinMode(PIN_SENSOR, INPUT);
  *digitalPinToPCMSK(PIN_SENSOR) |= bit(digitalPinToPCMSKbit(PIN_SENSOR)); // enable pin change interrupt
  PCIFR |= bit(digitalPinToPCICRbit(PIN_SENSOR));                          // clear any outstanding interrupt
  PCICR |= bit(digitalPinToPCICRbit(PIN_SENSOR));                          // enable interrupt for the group

  Serial.println(F("Modem init."));
  if (!manager.init())
  {
    Serial.println("init failed");
  }

  driver.setFrequency(868);
  manager.setRetries(0);
  manager.setTimeout(100);

  Serial.println(F("Init variables"));
  pulseCount = 0;
  totalMilliLitres = 0;
  oldTime = 0;

  Serial.println(F("Setup complete."));
}

void loop()
{
  // Only process counters once per measurement_period
  if ((millis() - oldTime) > MEASUREMENT_PERIOD)
  {
    oldTime = millis();
    calculatingFlow();
  }

  // Each time the threshold is passed, send the totalMilliliters to the server arduino
  if (totalMilliLitres >= SENDING_THRESHOLD)
  {
    uint16_t totalMilliLitres_temp = totalMilliLitres;
    uint8_t data[2] = {totalMilliLitres_temp, (totalMilliLitres_temp >> 8)};
    if (sendData(data, sizeof(data)))
    {
      totalMilliLitres -= totalMilliLitres_temp;
    }
  }
}

bool sendData(uint8_t *data, uint8_t len)
{
  Serial.println(F("Sending to rf95_server"));
  // Send a message to manager_server
  if (manager.sendtoWait(data, len, SERVER_ADDRESS))
  {
    Serial.println(F("Data sent, ACK received"));
    return true;
  }
  else
  {
    Serial.println(F("No reply, is rf95_server running?"));
    return false;
  }
}

void calculatingFlow(void)
{
  //Store counter temporarily
  uint16_t pulseCount_temp = pulseCount;
  // Reset the pulse counter so we can start incrementing again
  pulseCount = 0;

  //apply the calibrationFactor to scale the output based on the number of pulses per measurement_period per units of measure
  // (litres/minute in this case) coming from the sensor.
  float flowRate = (float)(pulseCount_temp / calibrationFactor);

  // Divide the flow rate in litres/minute by 60 to determine how many litres have
  // passed through the sensor in this 1 second interval, then multiply by 1000 to
  // convert to millilitres.
  float flowMilliLitres = (flowRate * 1000 * MEASUREMENT_PERIOD) / 60000;

  // Add the millilitres passed in this second to the cumulative total
  totalMilliLitres += (uint16_t)round(flowMilliLitres);

  // Print the cumulative total of litres flowed since starting
  Serial.print("Liquid Quantity: ");
  Serial.print(totalMilliLitres);
  Serial.println("mL");
}