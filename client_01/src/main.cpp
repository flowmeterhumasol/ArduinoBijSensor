#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1

byte PIN_SENSOR   = 10;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 98;
volatile uint16_t pulseCount;  
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;

// Singleton instance of the radio driver
RH_RF95 driver(6, 2);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

//ISR
bool prevPulsState = true; 
bool FlowPuls = false; 
bool busyCalculating= false; 
bool busySending= false ;  

void setup() 
{
  // Dramco uno - enable 3v3 voltage regulator
 pinMode(8, OUTPUT);
digitalWrite(8, HIGH);

  Serial.begin(9600);
  while(!Serial) ; // Wait for serial port to be available
  if(!manager.init())
    Serial.println("init failed");
  driver.setFrequency(868);

  //Setup Interuptpin 10 
  //pinMode(PIN_SENSOR, INPUT_PULLUP);
  pinMode(PIN_SENSOR, INPUT);
  *digitalPinToPCMSK(PIN_SENSOR) |= bit (digitalPinToPCMSKbit(PIN_SENSOR));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(PIN_SENSOR)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(PIN_SENSOR)); // enable interrupt for the group

  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;
}


void CalculatingFlow()
{
    //Store counter temporarily
    uint16_t pulseCount_temp = pulseCount;

    // Reset the pulse counter so we can start incrementing again 
    pulseCount = 0;
  
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    //flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount_temp) / calibrationFactor;   // calculating per seconde 
    flowRate = ((60000.0 / (millis() - oldTime)) * pulseCount_temp) / calibrationFactor; // calculating per minute


    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTime = millis();
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
   // flowMilliLitres = (flowRate / 60) * 1000;   // calculating per seconde 
    flowMilliLitres = flowRate  * 1000;         // calculating per minute
    
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
      
    
    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");        
    Serial.print(totalMilliLitres);
    Serial.println("mL"); 
}

void Sending()
{
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  uint8_t data[] = "data";

  // Send a message to manager_server
  if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
  {
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
    uint8_t from;   

 if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
      Serial.print("got reply from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      totalMilliLitres -=20; 
    } 
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is rf95_server running?");
  }  
}

// Making an interrupt pin from pin 10 
ISR (PCINT0_vect){ //  pin change interrupt for D8 to D13
  PCIFR  |= bit (digitalPinToPCICRbit(PIN_SENSOR)); // clear any outstanding interrupt
  bool currentState = (bool)digitalRead(PIN_SENSOR);
  if(!FlowPuls && prevPulsState){
    if(!currentState){
      pulseCount++;
      FlowPuls = true; 
    }
  }
  // Only nescessary for debouncing (when making puls by hand)
  else if (FlowPuls && !prevPulsState){
    if(currentState){
      FlowPuls = false; 
    }
  }
  prevPulsState = currentState;
}

#warning "compiling lora ptp client code"
void loop()
{
 //if((millis() - oldTime) > 1000 && !busySending)    // Only process counters once per second
 if((millis() - oldTime) > 60000 && !busySending)    // Only process counters once per minute 
  {
    busyCalculating= true;
    CalculatingFlow();
    busyCalculating= false;
  }

  if (totalMilliLitres > 20 && !busyCalculating) // send message each 20ml
  {
    //noInterrupts ();
    busySending= true ;
    Sending(); 
    //interrupts ();
    busySending= false ;
  }
}



