#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#define PTP_CLIENT

//Variables flowsensor
byte PIN_SENSOR   = 10;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 98;
volatile uint16_t pulseCount;  
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;
uint8_t tapNumber[]= "01"; 

//Interupt bools
bool prevPulsState = true; 
bool FlowPuls = false; 

// Singleton instance of the radio driver
RH_RF95 rf95(6, 2);

void setup() 
{
  // Dramco uno - enable 3v3 voltage regulator
  //%%%%%%%% Heb ik da nodig k denk van ni dus efkes weggedaan 
  //pinMode(8, OUTPUT);
  //digitalWrite(8, HIGH);

  Serial.begin(9600);
  while(!Serial) ; // Wait for serial port to be available
  if(!rf95.init())
    Serial.println("init failed");
    rf95.setFrequency(868);
  
  //Setup flowsensor:
  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;

  //Setup Interuptpin 10 
  //pinMode(PIN_SENSOR, INPUT_PULLUP);
  pinMode(PIN_SENSOR, INPUT);
  *digitalPinToPCMSK(PIN_SENSOR) |= bit (digitalPinToPCMSKbit(PIN_SENSOR));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(PIN_SENSOR)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(PIN_SENSOR)); // enable interrupt for the group
 
}

#warning "compiling lora ptp client code"
void loop()
{
  //if((millis() - oldTime) > 1000)    // Only process counters once per second
  if((millis() - oldTime) > 60000)    // Only process counters once per minute 
  { 
    // Store time and counter temporarly and reset oldTime,
    // Because of this backup it is not nescesary to stop interrupts during the calculation 
    uint16_t tijdsverschil= millis() - oldTime;
    uint16_t pulseCount_temp = pulseCount; 
    oldTime = millis();

    // Reset the pulse counter so we can start incrementing again 
    pulseCount = 0;
  
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    //flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount_temp) / calibrationFactor;
    flowRate = ((60000.0 / tijdsverschil) * pulseCount_temp) / calibrationFactor;  
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;
    
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
      
    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");        
    Serial.print(totalMilliLitres);
    Serial.println("mL"); 
    Serial.print("\t");       // Print tab space
  }


  if ( totalMilliLitres > 3000 ){// per 3 liter zenden 
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server 
  rf95.send(tapNumber, sizeof(tapNumber));

  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Succes ");
      totalMilliLitres -= 3000;    
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