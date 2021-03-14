/*
Liquid flow rate sensor -DIYhacking.com Arvind Sanjeev

Measure the liquid/water flow rate using this code. 
Connect Vcc and Gnd of sensor to arduino, and the 
signal line to arduino digital pin 2.
 
 */
#include <Arduino.h>

byte statusLed    = 13;
byte PIN_SENSOR   = 10;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 98;
volatile uint16_t pulseCount;  
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;

bool prevPulsState = true; 
bool FlowPuls = false; 

void setup()
{
  
  // Initialize a serial connection for reporting values to the host
  Serial.begin(9600);
   
  // Set up the status LED line as an output
  //pinMode(statusLed, OUTPUT);
  //digitalWrite(statusLed, HIGH);  // We have an active-low LED attached
  
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

void loop()
{
   // Print pulscount
  /*  Serial.print("Pulsecount: ");        
    Serial.println(pulseCount);
    delay(1000);
   */
   
  if((millis() - oldTime) > 1000)    // Only process counters once per second
  //if((millis() - oldTime) > 60000)    // Only process counters once per minute 
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
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount_temp) / calibrationFactor;   // calculating per seconde 
    //flowRate = ((60000.0 / (millis() - oldTime)) * pulseCount_temp) / calibrationFactor; // calculating per minute


    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTime = millis();
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;   // calculating per seconde 
    //flowMilliLitres = flowRate  * 1000;         // calculating per minute
    
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
      
    
    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");        
    Serial.print(totalMilliLitres);
    Serial.println("mL"); 
    Serial.print("\t");       // Print tab space

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
