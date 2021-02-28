
volatile int pulseCount;  
unsigned int sensorPin=2; 
unsigned int interruptPin=0;                  //sensorPin=2 komt overeen met interruptPin0
unsigned long flowRate;
unsigned long currentTime;
unsigned long loopTime;                       //Tijdstip van de vorige doorlopen ifloop
unsigned int calibrationFactor=98;            // 98 pulsen per seconde voor elke L vloeistof pet minuut 
unsigned long flowMilliLitres ;
unsigned long totalMilliLitres;

void flow(){	//Bij elke interrupt wordt deze methode opgeroepen (dit is de pulsecounter)
	pulseCount++; 
}

void setup() {
	Serial.begin(9600);
	pinMode(sensorPin,INPUT);
	digitalWrite(sensorPin, HIGH);                //interne pullup 
 
  pulseCount =0;
  flowRate =0.0;
  flowMilliLitres =0.0;
  totalMilliLitres=0.0;
  
	attachInterrupt(interruptPin, flow, FALLING); //Geeft interrupt bij falling puls
	currentTime = millis(); 
	loopTime = currentTime;     
}
void loop() {
	currentTime = millis(); 
	if ( currentTime >= (loopTime+1000)){	        // +_ Elke seconde rekenen en printen
		detachInterrupt(interruptPin);              // interrupt disablen zodat ge flow rate kunt berekenen ( millis stopt ook met tellen dan) 
   
	  //(Correctiefactor (om exact per seconde te rekenen)* pulseCount/ calibrationfactor 
		flowRate = ((1000.0/(currentTime-loopTime)*pulseCount) /calibrationFactor);          
		flowMilliLitres = (flowRate / 60) * 1000;   // berekenen hoeveel milliliter er in deze seconde vloeiden 
		totalMilliLitres += flowMilliLitres; 
		loopTime = currentTime;                     // reset looptime 
		pulseCount = 0 ;                             // reset frequentie counter
		Serial.print("flowMilliLitres: ");
		Serial.println(flowMilliLitres,DEC); 
		Serial.print("TotMili:"); 
		Serial.println(totalMilliLitres,DEC);
    attachInterrupt(interruptPin, flow, FALLING);

	}}
