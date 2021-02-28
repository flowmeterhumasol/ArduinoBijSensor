
int sensorPin=2; 
unsigned long period; // tijd tussen 2 pulsen 

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	pinMode(sensorPin,INPUT);
}
void loop() {
	// put your main code here, to run repeatedly:

	period = (pulseIn(sensorPin,LOW)); //bij active high 
	if (period){
		Serial.print("Periode: ");
		Serial.println(period);  
	}

}
