// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W 

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

#define PTP_CLIENT

// Singleton instance of the radio driver
RH_RF95 rf95(6, 2);
unsigned long totalLiters = 67;
unsigned int kraannmr = 0;
String message; 


void setup() 
{
  // Dramco uno - enable 3v3 voltage regulator
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  Serial.begin(9600);
  while(!Serial) ; // Wait for serial port to be available
  if(!rf95.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    rf95.setFrequency(868);
  
}


#warning "compiling lora ptp client code"
void loop()
{
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  sprintf(message,"%d:%f", kraannmr,totalLiters);
  if (strlen(message)<RH_RF95_MAX_MESSAGE_LEN)
  //rf95.send((uint8_t *)message, sizeof(message));
  uint8_t data[] =  atoi(message.c_str ());

  //uint8_t data[] = "Hello World!";
  rf95.send(data, sizeof(data));

  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);    
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
  delay(400);
}