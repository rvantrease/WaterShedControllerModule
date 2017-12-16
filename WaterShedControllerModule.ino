// Based on Adafruits RadioHead fork https://github.com/adafruit/RadioHead
// -*- mode: C++ -*-
// WaterShedControllerModule
// Transmits messages about water shed sensor status

/* Modifications
    12/09/2017 - RLV (tested and approved)
      Initial cleanup of code
        Removed non-needed boards
          ATmega32U4, AVR_ATmega328P, ESP8266, ESP32
        Frequency 915.0Mhz
    12/10/2017 - RLV 
      Modified to allow more than one HC-SR04 sensor
*/

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/************ HC-SR04 Setup ***************/
#define trigPin_1 13
#define echoPin_1 12
#define delayLoop 5000  //delay in milliseconds

/************ Radio Setup ***************/
#define RF69_FREQ 915.0

// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     2

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  // HC-SR04 Setup
  pinMode(trigPin_1, OUTPUT);
  pinMode(echoPin_1, INPUT);

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

void loop() {
  delay(delayLoop);  // Wait 1 second between transmits, could also 'sleep' here!

  char myStr[10];
  testfunc(myStr);
  Serial.println(myStr);

  float pingInches = ping_In();
  int gallons = tankGallons(pingInches);

  char radiopacket[40] = "Gallons remaining ";

  itoa(gallons, radiopacket+18, 10);
  //itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  
  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      buf[len] = 0; // zero out remaining string
      
      Serial.print("Got reply from #"); Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.print("] : ");
      Serial.println((char*)buf);     
      Blink(LED, 100, 3); //blink LED 3 times, 100ms between blinks
    } else {
      Serial.println("No reply, is anyone listening?");
    }
  } else {
    Serial.println("Sending failed (no ack)");
  }
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

float ping_In() {
  float duration, distance, distance2, magicnumber;
  digitalWrite(trigPin_1, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin_1, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin_1, LOW);
  duration = pulseIn(echoPin_1, HIGH);
  //Formula 29.1 is a magic number, not sure where it came from, but it appears to work
  //Speed of sound is 343 m/s
  //2.54 cm in an inch
  //Divide by 2 because it is a roundtrip
  magicnumber = 29.1*2.54*2;
  distance = duration/magicnumber;
  return distance;
}

int tankGallons(float inchesFromTop) {
    /*  Used to convert inches to gallons for a 275 gallon IBC tote.  Tote
     *  is 46 inches from ground to top.  Estimating 4 inches to bottom of 
     *  tote.  Leaving 42 inches for 275 gallons.  Will need some fine tuning
     *  to accomodate curvature at top and bottom.
     */

    float tankHeight = 42;
    float tankCapacity = 275.0;

    float gallonsPerInch = tankCapacity/tankHeight;

    return tankCapacity - (gallonsPerInch * inchesFromTop);
}

//https://stackoverflow.com/questions/5660527/how-do-i-return-a-char-array-from-a-function?noredirect=1&lq=1
void testfunc(char* outStr){
  //outStr = "testing";
  char str[10] = "testing";
  //for(int i=0; i < 10; ++i){
  //  outStr[i] = str[i];
  //}
  //outStr = str;
  Serial.println(str);
//  strncpy(str, outStr);
}

