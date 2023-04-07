#include "SSD1306Wire.h"
#include <WiFi.h>
#include "ESPAsyncWebServer.h"

//OLED related variables
#define OLED_ADDR   0x3c
#define OLED_SDA    21//4     //TTGO board without SD Card has OLED SDA connected to pin 4 of ESP32
#define OLED_SCL    22//15    //TTGO board without SD Card has OLED SCL connected to pin 15 of ESP32
#define OLED_RST    16        //Optional, TTGO board contains OLED_RST connected to pin 16 of ESP32
#define SAMPLE_TIME 5000

//WiFi related stuff 
const char* ssid = "Pixelblaze";
const char* password = "123456789";
//192.168.1.34
IPAddress local_IP(192, 168, 4, 4);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
// #define SERVER_PORT 4080
// byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

SSD1306Wire  display(OLED_ADDR, OLED_SDA, OLED_SCL); 
int counter = 0;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);




struct Hall {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

struct LED {
  const uint8_t PIN;
  bool state;
};

Hall hall1 = {13, 0, false};
LED LEDBuiltin = {2, false};

//variables to keep track of the timing of recent interrupts
unsigned long hall_time = 0;  
unsigned long last_hall_time = 0; 


bool edge1 = true;
bool edge2 = true;
unsigned long LastTimeStamp = millis();
float current_speed = 0.0;
uint32_t last_key_pressed = 0;

bool report = false;

void IRAM_ATTR isr() {
  hall_time = millis();
  

  if (hall_time - last_hall_time > 1000){ //debouce logic (only looking at input that happens every X millis())   
    bool edge = digitalRead(hall1.PIN); // true = rising, false = falling 

    if (edge){ // if ISR triggered on rising edge
      hall1.numberKeyPresses++;
      hall1.pressed = true;
      digitalWrite(LEDBuiltin.PIN,HIGH);
      LEDBuiltin.state = true;
      Serial.printf("Rising");
      
    } else{ // if ISR triggered on falling edge   
      hall1.pressed = false;
      digitalWrite(LEDBuiltin.PIN,LOW);
      LEDBuiltin.state = false;
      last_hall_time = hall_time;
      report = true;
      Serial.printf("Falling \n");
    }
  }
}

void initOLED() {
   pinMode(OLED_RST, OUTPUT);
   //Give a low to high pulse to the OLED display to reset it
   //This is optional and not required for OLED modules not containing a reset pin
   digitalWrite(OLED_RST, LOW);
   delay(20);
   digitalWrite(OLED_RST, HIGH);
}
void showOLEDMessage(String line1, String line2, String line3) {
   display.init();                        // clears screen
   display.setFont(ArialMT_Plain_16);
   display.drawString(0, 0, line1);       //  adds to buffer
   display.drawString(0, 20, line2);
   display.drawString(0, 40, line3);
   display.display();                     // displays content in buffer
}


String readSpeed() {
  return String(current_speed);
  //return String(1.8 * bme.readTemperature() + 32);
}

void setup() {
  initOLED();
  Serial.begin(115200);
  delay(1000);
  Serial.print("Board loading...");
  showOLEDMessage("Board loading...","","");
  delay(1000);
   // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet /*, primaryDNS, secondaryDNS*/)) {
    Serial.println("STA Failed to configure");
  } 



  //Connecting to WiFI
  //WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");
  showOLEDMessage("Wifi is connecting...","","");
  
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
    }
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  showOLEDMessage("Connected to the WiFi network","Local ESP32 IP:",WiFi.localIP().toString());
  delay(3000);
  pinMode(hall1.PIN, INPUT_PULLUP);
  // attachInterrupt(hall1.PIN, isr, CHANGE);
  pinMode(LEDBuiltin.PIN, OUTPUT);

  Serial.printf("Setup Complete. Counting...");
  //showOLEDMessage("Setup Complete.\nCounting... ","","");


  server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readSpeed().c_str());
  });

  // Start server
  server.begin();
  showOLEDMessage("Web server started... ","Setup Complete...","");
  delay(3000);
}



void check_speed()
    {
      if( millis() - LastTimeStamp > SAMPLE_TIME)
        {
          //clculate RPM. Good speed is ~100 RPM
          current_speed = 0.0;
          float num_pressed =  float(hall1.numberKeyPresses - last_key_pressed); 
          int time_passed = (millis() - LastTimeStamp); //TIme Passed in milliseconds
 
          current_speed = num_pressed / time_passed * 60000; // speed in RPM
          LastTimeStamp = millis();
          Serial.printf("Num pressed %f \n", num_pressed);
          Serial.printf("Time passed %d \n", time_passed);
          Serial.printf("Current speed is %f RPM \n", current_speed);
          showOLEDMessage("Speed is",String(current_speed) + " RPM","");
          last_key_pressed = hall1.numberKeyPresses;
        }
      
      }

void loop() {
/*  if (report) {
    Serial.printf("Hall sensor has been triggered %u times\n", hall1.numberKeyPresses);
    showOLEDMessage("Hall count:",String(hall1.numberKeyPresses),"");
    report = false;
  }
*/
    
    delay(10);
    bool edge2 = digitalRead(hall1.PIN); // true = rising, false = falling
    if(edge2 != edge1)
        {
        edge1 = edge2;
        if(!edge1) 
            {
            hall1.numberKeyPresses++;
            //showOLEDMessage("Hall count:",String(hall1.numberKeyPresses),"");
            Serial.printf("Hall sensor has been triggered %u times\n", hall1.numberKeyPresses);
            
            //Serial.printf("Current time in millis %u \n", millis());
            }
        
        } 

    check_speed();
    //Serial.printf("Current speed is %f RPM \n", current_speed);

    

    
    //Serial.printf("Hall sensor read is %u \n", edge2);
    
    
    
    
}
