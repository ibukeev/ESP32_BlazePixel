#include "SSD1306Wire.h"

//OLED related variables
#define OLED_ADDR   0x3c
#define OLED_SDA    21//4     //TTGO board without SD Card has OLED SDA connected to pin 4 of ESP32
#define OLED_SCL    22//15    //TTGO board without SD Card has OLED SCL connected to pin 15 of ESP32
#define OLED_RST    16        //Optional, TTGO board contains OLED_RST connected to pin 16 of ESP32

SSD1306Wire  display(OLED_ADDR, OLED_SDA, OLED_SCL); 
int counter = 0;



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
      
    } else{ // if ISR triggered on falling edge   
      hall1.pressed = false;
      digitalWrite(LEDBuiltin.PIN,LOW);
      LEDBuiltin.state = false;
      last_hall_time = hall_time;
      report = true;
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

void setup() {
  initOLED();
  Serial.begin(115200);
  pinMode(hall1.PIN, INPUT_PULLUP);
  attachInterrupt(hall1.PIN, isr, CHANGE);
  pinMode(LEDBuiltin.PIN, OUTPUT);

  Serial.printf("Setup Complete. Counting...");
  showOLEDMessage("Setup Complete.\nCounting... ","","");
}

void loop() {
  if (report) {
    Serial.printf("Hall sensor has been triggered %u times\n", hall1.numberKeyPresses);
    showOLEDMessage("Hall count:",String(hall1.numberKeyPresses),"");
    report = false;
  }
}