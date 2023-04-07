#include "SSD1306Wire.h"
#include <WiFi.h> // ESP32 WiFi Library
#include <WebServer.h> // WebServer Library for ESP32
#include <ArduinoWebsockets.h>
#include <HTTPClient.h>

//OLED related variables
#define OLED_ADDR   0x3c
#define OLED_SDA    21//4     //TTGO board without SD Card has OLED SDA connected to pin 4 of ESP32
#define OLED_SCL    22//15    //TTGO board without SD Card has OLED SCL connected to pin 15 of ESP32
#define OLED_RST    16        //Optional, TTGO board contains OLED_RST connected to pin 16 of ESP32
#define SAMPLE_TIME 5000
#define CONNECTION_REFRESH_TIME 500

//WiFi related stuff
const char* ssid = "Pixelblaze";
const char* password = "123456789";
const char* websockets_connection_string = "ws://192.168.4.1:81"; //Enter server adress


float br_0 = 0.05;
float br_1 = 0.0;

using namespace websockets;
WebsocketsClient client;



//Configuring HTTP client to read speed from another board
HTTPClient http;




void onMessageCallback(WebsocketsMessage message) {
  Serial.print("Got Message: ");
  Serial.println(message.data());
}


void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Connnection Closed");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
  }
}

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


bool edge1 = true;
bool edge2 = true;
unsigned long LastTimeStamp = millis();
unsigned long LastHTTP_connection = millis();
float current_speed = 0.0;
float current_speed_bike_2 = 0.0;
uint32_t last_key_pressed = 0;

bool report = false;

void IRAM_ATTR isr() {
  hall_time = millis();


  if (hall_time - last_hall_time > 1000) { //debouce logic (only looking at input that happens every X millis())
    bool edge = digitalRead(hall1.PIN); // true = rising, false = falling

    if (edge) { // if ISR triggered on rising edge
      hall1.numberKeyPresses++;
      hall1.pressed = true;
      digitalWrite(LEDBuiltin.PIN, HIGH);
      LEDBuiltin.state = true;
      Serial.printf("Rising");

    } else { // if ISR triggered on falling edge
      hall1.pressed = false;
      digitalWrite(LEDBuiltin.PIN, LOW);
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

  //Connecting to WiFI
  WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

  pinMode(hall1.PIN, INPUT_PULLUP);
  // attachInterrupt(hall1.PIN, isr, CHANGE);
  pinMode(LEDBuiltin.PIN, OUTPUT);

  Serial.printf("Setup Complete. Counting...");
  showOLEDMessage("Setup Complete.\nCounting... ", "", "");

  // run callback when messages are received
  client.onMessage(onMessageCallback);

  // run callback when events are occuring
  client.onEvent(onEventsCallback);

  // Before connecting, set the ssl fingerprint of the server
  //client.setCACert(echo_org_ssl_ca_cert);

  

  // Connect to server
  client.connect(websockets_connection_string);
  // TODO. Handle the case when connection to PixelBlaze is not successfull first time.
  Serial.printf("..........\n");

  // Send a message
  //client.send("Hello Server");

  // Send a ping
  client.ping();
}



void check_speed()
{
  if ( millis() - LastTimeStamp > SAMPLE_TIME)
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
    showOLEDMessage("Speed is", String(current_speed) + " RPM", "");
    last_key_pressed = hall1.numberKeyPresses;


  }

}

float map_rpm_to_brightness(float RPM1, float RPM2)
{
  float brightness = 0.0 + 0.9 * (RPM1 + RPM2) / (300.0);
  //Serial.printf("First bike speed is  %f\n", RPM1);
  //Serial.printf("Second bike speed is  %f\n", RPM2);

  return brightness;

}

void loop() {
  /*  if (report) {
      Serial.printf("Hall sensor has been triggered %u times\n", hall1.numberKeyPresses);
      showOLEDMessage("Hall count:",String(hall1.numberKeyPresses),"");
      report = false;
    }
  */

  delay(10);

  //Counting clicks of the magnetic sensor
  //TODO. Danny to rewrite it in the way it is low level (interupts based) and efficient
  bool edge2 = digitalRead(hall1.PIN); // true = rising, false = falling
  if (edge2 != edge1)
  {
    edge1 = edge2;
    if (!edge1)
    {
      hall1.numberKeyPresses++;
      //showOLEDMessage("Hall count:",String(hall1.numberKeyPresses),"");
      //Serial.printf("Hall sensor has been triggered %u times\n", hall1.numberKeyPresses);
    }

  }

  //Checking this bike speed
  check_speed();


  // Reading speed from another board over HTTP
  if ( millis() - LastHTTP_connection > CONNECTION_REFRESH_TIME)
  {

    LastHTTP_connection = millis();
    http.begin("http://192.168.4.4/speed");
    int httpCode = http.GET();

    if (httpCode > 0)
    {
      String payload = http.getString();
      //Serial.println("HTTP response code from slave board:");
      //Serial.println(httpCode);
      // Serial.println("Payload (RPM speed) is:");
      // Serial.println(payload);
      current_speed_bike_2 = payload.toFloat();

    }
    else
    {
      Serial.println("Error on HTTP request");
    }
    http.end();
  }


  //Checking the current desired brightness level
  br_1 = map_rpm_to_brightness(current_speed, current_speed_bike_2);

  //only adjust brightness if brightness level changed
  if (br_1 != br_0)
  {
    Serial.printf("First bike speed is  %f\n", current_speed);
    Serial.printf("Second bike speed is  %f\n", current_speed_bike_2);

    //adjusting the brightness of Pixelblaze controller
    adjust_brightness(br_0, br_1);
    br_0 = br_1;
  }

}


void adjust_brightness(float old_brightness, float new_brightness)
{

    
    //Serial.println("Websocket connection status:");
    //Serial.println(client.available());
    while (!client.available())
          {
            //re-establish connection
            Serial.println("Websocket connection is down...");
            Serial.println("Attempting to reconnect...");
            client.connect(websockets_connection_string);
            Serial.printf("..........\n");
            // Send a ping
            client.ping();
          }
  String send_str  = "";
  send_str.concat("{ \"brightness\": ");
  send_str.concat(new_brightness);
  send_str.concat(", \"save\": false}");
  //client.poll();
  //Serial.println(send_str);
  client.send(send_str);


}
