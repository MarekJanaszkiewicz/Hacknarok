//Libraries
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
//#include "Bitmaps.h"
#include "PMS.h"
#include "string.h"
#include <QMC5883LCompass.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp(&Wire1); // I2C
QMC5883LCompass compass;

#define TRIG_PIN1 2
#define ECHO_PIN1 3
#define TRIG_PIN2 4
#define ECHO_PIN2 5
#define TRIG_PIN_DIST 6
#define ECHO_PIN_DIST 7

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define SerialOutput 0;

//Screen initialisation
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);

const char* ssid = "KPT-Conference"; // Replace with your WiFi SSID
const char* password = "E2ue6Tm&"; // Replace with your WiFi password

// Replace with your actual API key and city ID from OpenWeatherMap
const char* apiKey = "5b28672779bd07f1972832766f185d40";
const char* lat = "64.1355"; // Latitude for Reykjavik
const char* lon = "-21.8954"; // Longitude for Reykjavik
const char* serverName = "http://api.openweathermap.org/data/2.5/weather";

WiFiClient client;

//Global variables
const int SDA_PIN = 0;
const int SCL_PIN = 1;
const int s1Pin = 10;
const int s2Pin = 11;
const int keyPin = 12;
short screen = 0;
short subScreen = 0;
short int numberOfScreens = 6;
bool screenChangedFlag = false;
bool weightConfigurationFlag = false;
short int currentScreen = 0;
int s1 = 0;
int s2 = 0;
int refreshTime = 2000000;
//Time oriented variables
volatile unsigned long s1TriggerTimeBegin = micros();
volatile unsigned long s1TriggerTimeEnd = micros();
volatile unsigned long s2TriggerTimeBegin = micros();
volatile unsigned long s2TriggerTimeEnd = micros();
volatile unsigned long keyTriggerTimeBegin = micros();
volatile unsigned long keyTriggerTimeEnd = micros();
volatile unsigned long refreshTimePrevious = micros();
long distance_dist;

float temperature = 0;
const char* weatherMain = 0;
float windSpeed = 0;
int windDeg = 0;
int visibility = 0;

int Boat_lift = 5000;
int Boat_height_no_load = 70;

int PM10 = 0;
int PM10_ALLERT = 150;
float Temperature;
float Pressure;

static const uint32_t PMS_READ_DELAY = 500;
static const uint32_t PMS_READ_INTERVAL = 4500;
uint32_t timerInterval = PMS_READ_DELAY;
float Avg_distance;

unsigned long previousMillis_weight = 0; // Zmienna do przechowywania 
unsigned long previousMillis_distance = 0; // Zmienna do przechowywania 
const unsigned long Ultrasound_weigh_interval = 500; 
const unsigned long Ultrasound_distance_interval = 1000; 

int x, y, z;

     char val[10];
     char val1[10];
     char weight[10];

int startingWeightDistance = 0;
int baseBouyancy = 0;
bool acceptBouy = true;
bool buoyancySelect = false;
// Dodać do sprawdzania w ifie na pocz przerwań

void CalibrateWeight() {
  display.clearDisplay();

  for (int i = 3; i > 0; i--) {
    display.setCursor((SCREEN_WIDTH - (6 * 12)) / 2 - 3, 3);
    display.println("START?");
    display.println();
    display.print("  ");
    display.print(i);
    display.println("  ");
    display.display();
    delay(1000);
  }

  weight_Read();
  Boat_height_no_load = Avg_distance;
  Serial.print(Boat_height_no_load);
  buoyancySelect = true;

  while (!acceptBouy) {
    display.clearDisplay();
    display.setTextSize(2);

      // ENKODER W PRAWO 
      //  baseBouyancy += 100;
        
     // ENKODER W LEWO
     //   baseBouyancy -= 100;

    // TO PEWNIE DODAĆ DO ENKODERA, GDY TA FLAGA
    display.setCursor((SCREEN_WIDTH - (6 * 12)) / 2 - 7, 3);
    display.println("Set Bouyancy:");
    display.print(baseBouyancy);
    display.display();
    delay(200);
   }

      display.clearDisplay();
      display.setCursor((SCREEN_WIDTH - (6 * 12)) / 2 - 8, 3);
      display.println("Selected Bouyancy");
      display.print(baseBouyancy);
      display.display();
      delay(1000);
}









PMS pms(Serial2);

void pms7003Read()
{
  delay(PMS_READ_DELAY);

  PMS::DATA data;

  while (Serial.available()) { Serial.read(); }

  pms.requestRead();

  if (pms.readUntil(data))
  {
    #ifdef SerialOutput
     Serial.println();

     Serial.print("PM 1.0 (ug/m3): ");
     Serial.println(data.PM_AE_UG_1_0);

     Serial.print("PM 2.5 (ug/m3): ");
     Serial.println(data.PM_AE_UG_2_5);

     Serial.print("PM 10.0 (ug/m3): ");
     PM10 = data.PM_AE_UG_10_0;
     Serial.println(PM10);


     Serial.println();
  }
  else
  {
    Serial.println("No data.");
  }
  #endif
}

void Air_quality_alert()
{
  if(PM10 > PM10_ALLERT)
  {
    Serial.print("!!AIR QUALITY ALLETR!! PM10 VALUE:");
    Serial.println(PM10);
    char PM10_TXT[10];
    dtostrf(PM10,6,0,PM10_TXT);
    delay(10);
  }
}
void BPM280Read()
{
  delay(5);
if (bmp.takeForcedMeasurement()) {
    Temperature = bmp.readTemperature();
   Pressure = bmp.readPressure();
   #ifdef SerialOutput
    Serial.print(F("Temperature = "));
    
    Serial.print(Temperature);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
   
    Serial.print(Pressure);
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
  } else {
    Serial.println("Forced measurement failed!");
  }
  #endif
}

void weight_Read(){

digitalWrite(TRIG_PIN1, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN1, LOW);

    long duration1 = pulseIn(ECHO_PIN1, HIGH);
    long distance1 = 3 + (duration1/2) / 29.1;

    digitalWrite(TRIG_PIN2, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN2, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN2, LOW);

    long duration2 = pulseIn(ECHO_PIN2, HIGH);
    long distance2 = 3 + (duration2/2) / 29.1;

   //Serial.print(" Distance 1: ");
     //Serial.print(distance1);
    //Serial.print(" Distance 2: ");
    //Serial.print(distance2);
     //Serial.print(" Average distance: ");
     Avg_distance = float((distance1+distance2)/2)/100;
     //Serial.print(Avg_distance);
     //Serial.println(" m");

      float kg_cm_no_load = Boat_lift/Boat_height_no_load;
      float cm_to_weight = (Boat_height_no_load - 100*Avg_distance) * kg_cm_no_load;

      if(cm_to_weight < 0){
        cm_to_weight = 0;}
      
      char tekst1[] = "Distance to water is: ";
      char tekst2[] = "Calibrated kg/cm value: ";
      char tekst3[] = " Kg\n";

      char metr[] = " m ";
      dtostrf(Avg_distance,4,2,val);
      dtostrf(kg_cm_no_load,6,2,val1);
      dtostrf(cm_to_weight,6,1,weight);

     // Serial.print(weight);


}

void distance_Read(){

 digitalWrite(TRIG_PIN_DIST, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_DIST, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_DIST, LOW);

    long duration_dist = pulseIn(ECHO_PIN_DIST, HIGH);
    distance_dist = 3 + (duration_dist/2) / 29.1;

     Serial.print(" Boat distance: ");
     Serial.println(distance_dist);

}

void download_weather(){

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    String serverPath = String(serverName) + "?lat=" + lat + "&lon=" + lon + "&units=metric&appid=" + apiKey;

    http.begin(client, serverPath.c_str());
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    
      // Parse JSON response
      JsonDocument doc;
      deserializeJson(doc, response);

      // Extract values
      temperature = doc["main"]["temp"];
      weatherMain = doc["weather"][0]["main"];
      windSpeed = doc["wind"]["speed"];
      windDeg = doc["wind"]["deg"];
      visibility = doc["visibility"];

      // Print values
      Serial.print("Temperature: ");
      Serial.println(temperature);
      Serial.print("Weather Main: ");
      Serial.println(weatherMain);
      Serial.print("Wind Speed: ");
      Serial.println(windSpeed);
      Serial.print("Wind Degree: ");
      Serial.println(windDeg);
      Serial.print("Visibility: ");
      Serial.println(visibility);

  } else {
      Serial.print("Error on sending GET: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
}

String compassRead(){
  compass.read();
  byte a = compass.getAzimuth();
   char myArray[3];
  compass.getDirection(myArray, a);
   Serial.print(myArray[0]);
   Serial.print(myArray[1]);
   Serial.print(myArray[2]);
   return myArray;
}



void setup() {
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
    pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
    pinMode(TRIG_PIN_DIST, OUTPUT);
  pinMode(ECHO_PIN_DIST, INPUT);

  WiFi.begin(ssid, password);

   while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("! ");
    Serial.println("Connecting to WiFi...");
  }

  digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED when connected
  Serial.println("Connected to WiFi");

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);

  Wire1.setSDA(18);
  Wire1.setSCL(19);

    if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));}

    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.begin(9600);   // USB Serial

  Serial2.setTX(8);
  Serial2.setRX(9);

  Serial2.begin(9600);  // GPIO8 & GPIO9
  delay(100);
  pms.passiveMode();
  delay(5000);
  pms.wakeUp();
  //I2C
  
  Wire.begin();
  compass.init();
  //Encoder and interrupts
  pinMode(s1Pin, INPUT);
  pinMode(s2Pin, INPUT);
  pinMode(13, INPUT);
  pinMode(14, INPUT);
  pinMode(keyPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(keyPin),keyPinInterrupt,CHANGE);
  attachInterrupt(digitalPinToInterrupt(s1Pin),s1PinInterrupt,RISING);
  attachInterrupt(digitalPinToInterrupt(s2Pin),s2PinInterrupt,RISING);

  //Welcome screen
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Hello Mighty Viking!");
  display.println(" ");
  display.println("Are you winning son?");
  display.println(" ");
  display.println("   Use the funny ");
  display.println("joystick to navigate");
  display.display();
}

void DisplayMain(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Hello Mighty Viking!");
  display.println(" ");
  display.println("Are you winning son?");
  display.println(" ");
  display.println("   Use the funny ");
  display.println("joystick to navigate");
  display.display();
}

void DisplayCompass(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("    Kompas");
  display.println(" ");
  display.display();
}

void DisplayCompassSub(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(compassRead());
  display.println(" ");
  display.display();
}

void DisplayTemp(){
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
        display.clearDisplay();
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("Temperature:");
        display.println(Temperature);
        display.println("Pressure:");
        display.println(Pressure);
        display.display();
}

void DisplayTempProg(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Weather prognosis");
  display.println(" ");
  display.println("Waiting for download...");
  display.display();
  delay(2000);
  download_weather();
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Weather prognosis");
  display.print("Temperature: ");
  display.println(temperature);
  display.print("Weather: ");
  display.println(weatherMain);
  display.print("Wind speed: ");
  display.println(windSpeed);
  display.print("Visibility: ");
  display.println(visibility);
  display.display();
  delay(3000);
  screen = 2;
  subScreen = 0;
  screenChangedFlag = true;
}

void DisplayAir(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Air quality");
  display.println(PM10);
  display.println("Overall quality:");
  display.println(" ");
  if(PM10>150){
    display.println("ALERT! ALERT! ALERT!");
    display.println("Hazardous environment");

  }else{
    display.println("Air quality OK");
  }
  display.display();
}

void DisplayCal(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1.5);
  display.setCursor(0, 0);
  display.println("Calibration");
  //display.println(" ");
  //display.println("");
  //display.println("");
  display.display();
}
void DisplayWe(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 15);
  display.println("Boat Info");
  display.println("Weight      Distance");
  display.println(" ");
  display.print(weight);
  display.print("      ");
  display.println(distance_dist);
  display.display();
}
void DisplayCalSub(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Choose between:");
  display.println(" ");
  display.println("Compass");
  display.println(" ");
  display.println("Boat weight  ");
  display.display();
}
void DisplayCalGy(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Calibration in 5 seconds");
  display.display();
  delay(5000);
  display.clearDisplay();
  display.println("CALIBRATING. Keep moving your sensor...");
  display.display();
  compass.calibrate();
  display.clearDisplay();
  display.println("Calibration complete.");
  display.display();
  delay(1000);
  screen = 4;
  subScreen = 0;
  screenChangedFlag = true;
  }
void DisplayCalWe(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Weight");
  display.println("Turn the knob to change weight");
  display.println(Boat_lift);
  display.println("press the encoder");
  display.println("to save");
  display.display();
}
void DisplayExit(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Exit");
  display.println(" ");
  display.println("press the encoder");
  display.println("");
  display.display();
}
//Functions
void drawBitmap1(int x, int y, int sx, int sy, const uint16_t *data){
 int tc = 0;
 for(int Y = 0; Y < sy; Y++)
 {
  for(int X = 0; X < sx; X++)
  {
   display.drawPixel(X+x, Y+y, pgm_read_word(&data[tc]));
   if(tc < (sx*sy)) tc++;
  }
 }
}
void displayScreen(int screenNumber=screen,int subScreenNumber=subScreen){
  switch(screenNumber){
    case 0: //Main screen
        //display.clearDisplay();
        //drawBitmap1(32, 30, 26, 16,ArrowE);
        //display.display();
        DisplayMain();
        break;
    case 1: //Compass
      if(subScreenNumber == 0){ //Compass main
        DisplayCompass();
      } else if(subScreen == 1){ //Compass subscreen
        DisplayCompassSub();
      }
      break;
    case 2: //Temperature and barometer
        if(subScreen == 0){ //Main
        DisplayTemp();
        break;
      } else if(subScreen == 1){ //Prognosis subscreen
        DisplayTempProg();
        break;
      }
      break;
    case 3: //Air quality
        DisplayAir();
        break;
    case 4: //Calibration
        if(subScreenNumber == 0){ //Calibration main
        DisplayCal();
        break;
      } else if(subScreen == 1){ //Choose which sensor to calibrate
        DisplayCalGy();
        break;
      }else if(subScreen == 2){
        DisplayCalWe();
        break;
      }else if(subScreen == 3){
        DisplayExit();
        break;
      }
    case 5:
        DisplayWe();
        break;
  }
  delay(100);
}

void keyPinInterrupt(){
  keyTriggerTimeEnd = micros();
  unsigned long keyTriggerTimeDifference = keyTriggerTimeEnd - keyTriggerTimeBegin;
  if(keyTriggerTimeDifference < 500000){
    return;
  }else{
  //Serial.print("Encoder was clicked.\n");
  keyTriggerTimeBegin = micros();
  if(screen == 5){
    acceptBouy = !acceptBouy;
    buoyancySelect = !buoyancySelect;
    CalibrateWeight();
  }else{
  if(screen == 4 && subScreen > 0){
    switch(subScreen){
      case 1:
      //kalibracja żyroskopu
      Serial.print("kalibracja żyroskopu w trakcie");
      case 2:
      weightConfigurationFlag = true;
      //CalibrateWeight();
      Serial.print("kalibracja wagi");
      case 3:
      screen = 4;
      subScreen = 0;
      displayScreen();
      return;
    }
  }
  if(subScreen == 1){
    subScreen = 0;
  }else{subScreen++;}
    displayScreen();
  }}
}

void s1PinInterrupt(){
  s1 =1;
  s1TriggerTimeEnd = micros();
  unsigned long s1TriggerTimeDifference = s1TriggerTimeEnd-s1TriggerTimeBegin;
  if (s1TriggerTimeDifference>40000 && s1 && !s2 ) {
    if(buoyancySelect){
      baseBouyancy = baseBouyancy - 100;
    }else{
      Serial.print("Encoder moved left.\n");
      if(subScreen > 0 && screen!=4){
        return;
      }else if(subScreen>0 && screen ==4){
        if(subScreen == 1){
          subScreen = 3;
        }else{
        subScreen = subScreen - 1;
        }
        displayScreen();
      }else{
      if(screen == 0 ){
        screen = numberOfScreens - 1;
      }
      else{
        screen = screen - 1;
      }
      screenChangedFlag = true;
      s1TriggerTimeBegin = micros();}}
    }
    if(s1 && s2){
    s1 = 0;
    s2 = 0;
    }
}

void s2PinInterrupt(){
  s2 = 1;
  s2TriggerTimeEnd = micros();
  unsigned long s2TriggerTimeDifference = s2TriggerTimeEnd-s2TriggerTimeBegin;
  if (s2TriggerTimeDifference>40000 && s2 && !s1) {
    if(buoyancySelect){
      baseBouyancy = baseBouyancy - 100;
    }else{
      Serial.print("Encoder moved right.\n");
      if(subScreen > 0 && screen != 4){
        return;
      }else if(subScreen>0 && screen ==4){
        if(subScreen == 3){
          subScreen = 1;
        }else{
        subScreen = subScreen + 1;
        }
        displayScreen();
      }else{
      if(screen == numberOfScreens-1){
        screen = 0;
      }
      else{
        screen = screen + 1;
      }
      screenChangedFlag = true;
      s2TriggerTimeBegin = micros();}}
    }
    if(s1 && s2){
    s1 = 0;
    s2 = 0;
    }
}





void loop() {
   




  if(screenChangedFlag){
    displayScreen();
    screenChangedFlag = false;
  }
  if(weightConfigurationFlag && (micros()-refreshTimePrevious>=500000)){
    displayScreen();
  }
  if(micros()-refreshTimePrevious>=refreshTime){
    displayScreen();
    refreshTimePrevious = micros();
  }

  static uint32_t timerLast = 0;
 uint32_t timerNow = millis();
  if (timerNow - timerLast >= timerInterval) {
    timerLast = timerNow;
    pms7003Read();
    timerInterval = timerInterval == PMS_READ_DELAY ? PMS_READ_INTERVAL : PMS_READ_DELAY;
  }


 unsigned long currentMillis_weight = millis(); // Aktualny czas
if (currentMillis_weight - previousMillis_weight >= Ultrasound_weigh_interval) {

        weight_Read();
        Air_quality_alert();
        BPM280Read();
     
previousMillis_weight = currentMillis_weight;
  }


unsigned long currentMillis_distance = millis(); // Aktualny czas
 if (currentMillis_distance - previousMillis_distance >= Ultrasound_distance_interval) {

    distance_Read();

    previousMillis_distance = currentMillis_distance;
  }


}


