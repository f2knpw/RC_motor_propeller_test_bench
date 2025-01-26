//Rx servo reader library : https :  //github.com/rewegit/esp32-rmt-pwm-reader
#include <esp32-rmt-pwm-reader.h>
// #include "esp32-rmt-pwm-reader.h" // if the lib is located directly in the project directory

// init channels and pins
uint8_t pins[] = { 32, 33 };  // desired input pins
int numberOfChannels = sizeof(pins) / sizeof(uint8_t);

#include <WiFiClient.h>
#include <TelnetStream.h>
#include "driver/adc.h"
#include <esp_wifi.h>
#include <esp_bt.h>

#include <esp_task_wdt.h>
//1 seconds WDT
#define WDT_TIMEOUT 1
int nbTimeout = 0;
int nbAlarm = 0;
long bootTime;

//Json
#include <ArduinoJson.h>  //https://github.com/bblanchon/ArduinoJson


String displayStatus = "Initializing";

//OLED
#define OLED
#ifdef OLED
#define OLED_SCL_PIN 5
#define OLED_SDA_PIN 17
#include "SSD1306.h"
SSD1306 display(0x3c, OLED_SDA_PIN, OLED_SCL_PIN);  // for 0.96" 128x64 LCD display ; i2c ADDR & SDA, SCL
#endif
String displayDebug;

//Radio Rx
int throttle = -1;


//sensors selection
#define HAS_HX711  //uncomment for weighing rain gauge
//#define HAS_DS18B20     //uncomment for DS18B20 temperature sensor
#define HAS_SPEED_SENSOR  //uncomment for propeller speed sensor


float temperature = 0;
int smooth = 10;  //acquire smooth*values for each ADC
float Vin = 0.;   //input Voltage (lipo battery voltage)
float prevVin = 0;
float current = 0.;  //lipo battery current
float prevCurrent = 0;
float RPM = 0;

#define VIN_PIN 39  //ADC pin for solar panel voltage measurement
#define CUR_PIN 36
int zeroCurrent = 0;

//temperature sensor
#define ONE_WIRE_BUS 13  // Data wire is plugged into pin 13 on the ESP32
#ifdef HAS_DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempScale1(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
#endif

//scale
#define PIN_CLOCK 19  //output to generate clock on Hx711
#define PIN_DOUT 23   //input Dout from Hx711

long calibZero = 0;      //No load sensor Output
long calib = 130968;     //sensor output - calibZero for Weight calibration --> will be auto calibrated later
int calibWeight = 1000;  //weight at which calibration is done --> expressed in grams.
float AverageWeight = 0;
float CurrentRawWeight = 0;

#define FILTER_SAMPLES 5000         // 1 = no filtering (faster single acquisition but noise), filterSamples should  be an odd number
#define REJECT_RATIO 40             //points to reject % left and right before averaging (if filter sample >1)
float smoothArray[FILTER_SAMPLES];  // array for holding raw sensor values for sensor1

//speed sensor
#define TAC_PIN 27  //IR sensor acting as tachometer
int tops = 0;       //nb tops when anemometer rotates
long hallTimeout;   //to debounce
int speedSensorMeasuringTime;
#define NB_BLADES 2 // 2 tops for a normal 2 blades propeller

#define LED_PIN 22


//Preferences
#include <Preferences.h>
Preferences preferences;

//touchpad
touch_pad_t touchPin;
int threshold = 40;  //Threshold value for touchpads pins

void callback() {
  //placeholder callback function
}
boolean TouchWake = false;



//WifiManager
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager

//flag for saving data
bool shouldSaveConfig = false;
bool touch3detected = false;  //touch3 used to launch WifiManager (hold it while reseting)
bool touch2detected = false;  //touch2 used to calibrate loadcell with known weight (hold it while reseting)

//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


String ssid = "";
String password = "";
boolean hasWifiCredentials = false;

//#define W_DEBUG     //debug Wifi and firebase
//#define G_DEBUG     //debug GCM serveur
//#define DEBUG_OUT
//#define xDEBUG
//#define xxDEBUG
//#define UDP_DEBUG
//#define DEBUG
//#define TEST
#define PREFERENCES_DEBUG
//#define DEBUG_TELNET
//#define RAW_WEIGHT_DEBUG
//#define DEBUG_W
//#define DEBUG_VIN
//#define DEBUG_CURRENT
//#define DEBUG_RPM

//UDP --------------
unsigned int localPort = 5000;  // local port to listen on
char packetBuffer[64];          //buffer to hold incoming packet
char AndroidConnected = 0;
long Timeout;
String device = "RCmotorTester";
String theMAC = "";

WiFiUDP Udp;
long LastUDPnotification;
//end UDP-----------

#ifdef HAS_SPEED_SENSOR
void IRAM_ATTR hall_ISR()  //hall sensor interrupt routine
{
  //if ((millis() - hallTimeout) > 10) //leave commented : no need to debounce
  {
    hallTimeout = millis();
    tops++;
    //Serial.println( tops);  //should comment this line to avoid crashes
  }
}
#endif

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);

  //sparks drivers


  // init Rx channels
  pwm_reader_init(pins, numberOfChannels);
  pwm_set_channel_pulse_neutral(0, 1000);  // throttle neutral is set to "zero"
  pwm_set_channel_pulse_neutral(1, 1000);  //vol neutral is set to "zero"

  // here you can change channel defaults values before reading (if needed)
  // e.g. pwm_set_channel_pulse_min() /max/neutral
  // e.g. set auto_zero/auto_min_max for channel 0-2
  // for (int ch = 0; ch < 3; ch++) {
  //   pwm_set_auto_zero(ch, true);     // set channel to auto zero
  //   pwm_set_auto_min_max(ch, true);  // set channel to auto min/max calibration
  // }

  // begin reading
  esp_err_t err = pwm_reader_begin();
  if (err != ESP_OK) {
    Serial.printf("begin() err: %i", err);
  } else {
    Serial.println("***************");
    Serial.println("program started");
    Serial.println("***************");
    Serial.println(" ");
  }



  //Preferences
  preferences.begin("RC_motor_tester", false);

  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only

  calibWeight = preferences.getInt("calibWeight", 1000);
  calib = preferences.getLong("calib", -222444);  // value for a milk pack ~1000g

  ssid = preferences.getString("ssid", "");  // Get the ssid  value, if the key does not exist, return a default value of ""
  password = preferences.getString("password", "");
#ifdef PREFERENCES_DEBUG
  Serial.println("_________________");
  Serial.println("read preferences :");

  Serial.print("calib0 HX711 : ");
  Serial.println(calibZero);
  Serial.print("calib HX711 : ");
  Serial.println(calib);
  Serial.print("calib weight (g) : ");
  Serial.println(calibWeight);
  Serial.println("_________________");
#endif
  //preferences.end();  // Close the Preferences

#ifdef OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  displayLCD();
#endif

  if (ftouchRead(T3) < threshold) touch3detected = true;  //detect touchpad for CONFIG_PIN

  //  //connect to WiFi

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(300);

  if (touch3detected)  //then launch WifiManager
  {
    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal("JP RCMotorTester")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }
  delay(2000);
  //  //save the custom WifiManager's parameters if needed
  if (shouldSaveConfig) {
    Serial.println("saving Wifi credentials ");
    //read updated parameters

    preferences.putString("password", WiFi.psk());
    preferences.putString("ssid", WiFi.SSID());
    delay(2000);
    ESP.restart();
    delay(5000);
  }




  //connect to WiFi
  WiFi.begin(ssid.c_str(), password.c_str());
  long start = millis();
  hasWifiCredentials = false;

  while ((WiFi.status() != WL_CONNECTED) && (millis() - start < 20000)) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) hasWifiCredentials = true;

  //if you get here you may be connected to the WiFi
  Serial.print("connected to Wifi: ");
  Serial.println(hasWifiCredentials);



  //if (hasWifiCredentials)
  {
    TelnetStream.begin();  //used to debug over telnet
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    //Start UDP
    Udp.begin(localPort);
  }

  Serial.println(" ");
  Serial.println("start decoding : \n");


  //delay(3000);

  //Serial.println("Configuring WDT...");
  // esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  // esp_task_wdt_add(NULL);                //add current thread to WDT watch

  bootTime = millis();


  //scale init
  if (ftouchRead(T2) < threshold) touch2detected = true;  //detect touchpad for weight calibration
#ifdef HAS_HX711
  pinMode(PIN_CLOCK, OUTPUT);  // initialize digital pin 4 as an output.(clock)
  digitalWrite(PIN_CLOCK, HIGH);
  delayMicroseconds(100);        //be sure to go into sleep mode if > 60µs
  digitalWrite(PIN_CLOCK, LOW);  //exit sleep mode*/
  pinMode(PIN_DOUT, INPUT);      // initialize digital pin 5 as an input.(data Out)

  GetRawWeight();
  calibZero = CurrentRawWeight;  // during boot the motor doesn't spin = no thrust on the propeller
//preferences.putLong("calibZero", calibZero); // no need to store it as computed during each boot and used during loop
#ifdef DEBUG_W
  Serial.print("calibZero raw value = ");
  Serial.println(calibZero);
#endif
#endif  //HAS_HX711

  //anemometer
#ifdef HAS_SPEED_SENSOR
#ifdef DEBUG
  Serial.println("speed sensor enabled");
#endif
  pinMode(TAC_PIN, INPUT_PULLUP);
  attachInterrupt(TAC_PIN, hall_ISR, FALLING);  //will count tops on Anemometer hall Sensor
  hallTimeout = millis();
  tops = 0;
  speedSensorMeasuringTime = millis();
#endif

#ifdef HAS_DS18B20
  int kk;
  tempScale1.begin();                  // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  for (int i = 0; i < 2; i++) {        //read twice to warm up
    tempScale1.requestTemperatures();  //needed as first reading may be stuck to 25°
    kk = tempScale1.getTempCByIndex(0);
  }
#endif

  //lipo voltage and current
  //ADC
  //analogSetClockDiv(255);
  //analogReadResolution(12);             // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);              // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetAttenuation(ADC_11db);  //Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db=0, ADC_2_5db=1, ADC_6db=2, ADC_11db=3
                                   //lipo current
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    smoothArray[i] = analogRead(CUR_PIN);  //ADC smoothing the zeroCurrent delivered by the sensor (should be 2.5V)
  }
  zeroCurrent = medianFilter();
  prevCurrent = 0;
#ifdef DEBUG_CURRENT
  Serial.print("zero Current ");
  Serial.println(zeroCurrent);
#endif
  //lipo voltage at startup
  for (int i = 0; i < 2000; i++) prevVin += analogRead(VIN_PIN);  //ADC smmothing
  prevVin = prevVin / 2000;
  prevVin = volts(prevVin);

#ifdef DEBUG_TELNET
  TelnetStream.print("start");
#endif

  LastUDPnotification = millis();
}

//***********************************************************************************************************************
void loop() {
  esp_task_wdt_reset();  //reset the watchdog
//*******************
//acquire all sensors
//*******************
//outside temperature sensor
#ifdef HAS_DS18B20
  temperature = 0;
  int i;
  int kk;

  for (i = 0; i < 20; i++) {
    tempScale1.requestTemperatures();
    temperature += tempScale1.getTempCByIndex(0);
  }
  temperature /= i;
#ifdef DEBUG
  Serial.print("temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");
#endif
#endif

  //lipo voltage
  for (int i = 0; i < smooth; i++) Vin += analogRead(VIN_PIN);  //ADC smmothing
  Vin = Vin / smooth;
#ifdef DEBUG_VIN
  Serial.print("Vin ");
  Serial.print(Vin);
  Serial.print(" / ");
#endif
  Vin = volts(Vin);
  if (Vin > prevVin) Vin = prevVin;
#ifdef DEBUG_VIN
  Serial.println(Vin);
#endif

  //lipo current
  for (int i = 0; i < smooth; i++) current += analogRead(CUR_PIN);  //ADC smoothing
  current = current / smooth;


#ifdef DEBUG_CURRENT
  Serial.print("Current ");
  Serial.print(current);
  Serial.print(" / ");
#endif
  current = amps(current);
  if ((current) < 0) current = prevCurrent;
  prevCurrent = current;
#ifdef DEBUG_CURRENT
  Serial.println(current);
#endif

  //propeller thrust
#ifdef HAS_HX711
  GetRawWeight();  //HX711 sensor
  AverageWeight = (calibZero - CurrentRawWeight) * calibWeight / (calib);
#ifdef DEBUG
  Serial.print("weight = ");
  Serial.print(AverageWeight);
  Serial.println(" g");
#endif
#endif  //HAS_HX711

//propeller RPM
#ifdef HAS_SPEED_SENSOR
  RPM = tops * 60*1000 / (millis() - speedSensorMeasuringTime)/NB_BLADES;
#ifdef DEBUG_RPM
  Serial.print("RPM = ");
  Serial.println(RPM);
#endif
  tops = 0;
  speedSensorMeasuringTime = millis();
#endif

  //***************************
  //decoding the Radio receiver
  //***************************
  if (pwm_get_state_name(0) == "STABLE")  // if we receive "throttle" channel then switch to manual mode
  {
    // Reading the actual pulse width of Throttle channel
    throttle = constrain((pwm_get_rawPwm(0) - 1000), 0, 1000);  //clip throttle value between 0 and 5
  }

  // Do something with the pulse width... throttle



  //************
  // UDP process
  //************
  int packetSize = Udp.parsePacket();  //if there's data available, read a packet coming from Android phone
  if (packetSize) {
    Timeout = millis();  //rearm software watchdog

#if defined xxxDEBUG
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
#endif
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
#if defined UDP_DEBUG
    Serial.print("UDP Contents: ");
    Serial.println(packetBuffer);
#endif
    String Res;
    String test;
    int i;
    test = packetBuffer;

    if (test == "ID")  // send a reply, to the IP address and port that sent us the packet we received
    {
      AndroidConnected = 1;
      Res = device;
      sendUDP(Res);

    } else if (test.startsWith("TOPIC"))  // send a reply, to the IP address and port that sent us the packet we received
    {
      //{"sender":"18FE349E7690","device":"WaterLeak"}
      Res = WiFi.macAddress();
      Res.replace(":", "");
      sendUDP(Res);
    }

    else if (test.startsWith("{"))  // decode json
    {
      // message = test.substring(6);
      readCmd(test);
    }

    else if (test.startsWith("STOP"))  // send a reply, to the IP address and port that sent us the packet we received
    {
      sendUDP("okSTOP");
    }
  }

  //send sensors data as fast as possible (else uncomment the following line)
  //if (((millis() - LastUDPnotification) > 10))  // send UDP message to Android App (no need to be connected, a simple notification, send and forget)
  {

    String res;
    res = "{\"A\":\"" + String(current) + "\",\"V\": " + String(Vin) + ",\"W\": " + String(AverageWeight) + ",\"R\": " + String(RPM) + ",\"T\": " + String(temperature) + ",\"S\": " + String(throttle) + "}";
    //LastUDPnotification = millis();
#ifdef DEBUG_TELNET
    TelnetStream.println(res);
#endif
    sendUDP(res);  //try to send via UDP
  }
#ifdef OLED
  displayLCD();  //and update OLED display
#endif
}  //end of Loop



#ifdef OLED
void displayLCD(void)  //refresh the LCD screen
{
  display.clear();
  display.drawString(0, 0, "current : " + (String)current + " A");
  display.drawString(0, 10, "voltage : " + (String)Vin + " V");
  display.drawString(0, 20, "thrust  : " + (String)AverageWeight + " kg");
  display.drawString(0, 30, "RPM  : " + (String)RPM);
  display.drawString(0, 40, "temp : " + (String)temperature + " °C");

  display.display();
}
#endif

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int ftouchRead(int gpio)  // this will filter false readings of touchRead() function...
{
  int val = 0;
  int readVal;
  for (int i = 0; i < 10; i++) {
    readVal = touchRead(gpio);
    val = max(val, readVal);
  }
  return val;
}


void sendUDP(String Res) {
  char ReplyBuffer[Res.length() + 1];  // a string to send back
  Res.toCharArray(ReplyBuffer, Res.length() + 1);
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.print(ReplyBuffer);  //was write...
  Udp.endPacket();
}

void readCmd(String test) {
  if (test.startsWith("{"))  //then it may contain JSON
  {
    StaticJsonDocument<2000> doc;
    DeserializationError error = deserializeJson(doc, test);
    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      Serial.println("deserializeJson() failed");  //answer with error : {"answer" : "error","detail":"decoding failed"}
      sendUDP("{\"answer\" : \"error\",\"detail\":\"decoding failed\"}");
    } else {
      // Fetch values --> {"Cmd":"Start"}
      String Cmd = doc["Cmd"];
      if (Cmd == "Start")  //start pump
      {
#ifdef DEBUG_OUT
        Serial.print("Start pump ");
#endif

      } else if (Cmd == "Calib")  //set high
      {
        String value = doc["value"];
        calibWeight = value.toInt();
        GetRawWeight();  //HX711 sensor

        calib = calibZero - CurrentRawWeight;
        Serial.print("calibration HX711... ");
        Serial.println(calib);
        Serial.print(" for weight (g) ");
        Serial.println(calibWeight);

        preferences.putLong("calib", calib);
        preferences.putInt("calibWeight", calibWeight);
      } else if (Cmd == "CalibCurrent")  //set high
      {

        Serial.print("calibration current... ");
        Serial.println(calib);
      }
    }
  }
}

#ifdef HAS_HX711
void GetRawWeight(void) {
  unsigned long RawWeight;
  // wait for the chip to become ready
  long startTime;
  //delay(5000);             //let the HX711 warm up
  AverageWeight = 0;
  int j = 0;
  for (j = 0; j < 1; j++) {
    startTime = millis();

    while ((digitalRead(PIN_DOUT) == HIGH) && ((millis() - startTime) < 1000))
      ;  //wait for data conversion ready

    if ((millis() - startTime) > 1000)  //or time out...
    {
      Serial.println("weight error");
    }
    RawWeight = 0;
    // pulse the clock pin 24 times to read the data
    for (char i = 0; i < 24; i++) {
      digitalWrite(PIN_CLOCK, HIGH);
      delayMicroseconds(1);
      RawWeight = RawWeight << 1;
      if (digitalRead(PIN_DOUT) == HIGH) RawWeight++;
      digitalWrite(PIN_CLOCK, LOW);
    }
    // set the channel and the gain factor (A 128) for the next reading using the clock pin (one pulse)
    digitalWrite(PIN_CLOCK, HIGH);
    delayMicroseconds(1);
    RawWeight = RawWeight ^ 0x800000;
    digitalWrite(PIN_CLOCK, LOW);

    AverageWeight += RawWeight;
  }
  //digitalWrite(PIN_CLOCK, HIGH);    //to enter into power saving mode
  CurrentRawWeight = AverageWeight / j;
#ifdef xRAW_WEIGHT_DEBUG
  Serial.print("Raw weight : \t");
  Serial.println(RawWeight);
#endif
#ifdef RAW_WEIGHT_DEBUG
  Serial.print("Raw average weight : ");
  Serial.println(CurrentRawWeight);
#endif
}
#endif


float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
}
float volts(float raw)  //simple linear calibration...
{
  return raw * 11.64 / 3780;  //calib reading raw value into log
}

float amps(float raw)  //simple linear calibration without substraction of the zero current (which is measured at boot time into setup)...
{
  return ((zeroCurrent - raw) * 8. / 105.);
}

int medianFilter(void) {
  //median filter
  int raw = smoothArray[0];
  if (FILTER_SAMPLES > 1) {
    boolean done;
    float temp;
    int k, top, bottom;
    done = 0;            // flag to know when we're done sorting
    while (done != 1) {  // simple swap sort, sorts numbers from lowest to highest
      done = 1;
      for (int j = 0; j < (FILTER_SAMPLES - 1); j++) {
        if (smoothArray[j] > smoothArray[j + 1]) {  // numbers are out of order - swap
          temp = smoothArray[j + 1];
          smoothArray[j + 1] = smoothArray[j];
          smoothArray[j] = temp;
          done = 0;
        }
      }
    }
    // throw out top and bottom REJECT_RATIO % of samples - limit to throw out at least one from top and bottom
    bottom = max(((FILTER_SAMPLES * REJECT_RATIO) / 100), 1);
    top = min((((FILTER_SAMPLES * (100 - REJECT_RATIO)) / 100) + 1), (FILTER_SAMPLES - 1));  // the + 1 is to make up for asymmetry caused by integer rounding
    k = 0;
    raw = 0;
    for (int j = bottom; j < top; j++) {
      raw += smoothArray[j];  // total remaining indices
      k++;
    }
    raw = raw / k;  // divide by number of samples and return the value
    //end median filter
  }
  return raw;
}
