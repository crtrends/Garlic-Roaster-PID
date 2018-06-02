#include <arduino.h>//necessary if you decide to convert to .cpp file type
const char firmware_name[] = {"Smoker PID "};
const char firmware_version[] = {"v0.2"};

/*
   Assembled by Colin Trachte.
   Contact Info:
   Licensing: See respective libraries. Other than that, keep me in the credits.
   //I've tried to keep track of all the sources used.
   Credits:
   LCD: https://github.com/duinoWitchery/hd44780
   Real Time Clock: https://github.com/Makuna/Rtc/wiK[pid_profile][1]/RtcDS3231-object
   https://github.com/Makuna/Rtc/wiK[pid_profile][1]/RtcDateTime-object
   PID Tutorial: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
   PID Autotune: http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/
   Onewire address finder: http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html
   OTA Update Tutorial: https://www.bakke.online/index.php/2017/06/02/self-updating-ota-firmware-for-esp8266/
   OTA Example code: https://gist.github.com/mhelff/e857be405e4c77bdfec7b950c34f1b3f
   Wifi Manager: https://github.com/tzapu/WiFiManager
*/

/*********************************Definitions*******************************************/
//a uint8_t is an 8-bit unsigned integer with a value of 0-255. This data type is also sometimes called a "uint8_t," which is confusing, so we always call it a uint8_t.
//arrays start at zero. so, a six slot array will have indices 0-5, not 1-6.
//define is actually a global find/replace command. #define foo bar will replace foo with bar throughout the code.
//the variable "now" is an unsigned double commonly used in open source programs (seconds since jan 1, 1970)

/*********************************Pin Mapping*******************************************/
#define pin_sleep D0// this pin has to be tied to the rst pin for sleep mode to work.
#define pin_scl D1
#define pin_sda D2
#define pin_onewire D3
#define pin_led_builtin D4 //Pin D4 read/write is inverted. so, True = off, False = on.
const uint8_t pin_relay[] = {D5, D6, D7, D8};

/*********************************Libraries*******************************************/
#include <ESP8266WiFi.h>//this library is for esp8266 with no arduino involved.
#include <ESP8266WebServer.h>//WiFiManager
#include <DNSServer.h>//WiFiManager
#include <ArduinoOTA.h>//enable OTA Updates
#include <WiFiManager.h>//
//#include <ESP8266HTTPClient.h>
//#include <ESP8266httpUpdate.h>
#include <PubSubClient.h> //MQTT library
#include <Wire.h>//I2C
#include <OneWire.h>//temp sensors use onewire comm protocol
#include <hd44780.h>//LCD screen library for chunky LCD with I2C adapter.
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Button.h>//simplifies the use of single buttons
#include <PID_v1.h>//make one value change over time until it is close enough to a set point.
#include <PID_AutoTune_v0.h>
#include <DallasTemperature.h>//for use with ds18b20 sensor
#include <EEPROM.h>//There are 4096 bytes of EEPROM on the 8266, with indices from 0 to 4095. Arduino has 512.
#include <RtcDS3231.h>//Real Time Clock is at address 0x68
#include <CayenneMQTTESP8266.h>//smartphone remote control app

/*********************************Pseudo - Objects*******************************************/
//objects have a name, a status indicator, booleans for disabling and sleep mode, and an update speed.
//we also need to keep track of how many objects there are, for loops through the array.
//large chunks of a program can be saved and loaded from spreadsheets, in theory. Not yet implemented.
//https://stackoverflow.com/questions/18577404/how-can-a-mixed-data-type-int-float-char-etc-be-stored-in-an-array?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa

#define object_count 4
#define onewire_count 4
#define relay_count 4

//english names help make switch statements understandable. We use prefix o_ for "object."
//all objects are stored in what is basically a spreadsheet. Objects are identified by column number.
#define o_lcd 0
#define o_temp 1
#define o_relay 2
#define o_clock 3

#define pid_coarse 0
#define pid_fine 1

//#define CAYENNE_DEBUG
#define CAYENNE_PRINT Serial

char object_name[object_count][12] = {{"LCD"}, {"Temp Sensor"}, {"Relay"}, {"Clock"}};
bool object_enabled[object_count] = {false, true, true, true};//Disabling an object prevents it from running any code whatsoever. an important debug feature.
bool object_awake[2][object_count] =
{
  {true, true, true, true},
  {false, false, false, false} //do we want it to be awake? is it currently awake?
};
uint8_t object_status[2][object_count];//two rows, object_count columns
unsigned long object_mtimer[][object_count] =
{
  {0, 0, 0, 0}, //when was the object last updated?
  {250, 1000, 1000, 1000}//how often is the object updated?
};

//a "spreadsheet" which describes status codes
char status_name[][11] = {{"sleeping"}, {"awake"}};//status codes can tell us more about what the object is doing

//a "spreadsheet" which describes bezier curves. The user can define curves by placing dots.
//char path_name[];

uint16_t eeprom_address = 0; // start reading from the first byte (address 0) of the EEPROM
uint8_t eeprom_value;

unsigned long uptime = 0;
//RtcDateTime(uint16_t year, uint8_t month, uint8_t dayOfMonth, uint8_t hour, uint8_t minute, uint8_t second)
//RtcDateTime compileDateTime(__DATE__, __TIME__);

uint8_t y, mo, dm, dw, h, m, s; //where dw is the day of week and dm is day of month.
const char day_name[][11] = {{"Sunday"}, {"Monday"}, {"Tuesday"}, {"Wednesday"}, {"Thursday"}, {"Friday"}, {"Saturday"}};
const char day_abbr[][11] = {{"Sun"}, {"Mon"}, {"Tue"}, {"Wed"}, {"Thu"}, {"Fri"}, {"Sat"}};

/*timer system to be used like an alarm clock
  //stimers are based on RTC with a resolution of 1 second.
  uint8_t stimer_count = 1;
  unsigned long stimer[][stimer_count] = {0};
  uint8_t stimer_event[] = {0};//event code tells the timer what to do when it goes off.
  uint8_t stimer_callback[] = {0};//callback code tells the timer what to do after it goes off.

  //millis()-based timers are called mtimers.
  uint8_t mtimer_count = 1;
  unsigned long mtimer[][mtimer_count] = {0};
  uint8_t mtimer_event[] = {0};//event code for each timer
  uint8_t mtimer_callback[] = {0};//callbacks tell the timer what to do next after the event

  //micros()-based timers are called utimers.
  uint8_t utimer_count = 1;
  unsigned long utimer[][utimer_count] = {0};
  uint8_t utimer_event[] = {0};//event code for each timer
  uint8_t utimer_callback[] = {0};//callbacks tell the timer what to do next after the event
*/
double temp_reading[4];

//Specify the links and initial tuning parameters
double Input, Output;
double Setpoint = 75;//this is what we want to eventually control using some sort
double pid_deadband = .5;//ignore any short, sudden PID outputs. we don't want to wear out the relay.
uint8_t pid_profile = pid_coarse;
uint8_t pid_profile_cutoff[] = {1};//how close to setpoint before transition from coarse to fine control.
double K[][3] =
{
  {4, .2, 1}, //coarse
  {1, .05, .25}//fine
};
//Onewire Temp sensors can't be read faster than once per second.
//we should try to get at least 30 data points - a statistically significant minimum.
int WindowSize = 30000;//millis of data to remember for purposes of PID.
unsigned long windowStartTime;

//autotune seems to work but isn't producing good results. so, tuning = false for now.
boolean tuning = false;
byte ATuneModeRemember = 2;
double aTuneStep = 2, aTuneNoise = 1, aTuneStartValue = 0;
unsigned int aTuneLookBack = 30;//seconds of history to go off of

/*********************************Objects*******************************************/
OneWire ow_bus(pin_onewire);//Setup a OneWire instance to communicate with any OneWire devices
DallasTemperature temp_sensors(&ow_bus);// Pass our oneWire reference to Dallas Temperature.
RtcDS3231<TwoWire> rtc(Wire); //global clock object. Interestingly, this particular clock also has a temp sensor.
hd44780_I2Cexp lcd;
PID pid_relay(&Input, &Output, &Setpoint, K[pid_profile][0], K[pid_profile][1], K[pid_profile][2], DIRECT);
PID_ATune aTune(&Input, &Output);

/*********************************Functions*******************************************/
//functions must be initialized before we get to setup and loop where they are used.
void setup_serial()
{
  Serial.begin(115200);
  delay(10);
  Serial.println("USB Serial Begin");
  Serial.print(firmware_name);
  Serial.print(firmware_version);
  Serial.print(" Compiled ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
}

void AutoTuneHelper(boolean start)
{
  if (start)
  {
    ATuneModeRemember = pid_relay.GetMode();
  }
  else
  {
    pid_relay.SetMode(ATuneModeRemember);
  }
}

void setup_object(uint8_t object_id)
{
  if (object_enabled[object_id])//don't even try if the object is disabled
  {
    switch (object_id)
    {
      case o_lcd:
        {
          int lcd_status = lcd.begin(16, 2);

          if (lcd_status) // non zero status means it was unsuccesful
          {
            lcd_status = -lcd_status; // convert negative status value to positive number

            // begin() failed so blink error code using the onboard LED if possible
            hd44780::fatalError(lcd_status); // does not return
          }
        }
        break;
      case o_temp:
        {
          temp_sensors.begin();
          Serial.print("Parasitic power should be off (0): ");
          Serial.println(temp_sensors.isParasitePowerMode());
        }
        break;
      case o_relay:
        {
          for (uint8_t i = 0; i < relay_count; i++)
          {
            pinMode(pin_relay[i], OUTPUT);
          }

          //initialize the variables linked to the PID
          windowStartTime = millis();
          pid_relay.SetOutputLimits(0, WindowSize);//tell the PID to range between 0 and the full window size
          pid_relay.SetMode(AUTOMATIC);//turn the PID on

          if (tuning)
          {
            //Set the output to the desired starting frequency.
            Output = aTuneStartValue;
            aTune.SetNoiseBand(aTuneNoise);
            aTune.SetOutputStep(aTuneStep);
            aTune.SetLookbackSec((int)aTuneLookBack);
            AutoTuneHelper(true);
          }
        }
        break;
      case o_clock:
        {
          rtc.Begin();
          RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__) + 60;
          //turn off pins that are inaccessible anyway.
          rtc.Enable32kHzPin(false);
          rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);

          if (!rtc.IsDateTimeValid())
          {
            Serial.println("RTC lost confidence in the DateTime!");
            rtc.SetDateTime(compiled);
          }

          if (!rtc.GetIsRunning())
          {
            Serial.println("RTC was not actively running, starting now");
            rtc.SetIsRunning(true);//turns on the clock if it was off.
          }

          RtcDateTime now = rtc.GetDateTime();
          if (now < compiled)
          {
            Serial.println("RTC is older than compile time! Syncing...)");
            rtc.SetDateTime(compiled);
          }

          print_time(now);

          /* calculate a date which is 7 days and 30 seconds into the future
            RtcDateTime future (now + 7 * 86400L + 30);
          */
        }
        break;
    }
  }
}

void start_object(uint8_t object_id)
{
  switch (object_id)
  {
    case o_lcd:
      {
      }
      break;
    case o_temp:
      {
      }
      break;
    case o_relay:
      {
      }
      break;
    case o_clock:
      {
      }
      break;
  }
}

void update_object(uint8_t object_id)
{
  uint8_t level_id = object_status[0][object_id];//current object status
  switch (object_id)
  {
    case o_lcd:
      {
      }
      break;
    case o_temp:
      {
        Serial.print("Temperature F:");
        temp_sensors.requestTemperatures(); //Send the command to take a temperature reading
        for (uint8_t i = 0; i < onewire_count; i++)
        {
          Serial.print(" ");
          Serial.print(i);
          Serial.print(": ");
          temp_reading[i] = temp_sensors.getTempFByIndex(i);//get the temperature reading from the sensors
          Serial.print(temp_reading[i]);
        }
        Serial.println();
      }
      break;
    case o_relay:
      {
        Input = temp_reading[0];

        if (tuning)
        {
          byte val = (aTune.Runtime());//The autotune waits until the last 3 maxima have been within 5% of each other.
          if (val != 0)//we're done, set the tuning parameters
          {
            K[pid_profile][0] = aTune.GetKp();
            K[pid_profile][1] = aTune.GetKi();
            K[pid_profile][2] = aTune.GetKd();
            Serial.println("Autotune Finished");
            pid_relay.SetTunings(K[pid_profile][0], K[pid_profile][1], K[pid_profile][2]);
            AutoTuneHelper(false);
            tuning = false;
          }
        }
        else
        {
          pid_relay.Compute();
          
          if (abs(Setpoint - Input) < pid_profile_cutoff[0])//if we're close to setpoint, use conservative parameters
          {
            pid_profile = pid_fine;
            Serial.println("using fine pid profile");
          }
          else
          {
            pid_profile = pid_coarse;//we're far from setpoint, use aggressive tuning parameters
            Serial.println("using coarse pid profile");
          }
          pid_relay.SetTunings(K[pid_profile][0], K[pid_profile][1], K[pid_profile][2]);          
        }

        if (Output > pid_deadband)
        {
          digitalWrite(pin_relay[0], HIGH);
          digitalWrite(pin_led_builtin, LOW);//indicator light is ON
        }
        else
        {
          digitalWrite(pin_relay[0], LOW);
          digitalWrite(pin_led_builtin, HIGH);
        }

        Serial.println(Output);
        Serial.print("Proportional Constant: ");
        Serial.println(K[pid_profile][0]);
        Serial.print("Integral Constant: ");
        Serial.println(K[pid_profile][1]);
        Serial.print("Derivative Constant: ");
        Serial.println(K[pid_profile][2]);
      }
      break;
    case o_clock:
      {
        if (!rtc.IsDateTimeValid())
        {
          Serial.println("Clock disconnected. Likely power issue.");
        }
        RtcTemperature temperature = rtc.GetTemperature();  //read temperature
        float rtc_temp = temperature.AsFloatDegF();
        Serial.print("Clock Temp: ");
        Serial.print(rtc_temp);
        Serial.println(" F");
      }
      break;
  }
}

void stop_object(uint8_t object_id)
{
  switch (object_id)
  {
    case o_lcd:
      {
      }
      break;
    case o_temp:
      {
      }
      break;
    case o_relay:
      {
      }
      break;
    case o_clock:
      {
      }
      break;
  }
}

void loop_objects()
{
  for (uint8_t i = 0; i < object_count; i++)
  {
    if (object_enabled[i])//don't even try if the object is disabled
    {
      if (object_awake[0][i])//if we want the object to be awake,
      {
        if (object_awake[1][i])//if the object is already awake,
        {
          if ((abs(millis() - object_mtimer[0][i]) > object_mtimer[1][i])) //if enough time has passed since the object was last updated
          {
            update_object(i);//use a switch/case function to do whatever ongoing maintenance is needed for the object.
            object_mtimer[0][i] = millis();
          }
        }
        else //if the object is sleeping but we want to wake it up,
        {
          start_object(i);//use a switch/case function to do any initial setup required when the object starts
          object_awake[1][i] = true; //mark the object as having been started
          Serial.print(object_name[i]);
          Serial.println(" object woke up");
        }
      }
      else//if we want the object to sleep,
      {
        if (object_awake[1][i])//if the object is awake and we want it to sleep,
        {
          stop_object(i);//do any cleanup required to stop the object properly
          object_awake[1][i] = false;//mark the object as having been stopped
          Serial.print(object_name[i]);
          Serial.println(" object is now sleeping");
        }
      }
    }
  }
}

void print_status()
{
  Serial.print("I have been online for: ");
  Serial.print(uptime);
  Serial.println(" minutes.");
  for (uint8_t i = 0; i < object_count; i++)
  {
    Serial.print(object_name[i]);
    Serial.print(" status: ");
    Serial.print(object_status[0][i]);
    Serial.print(object_status[1][i]);
    Serial.print(", wakeup status: ");
    Serial.print(object_awake[0][i]);
    Serial.println(object_awake[1][i]);
  }
  Serial.println("");
}

void lcd_print(char* line1, char* line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  Serial.print("Sent text to LCD: ");
  Serial.print(line1);
  Serial.print(" ");
  Serial.println(line2);
}

void print_time(RtcDateTime currentTime)
{
  char str[15];   //declare a string as an array of chars
  sprintf(str, "%d/%d/%d %d:%d:%d",     //%d allows to print an integer to the string
          currentTime.Month(),  //get month method
          currentTime.Day(),    //get day method
          currentTime.Year(),   //get year method
          currentTime.Hour(),   //get hour method
          currentTime.Minute(), //get minute method
          currentTime.Second()  //get second method
         );
  Serial.print(currentTime.DayOfWeek());
  Serial.print(" ");
  Serial.println(str); //print the time and date to the serial port
}

/*
  // Default function for sending sensor data at intervals to Cayenne.
  // You can also use functions for specific channels, e.g CAYENNE_OUT(1) for sending channel 1 data.
  CAYENNE_OUT_DEFAULT()
  {
  // Write data to Cayenne here. This example just sends the current uptime in milliseconds on virtual channel 0.
  Cayenne.virtualWrite(0, millis());
  // Some examples of other functions you can use to send data.
  //Cayenne.celsiusWrite(1, 22.0);
  //Cayenne.luxWrite(2, 700);
  //Cayenne.virtualWrite(3, 50, TYPE_PROXIMITY, UNIT_CENTIMETER);
  }

  // Default function for processing actuator commands from the Cayenne Dashboard.
  // You can also use functions for specific channels, e.g CAYENNE_IN(1) for channel 1 commands.
  CAYENNE_IN_DEFAULT()
  {
  CAYENNE_LOG("Channel %u, value %s", request.channel, getValue.asString());
  //Process message here. If there is an error set an error message using getValue.setError(), e.g getValue.setError("Error message");
  }
*/
void eeprom_save(int addr, uint8_t val)
{
  EEPROM.write(addr, val);
  EEPROM.commit();
  yield();
}

uint8_t eeprom_load(int addr)
{
  return EEPROM.read(addr);
}

void goto_sleep(int SLEEPTIME)
{
  WiFi.disconnect(true);
  yield();
  ESP.deepSleep(SLEEPTIME, WAKE_RF_DISABLED);// WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up
}

void wifi_stop()
{
  WiFi.mode(WIFI_OFF);//no need for wifi yet
  WiFi.forceSleepBegin();
  yield();
}

void setup_wifi()
{
  WiFi.forceSleepWake();
  yield();

  WiFi.persistent(true);//wifi.begin doesn't use stored credentials, but wifi manager does.
  WiFi.mode(WIFI_STA);
  yield();
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(30);//try last good settings after 30 seconds
  /*//this code forces the module into wifi login mode.
    if (!wifiManager.startConfigPortal("OnDemandAP"))
    {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
    }*/
  wifiManager.autoConnect("AutoConnectAP");//https://github.com/tzapu/WiFiManager
}

void setup_ota()
{
  ArduinoOTA.onStart([]() {
    Serial.println("Start OTA");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("End OTA");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void eeprom_test()
{
  //read the first 1024 bytes of flash memory to serial.
  //empty bytes will read "255" over and over.
  Serial.println("Eeprom test: ");
  for (uint8_t j = 0; j < 32; j++)
  {
    for (uint8_t i = j; i < (j + 32); i++)
    {
      Serial.print(eeprom_load(i));
    }
    Serial.println();
  }
}

void setup()
{
  wifi_stop();//by default, wifi is on during setup. we turn it off until we need it.
  setup_serial();
  //it is possible that the first few hundred bytes of EEPROM are used to store wifi credentials...
  EEPROM.begin(1024);//we could use up to 4096 bytes. not sure if maxing it out is a good idea.
  //eeprom_save(10, 5);
  pinMode(pin_led_builtin, OUTPUT);
  for (uint8_t i = 0; i < object_count; i++)
  {
    setup_object(i);
    if (object_enabled[o_lcd])
    {
      lcd.clear();
      lcd_print(object_name[i], "Online");//this only works if lcd is object 0.
    }
    delay(850);
  }
  //eeprom_test();
  setup_wifi();
  setup_ota();
}

void loop()
{
  ArduinoOTA.handle();//watch for any updates being pushed over LAN
  loop_objects();
  //not ready yet: Cayenne.loop();
}
