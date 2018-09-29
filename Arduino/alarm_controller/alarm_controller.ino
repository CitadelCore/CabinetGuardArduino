#include <Wire.h>
#include <LiquidCrystal.h>
#include <Timer.h>
#include <ArduinoJson.h>
#include <avr/io.h> 
#include <avr/wdt.h>

/* CoreCabinet FMC controller code.
 *  ALL RIGHTS RESERVED UNLESS STATED OTHERWISE.
 *  NOT TO BE REDISTRIBUTED, SOLD, OR MODIFIED.
 * © 2018 Joseph Marsden <josephmarsden@towerdevs.xyz> 
*/

#define BUTTON_ADC_PIN A0
#define LCD_BACKLIGHT_PIN 3
#define PIEZO_PIN 13 // Digital pin set to D13. This is also the D13 LED pin, so the LED will sync up with the beeps.

// ADC values for button selection
#define RIGHT_10BIT_ADC           0  // right
#define UP_10BIT_ADC            145  // up
#define DOWN_10BIT_ADC          329  // down
#define LEFT_10BIT_ADC          505  // left
#define SELECT_10BIT_ADC        741  // right
#define BUTTONHYSTERESIS         10  // hysteresis for valid button sensing window

// Button status constants
#define BUTTON_NONE               0  // 
#define BUTTON_RIGHT              1  // 
#define BUTTON_UP                 2  // 
#define BUTTON_DOWN               3  // 
#define BUTTON_LEFT               4  // 
#define BUTTON_SELECT             5  // 

// Backlight macros
#define LCD_BACKLIGHT_OFF()     digitalWrite LCD_BACKLIGHT_PIN, LOW)
#define LCD_BACKLIGHT_ON()      digitalWrite(LCD_BACKLIGHT_PIN, HIGH)
#define LCD_BACKLIGHT(state)    { if(state){digitalWrite(LCD_BACKLIGHT_PIN, HIGH);}else{digitalWrite(LCD_BACKLIGHT_PIN, LOW);} }
void CONFIRMATION_BEEP()     { tone(PIEZO_PIN, 880, 250); }
void ERROR_BEEP_ONCE()            { tone(PIEZO_PIN, 600, 100); }
void ERROR_BEEP()            { ERROR_BEEP_ONCE(); delay(1000); ERROR_BEEP_ONCE(); }
void ALARM_BEEP()            { tone(PIEZO_PIN, 300, 250); }
void CLEAR_LCD_LINE(LiquidCrystal lcd, int line) { lcd.setCursor(0, line); lcd.print(F("                ")); }

DynamicJsonBuffer *jsonBuffer = new DynamicJsonBuffer(256);
DynamicJsonBuffer& GET_JSON_BUFFER() { return *jsonBuffer; }
void PRINT_SERIAL_JSON(JsonObject& json) { json.printTo(Serial); Serial.println(); }
void SEND_JSON_STATUS(String status) { DynamicJsonBuffer& jsonBuffer = GET_JSON_BUFFER(); JsonObject& root = jsonBuffer.createObject(); root["status"] = status; PRINT_SERIAL_JSON(root); jsonBuffer.clear(); }
void SEND_CMD_JSON_STATUS(String status, String command) { DynamicJsonBuffer& jsonBuffer = GET_JSON_BUFFER(); JsonObject& root = jsonBuffer.createObject(); root["status"] = status; root["command"] = command; PRINT_SERIAL_JSON(root); jsonBuffer.clear(); }

// Alarm definitions
#define SENSOR_GRACE_PERIOD 5000 // Period in miliseconds a user has when a sensor is triggered to either disarm the alarm or put the sensor back into place.
#define ALARM_SILENCE_PERIOD 60L // Period in seconds until the alarm resets silences itself after it's triggered. This period is reset if the alarm is triggered again.
#define ALARM_SENSOR_PIN 2 // PIN that should always be HIGH
#define ALARM_OVERRIDE_PIN 11 // When this pin goes HIGH the alarm will be disarmed
#define ALARM_TAMPER_PIN 12 // PIN that should always be HIGH
#define ALARM_PANIC_PIN 10 // Optional PIN that should be connected to a momentary switch.

// Version
String CONTROLLER_VERSION = "v1.0";

/* System timers */
Timer graceTimer;
Timer graceBeepTimer;
Timer alarmBeepTimer;
Timer alarmTimeoutTimer;
Timer pollerTimer;

int graceTimerId;
int graceBeepTimerId;
int alarmBeepTimerId;
int alarmTimeoutTimerId;
int pollerTimerId;

/* Variables */

bool buttonJustPressed  = false;         //this will be true after a ReadButtons() call if triggered
bool buttonJustReleased = false;         //this will be true after a ReadButtons() call if triggered
byte buttonWas          = BUTTON_NONE;   //used by ReadButtons() for detection of button events
String incomingString;

bool inGracePeriod = false;

// Whether alarm system is armed
// TODO: Store this in EEPROM?
bool armed = false;

// Whether alarm system is alarmed
bool alarmed = false;

// Whether arming is overridden by keyswitch
bool keyswitchOverride = false;

// Whether arming is overridden by CMC
bool remoteOverride = false;

bool masterOverride = false;
bool armedBeforeOverride = false;
bool armOverride = false;

// Whether the controller has been polled in the last 10 seconds
bool polledRecently = false;

// Initialize display
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

/* Methods */

// Actually triggers the alarm
void TriggerAlarm() {
  if (!armed) { return; }
  
  SEND_JSON_STATUS(F("alert"));
  alarmBeepTimer.stop(alarmBeepTimerId);
  graceBeepTimer.stop(graceBeepTimerId);
  graceTimer.stop(graceTimerId);
  alarmTimeoutTimer.stop(alarmTimeoutTimerId);
  inGracePeriod = false;
  alarmed = true;

  alarmBeepTimerId = alarmBeepTimer.every(500, ALARM_BEEP, ALARM_SILENCE_PERIOD * 2);
  alarmTimeoutTimerId = alarmTimeoutTimer.after((unsigned int)(ALARM_SILENCE_PERIOD * 1000), Silence);

  CLEAR_LCD_LINE(lcd, 0);
  lcd.setCursor(0, 0);
  lcd.print(F("STATE: SYS ALERT"));
}

// Starts alarm grace period
void Alarm() {
  // Don't re-start the timer if we're in the period or alarmed, or we are not armed
  if (!inGracePeriod && !alarmed && armed) {
    inGracePeriod = true;
    graceTimerId = graceTimer.after(SENSOR_GRACE_PERIOD, TriggerAlarm);
    graceBeepTimerId = graceBeepTimer.every(200, ERROR_BEEP_ONCE, 25);
    }

  if (!armed) {
    SEND_JSON_STATUS(F("alarm_activated_not_armed"));
    }
  }

void SensorTriggered() {
  SEND_JSON_STATUS(F("sensor_triggered"));
  Alarm();
  }

void SensorUntriggered() {
  SEND_JSON_STATUS(F("sensor_untriggered"));
  inGracePeriod = false;
  graceTimer.stop(graceTimerId);
  graceBeepTimer.stop(graceBeepTimerId);
  }
  
void Silence() {
  alarmed = false;
  alarmBeepTimer.stop(alarmBeepTimerId);
  alarmTimeoutTimer.stop(alarmTimeoutTimerId);

  if (armed) {
    Arm();
    }

  if (!armed) {
    Disarm();
    }
  }
  
void Arm() {
  if (masterOverride) { return; }
  
  if (!alarmed && !masterOverride) {
    CLEAR_LCD_LINE(lcd, 0);
    lcd.setCursor(0, 0);
    lcd.print(F("STATE: ARMED"));
  }
  
  armed = true;
}
void Disarm() {
  if (masterOverride) { return; }
  
  if (!alarmed && !masterOverride) {
    CLEAR_LCD_LINE(lcd, 0);
    lcd.setCursor(0, 0);
    lcd.print(F("STATE: DISARMED"));
  }
    
  armed = false;
}

void PollTimeout() {
  if (polledRecently) {
      polledRecently = false;
    } else {
      CLEAR_LCD_LINE(lcd, 1);
      lcd.setCursor(0, 1);
      lcd.print(F("CMC NOT POLLING"));
      }
  }

void SetOverride(bool setOverride, bool arm) {
  if (setOverride) {
    armOverride = arm;
    
    CLEAR_LCD_LINE(lcd, 0);
    lcd.setCursor(0, 0);
    lcd.print(F("STATE: OVERRIDE"));
    
    armedBeforeOverride = armed;
    masterOverride = true;
    if (arm == true) {
      Arm();
    } else {
      Disarm();
    }
  } else {
    masterOverride = false;
    if (armedBeforeOverride == true) {
      Arm();
      } else {
      Disarm();
      }
  }
}

void setup() {
  MCUSR &= ~_BV(WDRF);
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;
  
  // Setup serial
  Serial.begin(9600);
  while (!Serial) {
    ;
    }
    
  SEND_JSON_STATUS(F("system_initialization"));
  
  // Setup ADC pins
  pinMode(BUTTON_ADC_PIN, INPUT);
  digitalWrite(BUTTON_ADC_PIN, LOW);

  // Setup backlight pins
  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);

  pinMode(ALARM_SENSOR_PIN, INPUT);
  pinMode(ALARM_TAMPER_PIN, INPUT);
  
  lcd.begin(16, 2);
  CLEAR_LCD_LINE(lcd, 0);
  lcd.setCursor(0, 0);

  if (digitalRead(ALARM_OVERRIDE_PIN) == HIGH && !keyswitchOverride) { SEND_JSON_STATUS(F("force_disarmed")); keyswitchOverride = true; SetOverride(true, false); }
  
  Disarm();
  
  // Beep for startup confirmation
  CONFIRMATION_BEEP();
  SEND_JSON_STATUS(F("system_ready"));

  pollerTimerId = pollerTimer.every(20000, PollTimeout);
  PollTimeout();
}

void loop() {
  graceTimer.update();
  graceBeepTimer.update();
  alarmBeepTimer.update();
  alarmTimeoutTimer.update();
  pollerTimer.update();
  
  //if ((digitalRead(ALARM_SENSOR_PIN) == LOW || digitalRead(ALARM_TAMPER_PIN) == LOW) && !alarmed && !inGracePeriod && armed) { SensorTriggered(); } else if ((digitalRead(ALARM_SENSOR_PIN) == HIGH && digitalRead(ALARM_TAMPER_PIN) == HIGH) && inGracePeriod) { SensorUntriggered(); }
  if ((digitalRead(ALARM_SENSOR_PIN) == LOW) && !alarmed && !inGracePeriod && armed) { SensorTriggered(); } else if ((digitalRead(ALARM_SENSOR_PIN) == HIGH) && inGracePeriod) { SensorUntriggered(); }
  if (digitalRead(ALARM_TAMPER_PIN) == LOW && !alarmed) { SEND_JSON_STATUS(F("tamper_detection_tripped")); TriggerAlarm(); } 
  if (digitalRead(ALARM_PANIC_PIN) == HIGH && !alarmed && armed) { SEND_JSON_STATUS(F("panic_button_tripped")); TriggerAlarm(); }
  if (digitalRead(ALARM_OVERRIDE_PIN) == HIGH && !keyswitchOverride && !remoteOverride) { SEND_JSON_STATUS(F("keyswitch_override")); keyswitchOverride = true; SetOverride(true, false); }
  if (digitalRead(ALARM_OVERRIDE_PIN) == LOW && keyswitchOverride && !remoteOverride) { SEND_JSON_STATUS(F("keyswitch_nooverride")); keyswitchOverride = false; SetOverride(false, false); }
  
  if (Serial.available() > 0) {
    incomingString = Serial.readString();
    bool wasInvalid = false;

    if (incomingString == F("help")) {
      DynamicJsonBuffer& jsonBuffer = GET_JSON_BUFFER();
      JsonObject& root = jsonBuffer.createObject(); 
      root["status"] = F("commands");
      JsonArray& cmds = root.createNestedArray(F("commands"));
      cmds.add(F("help"));
      cmds.add(F("alarm"));
      cmds.add(F("silence"));
      cmds.add(F("arm"));
      cmds.add(F("disarm"));
      cmds.add(F("reset"));
      PRINT_SERIAL_JSON(root);
      jsonBuffer.clear();
      } else if (incomingString == F("beep")) {
        CONFIRMATION_BEEP();
      } else if (incomingString == F("alarm")) {
        TriggerAlarm();
      } else if (incomingString == F("silence")) {
        Silence();
      } else if (incomingString == F("arm")) {
        Arm();
      } else if (incomingString == F("disarm")) {
        Disarm();
      } else if (incomingString == F("reset")) {
        wdt_enable(WDTO_1S);
      } else if (incomingString == F("override")) {
        remoteOverride = true;
        keyswitchOverride = false;
        SetOverride(true, armed);
      } else if (incomingString == F("cancel_override")) {
        remoteOverride = false;
        keyswitchOverride = false;
        SetOverride(false, armed);
      } else if (incomingString == F("syncStatus")) {
        // Called by the remote controller
        // Syncs alarm status
        DynamicJsonBuffer& statusJsonBuffer = GET_JSON_BUFFER();
        JsonObject& statusRoot = statusJsonBuffer.createObject();
        statusRoot["status"] = F("syncStatus");
        JsonArray& statusValues = statusRoot.createNestedArray(F("statusValues"));

        if (alarmed == true) {
          statusValues.add(F("alarmed"));
        } else {
          statusValues.add(F("silenced"));
        }

        // We still want the FMC to report armed status,
        // even if it is in an override condition and forced disarmed but armed previously.
        if (armed == true || (masterOverride && armedBeforeOverride)) 
        {
          statusValues.add(F("armed"));
        } else {
          statusValues.add(F("disarmed"));
        }

        // Report the override status.
        if (masterOverride == true)
        {
          statusValues.add(F("override"));
        } else {
          statusValues.add(F("no_override"));
        }
        //else if (digitalRead(ALARM_OVERRIDE_PIN) == LOW && keyswitchOverride == false) 
        //{
          //statusValues.add(F("force_armed"));
        //}

        PRINT_SERIAL_JSON(statusRoot);
        polledRecently = true;
        CLEAR_LCD_LINE(lcd, 1);
        lcd.setCursor(0, 1);
        lcd.print(F("CMC HEARTBEAT OK"));
      } else {
        wasInvalid = true;
      }

    if (!wasInvalid) { SEND_CMD_JSON_STATUS(F("success"), incomingString); } else { SEND_CMD_JSON_STATUS(F("failure"), incomingString); }
  }
  
  // put your main code here, to run repeatedly:
  byte button; // Button pressed in the last ReadButtons() cycle
  byte timestamp;

  button = ReadButtons();

  if (buttonJustPressed || buttonJustReleased) {
    CLEAR_LCD_LINE(lcd, 1);
    }

  switch(button) {
    case BUTTON_NONE: {
      break;
      }
    case BUTTON_RIGHT: {
      break;
      }
    case BUTTON_UP: {
      break;
      }
    case BUTTON_DOWN: {
      break;
      }
    case BUTTON_LEFT: {
      break;
      }
    case BUTTON_SELECT: {
      lcd.setCursor(0, 1);
      lcd.print(F("No connection."));
      break;
      }
    default: {
      break;
      }
    }
}

/*--------------------------------------------------------------------------------------
  ReadButtons()
  Detect the button pressed and return the value
  Uses global values buttonWas, buttonJustPressed, buttonJustReleased.
--------------------------------------------------------------------------------------*/
byte ReadButtons()
{
   unsigned int buttonVoltage;
   byte button = BUTTON_NONE;   // return no button pressed if the below checks don't write to btn
   
   //read the button ADC pin voltage
   buttonVoltage = analogRead( BUTTON_ADC_PIN );
   //sense if the voltage falls within valid voltage windows
   if( buttonVoltage < ( RIGHT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_RIGHT;
   }
   else if(   buttonVoltage >= ( UP_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( UP_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_UP;
   }
   else if(   buttonVoltage >= ( DOWN_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( DOWN_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_DOWN;
   }
   else if(   buttonVoltage >= ( LEFT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( LEFT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_LEFT;
   }
   else if(   buttonVoltage >= ( SELECT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( SELECT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_SELECT;
   }
   //handle button flags for just pressed and just released events
   if( ( buttonWas == BUTTON_NONE ) && ( button != BUTTON_NONE ) )
   {
      //the button was just pressed, set buttonJustPressed, this can optionally be used to trigger a once-off action for a button press event
      //it's the duty of the receiver to clear these flags if it wants to detect a new button change event
      buttonJustPressed  = true;
      buttonJustReleased = false;
   }
   if( ( buttonWas != BUTTON_NONE ) && ( button == BUTTON_NONE ) )
   {
      buttonJustPressed  = false;
      buttonJustReleased = true;
   }
   
   //save the latest button value, for change event detection next time round
   buttonWas = button;
   
   return(button);
}
