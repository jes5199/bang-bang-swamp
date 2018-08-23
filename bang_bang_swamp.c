// Select "Arduino/Genuino Uno" in the Arduino IDE

#define eepromVersion 13

#define waterLevelPin A1
#define waterLevelCalibrationPin A2
#define buttonPin A3

#define refillPumpPin 3
#define bilgePumpPin 5
#define blowerPin 6
#define pixelPin 11
#define maxPin 11

#define buttonReadingCount 5
#define upButton    1
#define rightButton 2
#define downButton  3
#define leftButton  4

#define tcaAddr 0x71
#define sensorAddr 0x5C
#define sensorCount 3
#define sensorRetries 3
#define segmentAddr 0x70
#define sensorQueryDelay 10
#define sensorCooldown 2000

#define outsideSensorNumber 0
#define insideSensorNumber 1
#define blowerSensorNumber 2

// milliseconds
#define uiTimeout 5000
#define displayTimeout 5000

#define waterLevelsSetting 0
#define blowerSpeedSetting 1
#define targetTemperatureSetting 2
#define targetHumiditySetting 3

#define extraSettingsSetting  4
#define extraSettingsSetting2 5
#define extraSettingsSetting3 6
#define extraSettingsSetting4 7

#define minReservoirLevelSetting 8
#define maxReservoirLevelSetting 9
#define currentReservoirLevelSetting 10
#define bilgePumpRunTimeSetting 11
#define bilgePumpWaitTimeSetting 12
#define overheatSetting 13
#define temperatureSwingSetting 14
#define humiditySwingSetting 15
#define maxHumiditySetting 16
#define maxHumiditySwingSetting 17
#define refillPumpTimePerGallonSetting 18
#define factoryResetSetting 30

#define maxSetting 32

#define blowerSpeedStep 5
#define minBlowerSpeed 0
#define maxBlowerSpeed 255

#define bilgePumpTimeStep 5

#define maxExtraWater 0xF0
#define extraWaterStep 0x10
#define maxTemperature 99
#define minTemperature 32

#define maxHumidity 99
#define minHumidity 0

#define bottomPixel 1
#define refillStartPixel 12
#define refillStopPixel 14
#define pixelCount 16
#define fanAnimatationDelay 100
#define alertAnimationDelay 500

#define waterLevelReadingCount 11
#define waterSeriesResistorValue 580
#define waterOneInchResistanceRatio 0.95
#define waterCalibrationInchRatio 0.63
#define waterCalibrationInch 7

#define maxReservoirLevel 120
#define minReservoirLevel 10

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <stdbool.h>

#define uint8_t unsigned char

#else
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_NeoPixel.h>

Adafruit_7segment matrix = Adafruit_7segment();
Adafruit_NeoPixel strip = Adafruit_NeoPixel(pixelCount, pixelPin, NEO_GRBW + NEO_KHZ800);

#endif

struct SensorData {
  int temperature;
  int humidity;
};

struct SwampWater {
  int readingCount;
  int readingIndex;
  float readings[waterLevelReadingCount];
};

struct SwampData {
  struct SensorData outside;
  struct SensorData inside;
  struct SensorData blower;

  int reservoir;
};

struct SwampI2C {
  int selection;
  unsigned long sensor_query_time;
  int retries;
  int got_sensor_response;
  unsigned long sensor_last_read_time[sensorCount];
};

struct SwampPWM {
  int pin[maxPin];
};

struct SwampUI {
  int selection;
  unsigned long last_activity_time;
};

struct SwampSettings {
  int target_temperature;
  int temperature_swing;
  int overheat;

  int max_humidity;
  int max_humidity_swing;

  int target_humidity;
  int target_humidity_swing;

  int estimated_extra_water;

  uint8_t blower_pwm;

  int bilge_pump_run_time;
  int bilge_pump_wait_time;

  int min_reservoir_level;
  int max_reservoir_level;
  int reservoir_level_swing;

  int water_level_calibrate_tenths_of_inches;
  float water_level_calibrate_ratio;

  int refill_pump_time_per_gallon;
};

enum Temperature {
  COLD,
  COOL,
  HOT,
  OVERHEATED
};

enum Humidity {
  DRY,
  WET
};

enum WaterLevel {
  LOW_WATER,
  ENOUGH_WATER,
  HIGH_WATER
};

struct Sensor {
  enum Temperature temp;
  enum Humidity hum;
};

struct SwampSense {
  struct Sensor outside;
  struct Sensor inside;
  struct Sensor blower;
  enum WaterLevel reservoir;
  bool can_refill;
};

enum Power {
  OFF,
  ON,
};

struct Animation {
  int refill;
  int alert;
  unsigned long int color1;
  unsigned long int color2;
};

#define RED       0x1000
#define PINK      0x1001
#define BLUE      0x0010
#define BABYBLUE  0x0011
#define CYAN      0x0110
#define LIGHTCYAN 0x0111
#define YELLOW    0x1100
#define GREEN     0x0100
#define MAGENTA   0x1010
#define GREY      0x0001
#define WHITE     0x1110

struct SwampAction {
  enum Power blower_fan;
  enum Power bilge_pump;
  enum Power refill_pump;
  struct Animation lights;
};

struct SwampButtons {
  int pressed;
};

struct SwampTimers {
  unsigned long bilge_pump_start_time;
  unsigned long bilge_pump_stop_time;

  unsigned long refill_start_time;
};

#ifdef __EMSCRIPTEN__
unsigned long millis() {
  return EM_ASM_INT({return (new Date()).getTime()});
}
#endif

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  #ifdef __EMSCRIPTEN__
  EM_ASM(selectSensor($0), i);
  #else
  Wire.beginTransmission(tcaAddr);
  Wire.write(1 << i);
  Wire.endTransmission();
  //Serial.print("TCA "); Serial.println(i);
  #endif
}

void querySelectedSensor() {
  #ifdef __EMSCRIPTEN__
  EM_ASM(querySensor());
  #else
  Wire.beginTransmission(sensorAddr);
  Wire.write(3); //read
  Wire.write(0); //from 0
  Wire.write(4); //4 bytes
  Wire.endTransmission();
  #endif
}

bool readSelectedSensor(struct SensorData *data) {
  #ifdef __EMSCRIPTEN__
  data->temperature = EM_ASM_INT({return getSelectedSensor().temperature});
  data->humidity    = EM_ASM_INT({return getSelectedSensor().humidity   });

  #else
  uint8_t reply[8];
  int temperature;
  int humidity;

  Wire.requestFrom(sensorAddr, 8);
  for (uint8_t i=0; i<8; i++) {
    reply[i] = Wire.read();
  }

  if (reply[0] != 3) return false; // "read" message
  if (reply[1] != 4) return false; // bytes req'd

  humidity = reply[2];
  humidity *= 256;
  humidity += reply[3];
  humidity /= 10;

  temperature = reply[4] & 0x7F;
  temperature *= 256;
  temperature += reply[5];
  if (reply[4] >> 7) temperature = -temperature;

  temperature = temperature * 0.18 + 32; // convert 10x C to 1x F

  data->temperature = temperature;
  data->humidity    = humidity;
  #endif
  return true;
}

void magicallyReadAllSensors(struct SwampData *data) {
  // deprecated too easy
  #ifdef __EMSCRIPTEN__
  data->outside.temperature = EM_ASM_INT({return getSensor("outside").temperature});
  data->outside.humidity    = EM_ASM_INT({return getSensor("outside").humidity   });
  data->inside.temperature  = EM_ASM_INT({return getSensor("inside").temperature});
  data->inside.humidity     = EM_ASM_INT({return getSensor("inside").humidity   });
  data->blower.temperature  = EM_ASM_INT({return getSensor("blower").temperature});
  data->blower.humidity     = EM_ASM_INT({return getSensor("blower").humidity   });
  data->reservoir           = EM_ASM_INT({return getAnalog("reservoir")         });
  #endif
}

int isSensorFinished(struct SwampI2C i2c) {
  return (i2c.got_sensor_response || i2c.retries >= sensorRetries);
}

struct SensorData * getSensorNumber(int n, struct SwampData *data) {
  switch(n) {
    case outsideSensorNumber: return &data->outside;
    case insideSensorNumber:  return &data->inside;
    case blowerSensorNumber:  return &data->blower;
  }
  return NULL;
}

void doI2cReadStep(struct SwampI2C *i2c, struct SwampData *data) {
  if(isSensorFinished(*i2c)) {
    i2c->selection += 1;
    if(i2c->selection >= sensorCount) {
      i2c->selection = 0;
    }
    if(i2c->sensor_last_read_time[i2c->selection] + sensorCooldown > millis() ) {
      // too soon to read again
    } else {
      tcaSelect(i2c->selection);
      i2c->sensor_query_time = 0;
      i2c->got_sensor_response = 0;
      i2c->retries = 0;
    }
  } else if(!i2c->sensor_query_time) {
    //Serial.print("query sensor "); Serial.println(i2c->selection);
    querySelectedSensor();
    i2c->sensor_query_time = millis();
  } else if(millis() - i2c->sensor_query_time > sensorQueryDelay) {
    struct SensorData *sensor_data = getSensorNumber(i2c->selection, data);
    if(readSelectedSensor(sensor_data)) {
      i2c->got_sensor_response = 1;
      //Serial.print("got sensor "); Serial.println(i2c->selection);
    } else {
      //Serial.print("retry sensor "); Serial.println(i2c->selection);
      i2c->retries += 1;
      i2c->sensor_query_time = 0;
    }
    if(isSensorFinished(*i2c)) {
      i2c->sensor_last_read_time[i2c->selection] = millis();
    }
  }
}

float readWaterPinRatio() {
  int w_analog = analogRead(waterLevelPin);
  int c_analog = analogRead(waterLevelCalibrationPin);
  float w_resistance = waterSeriesResistorValue / ((1023.0 / w_analog) - 1.0);
  float c_resistance = waterSeriesResistorValue / ((1023.0 / c_analog) - 1.0);

  //Serial.print("W: "); Serial.println(w_resistance);
  //Serial.print("C: "); Serial.println(c_resistance);
  float ratio = w_resistance / c_resistance;
  return ratio;
}

float readWaterPin(struct SwampSettings settings) {
  float ratio = readWaterPinRatio();
  float inches = (10 +
                      (settings.water_level_calibrate_tenths_of_inches-10) * (ratio - waterOneInchResistanceRatio)
                    / (settings.water_level_calibrate_ratio - waterOneInchResistanceRatio)
                 ) / 10;
  //Serial.print(inches); Serial.println(" inch");
  return inches;
}

void readWaterLevel(struct SwampWater *water, struct SwampData *data, struct SwampSettings settings) {
  float value = readWaterPin(settings);
  water->readings[water->readingIndex] = value;
  water->readingIndex += 1;
  water->readingIndex %= waterLevelReadingCount;
  if(water->readingCount < waterLevelReadingCount) { water->readingCount += 1; }

  float sortedReadings[waterLevelReadingCount];
  for(int i = 0; i < water->readingCount; i++) { sortedReadings[i] = water->readings[i]; }
  for(int i = 0; i < water->readingCount; i++) {
    for(int j = i+1; j < water->readingCount; j++) {
      if(sortedReadings[i] > sortedReadings[j]) {
        float tmp = sortedReadings[i];
        sortedReadings[i] = sortedReadings[j];
        sortedReadings[j] = tmp;
      }
    }
  }

  float meanReading = sortedReadings[water->readingCount / 2];
  int readingInTenths = (meanReading + 0.5) * 10;
  //Serial.print(meanReading); Serial.println(" inch");

  data->reservoir = readingInTenths;
  //Serial.println(data->reservoir);
}

void readSensors(struct SwampI2C *i2c, struct SwampData *data, struct SwampWater *water, struct SwampSettings settings) {
  doI2cReadStep(i2c, data);
  readWaterLevel(water, data, settings);
}

void debuglog(char *str) {
  #ifdef __EMSCRIPTEN__
  EM_ASM(console.log(UTF8ToString($0)), str);
  #endif
}

void debugint(int val) {
  #ifdef __EMSCRIPTEN__
  EM_ASM(console.log($0), val);
  #endif
}

void debuglong(unsigned long val) {
  #ifdef __EMSCRIPTEN__
  EM_ASM(console.log($0), val);
  #endif
}

void interpretSensors(struct SwampData data, struct SwampSettings settings, struct SwampSense *sense) {
  if(sense->outside.temp == COLD) {
    if(data.outside.temperature > settings.target_temperature) {
      sense->outside.temp = COOL;
      debuglog("outside became COOL");
    }
  }
  if(sense->outside.temp == COOL) {
    if(data.outside.temperature > data.inside.temperature) {
      sense->outside.temp = HOT;
      debuglog("outside became HOT");
    }
    if(data.outside.temperature < settings.target_temperature - settings.temperature_swing) {
      sense->outside.temp = COLD;
      debuglog("outside became COLD");
    }
  }
  if(sense->outside.temp == HOT) {
    if(data.outside.temperature < data.inside.temperature - settings.temperature_swing) {
      sense->outside.temp = COOL;
      debuglog("outside became COOL");
    }
    if(data.outside.temperature > data.inside.temperature + settings.overheat) {
      sense->outside.temp = OVERHEATED;
      debuglog("outside became OVERHEATED");
    }
  }

  if(sense->blower.temp == COLD) {
    if(data.blower.temperature > settings.target_temperature) {
      sense->blower.temp = COOL;
      debuglog("blower became COOL");
    }
  }
  if(sense->blower.temp == COOL) {
    if(data.blower.temperature > data.inside.temperature) {
      sense->blower.temp = HOT;
      debuglog("blower became HOT");
    }
    if(data.blower.temperature < settings.target_temperature - settings.temperature_swing) {
      sense->blower.temp = COLD;
      debuglog("blower became COLD");
    }
  }
  if(sense->blower.temp == HOT) {
    if(data.blower.temperature < data.inside.temperature - settings.temperature_swing) {
      sense->blower.temp = COOL;
      debuglog("blower became COOL");
    }
  }

  if(sense->inside.temp == COLD) {
    if(data.inside.temperature > settings.target_temperature + settings.temperature_swing) {
      sense->inside.temp = HOT;
      debuglog("inside became HOT");
    }
  } else {
    if(data.inside.temperature <= settings.target_temperature) {
      sense->inside.temp = COLD;
      debuglog("inside became COLD");
    }
  }

  if(sense->outside.hum == DRY) {
    if(data.outside.humidity >= settings.target_humidity) {
      sense->outside.hum = WET;
      debuglog("outside became WET");
    }
  } else {
    if(data.outside.humidity < settings.target_humidity - settings.target_humidity_swing) {
      sense->outside.hum = DRY;
      debuglog("outside became DRY");
    }
  }

  if(sense->blower.hum == DRY) {
    if(data.blower.humidity >= settings.target_humidity) {
      sense->blower.hum = WET;
      debuglog("blower became WET");
    }
  } else {
    if(data.blower.humidity < settings.target_humidity - settings.target_humidity_swing) {
      sense->blower.hum = DRY;
      debuglog("blower became DRY");
    }
  }

  if(sense->inside.hum == DRY) {
    if(data.inside.humidity > settings.max_humidity) {
      sense->inside.hum = WET;
      debuglog("inside became WET");
    }
  } else {
    if(data.inside.humidity < settings.max_humidity - settings.max_humidity_swing) {
      sense->inside.hum = DRY;
      debuglog("inside became DRY");
    }
  }

  if(sense->reservoir != LOW_WATER && data.reservoir <= settings.min_reservoir_level) {
    sense->reservoir = LOW_WATER;
    debuglog("reservoir became LOW");
  }
  if(sense->reservoir == LOW_WATER && data.reservoir >= settings.min_reservoir_level + settings.reservoir_level_swing) {
      sense->reservoir = ENOUGH_WATER;
      debuglog("reservoir became ENOUGH");
  }
  if(sense->reservoir == ENOUGH_WATER && data.reservoir >= settings.max_reservoir_level) {
    sense->reservoir = HIGH_WATER;
    debuglog("reservoir became HIGH");
  }
  if(sense->reservoir == HIGH_WATER && sense->reservoir <= settings.max_reservoir_level - settings.reservoir_level_swing) {
      sense->reservoir = ENOUGH_WATER;
    debuglog("reservoir became ENOUGH");
  }

  sense->can_refill = settings.estimated_extra_water > 0;
}

void applyTruthTable(struct SwampSense sense, struct SwampAction *action) {
  if(   sense.inside.temp == HOT
     && sense.inside.hum  == WET
     && (sense.outside.temp == HOT || sense.outside.temp == OVERHEATED)) {
    // overhumid. red/blue alert and shutdown.
    // this is the only state that overrides refills
    action->blower_fan = OFF;
    action->bilge_pump = OFF;
    action->refill_pump = OFF;
    action->lights.color1 = RED;
    action->lights.color2 = BLUE;
    action->lights.refill = 0;
    action->lights.alert = 1;
    return;
  }

  if(sense.reservoir != HIGH_WATER && sense.can_refill) {
    // refill!
    action->refill_pump = ON;
    action->lights.refill = 1;
  } else {
    action->refill_pump = OFF;
    action->lights.refill = 0;
  }

  if(sense.inside.temp == COLD) {
    // If it's cold indoors, no reason to run anything
    // except maybe refill, which is above.
    action->blower_fan = OFF;
    action->bilge_pump = OFF;
    action->lights.alert = 0;
    action->lights.color1 = 0;
    action->lights.color2 = 0;
    return;
  }

  if(sense.outside.temp == COLD) {
    // if it's cold out, no need for waterworks
    // maybe we can dry out a little
    action->blower_fan = ON;
    action->bilge_pump = OFF;
    action->lights.alert = 0;
    action->lights.color1 = GREY;
    action->lights.color2 = 0;
    return;
  }

  if(sense.outside.temp == COOL) {
    // if it's cool outside, it's worth blowing dry
    action->blower_fan = ON;
    action->lights.alert = 0;

    if(sense.inside.hum == WET) {
      // and useful to dry us out some if we're over the limit
      action->bilge_pump = OFF;
      action->lights.color1 = MAGENTA;
      action->lights.color2 = 0;
    } else {
      // but it's usually better to blow wet
      action->lights.color1 = GREEN;
      if(sense.blower.hum == DRY) {
        action->bilge_pump = ON;
        if(sense.reservoir != LOW_WATER) {
          action->lights.color2 = BLUE;
        } else {
          action->lights.color2 = YELLOW;
        }
      } else {
        action->bilge_pump = OFF;
        action->lights.color2 = 0;
      }
    }
    return;
  }

  // OK. It's hot out. Normal operation.

  if(sense.blower.temp == HOT) {
    if(sense.outside.temp == OVERHEATED) {
      action->bilge_pump = OFF;
      action->blower_fan = OFF;
      action->lights.color1 = RED;
      action->lights.color2 = GREY;
    } else {
      // maybe we're bootstrapping
      action->blower_fan = ON;
      // blowing hot air is a problem situation
      action->lights.alert = 1;

      if(sense.blower.hum == WET) {
        // blowing wet, hot air? Overheated!! add ice??
        action->bilge_pump = OFF;
        action->lights.alert = 1;
        action->lights.color1 = RED;
        action->lights.color2 = 0;
        return;
      } else {
        action->bilge_pump = ON;
        action->lights.color1 = YELLOW;
        // blowing dry, hot air? Add water!
        if(sense.reservoir != LOW_WATER) {
          // if we have water
          action->lights.color2 = BLUE;
        } else {
          // well. dry but maybe we can refill.
          action->lights.color2 = 0;
        }
      }
    }
    return;
  }

  // okay, gonna blow some cool or cold air
  action->blower_fan = ON;
  action->lights.alert = 0;

  if(sense.blower.temp == COOL) {
    // Not quite cold. Might count as hot once the indoor temp drops a little.
    action->lights.color1 = PINK;
  } else {
    action->lights.color1 = WHITE;
  }

  if(sense.blower.hum == DRY) {
    // run water
    action->bilge_pump = ON;
    if(sense.reservoir != LOW_WATER) {
      action->lights.color2 = BLUE;
    } else {
      action->lights.color2 = YELLOW;
    }
  } else {
    action->bilge_pump = OFF;
    action->lights.color2 = 0;
  }
}

void turnOffBlower(struct SwampPWM *pwm) {
  #ifdef __EMSCRIPTEN__
  EM_ASM({blower = false;});
  #else
  digitalWrite(blowerPin, LOW);
  pwm->pin[blowerPin] = 0;
  #endif
}

void turnOnBlower(enum Power level, struct SwampSettings settings, struct SwampPWM *pwm) {
  if(level == ON && settings.blower_pwm) {
    #ifdef __EMSCRIPTEN__
    EM_ASM({blower = $0;}, settings.blower_pwm);
    #else
    if(pwm->pin[blowerPin] != settings.blower_pwm) {
      analogWrite(blowerPin, settings.blower_pwm);
      pwm->pin[blowerPin] = settings.blower_pwm;
    }
    #endif
  } else {
    turnOffBlower(pwm);
  }
}

void turnOnBilgePump() {
  #ifdef __EMSCRIPTEN__
  EM_ASM({bilge_pump = true});
  #else
  digitalWrite(bilgePumpPin, HIGH);
  #endif
}

void turnOffBilgePump() {
  #ifdef __EMSCRIPTEN__
  EM_ASM({bilge_pump = false});
  #else
  digitalWrite(bilgePumpPin, LOW);
  #endif
}

void turnOnRefillPump() {
  #ifdef __EMSCRIPTEN__
  EM_ASM({refill_pump = true});
  #else
  Serial.println("turn on refill");
  digitalWrite(refillPumpPin, HIGH);
  #endif
}

void turnOffRefillPump() {
  #ifdef __EMSCRIPTEN__
  EM_ASM({refill_pump = false});
  #else
  Serial.println("turn off refill");
  digitalWrite(refillPumpPin, LOW);
  #endif
}

void dutyCycle(unsigned long now, unsigned long *start_time, unsigned long *stop_time, int run_time, int wait_time) {
    if(*start_time) {
      if(*start_time + run_time * 1000 < now) {
        *start_time = 0;
        *stop_time = now;
      }
    }
    if(*stop_time) {
      if(*stop_time + wait_time * 1000 <= now) {
        *start_time = now;
        *stop_time = 0;
      }
    }
}

void takeAction(struct SwampAction action, struct SwampSettings settings, struct SwampTimers *timers, struct SwampPWM *pwm) {
  unsigned long now = millis();

  if(action.blower_fan == OFF){
    turnOffBlower(pwm);
  } else {
    turnOnBlower(action.blower_fan, settings, pwm);
  }

  if(action.bilge_pump == OFF){
    timers->bilge_pump_start_time = 0;
    if(! timers->bilge_pump_stop_time){
      timers->bilge_pump_stop_time = now;
    }
  } else {
    if(!timers->bilge_pump_start_time && !timers->bilge_pump_stop_time) {
      timers->bilge_pump_start_time = now;
    } else { // ON
      dutyCycle(now, &timers->bilge_pump_start_time, &timers->bilge_pump_stop_time,
                settings.bilge_pump_run_time, settings.bilge_pump_wait_time);
    }
  }
  if(timers->bilge_pump_start_time) {
    turnOnBilgePump();
  } else {
    turnOffBilgePump();
  }

  if(action.refill_pump == ON) {
    if(! timers->refill_start_time) {
      turnOnRefillPump();
      timers->refill_start_time = now;
    }
  } else {
    turnOffRefillPump();
    timers->refill_start_time = 0;
  }
}

void estimateRefill(struct SwampSettings *settings, struct SwampTimers *timers) {
  if(! timers->refill_start_time) { return; }
  unsigned long now = millis();

  long int timeUnit = 4 * settings->refill_pump_time_per_gallon;
  if(timeUnit < 1) { timeUnit = 1; }
  if(timers->refill_start_time + timeUnit <= now) {
    timers->refill_start_time += timeUnit;
    //Serial.println(settings->estimated_extra_water);
    if(settings->estimated_extra_water > 0) {
      settings->estimated_extra_water -= 1;
      if(settings->estimated_extra_water % 16 == 0) {
        writeEEPROM(settings);
      }
    }
  }
}

void readButtons(struct SwampButtons *buttons) {
  #ifdef __EMSCRIPTEN__
  buttons->pressed = EM_ASM_INT({return getButtons()});
  #else
  int input = analogRead(buttonPin);
  static int button = 0;
  static int oldButton = 0;
  static int newButton = 0;
  static int newCount = 0;

  if(input > 1000) {
    button = 0;
  } else if(input < 20) {
    button = 1;
  } else if(input < 50) {
    button = 2;
  } else if(input < 70) {
    button = 3;
  } else if(input < 100) {
    button = 4;
  } else {
    button = 0;
  }
  if(button == newButton) {
    newCount += 1;
  } else {
    newCount = 0;
    newButton = button;
  }

  if(newCount > buttonReadingCount && newButton != oldButton) {
    oldButton = newButton;
    if(newButton != 0) {
      Serial.print("button "); Serial.println(newButton);
      buttons->pressed = newButton;
    }
  }
  #endif
}

void writeEEPROM(struct SwampSettings *settings) {
#ifdef __EMSCRIPTEN__
  //todo
#else
  EEPROM.write(0, eepromVersion);
  char *ptr;
  ptr = (char*)settings;

  for(int i = 0; i < sizeof(SwampSettings); i++) {
    EEPROM.write(1 + i, *(ptr+i));
  }
#endif
}

void readEEPROM(struct SwampSettings *settings) {
#ifdef __EMSCRIPTEN__
  //todo
#else
  char version;
  version = EEPROM.read(0);
  if(version != eepromVersion) { return; }

  Serial.println("Found a valid-looking EEPROM");

  char *ptr;
  ptr = (char*)settings;

  for(int i = 0; i < sizeof(SwampSettings); i++) {
    *(ptr+i) = EEPROM.read(1 + i);
  }
#endif
}

void doUI(struct SwampUI *ui, struct SwampButtons *buttons, struct SwampSettings *settings) {
  if(buttons->pressed) {
    if(buttons->pressed == leftButton) {
      if(ui->selection >= extraSettingsSetting && ui->selection <= extraSettingsSetting4) {
        ui->selection = extraSettingsSetting - 1;
      } else {
        if(ui->selection > 0) { ui->selection -= 1; }
      }
    } else if(buttons->pressed == rightButton) {
      if(ui->selection >= extraSettingsSetting && ui->selection <= extraSettingsSetting4) {
        ui->selection = extraSettingsSetting;
      } else {
        if( ui->selection < maxSetting
          ) {
          ui->selection += 1;
        }
      }
    } else { // up or down
      if(ui->selection == waterLevelsSetting) {
        if(buttons->pressed == upButton) {
          if(settings->estimated_extra_water < maxExtraWater - extraWaterStep) {
            settings->estimated_extra_water += extraWaterStep;
          } else {
            settings->estimated_extra_water = maxExtraWater;
          }
        }
        if(buttons->pressed == downButton) {
          if(settings->estimated_extra_water > extraWaterStep) {
            settings->estimated_extra_water -= extraWaterStep;
          } else {
            settings->estimated_extra_water = 0;
          }
        }
      }

      if(ui->selection == blowerSpeedSetting) {
        if(buttons->pressed == upButton) {
          if(settings->blower_pwm < maxBlowerSpeed) {
            settings->blower_pwm += blowerSpeedStep;
          }
        }
        if(buttons->pressed == downButton) {
          if(settings->blower_pwm > minBlowerSpeed) {
            settings->blower_pwm -= blowerSpeedStep;
          }
        }
      }

      if(ui->selection == targetTemperatureSetting) {
        if(buttons->pressed == upButton) {
          if(settings->target_temperature < maxTemperature) {
            settings->target_temperature += 1;
          }
        }
        if(buttons->pressed == downButton) {
          if(settings->target_temperature > minTemperature) {
            settings->target_temperature -= 1;
          }
        }
      }

      if(ui->selection == targetHumiditySetting) {
        if(buttons->pressed == upButton) {
          if(settings->target_humidity < maxHumidity) {
            settings->target_humidity += 1;
          }
        }
        if(buttons->pressed == downButton) {
          if(settings->target_humidity > minHumidity) {
            settings->target_humidity -= 1;
          }
        }
      }

      if(ui->selection == minReservoirLevelSetting) {
        if(buttons->pressed == upButton) {
          if(settings->min_reservoir_level < maxReservoirLevel) {
            settings->min_reservoir_level += 1;
          }
        }
        if(buttons->pressed == downButton) {
          if(settings->min_reservoir_level > minReservoirLevel) {
            settings->min_reservoir_level -= 1;
          }
        }
      }

      if(ui->selection == maxReservoirLevelSetting) {
        if(buttons->pressed == upButton) {
          if(settings->max_reservoir_level < maxReservoirLevel) {
            settings->max_reservoir_level += 1;
          }
        }
        if(buttons->pressed == downButton) {
          if(settings->max_reservoir_level > minReservoirLevel) {
            settings->max_reservoir_level -= 1;
          }
        }
      }

      if(ui->selection == currentReservoirLevelSetting) {
        if(buttons->pressed == upButton) {
          settings->water_level_calibrate_tenths_of_inches += 1;
        }
        if(buttons->pressed == downButton) {
          settings->water_level_calibrate_tenths_of_inches -= 1;
        }
      }

      if(ui->selection == bilgePumpRunTimeSetting) {
        if(buttons->pressed == upButton) {
          settings->bilge_pump_run_time += 5;
        }
        if(buttons->pressed == downButton) {
          settings->bilge_pump_run_time -= 5;
        }
      }

      if(ui->selection == bilgePumpWaitTimeSetting) {
        if(buttons->pressed == upButton) {
          settings->bilge_pump_wait_time += 5;
        }
        if(buttons->pressed == downButton) {
          settings->bilge_pump_wait_time -= 5;
        }
      }

      if(ui->selection == overheatSetting) {
        if(buttons->pressed == upButton) {
          settings->overheat += 1;
        }
        if(buttons->pressed == downButton) {
          settings->overheat -= 1;
        }
      }

      if(ui->selection == temperatureSwingSetting) {
        if(buttons->pressed == upButton) {
          settings->temperature_swing += 1;
        }
        if(buttons->pressed == downButton) {
          settings->temperature_swing -= 1;
        }
      }

      if(ui->selection == humiditySwingSetting) {
        if(buttons->pressed == upButton) {
          settings->target_humidity_swing += 1;
        }
        if(buttons->pressed == downButton) {
          settings->target_humidity_swing -= 1;
        }
      }

      if(ui->selection == maxHumiditySetting) {
        if(buttons->pressed == upButton) {
          settings->max_humidity += 1;
        }
        if(buttons->pressed == downButton) {
          settings->max_humidity -= 1;
        }
      }

      if(ui->selection == maxHumiditySwingSetting) {
        if(buttons->pressed == upButton) {
          settings->max_humidity_swing += 1;
        }
        if(buttons->pressed == downButton) {
          settings->max_humidity_swing -= 1;
        }
      }

      if(ui->selection == refillPumpTimePerGallonSetting) {
        if(buttons->pressed == upButton) {
          settings->refill_pump_time_per_gallon += 60;
        }
        if(buttons->pressed == downButton) {
          settings->refill_pump_time_per_gallon -= 60;
        }
      }

      if(ui->selection == factoryResetSetting) {
        if(buttons->pressed == upButton) {
          resetSettings(settings);
          hi(); delay(1000);
          ui->selection = 1;
        }
      }

      if(ui->selection == extraSettingsSetting) {
        if(buttons->pressed == upButton) {
          ui->selection += 1;
        }
      } else if(ui->selection == extraSettingsSetting2) {
        if(buttons->pressed == upButton) {
          ui->selection += 1;
        } else {
          ui->selection = extraSettingsSetting;
        }
      } else if(ui->selection == extraSettingsSetting3) {
        if(buttons->pressed == downButton) {
          ui->selection += 1;
        } else {
          ui->selection = extraSettingsSetting;
        }
      } else if(ui->selection == extraSettingsSetting4) {
        if(buttons->pressed == downButton) {
          ui->selection += 1;
        } else {
          ui->selection = extraSettingsSetting;
        }
      }
    }

    ui->last_activity_time = millis();
    buttons->pressed = 0;
  } else if(ui->last_activity_time && millis() - ui->last_activity_time > uiTimeout) {
    ui->last_activity_time = 0;
    ui->selection = 1;
    writeEEPROM(settings);
  }
}

void displayTemperatureAndHumidity(int temperature, int humidity, int sensorNumber) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitNum(0, temperature / 10);
  matrix.writeDigitNum(1, temperature % 10, sensorNumber != insideSensorNumber);

  matrix.writeDigitRaw(2, 2);

  matrix.writeDigitNum(3, humidity / 10);
  matrix.writeDigitNum(4, humidity % 10, sensorNumber != outsideSensorNumber);

  matrix.writeDisplay();
#endif
}

void resetSettings(struct SwampSettings *settings) {
  settings->target_temperature = 67;
  settings->temperature_swing = 5;
  settings->overheat = 20;

  settings->max_humidity = 80;
  settings->max_humidity_swing = 15;

  settings->target_humidity = 90;
  settings->target_humidity_swing = 10;

  settings->bilge_pump_run_time = 120;
  settings->bilge_pump_wait_time = 180;

  settings->estimated_extra_water = 0;

  settings->blower_pwm = 255;

  settings->min_reservoir_level = 20;
  settings->max_reservoir_level = 100;
  settings->reservoir_level_swing = 10;

  settings->water_level_calibrate_ratio  = waterCalibrationInchRatio;
  settings->water_level_calibrate_tenths_of_inches = waterCalibrationInch * 10;

  settings->refill_pump_time_per_gallon = 45 * 60;
}

void displayH20() {
#ifdef __EMSCRIPTEN__
  // todo
#else
  // the water levels will be displayed on the neopixels
  // so we just show the word "H20" here
  matrix.writeDigitRaw(0, 2 + 4 + 16 + 32 + 64);
  matrix.writeDigitNum(1, 2);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, 0);
  matrix.writeDigitRaw(4, 0);
  matrix.writeDisplay();
#endif
}

void displayFan(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  // "F xx"
  matrix.writeDigitNum(0, 0xF);
  matrix.writeDigitRaw(1, 0);
  matrix.writeDigitRaw(2, 0);

  matrix.writeDigitNum(3, settings.blower_pwm / 16);
  matrix.writeDigitNum(4, settings.blower_pwm % 16);
  matrix.writeDisplay();
#endif
}

void displayTargetTemperature(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  // "dd&deg;F"
  matrix.writeDigitNum(0, settings.target_temperature / 10);
  matrix.writeDigitNum(1, settings.target_temperature % 10);
  matrix.writeDigitRaw(2, 0);

  matrix.writeDigitRaw(3, 1 + 2 + 32 + 64); // &deg;
  matrix.writeDigitNum(4, 0xF);
  matrix.writeDisplay();
#endif
}

void displayTargetHumidity(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  // "% dd"
  matrix.writeDigitRaw(0, 1 + 2 + 32 + 64); // &deg;
  matrix.writeDigitRaw(1, 4 + 8 + 16 + 64); // o
  matrix.writeDigitRaw(2, 0);

  matrix.writeDigitNum(3, settings.target_humidity / 10);
  matrix.writeDigitNum(4, settings.target_humidity % 10);
  matrix.writeDisplay();
#endif
}

void displayLock() {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitNum(0, 5);
  matrix.writeDigitNum(1, 1);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, 9);
  matrix.writeDigitNum(4, 9);
  matrix.writeDisplay();
#endif
}

void displayJes() {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 0);
  matrix.writeDigitRaw(1, 2 + 4 + 8);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, 0xE);
  matrix.writeDigitRaw(4, 1 + 4 +  8 + 32 + 64);
  matrix.writeDisplay();
#endif
}

void displayAnna() {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitNum(0, 10, 0);
  matrix.writeDigitRaw(1, 4 + 16 + 64);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitRaw(3, 4 + 16 + 64);
  matrix.writeDigitNum(4, 10, 0);
  matrix.writeDisplay();
#endif
}

void displayHaze() {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 2 + 4 + 16 + 32 + 64);
  matrix.writeDigitNum(1, 0xA);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, 2);
  matrix.writeDigitNum(4, 0xE);
  matrix.writeDisplay();
#endif
}

void displayMinReservoirLevelSetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 2 + 4 + 8 + 16 + 32);
  matrix.writeDigitNum(1, settings.min_reservoir_level / 100);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, settings.min_reservoir_level / 10 % 10, true);
  matrix.writeDigitNum(4, settings.min_reservoir_level % 10);
  matrix.writeDisplay();
#endif
}

void displayMaxReservoirLevelSetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 1 + 2 + 4 + 16 + 32);
  matrix.writeDigitNum(1, settings.max_reservoir_level / 100);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, settings.max_reservoir_level / 10 % 10, true);
  matrix.writeDigitNum(4, settings.max_reservoir_level % 10);
  matrix.writeDisplay();
#endif
}

void displayCurrentReservoirLevelSetting(struct SwampData data) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 2 + 4 + 16 + 32 + 64);
  matrix.writeDigitRaw(1, 0);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, data.reservoir / 10, true);
  matrix.writeDigitNum(4, data.reservoir % 10);
  matrix.writeDisplay();
#endif
}


void displayBilgePumpRunTimeSetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitNum(0, 0xb, true);
  matrix.writeDigitNum(1, settings.bilge_pump_run_time / 60);
  matrix.writeDigitRaw(2, 2);
  matrix.writeDigitNum(3, (settings.bilge_pump_run_time % 60) / 10);
  matrix.writeDigitNum(4, settings.bilge_pump_run_time % 10);
  matrix.writeDisplay();
#endif
}

void displayBilgePumpWaitTimeSetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitNum(0, 0xb, true);
  matrix.writeDigitNum(1, settings.bilge_pump_wait_time / 60);
  matrix.writeDigitRaw(2, 2);
  matrix.writeDigitNum(3, (settings.bilge_pump_wait_time % 60) / 10);
  matrix.writeDigitNum(4, settings.bilge_pump_wait_time % 10, true);
  matrix.writeDisplay();
#endif
}

void displayOverheatSetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 1);
  matrix.writeDigitNum(1, settings.overheat / 10);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, settings.overheat % 10);
  matrix.writeDigitRaw(4, 1 + 2 + 32 + 64);
  matrix.writeDisplay();
#endif
}

void displayTemperatureSwingSetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 8 + 64);
  matrix.writeDigitNum(1, settings.temperature_swing / 10);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, settings.temperature_swing % 10);
  matrix.writeDigitRaw(4, 1 + 2 + 32 + 64);
  matrix.writeDisplay();
#endif
}

void displayHumiditySwingSetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 1 + 2 + 32 + 64); // &deg;
  matrix.writeDigitRaw(1, 4 + 8 + 16 + 64); // o
  matrix.writeDigitRaw(2, 2);
  matrix.writeDigitNum(3, settings.target_humidity_swing / 10);
  matrix.writeDigitNum(4, settings.target_humidity_swing % 10);
  matrix.writeDisplay();
#endif
}

void displayMaxHumiditySetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 1 + 2 + 32 + 64 + 128); // &deg;
  matrix.writeDigitRaw(1, 4 + 8 + 16 + 64 + 128); // o
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, settings.max_humidity / 10, true);
  matrix.writeDigitNum(4, settings.max_humidity % 10, true);
  matrix.writeDisplay();
#endif
}

void displayMaxHumiditySwingSetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 1 + 2 + 32 + 64 + 128); // &deg;
  matrix.writeDigitRaw(1, 4 + 8 + 16 + 64 + 128); // o
  matrix.writeDigitRaw(2, 2);
  matrix.writeDigitNum(3, settings.max_humidity_swing / 10, true);
  matrix.writeDigitNum(4, settings.max_humidity_swing % 10, true);
  matrix.writeDisplay();
#endif
}

void displayRefillPumpTimePerGallonSetting(struct SwampSettings settings) {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitNum(0, settings.refill_pump_time_per_gallon / 60 / 10);
  matrix.writeDigitNum(1, settings.refill_pump_time_per_gallon / 60 % 10);
  matrix.writeDigitRaw(2, 2);
  matrix.writeDigitRaw(3, 16 + 64); // r
  matrix.writeDigitNum(4, 0xf, true); // f
  matrix.writeDisplay();
#endif
}

void displayFactoryReset() {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitNum(0, 0xF);
  matrix.writeDigitNum(1, 0xA);
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitNum(3, 0xC);
  matrix.writeDigitRaw(4, 0);
  matrix.writeDisplay();
#endif
}

void displayErr() {
#ifdef __EMSCRIPTEN__
  // todo
#else
  matrix.writeDigitRaw(0, 1 + 8 + 16 + 32 + 64); // E
  matrix.writeDigitRaw(1, 16 + 64); // r
  matrix.writeDigitRaw(2, 0);
  matrix.writeDigitRaw(3, 16 + 64); // r
  matrix.writeDigitRaw(4, 0);
  matrix.writeDisplay();
#endif
}

void display(struct SwampUI ui, struct SwampI2C i2c, struct SwampData data, struct SwampSettings settings) {
  if(!isSensorFinished(i2c)) {
    return;
  }
  tcaSelect(insideSensorNumber);

  if(ui.last_activity_time == 0) {
    // display temp/humidity
    int displaySensorNumber = (millis() / displayTimeout) % sensorCount;
    struct SensorData *sensor_data = getSensorNumber(displaySensorNumber, &data);
    displayTemperatureAndHumidity(sensor_data->temperature, sensor_data->humidity, displaySensorNumber);
  } else {
    // display / edit settings
    switch(ui.selection) {
      case waterLevelsSetting: displayH20(); break;
      case blowerSpeedSetting: displayFan(settings); break;
      case targetTemperatureSetting: displayTargetTemperature(settings); break;
      case targetHumiditySetting: displayTargetHumidity(settings); break;
      case extraSettingsSetting: displayLock(); break;
      case extraSettingsSetting2: displayJes(); break;
      case extraSettingsSetting3: displayAnna(); break;
      case extraSettingsSetting4: displayHaze(); break;
      case minReservoirLevelSetting: displayMinReservoirLevelSetting(settings); break;
      case maxReservoirLevelSetting: displayMaxReservoirLevelSetting(settings); break;
      case currentReservoirLevelSetting: displayCurrentReservoirLevelSetting(data); break;
      case bilgePumpRunTimeSetting: displayBilgePumpRunTimeSetting(settings); break;
      case bilgePumpWaitTimeSetting: displayBilgePumpWaitTimeSetting(settings); break;
      case overheatSetting: displayOverheatSetting(settings); break;
      case temperatureSwingSetting: displayTemperatureSwingSetting(settings); break;
      case humiditySwingSetting: displayHumiditySwingSetting(settings); break;
      case maxHumiditySetting: displayMaxHumiditySetting(settings); break;
      case maxHumiditySwingSetting: displayMaxHumiditySwingSetting(settings); break;
      case refillPumpTimePerGallonSetting: displayRefillPumpTimePerGallonSetting(settings); break;
      case factoryResetSetting: displayFactoryReset(); break;
      default: displayErr();
    }
  }
}

void reservoirLED(struct SwampSettings settings) {
  int r1, g1, b1, w1;
  int i;
  if(settings.estimated_extra_water == maxExtraWater) {
    r1 = LIGHTCYAN / 0x1000 % 0x10;
    g1 = LIGHTCYAN / 0x0100 % 0x10;
    b1 = LIGHTCYAN / 0x0010 % 0x10;
    w1 = LIGHTCYAN / 0x0001 % 0x10;
    for(i = 0; i < 7; i++) {
      #ifndef __EMSCRIPTEN__
      strip.setPixelColor( (pixelCount + bottomPixel - 1 - i) % pixelCount, r1, g1, b1, w1);
      #endif
    }
  } else {
    int extraWaterLightCount = (7 * settings.estimated_extra_water) / (maxExtraWater - extraWaterStep);
    r1 = BABYBLUE / 0x1000 % 0x10;
    g1 = BABYBLUE / 0x0100 % 0x10;
    b1 = BABYBLUE / 0x0010 % 0x10;
    w1 = BABYBLUE / 0x0001 % 0x10;
    for(i = 0; i < extraWaterLightCount; i++) {
      #ifndef __EMSCRIPTEN__
      strip.setPixelColor( (pixelCount + bottomPixel - 1 - i) % pixelCount, r1, g1, b1, w1);
      #endif
    }
    r1 = BLUE / 0x1000 % 0x10;
    g1 = BLUE / 0x0100 % 0x10;
    b1 = BLUE / 0x0010 % 0x10;
    w1 = BLUE / 0x0001 % 0x10;
    if( ((14 * settings.estimated_extra_water) / (maxExtraWater - extraWaterStep)) % 2) {
      #ifndef __EMSCRIPTEN__
      strip.setPixelColor( (pixelCount + bottomPixel - 1 - extraWaterLightCount) % pixelCount, r1, g1, b1, w1);
      #endif
    }
    // Serial.println(14 * settings.estimated_extra_water / maxExtraWater);
  }
}


void animate(struct Animation lights, struct SwampSettings settings) {
  int r1, g1, b1, w1, r2, b2, g2, w2;

  r1 = lights.color1 / 0x1000 % 0x10;
  g1 = lights.color1 / 0x0100 % 0x10;
  b1 = lights.color1 / 0x0010 % 0x10;
  w1 = lights.color1 / 0x0001 % 0x10;

  r2 = lights.color2 / 0x1000 % 0x10;
  g2 = lights.color2 / 0x0100 % 0x10;
  b2 = lights.color2 / 0x0010 % 0x10;
  w2 = lights.color2 / 0x0001 % 0x10;

  if(lights.alert) {
    int frame = (millis() / alertAnimationDelay) % 2;
    for(int n = 0; n < pixelCount; n++) {
      if(n % 2) {
        if(frame) {
        #ifndef __EMSCRIPTEN__
        strip.setPixelColor(n, r1, g1, b1, w1);
        #endif
        } else {
        #ifndef __EMSCRIPTEN__
        strip.setPixelColor(n, r2, g2, b2, w2);
        #endif
        }
      } else {
        #ifndef __EMSCRIPTEN__
        strip.setPixelColor(n, 0, 0, 0, 0);
        #endif
      }
    }
  } else {
    int frame = (millis() / fanAnimatationDelay) % 4;
    for(int n = 0; n < pixelCount; n++) {
      int part = (n + frame) % 4;
      if(part == 0) {
        #ifndef __EMSCRIPTEN__
        strip.setPixelColor(n, r1, g1, b1, w1);
        #endif
      } else if(part == 1) {
        #ifndef __EMSCRIPTEN__
        strip.setPixelColor(n, r2, g2, b2, w2);
        #endif
      } else {
        #ifndef __EMSCRIPTEN__
        strip.setPixelColor(n, 0, 0, 0, 0);
        #endif
      }
    }
  }

  // show extra water level/timer if refilling
  if(lights.refill) {
    //int r3, g3, b3, w3;
    //r3 = LIGHTCYAN / 0x1000 % 0x10;
    //g3 = LIGHTCYAN / 0x0100 % 0x10;
    //b3 = LIGHTCYAN / 0x0010 % 0x10;
    //w3 = LIGHTCYAN / 0x0001 % 0x10;
    //#ifndef __EMSCRIPTEN__
    //for(int p = refillStartPixel; p <= refillStopPixel; p++) {
    //  strip.setPixelColor(p, r3, g3, b3, w3);
    //}
    //#endif
    reservoirLED(settings);
  }

  #ifndef __EMSCRIPTEN__
  strip.show();
  #endif
}

void waterLED(struct SwampSettings settings, struct SwampData data) {
  int i;
  for(i = 0; i < pixelCount; i++) {
    #ifndef __EMSCRIPTEN__
    strip.setPixelColor(i, 0, 0, 0, 0);
    #endif
  }

  int r2, b2, g2, w2;

  reservoirLED(settings);

  r2 = CYAN / 0x1000 % 0x10;
  g2 = CYAN / 0x0100 % 0x10;
  b2 = CYAN / 0x0010 % 0x10;
  w2 = CYAN / 0x0001 % 0x10;
  int reservoirLightCount = (7 * data.reservoir) / settings.max_reservoir_level;
  if(reservoirLightCount > 8) {reservoirLightCount = 8;}
  for(i = 0; i < reservoirLightCount; i++) {
    #ifndef __EMSCRIPTEN__
    strip.setPixelColor( (bottomPixel + 1 + i) % pixelCount, r2, g2, b2, w2);
    #endif
  }

  #ifndef __EMSCRIPTEN__
  strip.show();
  #endif
}

void doLights(struct Animation lights, struct SwampUI ui, struct SwampSettings settings, struct SwampData data) {
  if(ui.selection == waterLevelsSetting) {
    waterLED(settings, data);
  } else {
    animate(lights, settings);
  }
}

struct SwampData data;
struct SwampI2C i2c;
struct SwampPWM pwm;
struct SwampWater water;
struct SwampUI ui;
struct SwampSettings settings;
struct SwampSense sense;
struct SwampAction action;
struct SwampButtons buttons;
struct SwampTimers timers;

void hi() {
  #ifndef __EMSCRIPTEN__
  matrix.writeDigitRaw(0, 0);
  matrix.writeDigitRaw(1, 2 + 4 + 16 + 32 + 64);
  matrix.writeDigitRaw(3, 2 + 4);
  matrix.writeDigitRaw(4, 0);
  matrix.writeDisplay();
  #endif
}

void setup() {
  #ifndef __EMSCRIPTEN__
  Serial.begin(9600);
  Serial.println("hi, i hope you keep cool this summer <3");

  Wire.begin();

  pinMode(buttonPin, INPUT_PULLUP);

  tcaSelect(insideSensorNumber);
  matrix.begin(segmentAddr);
  matrix.setBrightness(0);
  hi(); delay(500);

  strip.begin();

  #endif

  sense.outside.temp = HOT;
  sense.outside.hum  = DRY;
  sense.inside.temp  = HOT;
  sense.inside.hum   = DRY;
  sense.blower.temp  = HOT;
  sense.blower.hum   = DRY;
  sense.reservoir   = HIGH_WATER;

  ui.selection = 1;
  ui.last_activity_time = 0;

  i2c.selection = 9;
  i2c.sensor_query_time = 0;
  i2c.got_sensor_response = 1;
  i2c.retries = 0;
  for(int s = 0; s < sensorCount; s++) {
    i2c.sensor_last_read_time[s] = 0;
  }

  for(int pin = 0; pin < maxPin; pin++) {
    pwm.pin[pin] = 0;
  }

  water.readingCount = 0;
  water.readingIndex = 0;

  buttons.pressed = 0;

  resetSettings(&settings);
  readEEPROM(&settings);
}

void simulate() {
  #ifdef __EMSCRIPTEN__
  EM_ASM({simulate();(});
  #endif
}

void loop() {
  readSensors(&i2c, &data, &water, settings);
  interpretSensors(data, settings, &sense);
  applyTruthTable(sense, &action);
  takeAction(action, settings, &timers, &pwm);
  estimateRefill(&settings, &timers);
  readButtons(&buttons);
  doUI(&ui, &buttons, &settings);
  display(ui, i2c, data, settings);
  doLights(action.lights, ui, settings, data);
  simulate();
}

#ifdef __EMSCRIPTEN__
int main() {
  setup();
  emscripten_set_main_loop(loop, 10, 1);
}
#endif

