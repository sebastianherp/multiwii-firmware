/*********** RC alias *****************/
enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL,     // not used currently
  PIDITEMS
};

enum box {
  #if ACC
    BOXANGLE,
    BOXHORIZON,
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    BOXBARO,
  #endif
  #if MAG
    BOXMAG,
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)  || defined(SERVO_MIX_TILT)
    BOXCAMSTAB,
  #endif
  #if defined(CAMTRIG)
    BOXCAMTRIG,
  #endif
  BOXARM,
  #if GPS
    BOXGPSHOME,
    BOXGPSHOLD,
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER) || defined(INFLIGHT_ACC_CALIBRATION)
    BOXPASSTHRU,
  #endif
  #if MAG
    BOXHEADFREE,
  #endif
  #if defined(BUZZER)
    BOXBEEPERON,
  #endif
  #if defined(LED_FLASHER)
    BOXLEDMAX, // we want maximum illumination
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    BOXLLIGHTS, // enable landing lights at any altitude
  #endif
  #if MAG
    BOXHEADADJ, // acquire heading for HEADFREE mode
  #endif
  CHECKBOXITEMS
};

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  #if ACC
    "ANGLE;"
    "HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    "BARO;"
  #endif
  #if MAG
    "MAG;"
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
    "CAMSTAB;"
  #endif
  #if defined(CAMTRIG)
    "CAMTRIG;"
  #endif
  "ARM;"
  #if GPS
    "GPS HOME;"
    "GPS HOLD;"
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER) || defined(INFLIGHT_ACC_CALIBRATION)
    "PASSTHRU;"
  #endif
  #if MAG
    "HEADFREE;"
  #endif
  #if defined(BUZZER)
    "BEEPER;"
  #endif
  #if defined(LED_FLASHER)
    "LEDMAX;"
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    "LLIGHTS;"
  #endif
  #if MAG
    "HEADADJ;"  
  #endif
;

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;
static uint16_t acc_1G;             // this is the 1G measured acceleration
static int16_t  acc_25deg;
static int16_t  headFreeModeHold;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static float    gyroScale = 0;
static int16_t  heading,magHold;
static uint8_t  vbat;                   // battery voltage in 0.1V steps
static uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
static uint8_t  rcOptions[CHECKBOXITEMS];
static int32_t  BaroAlt;
static int32_t  EstAlt;             // in cm
static int16_t  BaroPID = 0;
static int32_t  AltHold;
static int16_t  errorAltitudeI = 0;
#if defined(BUZZER)
  static uint8_t  toggleBeep = 0;
#endif
#if defined(ARMEDTIMEWARNING)
  static uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

static int16_t  debug[4];
static int16_t  sonarAlt; //to think about the unit

struct flags_struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t NUNCHUKDATA :1 ;
  uint8_t ANGLE_MODE :1 ;
  uint8_t HORIZON_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
  uint8_t GPS_HOME_MODE :1 ;
  uint8_t GPS_HOLD_MODE :1 ;
  uint8_t HEADFREE_MODE :1 ;
  uint8_t PASSTHRU_MODE :1 ;
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
} f;

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  static uint16_t cycleTimeMax = 0;       // highest ever cycle timen
  static uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
  static uint16_t powerMax = 0;           // highest ever current;
  static int32_t  BAROaltStart;       // offset value from powerup
  static int32_t  BAROaltMax;         // maximum value
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING)
  static uint32_t armedTime = 0;
#endif

static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;

// **********************
//Automatic ACC Offset Calibration
// **********************
#if defined(INFLIGHT_ACC_CALIBRATION)
  static uint16_t InflightcalibratingA = 0;
  static int16_t AccInflightCalibrationArmed;
  static uint16_t AccInflightCalibrationMeasurementDone = 0;
  static uint16_t AccInflightCalibrationSavetoEEProm = 0;
  static uint16_t AccInflightCalibrationActive = 0;
#endif

// **********************
// power meter
// **********************
#if defined(POWERMETER)
#define PMOTOR_SUM 8                     // index into pMeter[] for sum
  static uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
  static uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
  static uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
  static uint16_t powerValue = 0;          // last known current
#endif
static uint16_t intPowerMeterSum, intPowerTrigger1;

// **********************
// telemetry
// **********************
#if defined(LCD_TELEMETRY)
  static uint8_t telemetry = 0;
  static uint8_t telemetry_auto = 0;
#endif
#ifdef LCD_TELEMETRY_STEP
  static char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
  static uint8_t telemetryStepIndex = 0;
#endif

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

static int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

static int16_t rcData[RC_CHANS];   // interval [1000;2000]
static int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

#if defined(SPEKTRUM)
  volatile uint8_t  spekFrameFlags;
  volatile uint32_t spekTimeLast;
#endif

#if defined(OPENLRSv2MULTI)
  static uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
#endif

// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[NUMBER_MOTOR];
#if defined(SERVO)
  static int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
#endif

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[3], dynD8[3];

static struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE ! 
} global_conf;


static struct {
  uint8_t checkNewConf;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t angleTrim[2];
  uint16_t activate[CHECKBOXITEMS];
  uint8_t powerTrigger1;
  #ifdef FLYING_WING
    uint16_t wing_left_mid;
    uint16_t wing_right_mid;
  #endif
  #ifdef TRI
    uint16_t tri_yaw_middle;
  #endif
  #if defined HELICOPTER || defined(AIRPLANE)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
    int16_t servoTrim[8];
  #endif
  #if defined(GYRO_SMOOTHING)
    uint8_t Smoothing[3];
  #endif
  #if defined (FAILSAFE)
    int16_t failsafe_throttle;
  #endif
  #ifdef VBAT
    uint8_t vbatscale;
    uint8_t vbatlevel1_3s;
    uint8_t vbatlevel2_3s;
    uint8_t vbatlevel3_3s;
    uint8_t vbatlevel4_3s;
    uint8_t no_vbat;
  #endif
  #ifdef POWERMETER
    uint16_t psensornull;
    uint16_t pleveldivsoft;
    uint16_t pleveldiv;
    uint8_t pint2ma;
  #endif
  #ifdef CYCLETIME_FIXATED
    uint16_t cycletime_fixated;
  #endif
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE ! 
} conf;


// **********************
// GPS common variables
// **********************
  static int32_t  GPS_coord[2];
  static int32_t  GPS_home[2];
  static int32_t  GPS_hold[2];
  static uint8_t  GPS_numSat;
  static uint16_t GPS_distanceToHome;                          // distance to home in meters
  static int16_t  GPS_directionToHome;                         // direction to home in degrees
  static uint16_t GPS_altitude,GPS_speed;                      // altitude in 0.1m and speed in 0.1m/s
  static uint8_t  GPS_update = 0;                              // it's a binary toogle to distinct a GPS position update
  static int16_t  GPS_angle[2] = { 0, 0};                      // it's the angles that must be applied for GPS correction
  static uint16_t GPS_ground_course = 0;                       // degrees*10
  static uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
  static uint8_t  GPS_Enable  = 0;

  #define LAT  0
  #define LON  1
  // The desired bank towards North (Positive) or South (Negative) : latitude
  // The desired bank towards East (Positive) or West (Negative)   : longitude
  static int16_t  nav[2];
  static int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  // default POSHOLD control gains
  #define POSHOLD_P              .11
  #define POSHOLD_I              0.0
  #define POSHOLD_IMAX           20        // degrees

  #define POSHOLD_RATE_P         2.0
  #define POSHOLD_RATE_I         0.08      // Wind control
  #define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
  #define POSHOLD_RATE_IMAX      20        // degrees

  // default Navigation PID gains
  #define NAV_P                  1.4
  #define NAV_I                  0.20      // Wind control
  #define NAV_D                  0.08      //
  #define NAV_IMAX               20        // degrees

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial GPS only variables
  //navigation mode
  #define NAV_MODE_NONE          0
  #define NAV_MODE_POSHOLD       1
  #define NAV_MODE_WP            2
  static uint8_t nav_mode = NAV_MODE_NONE;            //Navigation mode

  #if defined(BUZZER)  
    static uint8_t beep_toggle = 0,
                   beep_confirmation = 0;
  #endif
  
/* static variables from loop() */
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;
  
/* baro variables */
#define BARO_TAB_SIZE   21
static int32_t  BaroPressure;
static int32_t  BaroTemperature;
static int32_t BaroPressureSum;


