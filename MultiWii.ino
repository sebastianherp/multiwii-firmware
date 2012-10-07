/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
July  2012     V2.1
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "def.h"
#include "global.h"

#define  VERSION  211


void setup() {
  #if !defined(GPS_PROMINI)
    SerialOpen(0,SERIAL0_COM_SPEED);
    #if defined(PROMICRO)
      SerialOpen(1,SERIAL1_COM_SPEED);
    #endif
    #if defined(MEGA)
      SerialOpen(1,SERIAL1_COM_SPEED);
      SerialOpen(2,SERIAL2_COM_SPEED);
      SerialOpen(3,SERIAL3_COM_SPEED);
    #endif
  #endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  for(global_conf.currentSet=0; global_conf.currentSet<3; global_conf.currentSet++) {  // check all settings integrity
    readEEPROM();
  }
  readGlobalSet();
  readEEPROM();                                    // load current setting data
  blinkLED(2,40,global_conf.currentSet+1);          
  configureReceiver();
  #if defined(OPENLRSv2MULTI)
    initOpenLRS();
  #endif
  initSensors();
  #if defined(I2C_GPS) || defined(GPS_SERIAL) || defined(GPS_FROM_OSD)
    GPS_set_pids();
  #endif
  previousTime = micros();
  #if defined(GIMBAL)
   calibratingA = 400;
  #endif
  calibratingG = 400;
  #if defined(POWERMETER)
    for(uint8_t i=0;i<=PMOTOR_SUM;i++)
      pMeter[i]=0;
  #endif
  #if defined(ARMEDTIMEWARNING)
    ArmedTimeWarningMicroSeconds = (ARMEDTIMEWARNING *1000000);
  #endif
  /************************************/
  #if defined(GPS_SERIAL)
    GPS_SerialInit();
    for(uint8_t i=0;i<=5;i++){
      GPS_NewData(); 
      LEDPIN_ON
      delay(20);
      LEDPIN_OFF
      delay(80);
    }
    if(!GPS_Present){
      SerialEnd(GPS_SERIAL);
      SerialOpen(0,SERIAL0_COM_SPEED);
    }
    #if !defined(GPS_PROMINI)
      GPS_Present = 1;
    #endif
    GPS_Enable = GPS_Present;    
  #endif
  /************************************/
 
  #if defined(I2C_GPS) || defined(TINY_GPS) || defined(GPS_FROM_OSD)
   GPS_Enable = 1;
  #endif
  
  #if defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) || defined(LCD_TELEMETRY_STEP)
    initLCD();
  #endif
  #ifdef LCD_TELEMETRY_DEBUG
    telemetry_auto = 1;
  #endif
  #ifdef LCD_CONF_DEBUG
    configurationLoop();
  #endif
  #ifdef LANDING_LIGHTS_DDR
    init_landing_lights();
  #endif
  ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  #if defined(LED_FLASHER)
    init_led_flasher();
    led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
  #endif
  f.SMALL_ANGLES_25=1; // important for gyro only conf

  debugmsg_append_str("initialization completed\n");
}

// ******** Main Loop *********
void loop () {
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta,deltaSum;
  int16_t PTerm,ITerm,PTermACC,ITermACC,PTermGYRO,ITermGYRO,DTerm;
  uint32_t timeDebug = 0;
  static uint32_t rcTime  = 0;

  #if defined(SPEKTRUM)
    if (spekFrameFlags == 0x01) readSpektrum();
  #endif
  
  #if defined(OPENLRSv2MULTI) 
    Read_OpenLRS_RC();
  #endif 

  #define RC_FREQ 50

  // debug timing code (use to track how long things take to compute)
  //timeDebug = micros();
  //debug[0] = micros() - timeDebug;

  // whole thing takes 130-170 µs
  if (currentTime > rcTime ) { // 50Hz
    rcTime += 20000;
    computeRC();
    handleRC();
    handleBoxes();
  }

  timeDebug = micros();
  getSensorData(); // 530 µs
  debug[0] = (debug[0]*9 + (micros() - timeDebug)) / 10;

  #if BARO
    Baro_update(); // 70 -> 170 -> 70 -> 1100 µs (MS561101BA)
  #endif

  #if GPS
    if(GPS_Enable) GPS_NewData();
  #endif
  #if SONAR
    Sonar_update();debug[2] = sonarAlt;
  #endif
  #ifdef LANDING_LIGHTS_DDR
    auto_switch_landing_lights();
  #endif

  // calculate attitude from sensor data
  #if ACC
    timeDebug = micros();
    getEstimatedAttitude(); // around 1100 µs
    debug[1] = (debug[1]*9 + (micros() - timeDebug)) / 10;
  #endif  
  #if BARO
    getEstimatedAltitude(); // around 550 µs when not immediatly returning (25 Hz "timer" in function)
  #endif
  
  annexCode(); // 500 - 1700 µs
  
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

#ifdef CYCLETIME_FIXATED
  if (conf.cycletime_fixated) {
    if ((micros()-timestamp_fixated)>conf.cycletime_fixated) {
       //debug[0]++;
    } else {
       while((micros()-timestamp_fixated)<conf.cycletime_fixated) ; // waste away
       //debug[1] = micros()-timestamp_fixated - conf.cycletime_fixated;
    }
    timestamp_fixated=micros();
  }
#endif
  //***********************************
  //**** Experimental FlightModes *****
  //***********************************
  #if defined(ACROTRAINER_MODE)
    if(f.ANGLE_MODE){
      if (abs(rcCommand[ROLL]) + abs(rcCommand[PITCH]) >= ACROTRAINER_MODE ) {
        f.ANGLE_MODE=0;
        f.HORIZON_MODE=0;
        f.MAG_MODE=0;
        f.BARO_MODE=0;
        f.GPS_HOME_MODE=0;
        f.GPS_HOLD_MODE=0;
      }
    }
  #endif
  #if defined(AP_MODE)
    if(f.ANGLE_MODE || f.HORIZON_MODE){
      if (abs(rcCommand[ROLL])>= AP_MODE || abs(rcCommand[PITCH]) >= AP_MODE) {
        f.BARO_MODE=0;
        f.GPS_HOME_MODE=0;
        f.GPS_HOLD_MODE=0;
      }
    }
  #endif
 //*********************************** 


  #if MAG
    if (abs(rcCommand[YAW]) <70 && f.MAG_MODE) {
      int16_t dif = heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif*conf.P8[PIDMAG]/30;  // 18 deg
    } else magHold = heading;
  #endif

  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    if (f.BARO_MODE) {
      if (abs(rcCommand[THROTTLE]-initialThrottleHold)>ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
        f.BARO_MODE = 0; // so that a new althold reference is defined
      }
      rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
    }
  #endif
  #if GPS
    if ( (!f.GPS_HOME_MODE && !f.GPS_HOLD_MODE) || !f.GPS_FIX_HOME ) {
      GPS_reset_nav(); // If GPS is not activated. Reset nav loops and all nav related parameters
    } else {
      float sin_yaw_y = sin(heading*0.0174532925f);
      float cos_yaw_x = cos(heading*0.0174532925f);
   #if defined(NAV_SLEW_RATE)     
      nav_rated[LON] += constrain(wrap_18000(nav[LON]-nav_rated[LON]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
      nav_rated[LAT] += constrain(wrap_18000(nav[LAT]-nav_rated[LAT]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
      GPS_angle[ROLL]   = (nav_rated[LON]*cos_yaw_x - nav_rated[LAT]*sin_yaw_y) /10;
      GPS_angle[PITCH]  = (nav_rated[LON]*sin_yaw_y + nav_rated[LAT]*cos_yaw_x) /10;
   #else 
      GPS_angle[ROLL]   = (nav[LON]*cos_yaw_x - nav[LAT]*sin_yaw_y) /10;
      GPS_angle[PITCH]  = (nav[LON]*sin_yaw_y + nav[LAT]*cos_yaw_x) /10;
   #endif
    }
  #endif

  //**** PITCH & ROLL & YAW PID ****
  int16_t prop;
  prop = max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])); // range [0;500]
  
  for(axis=0;axis<3;axis++) {
    if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis<2 ) { // MODE relying on ACC
      // 50 degrees max inclination
      errorAngle = constrain(2*rcCommand[axis] + GPS_angle[axis],-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
      #ifdef LEVEL_PDF
        PTermACC      = -(int32_t)angle[axis]*conf.P8[PIDLEVEL]/100 ;
      #else  
        PTermACC      = (int32_t)errorAngle*conf.P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      #endif
      PTermACC = constrain(PTermACC,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

      errorAngleI[axis]     = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
      ITermACC              = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    }
    if ( !f.ANGLE_MODE || f.HORIZON_MODE || axis == 2 ) { // MODE relying on GYRO or YAW axis
      if (abs(rcCommand[axis])<350) error =          rcCommand[axis]*10*8/conf.P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
                               else error = (int32_t)rcCommand[axis]*10*8/conf.P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
      error -= gyroData[axis];

      PTermGYRO = rcCommand[axis];
      
      errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
      if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
      ITermGYRO = (errorGyroI[axis]/125*conf.I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
    }
    if ( f.HORIZON_MODE && axis<2) {
      PTerm = ((int32_t)PTermACC*(500-prop) + (int32_t)PTermGYRO*prop)/500;
      ITerm = ((int32_t)ITermACC*(500-prop) + (int32_t)ITermGYRO*prop)/500;
    } else {
      if ( f.ANGLE_MODE && axis<2) {
        PTerm = PTermACC;
        ITerm = ITermACC;
      } else {
        PTerm = PTermGYRO;
        ITerm = ITermGYRO;
      }
    }

    if (abs(gyroData[axis])<160) PTerm -=          gyroData[axis]*dynP8[axis]/10/8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
                            else PTerm -= (int32_t)gyroData[axis]*dynP8[axis]/10/8; // 32 bits is needed for calculation   

    delta          = gyroData[axis] - lastGyro[axis];                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    deltaSum       = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
 
    if (abs(deltaSum)<640) DTerm = (deltaSum*dynD8[axis])>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
                      else DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;              // 32 bits is needed for calculation
                      
    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

  mixTable();
  writeServos();
  writeMotors();
}
