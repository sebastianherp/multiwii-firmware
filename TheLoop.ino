/* TheLoop.ino

All of the functions loop() executes in one place

*/


// this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
void annexCode() {
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  #define BREAKPOINT 1500
  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if   (rcData[THROTTLE]<BREAKPOINT) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE]<2000) {
      prop2 = 100 - (uint16_t)conf.dynThrPID*(rcData[THROTTLE]-BREAKPOINT)/(2000-BREAKPOINT);
    } else {
      prop2 = 100 - conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    #if defined(DEADBAND)
      if (tmp>DEADBAND) { tmp -= DEADBAND; }
      else { tmp=0; }
    #endif
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp/100;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
      prop1 = 100-(uint16_t)conf.rollPitchRate*tmp/500;
      prop1 = (uint16_t)prop1*prop2/100;
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100-(uint16_t)conf.yawRate*tmp/500;
    }
    dynP8[axis] = (uint16_t)conf.P8[axis]*prop1/100;
    dynD8[axis] = (uint16_t)conf.D8[axis]*prop1/100;
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*1000/(2000-MINCHECK); // [MINCHECK;2000] -> [0;1000]
  tmp2 = tmp/100;
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*100) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

  if(f.HEADFREE_MODE) { //to optimize
    float radDiff = (heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff; 
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  #if defined(POWERMETER_HARD)
    uint16_t pMeterRaw;               // used for current reading
    static uint16_t psensorTimer = 0;
    if (! (++psensorTimer % PSENSORFREQ)) {
      pMeterRaw =  analogRead(PSENSORPIN);
      powerValue = ( conf.psensornull > pMeterRaw ? conf.psensornull - pMeterRaw : pMeterRaw - conf.psensornull); // do not use abs(), it would induce implicit cast to uint and overrun
      if ( powerValue < 333) {  // only accept reasonable values. 333 is empirical
      #ifdef LCD_TELEMETRY
        if (powerValue > powerMax) powerMax = powerValue;
      #endif
      } else {
        powerValue = 333;
      }        
      pMeter[PMOTOR_SUM] += (uint32_t) powerValue;
    }
  #endif
  #if defined(BUZZER)
    #if defined(VBAT)
      static uint8_t vbatTimer = 0;
      static uint8_t ind = 0;
      uint16_t vbatRaw = 0;
      static uint16_t vbatRawArray[8];
      if (! (++vbatTimer % VBATFREQ)) {
        vbatRawArray[(ind++)%8] = analogRead(V_BATPIN);
        for (uint8_t i=0;i<8;i++) vbatRaw += vbatRawArray[i];
        vbat = vbatRaw / (conf.vbatscale/2);                  // result is Vbatt in 0.1V steps
      }
    #endif
    alarmHandler(); // external buzzer routine that handles buzzer events globally now
  #endif  
  
  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

  #if defined(LED_RING)
    static uint32_t LEDTime;
    if ( currentTime > LEDTime ) {
      LEDTime = currentTime + 50000;
      i2CLedRingState();
    }
  #endif

  #if defined(LED_FLASHER)
    auto_switch_led_flasher();
  #endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  #if !(defined(SPEKTRUM) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if Spektrum Sat in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
    #if defined(GPS_PROMINI)
      if(GPS_Enable == 0) {serialCom();}
    #else
      serialCom();
    #endif
  #endif

  #if defined(POWERMETER)
    intPowerMeterSum = (pMeter[PMOTOR_SUM]/conf.pleveldiv);
    intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE; 
  #endif

  #ifdef LCD_TELEMETRY_AUTO
    static char telemetryAutoSequence []  = LCD_TELEMETRY_AUTO;
    static uint8_t telemetryAutoIndex = 0;
    static uint16_t telemetryAutoTimer = 0;
    if ( (telemetry_auto) && (! (++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ) )  ){
      telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
      LCDclear(); // make sure to clear away remnants
    }
  #endif  
  #ifdef LCD_TELEMETRY
    static uint16_t telemetryTimer = 0;
    if (! (++telemetryTimer % LCD_TELEMETRY_FREQ)) {
      #if (LCD_TELEMETRY_DEBUG+0 > 0)
        telemetry = LCD_TELEMETRY_DEBUG;
      #endif
      if (telemetry) lcd_telemetry();
    }
  #endif

  #if GPS & defined(GPS_LED_INDICATOR)       // modified by MIS to use STABLEPIN LED for number of sattelites indication
    static uint32_t GPSLEDTime;              // - No GPS FIX -> LED blink at speed of incoming GPS frames
    static uint8_t blcnt;                    // - Fix and sat no. bellow 5 -> LED off
    if(currentTime > GPSLEDTime) {           // - Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ...
      if(f.GPS_FIX && GPS_numSat >= 5) {
        if(++blcnt > 2*GPS_numSat) blcnt = 0;
        GPSLEDTime = currentTime + 150000;
        if(blcnt >= 10 && ((blcnt%2) == 0)) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
      }else{
        if((GPS_update == 1) && !f.GPS_FIX) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
        blcnt = 0;
      }
    }
  #endif

  #if defined(LOG_VALUES) && (LOG_VALUES >= 2)
    if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
    if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
  #endif
  #if defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING)
    if (f.ARMED) armedTime += (uint32_t)cycleTime;
  #endif
  #if defined(VBAT)
    if (vbat > conf.no_vbat) { // only track possibly sane voltage values
      if (!f.ARMED) {
        vbatMin = vbat;
      } else {
        if (vbat < vbatMin) vbatMin = vbat;
      }
    }
  #endif
  #ifdef LCD_TELEMETRY
    #if BARO
      if (!f.ARMED) {
        BAROaltStart = BaroAlt;
        BAROaltMax = BaroAlt;
      } else {
        if (BaroAlt > BAROaltMax) BAROaltMax = BaroAlt;
      }
    #endif
  #endif
}


void handleRC() {
  uint8_t i=0;
  
  // Failsafe routine - added by MIS
#if defined(FAILSAFE)
  if ( failsafeCnt > (5*FAILSAFE_DELAY) && f.ARMED) {                  // Stabilize, and set Throttle to specified level
    for(i=0; i<3; i++) rcData[i] = MIDRC;                               // after specified guard time after RC signal is lost (in 0.1sec)
    rcData[THROTTLE] = conf.failsafe_throttle;
    if (failsafeCnt > 5*(FAILSAFE_DELAY+FAILSAFE_OFF_DELAY)) {          // Turn OFF motors after specified Time (in 0.1sec)
      f.ARMED = 0;   // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
      f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
    }
    failsafeEvents++;
  }
  if ( failsafeCnt > (5*FAILSAFE_DELAY) && !f.ARMED) {  //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
      f.ARMED = 0;   // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
      f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
  }
  failsafeCnt++;
#endif

  // end of failsafe routine - next change is made with RcOptions setting
  if (rcData[THROTTLE] < MINCHECK) {
    errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
    errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
    rcDelayCommand++;
    if (rcData[YAW] < MINCHECK && !f.ARMED) {                                             // THTOTTLE min, YAW left
      if(rcData[PITCH] < MINCHECK && rcDelayCommand == 20) {                              // PITCH down -> GYRO cal
        calibratingG=400;
        #if GPS 
          GPS_reset_home_position();
        #endif
      }
      i = 0;
      if(rcData[ROLL]  < MINCHECK && rcData[PITCH] > 1300 && rcData[PITCH] < 1700) i=1;    // ROLL left  -> SET 1
      if(rcData[PITCH] > MAXCHECK && rcData[ROLL]  > 1300 && rcData[ROLL]  < 1700) i=2;    // PITCH up   -> SET 2
      if(rcData[ROLL]  > MAXCHECK && rcData[PITCH] > 1300 && rcData[PITCH] < 1700) i=3;    // ROLL right -> SET 3
      if(i && rcDelayCommand == 20) {
        global_conf.currentSet = i-1;
        writeGlobalSet(0);
        readEEPROM();
        blinkLED(2,40,i);
      }
    } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
      if (rcDelayCommand == 20) {
        #ifdef TRI
          servo[5] = 1500; // we center the yaw servo in conf mode
          writeServos();
        #endif
        #ifdef FLYING_WING
          servo[0]  = conf.wing_left_mid;
          servo[1]  = conf.wing_right_mid;
          writeServos();
        #endif
        #ifdef AIRPLANE
          for(i = 4; i<7 ;i++) servo[i] = 1500;
          writeServos();
        #endif          
        #if defined(LCD_CONF)
          configurationLoop(); // beginning LCD configuration
        #endif
        previousTime = micros();
      }
    }
  #if defined(INFLIGHT_ACC_CALIBRATION)  
    else if (!f.ARMED && rcData[YAW] < MINCHECK && rcData[PITCH] > MAXCHECK && rcData[ROLL] > MAXCHECK){
      if (rcDelayCommand == 20){
        if (AccInflightCalibrationMeasurementDone){                // trigger saving into eeprom after landing
          AccInflightCalibrationMeasurementDone = 0;
          AccInflightCalibrationSavetoEEProm = 1;
        }else{ 
          AccInflightCalibrationArmed = !AccInflightCalibrationArmed; 
          #if defined(BUZZER)
          if (AccInflightCalibrationArmed){
            beep_toggle = 2;
          } else {
            beep_toggle = 3;
          }
          #endif
        }
      }
    } 
  #endif
    else if (conf.activate[BOXARM] > 0) {
      if ( rcOptions[BOXARM] && f.OK_TO_ARM
      #if defined(FAILSAFE)
        && failsafeCnt <= 1
      #endif 
      ) {
        f.ARMED = 1;
        headFreeModeHold = heading;
      } else if (f.ARMED) f.ARMED = 0;
      rcDelayCommand = 0;
    #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
    } else if ( (rcData[YAW] < MINCHECK )  && f.ARMED) {
      if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
    } else if ( (rcData[YAW] > MAXCHECK ) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
      if (rcDelayCommand == 20) {
        f.ARMED = 1;
        headFreeModeHold = heading;
      }
    #endif
    #ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
    } else if ( (rcData[ROLL] < MINCHECK)  && f.ARMED) {
      if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
    } else if ( (rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
      if (rcDelayCommand == 20) {
        f.ARMED = 1;
        headFreeModeHold = heading;
      }
    #endif
    #ifdef LCD_TELEMETRY_AUTO
    } else if (rcData[ROLL] < MINCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
      if (rcDelayCommand == 20) {
         if (telemetry_auto) {
            telemetry_auto = 0;
            telemetry = 0;
         } else
            telemetry_auto = 1;
      }
    #endif
    #ifdef LCD_TELEMETRY_STEP
    } else if (rcData[ROLL] > MAXCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
      if (rcDelayCommand == 20) {
        telemetry = telemetryStepSequence[++telemetryStepIndex % strlen(telemetryStepSequence)];
        LCDclear(); // make sure to clear away remnants
      }
    #endif
    } else
      rcDelayCommand = 0;
  } else if (rcData[THROTTLE] > MAXCHECK && !f.ARMED) {
    if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {        // throttle=max, yaw=left, pitch=min
      if (rcDelayCommand == 20) calibratingA=400;
      rcDelayCommand++;
    } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MINCHECK) { // throttle=max, yaw=right, pitch=min  
      if (rcDelayCommand == 20) f.CALIBRATE_MAG = 1; // MAG calibration request
      rcDelayCommand++;
    } else if (rcData[PITCH] > MAXCHECK) {
       conf.angleTrim[PITCH]+=2;writeParams(1);
       #if defined(LED_RING)
         blinkLedRing();
       #endif
    } else if (rcData[PITCH] < MINCHECK) {
       conf.angleTrim[PITCH]-=2;writeParams(1);
       #if defined(LED_RING)
         blinkLedRing();
       #endif
    } else if (rcData[ROLL] > MAXCHECK) {
       conf.angleTrim[ROLL]+=2;writeParams(1);
       #if defined(LED_RING)
         blinkLedRing();
       #endif
    } else if (rcData[ROLL] < MINCHECK) {
       conf.angleTrim[ROLL]-=2;writeParams(1);
       #if defined(LED_RING)
         blinkLedRing();
       #endif
    } else {
      rcDelayCommand = 0;
    }
  }
}

void handleBoxes() {
  uint8_t i = 0;
  uint16_t auxState = 0;
  
  #if defined(LED_FLASHER)
    led_flasher_autoselect_sequence();
  #endif
  
  #if defined(INFLIGHT_ACC_CALIBRATION)
    if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ){ // Copter is airborne and you are turning it off via boxarm : start measurement
      InflightcalibratingA = 50;
      AccInflightCalibrationArmed = 0;
    }  
    if (rcOptions[BOXPASSTHRU]) {      // Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
      if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone){
        InflightcalibratingA = 50;
      }
    }else if(AccInflightCalibrationMeasurementDone && !f.ARMED){
      AccInflightCalibrationMeasurementDone = 0;
      AccInflightCalibrationSavetoEEProm = 1;
    }
  #endif  
  
  for(i=0;i<4;i++)
    auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);
  for(i=0;i<CHECKBOXITEMS;i++)
    rcOptions[i] = (auxState & conf.activate[i])>0;

  // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false
  #if ACC
    if ( rcOptions[BOXANGLE] || (failsafeCnt > 5*FAILSAFE_DELAY) ) { 
      // bumpless transfer to Level mode
      if (!f.ANGLE_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.ANGLE_MODE = 1;
      }  
    } else {
      // failsafe support
      f.ANGLE_MODE = 0;
    }
    if ( rcOptions[BOXHORIZON] ) { 
      if (!f.HORIZON_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.HORIZON_MODE = 1;
      }
    } else {
      f.HORIZON_MODE = 0;
    }
  #endif

  if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
  #if !defined(GPS_LED_INDICATOR)
    if (f.ANGLE_MODE || f.HORIZON_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
  #endif

  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    if (rcOptions[BOXBARO]) {
        if (!f.BARO_MODE) {
          f.BARO_MODE = 1;
          AltHold = EstAlt;
          initialThrottleHold = rcCommand[THROTTLE];
          errorAltitudeI = 0;
          BaroPID=0;
        }
    } else {
        f.BARO_MODE = 0;
    }
  #endif
  #if MAG
    if (rcOptions[BOXMAG]) {
      if (!f.MAG_MODE) {
        f.MAG_MODE = 1;
        magHold = heading;
      }
    } else {
      f.MAG_MODE = 0;
    }
    if (rcOptions[BOXHEADFREE]) {
      if (!f.HEADFREE_MODE) {
        f.HEADFREE_MODE = 1;
      }
    } else {
      f.HEADFREE_MODE = 0;
    }
    if (rcOptions[BOXHEADADJ]) {
      headFreeModeHold = heading; // acquire new heading
    }
  #endif
  
  #if GPS
    #if defined(I2C_GPS)
    static uint8_t GPSNavReset = 1;
    if (f.GPS_FIX && GPS_numSat >= 5 ) {
      if (!rcOptions[BOXGPSHOME] && !rcOptions[BOXGPSHOLD] )
        {    //Both boxes are unselected
          if (GPSNavReset == 0 ) { 
             GPSNavReset = 1; 
             GPS_I2C_command(I2C_GPS_COMMAND_STOP_NAV,0);
          }
        }  
      if (rcOptions[BOXGPSHOME]) {
       if (!f.GPS_HOME_MODE)  {
          f.GPS_HOME_MODE = 1;
          GPSNavReset = 0;
          GPS_I2C_command(I2C_GPS_COMMAND_START_NAV,0);        //waypoint zero
        }
      } else {
        f.GPS_HOME_MODE = 0;
      }
      if (rcOptions[BOXGPSHOLD]) {
        if (!f.GPS_HOLD_MODE & !f.GPS_HOME_MODE) {
          f.GPS_HOLD_MODE = 1;
          GPSNavReset = 0;
          GPS_I2C_command(I2C_GPS_COMMAND_POSHOLD,0);
        }
      } else {
        f.GPS_HOLD_MODE = 0;
      }
    }
    #endif 
    #if defined(GPS_SERIAL) || defined(TINY_GPS) || defined(GPS_FROM_OSD)
    if (f.GPS_FIX && GPS_numSat >= 5 ) {
      if (rcOptions[BOXGPSHOME]) {
        if (!f.GPS_HOME_MODE)  {
          f.GPS_HOME_MODE = 1;
          GPS_set_next_wp(&GPS_home[LAT],&GPS_home[LON]);
          nav_mode    = NAV_MODE_WP;
        }
      } else {
        f.GPS_HOME_MODE = 0;
      }
      if (rcOptions[BOXGPSHOLD]) {
        if (!f.GPS_HOLD_MODE) {
          f.GPS_HOLD_MODE = 1;
          GPS_hold[LAT] = GPS_coord[LAT];
          GPS_hold[LON] = GPS_coord[LON];
          GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON]);
          nav_mode = NAV_MODE_POSHOLD;
        }
      } else {
        f.GPS_HOLD_MODE = 0;
      }
    }
    #endif
  #endif
  
  #if defined(FIXEDWING) || defined(HELICOPTER) || defined(INFLIGHT_ACC_CALIBRATION)
    if (rcOptions[BOXPASSTHRU]) {f.PASSTHRU_MODE = 1;}
    else {f.PASSTHRU_MODE = 0;}
  #endif
  
  #ifdef FIXEDWING 
    f.HEADFREE_MODE = 0;
  #endif
}

