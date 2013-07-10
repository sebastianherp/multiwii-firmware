/* GUI protocol implementation */

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "Output.h"
#include "GPS.h"
#include "MultiWii.h"
#include "Comm_GUI.h"
#include "Serial.h"

#define BIND_CAPABLE 0;  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module. 
#if defined(SPEK_BIND)
  #define BIND_CAPABLE 1;
#endif
// Capability is bit flags; next defines should be 2, 4, 8...

const uint32_t capability = 0+BIND_CAPABLE;

#ifdef DEBUGMSG
  #define DEBUG_MSG_BUFFER_SIZE 128
  static char debug_buf[DEBUG_MSG_BUFFER_SIZE];
  static uint8_t head_debug;
  static uint8_t tail_debug;
#endif

// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t cmdMSP[UART_NUMBER];

void evaluateCommand(uint8_t port);

void headSerialResponse(uint8_t port, uint8_t err, uint8_t s) {
  serialize8(port, '$');
  serialize8(port, 'M');
  serialize8(port, err ? '!' : '>');
  checksum[port] = 0; // start calculating a new checksum
  serialize8(port, s);
  serialize8(port, cmdMSP[port]);
}

void headSerialReply(uint8_t port, uint8_t s) {
  headSerialResponse(port, 0, s);
}

void inline headSerialError(uint8_t port, uint8_t s) {
  headSerialResponse(port, 1, s);
}

void tailSerialReply(uint8_t port) {
  serialize8(port, checksum[port]);
  UartSendData(port);
}

void serializeNames(uint8_t port, PGM_P s) {
  headSerialReply(port, strlen_P(s));
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(port, pgm_read_byte(c));
  }
}

uint8_t guiParser(uint8_t port, uint8_t c) {
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER];// = IDLE;  
  
  // regular data handling to detect and handle MSP and other data
  if (c_state[port] == IDLE) {
    c_state[port] = (c=='$') ? HEADER_START : IDLE;
    if (c_state[port] == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
  } else if (c_state[port] == HEADER_START) {
    c_state[port] = (c=='M') ? HEADER_M : IDLE;
  } else if (c_state[port] == HEADER_M) {
    c_state[port] = (c=='<') ? HEADER_ARROW : IDLE;
  } else if (c_state[port] == HEADER_ARROW) {
    if (c > INBUF_SIZE) {  // now we are expecting the payload size
      c_state[port] = IDLE;
    } else {
      indRX[port] = 0;
      dataSize[port] = c;
      offset[port] = 0;
      checksum[port] = 0;
      checksum[port] ^= c;
      c_state[port] = HEADER_SIZE;  // the command is to follow
    }
  } else if (c_state[port] == HEADER_SIZE) {
    cmdMSP[port] = c;
    checksum[port] ^= c;
    c_state[port] = HEADER_CMD;
  } else if (c_state[port] == HEADER_CMD && offset[port] < dataSize[port]) {
    checksum[port] ^= c;
    inBuf[offset[port]++][port] = c;
  } else if (c_state[port] == HEADER_CMD && offset[port] >= dataSize[port]) {
    if (checksum[port] == c) {  // compare calculated and transferred checksum
      evaluateCommand(port);  // we got a valid packet, evaluate it
    }
    c_state[port] = IDLE;
    return 1; // no more than one MSP per port and per cycle
  }
  return 0;
}

void  s_struct(uint8_t port, uint8_t *cb,uint8_t siz) {
  headSerialReply(port, siz);
  while(siz--) serialize8(port, *cb++);
}

void s_struct_w(uint8_t port, uint8_t *cb,uint8_t siz) {
 headSerialReply(port, 0);
  while(siz--) *cb++ = read8(port);
}

//#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand(uint8_t port) {
  uint32_t tmp=0; 

  switch(cmdMSP[port]) {
   case MSP_SET_RAW_RC:
     s_struct_w(port, (uint8_t*)&rcSerial,16);
     rcSerialCount = 50; // 1s transition 
     break;
   #if GPS
   case MSP_SET_RAW_GPS:
     f.GPS_FIX = read8(port);
     GPS_numSat = read8(port);
     GPS_coord[LAT] = read32(port);
     GPS_coord[LON] = read32(port);
     GPS_altitude = read16(port);
     GPS_speed = read16(port);
     GPS_update |= 2;              // New data signalisation to GPS functions
     headSerialReply(port, 0);
     break;
   #endif
   case MSP_SET_PID:
     s_struct_w(port, (uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
     break;
   case MSP_SET_BOX:
     s_struct_w(port, (uint8_t*)&conf.activate[0],CHECKBOXITEMS*2);
     break;
   case MSP_SET_RC_TUNING:
     s_struct_w(port, (uint8_t*)&conf.rcRate8,7);
     break;
   #if !defined(DISABLE_SETTINGS_TAB)
   case MSP_SET_MISC:
     struct {
       uint16_t a,b,c,d,e,f;
       uint32_t g;
       uint16_t h;
       uint8_t  i,j,k,l;
     } set_misc;
     s_struct_w(port, (uint8_t*)&set_misc,22);
     #if defined(POWERMETER)
       conf.powerTrigger1 = set_misc.a / PLEVELSCALE;
     #endif
     conf.minthrottle = set_misc.b;
     #ifdef FAILSAFE 
       conf.failsafe_throttle = set_misc.e;
     #endif  
     #if MAG
       conf.mag_declination = set_misc.h;
     #endif
     #if defined(VBAT)
       conf.vbatscale        = set_misc.i;
       conf.vbatlevel_warn1  = set_misc.j;
       conf.vbatlevel_warn2  = set_misc.k;
       conf.vbatlevel_crit   = set_misc.l;
     #endif
     break;
   case MSP_MISC:
     struct {
       uint16_t a,b,c,d,e,f;
       uint32_t g;
       uint16_t h;
       uint8_t  i,j,k,l;
     } misc;
     misc.a = intPowerTrigger1;
     misc.b = conf.minthrottle;
     misc.c = MAXTHROTTLE;
     misc.d = MINCOMMAND;
     #ifdef FAILSAFE 
       misc.e = conf.failsafe_throttle;
     #else  
       misc.e = 0;
     #endif
     #ifdef LOG_PERMANENT
       misc.f = plog.arm;
       misc.g = plog.lifetime + (plog.armed_time / 1000000); // <- computation must be moved outside from serial
     #else
       misc.f = 0; misc.g =0;
     #endif
     #if MAG
       misc.h = conf.mag_declination;
     #else
       misc.h = 0;
     #endif
     #ifdef VBAT
       misc.i = conf.vbatscale;
       misc.j = conf.vbatlevel_warn1;
       misc.k = conf.vbatlevel_warn2;
       misc.l = conf.vbatlevel_crit;
     #else
       misc.i = 0;misc.j = 0;misc.k = 0;misc.l = 0;
     #endif
     s_struct(port, (uint8_t*)&misc,22);
     break;
   #endif
   #if defined (DYNBALANCE)
     case MSP_SET_MOTOR:
       s_struct_w(port, (uint8_t*)&motor,16);
     break;
   #endif
   #ifdef MULTIPLE_CONFIGURATION_PROFILES
   case MSP_SELECT_SETTING:
     if(!f.ARMED) {
       global_conf.currentSet = read8(port);
       if(global_conf.currentSet>2) global_conf.currentSet = 0;
       writeGlobalSet(0);
       readEEPROM();
     }
     headSerialReply(port, 0);
     break;
   #endif
   case MSP_SET_HEAD:
     s_struct_w(port, (uint8_t*)&magHold,2);
     break;
   case MSP_IDENT:
     struct {
       uint8_t v,t,msp_v;
       uint32_t cap;
     } id;
     id.v     = VERSION;
     id.t     = MULTITYPE;
     id.msp_v = MSP_VERSION;
     id.cap   = capability|DYNBAL<<2|FLAP<<3;
     s_struct(port, (uint8_t*)&id,7);
     break;
   case MSP_STATUS:
     struct {
       uint16_t cycleTime,i2c_errors_count,sensor;
       uint32_t flag;
       uint8_t set;
     } st;
     st.cycleTime        = cycleTime;
     st.i2c_errors_count = i2c_errors_count;
     st.sensor           = ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4;
     #if ACC
       if(f.ANGLE_MODE)   tmp |= 1<<BOXANGLE;
       if(f.HORIZON_MODE) tmp |= 1<<BOXHORIZON;
     #endif
     #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
       if(f.BARO_MODE) tmp |= 1<<BOXBARO;
     #endif
     #if MAG
       if(f.MAG_MODE) tmp |= 1<<BOXMAG;
       #if !defined(FIXEDWING)
         if(f.HEADFREE_MODE)       tmp |= 1<<BOXHEADFREE;
         if(rcOptions[BOXHEADADJ]) tmp |= 1<<BOXHEADADJ;
       #endif
     #endif
     #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
       if(rcOptions[BOXCAMSTAB]) tmp |= 1<<BOXCAMSTAB;
     #endif
     #if defined(CAMTRIG)
       if(rcOptions[BOXCAMTRIG]) tmp |= 1<<BOXCAMTRIG;
     #endif
     #if GPS
       if(f.GPS_HOME_MODE) tmp |= 1<<BOXGPSHOME; 
       if(f.GPS_HOLD_MODE) tmp |= 1<<BOXGPSHOLD;
     #endif
     #if defined(FIXEDWING) || defined(HELICOPTER)
       if(f.PASSTHRU_MODE) tmp |= 1<<BOXPASSTHRU;
     #endif
     #if defined(BUZZER)
       if(rcOptions[BOXBEEPERON]) tmp |= 1<<BOXBEEPERON;
     #endif
     #if defined(LED_FLASHER)
       if(rcOptions[BOXLEDMAX]) tmp |= 1<<BOXLEDMAX;
       if(rcOptions[BOXLEDLOW]) tmp |= 1<<BOXLEDLOW;
     #endif
     #if defined(LANDING_LIGHTS_DDR)
       if(rcOptions[BOXLLIGHTS]) tmp |= 1<<BOXLLIGHTS;
     #endif
     #if defined(VARIOMETER)
       if(rcOptions[BOXVARIO]) tmp |= 1<<BOXVARIO;
     #endif
     #if defined(INFLIGHT_ACC_CALIBRATION)
       if(rcOptions[BOXCALIB]) tmp |= 1<<BOXCALIB;
     #endif
     #if defined(GOVERNOR_P)
       if(rcOptions[BOXGOV]) tmp |= 1<<BOXGOV;
     #endif
     #if defined(OSD_SWITCH)
       if(rcOptions[BOXOSD]) tmp |= 1<<BOXOSD;
     #endif
     if(f.ARMED) tmp |= 1<<BOXARM;
     st.flag             = tmp;
     st.set              = global_conf.currentSet;
     s_struct(port, (uint8_t*)&st,11);
     break;
   case MSP_RAW_IMU:
     #if defined(DYNBALANCE)
       for(uint8_t axis=0;axis<3;axis++) {imu.gyroData[axis]=imu.gyroADC[axis];imu.accSmooth[axis]= imu.accADC[axis];} // Send the unfiltered Gyro & Acc values to gui.
     #endif 
     s_struct(port, (uint8_t*)&imu,18);
     break;
   case MSP_SERVO:
     s_struct(port, (uint8_t*)&servo,16);
     break;
   case MSP_SERVO_CONF:
     s_struct(port, (uint8_t*)&conf.servoConf[0].min,56); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
     break;
   case MSP_SET_SERVO_CONF:
     s_struct_w(port, (uint8_t*)&conf.servoConf[0].min,56);
     break;
   case MSP_MOTOR:
     s_struct(port, (uint8_t*)&motor,16);
     break;
   case MSP_RC:
     s_struct(port, (uint8_t*)&rcData,RC_CHANS*2);
     break;
   #if GPS
   case MSP_RAW_GPS:
     headSerialReply(port, 16);
     serialize8(port, f.GPS_FIX);
     serialize8(port, GPS_numSat);
     serialize32(port, GPS_coord[LAT]);
     serialize32(port, GPS_coord[LON]);
     serialize16(port, GPS_altitude);
     serialize16(port, GPS_speed);
     serialize16(port, GPS_ground_course);        // added since r1172
     break;
   case MSP_COMP_GPS:
     headSerialReply(port, 5);
     serialize16(port, GPS_distanceToHome);
     serialize16(port, GPS_directionToHome);
     serialize8(port, GPS_update & 1);
     break;
   #endif
   case MSP_ATTITUDE:
     s_struct(port, (uint8_t*)&att,6);
     break;
   case MSP_ALTITUDE:
     s_struct(port, (uint8_t*)&alt,6);
     break;
   case MSP_ANALOG:
     s_struct(port, (uint8_t*)&analog,5);
     break;
   case MSP_RC_TUNING:
     s_struct(port, (uint8_t*)&conf.rcRate8,7);
     break;
   case MSP_PID:
     s_struct(port, (uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
     break;
   case MSP_PIDNAMES:
     serializeNames(port, pidnames);
     break;
   case MSP_BOX:
     s_struct(port, (uint8_t*)&conf.activate[0],2*CHECKBOXITEMS);
     break;
   case MSP_BOXNAMES:
     serializeNames(port, boxnames);
     break;
   case MSP_BOXIDS:
     headSerialReply(port, CHECKBOXITEMS);
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       serialize8(port, pgm_read_byte(&(boxids[i])));
     }
     break;
   case MSP_MOTOR_PINS:
     s_struct(port, (uint8_t*)&PWM_PIN,8);
     break;
   #if defined(USE_MSP_WP)    
   case MSP_WP:
     {
       int32_t lat = 0,lon = 0;
       uint8_t wp_no = read8(port);        //get the wp number  
       headSerialReply(port, 18);
       if (wp_no == 0) {
         lat = GPS_home[LAT];
         lon = GPS_home[LON];
       } else if (wp_no == 16) {
         lat = GPS_hold[LAT];
         lon = GPS_hold[LON];
       }
       serialize8(port, wp_no);
       serialize32(port, lat);
       serialize32(port, lon);
       serialize32(port, AltHold);           //altitude (cm) will come here -- temporary implementation to test feature with apps
       serialize16(port, 0);                 //heading  will come here (deg)
       serialize16(port, 0);                 //time to stay (ms) will come here 
       serialize8(port, 0);                  //nav flag will come here
     }
     break;
   case MSP_SET_WP:
     {
       int32_t lat = 0,lon = 0,alt = 0;
       uint8_t wp_no = read8(port);        //get the wp number
       lat = read32(port);
       lon = read32(port);
       alt = read32(port);                 // to set altitude (cm)
       read16();                       // future: to set heading (deg)
       read16();                       // future: to set time to stay (ms)
       read8();                        // future: to set nav flag
       if (wp_no == 0) {
         GPS_home[LAT] = lat;
         GPS_home[LON] = lon;
         f.GPS_HOME_MODE = 0;          // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
         f.GPS_FIX_HOME  = 1;
         if (alt != 0) AltHold = alt;  // temporary implementation to test feature with apps
       } else if (wp_no == 16) {       // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
         GPS_hold[LAT] = lat;
         GPS_hold[LON] = lon;
         if (alt != 0) AltHold = alt;  // temporary implementation to test feature with apps
         #if !defined(I2C_GPS)
           nav_mode      = NAV_MODE_WP;
           GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON]);
         #endif
       }
     }
     headSerialReply(port, 0);
     break;
   #endif
   case MSP_RESET_CONF:
     if(!f.ARMED) LoadDefaults();
     headSerialReply(port, 0);
     break;
   case MSP_ACC_CALIBRATION:
     if(!f.ARMED) calibratingA=512;
     headSerialReply(port, 0);
     break;
   case MSP_MAG_CALIBRATION:
     if(!f.ARMED) f.CALIBRATE_MAG = 1;
     headSerialReply(port, 0);
     break;
   #if defined(SPEK_BIND)
   case MSP_BIND:
     spekBind();  
     headSerialReply(port, 0);
     break;
   #endif
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(port, 0);
     break;
   case MSP_DEBUG:
     s_struct(port, (uint8_t*)&debug,8);
     break;
   #ifdef DEBUGMSG
   case MSP_DEBUGMSG:
     {
       uint8_t size = debugmsg_available();
       if (size > 16) size = 16;
       headSerialReply(port, size);
       debugmsg_serialize(port, size);
     }
     break;
   #endif
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(port, 0);
     break;
  }
  tailSerialReply(port);
}
