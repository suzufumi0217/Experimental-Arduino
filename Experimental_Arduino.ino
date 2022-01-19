/****************************************************************
   Example2_Advanced.ino
   ICM 20948 Arduino Library Demo
   Shows how to use granular configuration of the ICM 20948
   Owen Lyke @ SparkFun Electronics
   Original Creation Date: April 17 2019

   This code is beerware; if you see me (or any other SparkFun employee) at the
   local, and you've found our code helpful, please buy us a round!

   Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "MsTimer2.h"

//SPIを今回は使ってない
#define INT_PIN 7
//#define USE_SPI       // Uncomment this to use SPI


#define SERIAL_PORT Serial

#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 10000000// You can override the default SPI frequency
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define ADleft_VAL   0 //ここが重要，I2CのアドレスをそれぞれのIMUにふってる．
#define ADright_VAL   1
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when
// the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
ICM_20948_SPI myICM_right;  // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM_right;  // Otherwise create an ICM_20948_I2C object
#endif

#ifdef USE_SPI
ICM_20948_SPI myICM_left;  // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM_left;  // Otherwise create an ICM_20948_I2C object
#endif
//#define numOfThresholds 3

int i = 0;
int num_thresholds = 0;
int num_time = 0;
int trial_count = 0;
int prev_state = 0;
int current_state = 0;
float prev_w_hip, prev_aa_hip, current_aa_hip, prev_time, current_time, start_time, time_for_output;
float time_stamp1 = 0.0;
float time_stamp2 = 0.0;
float time_stamp3 = 0.0;
float time_stamp4 = 0.0;
float calibration_steps = 0;
int detection_steps = 0;

bool isHigh = true;
volatile bool isRecording = false;
volatile bool isMainRight = false;  //MainLegが左右どちらかを表す．bool変数を用意．
volatile bool isMainLeft = false;
volatile bool isDetection = false;
volatile bool isDetected = false;
volatile bool isReceivedC_params = false;
volatile bool isCalibration = false;
//volatile bool isStart = false;
volatile bool isRecievedD_param = false;
volatile bool isRecievedRorL = false;

int fsrAnalogPin_right = 0; // FSR0 is connected to analog 0
int fsrAnalogPin_left = 1; // FSR1 is connected to analog 1
int Rightheelstrike = 0;
int Leftheelstrike = 0;

float fsr_Right, fsr_Left;

float thresholds_R_max,
      thresholds_L_max,
      thresholds_FS;

//calibration
String sign;
char charC_param[20], charD_param[120];
int n_of_steps = 0;
float current_w_hip, current_FS;

//Detection
String RorL;
float step_duration, step_start_time, max_step_duration, threshold_hip_max;
volatile bool isfinishstep = false;
float prev_FS;
int n_ps; //integer for stucking in the preswing 
float preswing_dur = 0.2; //duration for pre-swing 
float FS_second_threshold = 1;

//Updating
volatile bool isRecievedNewD_param = false;

void isflag() {
  if (isCalibration) {
    isRecording = true;
  } else if (isDetection) {
    isDetected = true;
  }
}

void setup() {
  pinMode(INT_PIN, OUTPUT);
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {};

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  bool initialized = false;
  while ( !initialized ) {
    //右のIMUを設定
#ifdef USE_SPI
    myICM_right.begin( CS_PIN, SPI_PORT, SPI_FREQ ); // Here we are using the user-defined SPI_FREQ as the clock speed of the SPI bus
#else
    myICM_right.begin( WIRE_PORT, ADright_VAL );
#endif

    //左のIMUを設定
#ifdef USE_SPI
    myICM_left.begin( CS_PIN, SPI_PORT, SPI_FREQ ); // Here we are using the user-defined SPI_FREQ as the clock speed of the SPI bus
#else
    myICM_left.begin( WIRE_PORT, ADleft_VAL );
#endif

    SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    SERIAL_PORT.println( myICM_right.statusString() );
    SERIAL_PORT.println( myICM_left.statusString() );
    if ( myICM_right.status != ICM_20948_Stat_Ok || myICM_left.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println( "Trying again..." );
      delay(500);
    } else {
      initialized = true;
    }
  }

  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM_right.swReset( );
  myICM_left.swReset( );
  if ( myICM_right.status != ICM_20948_Stat_Ok || myICM_left.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM_right.statusString());
    SERIAL_PORT.println(myICM_left.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM_right.sleep( false );
  myICM_right.lowPower( false );
  myICM_left.sleep( false );
  myICM_left.lowPower( false );

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM_right.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous );
  if ( myICM_right.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM_right.statusString());
  }

  myICM_left.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous );
  if ( myICM_left.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM_left.statusString());
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  // gpm2
  // gpm4
  // gpm8
  // gpm16

  myFSS.g = dps1000;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  // dps250
  // dps500
  // dps1000
  // dps2000

  myICM_right.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );
  if ( myICM_right.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM_right.statusString());
  }

  myICM_left.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );
  if ( myICM_left.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM_left.statusString());
  }


  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d23bw9_n34bw4;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
  // acc_d111bw4_n136bw
  // acc_d50bw4_n68bw8
  // acc_d23bw9_n34bw4
  // acc_d11bw5_n17bw
  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d23bw9_n35bw9;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
  // gyr_d196bw6_n229bw8
  // gyr_d151bw8_n187bw6
  // gyr_d119bw5_n154bw3
  // gyr_d51bw2_n73bw3
  // gyr_d23bw9_n35bw9
  // gyr_d11bw6_n17bw8
  // gyr_d5bw7_n8bw9
  // gyr_d361bw4_n376bw5

  myICM_right.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  if ( myICM_right.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM_right.statusString());
  }

  myICM_left.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  if ( myICM_left.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM_left.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat_right = myICM_right.enableDLPF( ICM_20948_Internal_Acc, true );
  ICM_20948_Status_e gyrDLPEnableStat_right = myICM_right.enableDLPF( ICM_20948_Internal_Gyr, true );
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(myICM_right.statusString(accDLPEnableStat_right));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); SERIAL_PORT.println(myICM_right.statusString(gyrDLPEnableStat_right));

  ICM_20948_Status_e accDLPEnableStat_left = myICM_left.enableDLPF( ICM_20948_Internal_Acc, true );
  ICM_20948_Status_e gyrDLPEnableStat_left = myICM_left.enableDLPF( ICM_20948_Internal_Gyr, true );
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(myICM_left.statusString(accDLPEnableStat_left));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); SERIAL_PORT.println(myICM_left.statusString(gyrDLPEnableStat_left));

  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 10;
  mySmplrt.a = 10;
  myICM_right.setSampleRate( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt );
  myICM_left.setSampleRate( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt );
  SERIAL_PORT.print(F("setSampleRate returned: "));
  SERIAL_PORT.println(myICM_right.statusString());
  SERIAL_PORT.println(myICM_left.statusString());
  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Configuration complete!"));
  MsTimer2::set(10, isflag); //10msごとにisflagを呼び出している．
  MsTimer2::start();
}

void loop() {
  if (SERIAL_PORT.available() > 0) { // キー入力待ち
    sign = SERIAL_PORT.readStringUntil('\r');//文字取り出す
    if (sign == "c") {
      while (!isReceivedC_params) {
        if (SERIAL_PORT.available() > 0) {
          charC_param[num_time] = SERIAL_PORT.read();
          if (charC_param[num_time] == '\r') {
            charC_param[num_time] = '\0';
            calibration_steps = atof(strtok(charC_param, ","));
            max_step_duration = atof(strtok(NULL, ","));
            isReceivedC_params = true;
            num_time = 0;
          } else {
            num_time += 1;
          }
        }
      }
      SERIAL_PORT.println("start calibration");
      thresholds_FS = 1; // for calibration
      isCalibration = true;
      start_time = millis();
      //digitalWrite(INT_PIN, isHigh); //接続機器に対して，５Vを発することで同期を行っている．
    } else if (sign == "d") {
      while (!isRecievedD_param) {
        if (SERIAL_PORT.available() > 0) {
          charD_param[num_thresholds] = SERIAL_PORT.read();
          if (charD_param[num_thresholds] == '\r') {
            charD_param[num_thresholds] == '\0';
            //matlabからthresholdsを受け取って，代入する
            thresholds_R_max = atof(strtok(charD_param, ","));
            thresholds_L_max = atof(strtok(NULL, ","));
            thresholds_FS = atof(strtok(NULL, ","));
            detection_steps = atof(strtok(NULL, ","));
            max_step_duration = atof(strtok(NULL, ","));
            isRecievedD_param = true;
            SERIAL_PORT.println("Complete recieving D_params");
            SERIAL_PORT.println();
            num_thresholds = 0;
          } else {
            num_thresholds += 1;
          }
        }
      }
      isDetection = true;
      SERIAL_PORT.println("start detection");
      start_time = millis();
    } else if (sign == "u") {
      while (!isRecievedNewD_param) {
        if (SERIAL_PORT.available() > 0) {
          charD_param[num_thresholds] = SERIAL_PORT.read();
          if (charD_param[num_thresholds] == '\r') {
            charD_param[num_thresholds] == '\0';
            //matlabからthresholdsを受け取って，代入する
            thresholds_R_max = atof(strtok(charD_param, ","));
            thresholds_L_max = atof(strtok(NULL, ","));
            thresholds_FS = atof(strtok(NULL, ","));
            detection_steps = atof(strtok(NULL, ","));
            max_step_duration = atof(strtok(NULL, ","));
            isRecievedNewD_param = true;
            SERIAL_PORT.println("Complete recieving new D_params");
            SERIAL_PORT.println();
            num_thresholds = 0;
          } else {
            num_thresholds += 1;
          }
        }
      }
      isDetection = true;
      SERIAL_PORT.println("Restart detection");
      start_time = millis();
      digitalWrite(INT_PIN, isHigh); //接続機器に対して，５Vを発することで同期を行っている．
    }
  }
  if (isRecording) {
    while (!isRecievedRorL) {
      if (SERIAL_PORT.available() > 0) { //キー入力待ち
        RorL = SERIAL_PORT.readStringUntil('\r');//文字取り出す
        if (RorL == "r") {
          //mainleg をrightとする
          isMainRight = true;
          step_start_time = millis();
          isRecievedRorL = true;
        } else if (RorL == "l") {
          //mainlegをleftとする
          isMainLeft = true;
          step_start_time = millis();
          isRecievedRorL = true;
        }
      }
    }
    if ( myICM_right.dataReady() && myICM_left.dataReady()) {

      myICM_right.getAGMT();                // The values are only updated when you call 'getAGMT'
      myICM_left.getAGMT();

      //Footswitches get data
      fsr_Right = analogRead(fsrAnalogPin_right);
      fsr_Left = analogRead(fsrAnalogPin_left);

      if (isMainRight) {
        //変数に取得したデータを代入している
        current_w_hip = myICM_right.gyrZ();
        current_FS = fsr_Right / 1024 * 5;

        //matlabに送信するデータを作る
        current_time = millis();
        step_duration = (current_time - step_start_time) / 1000.00;
        time_for_output = (current_time - start_time) / 1000.00;

      } else if (isMainLeft) {
        //変数に取得したデータを代入している
        current_w_hip = - myICM_left.gyrZ();
        current_FS = fsr_Left / 1024 * 5;


        //matlabに送信するデータを作る
        current_time = millis();
        step_duration = (current_time - step_start_time) / 1000.00;
        time_for_output = (current_time - start_time) / 1000.00;
      }

      //check if one step is finish or not.
      //まだ完成してない"use FS for calibration time in each step"
      if (step_duration >= max_step_duration) {
        SERIAL_PORT.println("Finish Step");
        isRecievedRorL = false;
        isMainRight = false;
        isMainLeft = false;
        n_of_steps += 1;
      } else {
        SERIAL_PORT.print(step_duration, 3);
        SERIAL_PORT.print(",");
        SERIAL_PORT.print(current_FS);
        SERIAL_PORT.print(",");
        SERIAL_PORT.println(current_w_hip);
      }

      isRecording = false;

      //check if number of steps exceed calibration_steps
      if (n_of_steps >= calibration_steps) {
        SERIAL_PORT.println("Finish calibration section");
        isCalibration = false;
        isReceivedC_params  = false;
        n_of_steps = 0;
      }
    }
    else {
      SERIAL_PORT.println("Waiting");
      delay(100);
    }
  } else if (isDetected) {
    while (!isRecievedRorL) {
      if (SERIAL_PORT.available() > 0) { //キー入力待ち
        RorL = SERIAL_PORT.readStringUntil('\r');//文字取り出す
        if (RorL == "r") {
          //mainleg をrightとする
          isMainRight = true;
          //initialize variables
          current_state = 2;
          n_ps = 1;
          step_start_time = millis();
          isRecievedRorL = true;
          digitalWrite(INT_PIN, isHigh); //Power labに対して，５Vの電圧上昇を送信
        } else if (RorL == "l") {
          //mainlegをleftとする
          isMainLeft = true;
          //initialize variables
          current_state = 2;
          n_ps = 1;
          step_start_time = millis();
          isRecievedRorL = true;
          digitalWrite(INT_PIN, isHigh); //Power labに対して，５Vの電圧上昇を送信
        } else if (RorL == "e") {
          isDetection = false;
          isDetected = false;
          isRecievedRorL = true;
          isfinishstep = false;
          SERIAL_PORT.println("End Detection");
        }
      }
    }
    if ( myICM_right.dataReady() && myICM_left.dataReady()) {
      myICM_right.getAGMT();                // The values are only updated when you call 'getAGMT'
      myICM_left.getAGMT();

      //Footswitches get data
      fsr_Right = analogRead(fsrAnalogPin_right);
      fsr_Left = analogRead(fsrAnalogPin_left);
      //Initialize variables
      prev_FS = 0;
      

      if (isMainRight) {
        //変数に取得したデータを代入している
        prev_FS = current_FS;
        current_FS = fsr_Right / 1024 * 5;
        prev_state = current_state;
        prev_w_hip = current_w_hip;
        prev_aa_hip = current_aa_hip;
        current_w_hip = myICM_right.gyrZ();
        current_aa_hip = current_w_hip - prev_w_hip;
        threshold_hip_max = thresholds_R_max;
      } else if (isMainLeft) {
        //変数に取得したデータを代入している
        prev_FS = current_FS;
        current_FS = fsr_Left / 1024 * 5;
        prev_state = current_state;
        prev_w_hip = current_w_hip;
        prev_aa_hip = current_aa_hip;
        current_w_hip = - myICM_left.gyrZ();
        current_aa_hip = current_w_hip - prev_w_hip;
        threshold_hip_max = thresholds_L_max;
      }

      //Detect StateChange
      //T3
      if (prev_state == 2) { 
        if (current_FS < thresholds_FS && prev_FS > thresholds_FS) {
          current_state = 3;
        }
      }
      //T4
      else if (prev_state == 3) {
        if(n_ps * 0.01 < preswing_dur){
          current_state = 3;
          n_ps += 1;
        }else{
          current_state = 4;
        }
      }
      //T1
      else if (prev_state == 4) {
        if (prev_w_hip > threshold_hip_max / 4 && current_w_hip <= threshold_hip_max / 4 && current_FS < thresholds_FS) {
          current_state = 1;
        }
      }
      //T2
      else if (prev_state == 1) {
        if (  current_FS > FS_second_threshold ) {
          current_state = 5;
        }
      }
      //TE
      else if (prev_state == 5) {
        isMainRight = false;
        isMainLeft = false;
        isRecievedRorL = false;
        isfinishstep = true;//isfinishstepとか書いて下のprintに入らない様にするわ．
      }

      //Send Data to matlab
      current_time = millis();
      step_duration = (current_time - step_start_time) / 1000.00;
      time_for_output = (current_time - start_time) / 1000.00;
      //Check Step is finish or not.
      if (!isfinishstep) {
        SERIAL_PORT.print(step_duration, 3);
        SERIAL_PORT.print(",");
        SERIAL_PORT.print(current_state);
        SERIAL_PORT.print(",");
        SERIAL_PORT.print(current_FS);
        SERIAL_PORT.print(",");
        SERIAL_PORT.println(current_w_hip);
      } else {
        SERIAL_PORT.println("Finish Step");
        isfinishstep = false;
        isRecievedRorL = false;
        isMainRight = false;
        isMainLeft = false;
        n_of_steps += 1;
        digitalWrite(INT_PIN, !isHigh); //Power labに対して，５Vの電圧下降を送信
      }
      isDetected = false;

      //check if step_duration is over max_step_duration or not
      if (step_duration >= max_step_duration) {
        isRecievedRorL = false;
        isMainRight = false;
        isMainLeft = false;
        SERIAL_PORT.println("Time is up");
        n_of_steps += 1;
        digitalWrite(INT_PIN, !isHigh); //Power labに対して，５Vの電圧下降を送信
      }

      //check if detection finish or not
      if (n_of_steps >= detection_steps) {
        SERIAL_PORT.println("Finish section");
        isDetection = false;
        isRecievedD_param = false;
        isRecievedRorL = false;
        n_of_steps = 0;
      }
    } else {
      SERIAL_PORT.println("Waiting");
      delay(100);
    }

  }
}
