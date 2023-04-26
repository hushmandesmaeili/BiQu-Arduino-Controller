#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <math.h>

#define GEAR_RATIO 9
#define NUM_JOINTS 8
#define NUM_IMU_AXES 3
#define PREAMBLE_LENGTH 4
#define DATA_BYTE_LENGTH 2
#define DEADBAND 3

// Serial1, 2, 3, 4, 5, and 6 are built-in variables defined by the Teensy board definition
// these lines simply rename the serial objects to represent the odrive they're tied to
/**
 * Odrv0 - FL upper leg & FR upper leg (2mm connectors)
 * Odrv1 - FL Hip FE & FR Hip FE
 * Odrv2 - FR Hip AA & FL Hip AA
 * Odrv3 - HR Hip AA & HL Hip AA
 * Odrv4 - HL upper leg & HR upper leg
 * Odrv5 - HL Hip FE && HR Hip FE
 * 
 * OUTDATED CONFIGURATION -- CHECK full_leg_control_test FOR SETUP
 **/
HardwareSerial& serial_FLFR_UL = Serial5;
HardwareSerial& serial_FLFR_FE = Serial4;
HardwareSerial& serial_FRFL_AA = Serial3;
HardwareSerial& serial_HRHL_AA = Serial1;
HardwareSerial& serial_HLHR_FE = Serial6;
HardwareSerial& serial_HLHR_UL = Serial2;

// OdriveArduino objects 
ODriveArduino odrive0(serial_FLFR_UL);      // FL and FR Upper Leg
ODriveArduino odrive1(serial_FLFR_FE);      // FL and FR Hip FE
ODriveArduino odrive2(serial_FRFL_AA);      // FL and FR Hip AA
ODriveArduino odrive3(serial_HRHL_AA);      // HR and HL Hip AA
ODriveArduino odrive4(serial_HLHR_UL);      // HL and HR Upper Leg
ODriveArduino odrive5(serial_HLHR_FE);      // HL and HR Hip FE

// STRUCT
struct spine_biqu_data_t{
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
  int32_t flags[4];
  int32_t checksum;
};

float q_joint[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float qd_joint[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void setup()
{
  // Start Serial Monitor
  Serial.begin(115200); 
  Serial.println("Monitor On!");
  // Start Serial Communication with Odrives
  serial_FLFR_UL.begin(921600);
  serial_FLFR_FE.begin(921600);
  serial_FRFL_AA.begin(921600);
  serial_HRHL_AA.begin(921600);
  serial_HLHR_FE.begin(921600);
  serial_HLHR_UL.begin(921600);
}

void loop()
{
  if (Serial.available())
  {
    char c = Serial.read();


    // all legs calibration
    if (c == 'o')
    {
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
      Serial.print("All Leg: Requesting state "); Serial.print(requested_state); Serial.println("");
      // odrive0.run_state(0, requested_state, true);
      Serial.println("0-0 done");
      // odrive0.run_state(1, requested_state, true);
      Serial.println("0-1 done");
      // odrive1.run_state(0, requested_state, true);
      Serial.println("1-0 done");
      // odrive1.run_state(1, requested_state, true);
      Serial.println("1-1 done");
      //odrive2.run_state(0, requested_state, true);
      Serial.println("2-0 done");
      //odrive2.run_state(1, requested_state, true);
      Serial.println("2-1 done");
      odrive3.run_state(0, requested_state, true);
      Serial.println("3-0 done");
      odrive3.run_state(1, requested_state, true);
      Serial.println("3-1 done");
      odrive4.run_state(0, requested_state, true);
      Serial.println("4-0 done");
      odrive4.run_state(1, requested_state, true);
      Serial.println("4-1 done");
      odrive5.run_state(0, requested_state, true);
      Serial.println("5-0 done");
      odrive5.run_state(1, requested_state, true);
      Serial.println("5-1 done");
      
      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial.print("All Leg: Requesting state "); Serial.print(requested_state); Serial.println("");
      //odrive0.run_state(0, requested_state, false);
      //odrive0.run_state(1, requested_state, false);
      //odrive1.run_state(0, requested_state, false);
      //odrive1.run_state(1, requested_state, false);
      //odrive2.run_state(0, requested_state, false);
      //odrive2.run_state(1, requested_state, false);
      odrive3.run_state(0, requested_state, false);
      odrive3.run_state(1, requested_state, false);
      odrive4.run_state(0, requested_state, false);
      odrive4.run_state(1, requested_state, false);
      odrive5.run_state(0, requested_state, false);
      odrive5.run_state(1, requested_state, false); // don't wait
    }

    // Just calibrating one ODrive: front two upper legs
    if (c == 'j')
    {
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
      Serial.print("All Leg: Requesting state "); Serial.print(requested_state); Serial.println("");
      odrive0.run_state(0, requested_state, true);
      Serial.println("0-0 done");
      odrive0.run_state(1, requested_state, true);
      Serial.println("0-1 done");

      odrive1.run_state(0, requested_state, true);
      Serial.println("1-0 done");
      odrive1.run_state(1, requested_state, true);
      Serial.println("1-1 done");

      odrive5.run_state(0, requested_state, true);
      Serial.println("5-0 done");
      odrive5.run_state(1, requested_state, true);
      Serial.println("5-1 done");
      
      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial.print("All Leg: Requesting state "); Serial.print(requested_state); Serial.println("");
      odrive0.run_state(0, requested_state, false);
      odrive0.run_state(1, requested_state, false);

      odrive1.run_state(0, requested_state, false);
      odrive1.run_state(1, requested_state, false);

      odrive5.run_state(0, requested_state, false);
      odrive5.run_state(1, requested_state, false);
    }

    if (c == 'z')
    {
      float start = micros();

      // set position by batch
      odrive0.SetBatchPosition(0.0f, 0.0f);     // Serial 5
      odrive1.SetBatchPosition(0.0f, 0.0f);     // Serial 4
      odrive2.SetBatchPosition(0.0f, 0.0f);     // Serial 3
      odrive3.SetBatchPosition(0.0f, 0.0f);     // Serial 1
      odrive4.SetBatchPosition(0.0f, 0.0f);     // Serial 2
      odrive5.SetBatchPosition(0.0f, 0.0f);     // Serial 6
      // read feedback
      // String state = odrive0.readState();
      Serial5.print("f\n");
      Serial4.print("f\n");
      Serial3.print("f\n");
      Serial1.print("f\n");
      Serial2.print("f\n");
      Serial6.print("f\n");
      
      String str1 = "";
      String str2 = "";
      String str3 = "";
      String str4 = "";
      String str5 = "";
      String str6 = "";

      bool odr0_flag = true;
      bool odr1_flag = true;
      bool odr2_flag = true;
      bool odr3_flag = true;
      bool odr4_flag = true;
      bool odr5_flag = true;

      while (odr0_flag || odr1_flag || odr2_flag || odr3_flag || odr4_flag || odr5_flag)  {
        if (Serial1.available() && odr3_flag) {
            char c = Serial1.read();
            if (c == '\n')
                odr3_flag = false;
            str1 += c;
        }

        if (Serial2.available() && odr4_flag) {
            char c = Serial2.read();
            if (c == '\n')
                odr4_flag = false;
            str2 += c;
        }

        if (Serial3.available() && odr2_flag) {
            char c = Serial3.read();
            if (c == '\n')
                odr2_flag = false;
            str3 += c;
        }

        if (Serial4.available() && odr1_flag) {
            char c = Serial4.read();
            if (c == '\n')
                odr1_flag = false;
            str4 += c;
        }

        if (Serial5.available() && odr0_flag) {
            char c = Serial5.read();
            if (c == '\n')
                odr0_flag = false;
            str5 += c;
        }

        if (Serial6.available() && odr5_flag) {
            char c = Serial6.read();
            if (c == '\n')
                odr5_flag = false;
            str6 += c;
        }
      }
      
      float end = micros();

      float timeSpent = end - start;
      
      Serial.print("time spend (ms): "); Serial.print(timeSpent); Serial.println("");
      // Serial.println(state);
      Serial.println(str1);
      Serial.println(str2);
      Serial.println(str3);
      Serial.println(str4);
      Serial.println(str5);
      Serial.println(str6);
    }
    
    if (c == 's')
    {
      float start = micros();

      // set position by batch
      odrive0.SetBatchPosition(0.5f, 0.5f);     // Serial 5
      odrive1.SetBatchPosition(0.5f, 0.5f);     // Serial 4
      odrive2.SetBatchPosition(0.5f, 0.5f);     // Serial 3
      odrive3.SetBatchPosition(0.5f, 0.5f);     // Serial 1
      odrive4.SetBatchPosition(0.5f, 0.5f);     // Serial 2
      odrive5.SetBatchPosition(0.5f, 0.5f);     // Serial 6
      // read feedback
      // String state = odrive0.readState();
      Serial5.print("f\n");
      Serial4.print("f\n");
      Serial3.print("f\n");
      Serial1.print("f\n");
      Serial2.print("f\n");
      Serial6.print("f\n");
      
      String str1 = "";
      String str2 = "";
      String str3 = "";
      String str4 = "";
      String str5 = "";
      String str6 = "";

      bool odr0_flag = true;
      bool odr1_flag = true;
      bool odr2_flag = true;
      bool odr3_flag = true;
      bool odr4_flag = true;
      bool odr5_flag = true;

      while (odr0_flag || odr1_flag || odr2_flag || odr3_flag || odr4_flag || odr5_flag)  {
        if (Serial1.available() && odr3_flag) {
            char c = Serial1.read();
            if (c == '\n')
                odr3_flag = false;
            str1 += c;
        }

        if (Serial2.available() && odr4_flag) {
            char c = Serial2.read();
            if (c == '\n')
                odr4_flag = false;
            str2 += c;
        }

        if (Serial3.available() && odr2_flag) {
            char c = Serial3.read();
            if (c == '\n')
                odr2_flag = false;
            str3 += c;
        }

        if (Serial4.available() && odr1_flag) {
            char c = Serial4.read();
            if (c == '\n')
                odr1_flag = false;
            str4 += c;
        }

        if (Serial5.available() && odr0_flag) {
            char c = Serial5.read();
            if (c == '\n')
                odr0_flag = false;
            str5 += c;
        }

        if (Serial6.available() && odr5_flag) {
            char c = Serial6.read();
            if (c == '\n')
                odr5_flag = false;
            str6 += c;
        }
      }
      
      float end = micros();

      float timeSpent = end - start;
      
      Serial.print("time spend (ms): "); Serial.print(timeSpent); Serial.println("");
      // Serial.println(state);
      Serial.println(str1);
      Serial.println(str2);
      Serial.println(str3);
      Serial.println(str4);
      Serial.println(str5);
      Serial.println(str6);
    }

    if ( c == 'f')
    {
      Serial.println(odrive0.readState());
    }

    if ( c == 'h')
    {
      Serial.println(odrive0.getHelp());
    }
  }
}
