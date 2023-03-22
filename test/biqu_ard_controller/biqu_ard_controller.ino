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
 * Odrv4 - HL Hip FE && HR Hip FE
 * Odrv5 - HL upper leg & HR upper leg
 **/
HardwareSerial& serial_FLFR_UL = Serial1;
HardwareSerial& serial_FLFR_FE = Serial2;
HardwareSerial& serial_FRFL_AA = Serial3;
HardwareSerial& serial_HRHL_AA = Serial4;
HardwareSerial& serial_HLHR_FE = Serial5;
HardwareSerial& serial_HLHR_UL = Serial6;

// OdriveArduino objects 
ODriveArduino odrive0(serial_FLFR_UL);      // FL and FR Upper Leg
ODriveArduino odrive1(serial_FLFR_FE);      // FL and FR Hip FE
ODriveArduino odrive2(serial_FRFL_AA);      // FL and FR Hip AA
ODriveArduino odrive3(serial_HRHL_AA);      // HR and HL Hip AA
ODriveArduino odrive4(serial_HLHR_FE);      // HL and HR Hip FE
ODriveArduino odrive5(serial_HLHR_UL);      // HL and HR Upper Leg

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
}

float q_joint[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
float qd_joint[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}

void calibration()
{
  int requested_state;

  requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  if (!odrive.run_state(0, requested_state, true))
    return;
}

void read_q_joint(OdriveArduino odrive) {
  
}

// reads current position estimate from odrive and stores in global array
void update_q_joint() {
  serial_FLFR_UL.write("r axis0.encoder.pos_estimate\n"); 
  cur_raw_pos[0] = serial_FLFR_UL.readFloat();
  cur_joint_pos[0] = fmod(cur_raw_pos[0]*(360/GEAR_RATIO), 360);

  serial_FLFR_UL.write("r axis1.encoder.pos_estimate\n"); 
  cur_raw_pos[1] = serial_FLFR_UL.readFloat();
  cur_joint_pos[1] = fmod(cur_raw_pos[1]*(360/GEAR_RATIO), 360);

  serial_FLFR_FE.write("r axis0.encoder.pos_estimate\n");
  cur_raw_pos[2] = serial_FLFR_FE.readFloat();
  cur_joint_pos[2] = fmod(cur_raw_pos[2]*(360/GEAR_RATIO), 360);

  serial_FLFR_FE.write("r axis1.encoder.pos_estimate\n");
  cur_raw_pos[3] = serial_FLFR_FE.readFloat();
  cur_joint_pos[3] = fmod(cur_raw_pos[3]*(360/GEAR_RATIO), 360);

  serial_FRFL_AA.write("r axis0.encoder.pos_estimate\n");
  cur_raw_pos[4] = serial_FRFL_AA.readFloat();
  cur_joint_pos[4] = fmod(cur_raw_pos[4]*(360/GEAR_RATIO), 360);

  serial_FRFL_AA.write("r axis1.encoder.pos_estimate\n");
  cur_raw_pos[5] = serial_FRFL_AA.readFloat();
  cur_joint_pos[5] = fmod(cur_raw_pos[5]*(360/GEAR_RATIO), 360);

  serial_HLHR_FE.write("r axis0.encoder.pos_estimate\n");
  cur_raw_pos[6] = serial_HLHR_FE.readFloat();
  cur_joint_pos[6] = fmod(cur_raw_pos[6]*(360/GEAR_RATIO), 360);

  serial_HLHR_FE.write("r axis1.encoder.pos_estimate\n");
  cur_raw_pos[7] = serial_HLHR_FE.readFloat();
  cur_joint_pos[7] = fmod(cur_raw_pos[7]*(360/GEAR_RATIO), 360);

  serial_FLFR_FE.write("r axis1.encoder.pos_estimate\n");
  cur_raw_pos[3] = serial_FLFR_FE.readFloat();
  cur_joint_pos[3] = fmod(cur_raw_pos[3]*(360/GEAR_RATIO), 360);

  serial_FRFL_AA.write("r axis0.encoder.pos_estimate\n");
  cur_raw_pos[4] = serial_FRFL_AA.readFloat();
  cur_joint_pos[4] = fmod(cur_raw_pos[4]*(360/GEAR_RATIO), 360);

  serial_FRFL_AA.write("r axis1.encoder.pos_estimate\n");
  cur_raw_pos[5] = serial_FRFL_AA.readFloat();
  cur_joint_pos[5] = fmod(cur_raw_pos[5]*(360/GEAR_RATIO), 360);

  serial_HLHR_FE.write("r axis0.encoder.pos_estimate\n");
  cur_raw_pos[6] = serial_HLHR_FE.readFloat();
  cur_joint_pos[6] = fmod(cur_raw_pos[6]*(360/GEAR_RATIO), 360);

  serial_HLHR_FE.write("r axis1.encoder.pos_estimate\n");
  cur_raw_pos[7] = serial_HLHR_FE.readFloat();
  cur_joint_pos[7] = fmod(cur_raw_pos[7]*(360/GEAR_RATIO), 360);

  // verify everything is 0-360 for consistency
  for (int i = 0; i < NUM_JOINTS; i++) {
    cur_joint_pos[i] = cur_joint_pos[i] < 0 ? cur_joint_pos[i] + 360 : cur_joint_pos[i];
  }
}

void setup()
{
  // Start Serial Monitor
  Serial.begin(115200); 

  // Start Serial Communication with Odrives
  serial_FLFR_UL.begin(115200);
  serial_FLFR_FE.begin(115200);
  serial_FRFL_AA.begin(115200);
  serial_HRHL_AA.begin(115200);
  serial_HLHR_FE.begin(115200);
  serial_HLHR_UL.begin(115200);
}

void loop()
{
  if (Serial.available())
  {
    char c = Serial.read();

    // Run calibration depending which leg module
    if (c == 'u')
    {
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Upper Leg: Requesting state " << requested_state << '\n';
      odrive0.run_state(0, requested_state, true);
      odrive0.run_state(1, requested_state, true); // are these all blocking?
      odrive5.run_state(0, requested_state, true);
      odrive5.run_state(1, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Upper Leg: Requesting state " << requested_state << '\n';
      odrive0.run_state(0, requested_state, true);
      odrive0.run_state(1, requested_state, true);
      odrive5.run_state(0, requested_state, true);
      odrive5.run_state(1, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Upper Leg: Requesting state " << requested_state << '\n';
      odrive0.run_state(0, requested_state, false);
      odrive0.run_state(1, requested_state, false); 
      odrive5.run_state(0, requested_state, false);
      odrive5.run_state(1, requested_state, false); // don't wait
    }

    if (c == 'f')
    {
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Hip FE: Requesting state " << requested_state << '\n';
      odrive1.run_state(0, requested_state, true);
      odrive1.run_state(1, requested_state, true);
      odrive4.run_state(0, requested_state, true);
      odrive4.run_state(1, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Hip FE: Requesting state " << requested_state << '\n';
      odrive1.run_state(0, requested_state, true);
      odrive1.run_state(1, requested_state, true);
      odrive4.run_state(0, requested_state, true);
      odrive4.run_state(1, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Hip FE: Requesting state " << requested_state << '\n';
      odrive1.run_state(0, requested_state, false);
      odrive1.run_state(1, requested_state, false); 
      odrive4.run_state(0, requested_state, false);
      odrive4.run_state(1, requested_state, false); // don't wait
    }

    if (c == 'a')
    {
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Hip AA: Requesting state " << requested_state << '\n';
      odrive2.run_state(0, requested_state, true);
      odrive2.run_state(1, requested_state, true);
      odrive3.run_state(0, requested_state, true);
      odrive3.run_state(1, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Hip AA: Requesting state " << requested_state << '\n';
      odrive2.run_state(0, requested_state, true);
      odrive2.run_state(1, requested_state, true);
      odrive3.run_state(0, requested_state, true);
      odrive3.run_state(1, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Hip AA: Requesting state " << requested_state << '\n';
      odrive2.run_state(0, requested_state, false);
      odrive2.run_state(1, requested_state, false); 
      odrive3.run_state(0, requested_state, false);
      odrive3.run_state(1, requested_state, false); // don't wait
    }

    if (c == 't')
    {
      float temp_pos_states[12];
      float temp_vel_states[12];

      Serial.print("start: "); Serial.prinln(millis());
      
      // Send command
      odrive0.SetPosition(0, 1.0f);
      odrive0.SetPosition(1, 1.0f);

      odrive1.SetPosition(0, 1.0f);
      odrive1.SetPosition(1, 1.0f);
      
      odrive2.SetPosition(0, 1.0f);
      odrive2.SetPosition(1, 1.0f);

      odrive3.SetPosition(0, 1.0f);
      odrive3.SetPosition(1, 1.0f);

      odrive4.SetPosition(0, 1.0f);
      odrive4.SetPosition(1, 1.0f);

      odrive5.SetPosition(0, 1.0f);
      odrive5.SetPosition(1, 1.0f);

      // Read Position States
      temp_pos_states[0] = odrive0.GetPosition(0);
      temp_pos_states[1] = odrive0.GetPosition(1);

      temp_pos_states[2] = odrive1.GetPosition(0);
      temp_pos_states[3] = odrive1.GetPosition(1);

      temp_pos_states[4] = odrive2.GetPosition(0);
      temp_pos_states[5] = odrive2.GetPosition(1);

      temp_pos_states[6] = odrive3.GetPosition(0);
      temp_pos_states[7] = odrive3.GetPosition(1);

      temp_pos_states[8] = odrive4.GetPosition(0);
      temp_pos_states[9] = odrive4.GetPosition(1);
      
      temp_pos_states[10] = odrive5.GetPosition(0);
      temp_pos_states[11] = odrive5.GetPosition(1);
      
      // Read Velocity States
      temp_vel_states[0] = odrive0.GetVelocity(0);
      temp_vel_states[1] = odrive0.GetVelocity(1);

      temp_vel_states[2] = odrive1.GetVelocity(0);
      temp_vel_states[3] = odrive1.GetVelocity(1);
    
      temp_vel_states[4] = odrive2.GetVelocity(0);
      temp_vel_states[5] = odrive2.GetVelocity(1);

      temp_vel_states[6] = odrive3.GetVelocity(0);
      temp_vel_states[7] = odrive3.GetVelocity(1);
      
      temp_vel_states[8] = odrive4.GetVelocity(0);
      temp_vel_states[9] = odrive4.GetVelocity(1);

      temp_vel_states[10] = odrive5.GetVelocity(0);
      temp_vel_states[11] = odrive5.GetVelocity(1);
      

      Serial.println("Position")
      for(int i = 0; i < 12; i++){
        Serial.print(temp_pos_states[i]); Serial.print(" ");
      }

      Serial.println("Velocity")
      for(int i = 0; i < 12; i++){
        Serial.print(temp_vel_states[i]); Serial.print(" ");
      }
      
      Serial.print("end: "); Serial.prinln(millis());
    }
  }
}
