#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include "SPISlave_T4.h"
#include "leg_message.h"
#include "math_ops.h"
#include <sstream>
#include <vector>

int jj= 0;
int interruptPin = 10;
int interruptPin_jumped = 9;

// length of receive/transmit buffers
#define RX_LEN 130
#define TX_LEN 130

// length of outgoing/incoming messages
#define DATA_LEN 58
#define CMD_LEN  130

// TODO: Change values for BiQu
/// Value Limits ///
 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -65.0f
 #define V_MAX 65.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f
 
//  /// Joint Soft Stops ///
//  #define A_LIM_P 1.5f
//  #define A_LIM_N -1.5f
//  #define H_LIM_P 5.0f
//  #define H_LIM_N -5.0f
//  #define K_LIM_P 0.2f
//  #define K_LIM_N 7.7f
//  #define KP_SOFTSTOP 100.0f
//  #define KD_SOFTSTOP 0.4f;

/// MOTOR CONSTANTS ///
#define MOTOR_KT 0.018f;     // N.m/A

uint32_t _iteration;

///*** ODRIVE SETUP AND VARIABLES ***///
// Serial1, 2, 3, 4, 5, and 6 are built-in variables defined by the Teensy board definition
// these lines simply rename the serial objects to represent the odrive they're tied to
/**
 * Odrv0 - FL upper leg & FR upper leg (2mm connectors)
 * Odrv1 - FL Hip FE & FL Hip AA
 * Odrv2 - FR Hip FE & FR Hip AA
 * Odrv3 - HR Hip AA & HL Hip AA
 * Odrv4 - HL Hip FE && HR Hip FE 
 * Odrv5 - HL upper leg & HR upper leg
 **/
HardwareSerial& serial_FLFR_UL = Serial5;
HardwareSerial& serial_FLFE_AA = Serial4;
HardwareSerial& serial_FRFE_AA = Serial3;
HardwareSerial& serial_HRHL_AA = Serial6;
HardwareSerial& serial_HLHR_FE = Serial2;
HardwareSerial& serial_HLHR_UL = Serial1;

// OdriveArduino objects 
ODriveArduino odrive0(serial_FLFR_UL);      // axis0: FLUL ; axis1: FRUL
ODriveArduino odrive1(serial_FLFE_AA);      // axis0: FLFE ; axis1: FLAA
ODriveArduino odrive2(serial_HLHR_FE);      // FR Hip FE and FR Hip AA
ODriveArduino odrive3(serial_HRHL_AA);      // HR and HL Hip AA
ODriveArduino odrive4(serial_HLHR_UL);      // HL and HR Hip FE 
ODriveArduino odrive5(serial_FRFE_AA);      // axis0: FRFE ; axis1: FRAA

String feedback_str_odrive0;
String feedback_str_odrive1;
String feedback_str_odrive2;
String feedback_str_odrive3;
String feedback_str_odrive4;
String feedback_str_odrive5;
String feedback_strs[6];


///*** SPI SETUP AND VARIABLES ***///
struct spi_data_t spi_data; // data from spine to up
spi_command_t spi_command; // data from up to spine
torque_t  torque_out;
current_t  current_out;

// spi buffers
uint16_t rx_buf[RX_LEN];
uint16_t tx_buf[TX_LEN];

bool spi_enabled = 0;

volatile int bytecount;
volatile bool dummy_bytes;

bool process_bytes = false;

SPISlave_T4 mySPI(0, SPI_16_BITS);    // Same as MiniCheetah firmware, MODE0, SPI_BITS=16


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");

  /// ODRIVES SETUP ///
  // Start Serial Communication with Odrives
  serial_FLFR_UL.begin(921600);
  // serial_FLFE_AA.begin(921600);
  serial_FRFE_AA.begin(921600);
  // serial_HRHL_AA.begin(921600);
  // serial_HLHR_FE.begin(921600);
  // serial_HLHR_UL.begin(921600);

  init_odrive_single_leg();

  /// SPI SETUP FOR RASPBERRY PI COMMUNICATION ///
  memset(&tx_buf, 0, TX_LEN * sizeof(uint16_t));
  memset(&spi_data, 0, sizeof(spi_data_t));
  memset(&spi_command,0,sizeof(spi_command_t));
  memset(&torque_out,0,sizeof(torque_t));
  memset(&current_out,0,sizeof(current_t));

  spi_data = {{1.1, 1.2, 1.3, 1.4}, {2.1, 2.2, 2.3, 2.4}, {3.1, 3.2, 3.3, 3.4}, 
              {4.1, 4.2, 4.3, 4.4}, {5.1, 5.2, 5.3, 5.4}, {6.1, 6.2, 6.3, 6.4}, 
              {16, 17, 18, 19}, 20};

  for(int i = 0; i < DATA_LEN; i++)
    tx_buf[i] = ((uint16_t*)(&spi_data))[i];

  bytecount = -1;
  dummy_bytes = true;

  mySPI.begin(MSBFIRST, SPI_MODE0);
  mySPI.onReceive(spi_isr);
  Serial.println("Setup completed");
}

void loop() {

  if (process_bytes) {
    float start = micros();
    // After reading, save rx_buf into spi_command
    rx_to_command();
    // debug_rx_buf();
    // debug_command();
    debug_command_single_leg(0);

    // // Read states from ODrives
    // feedback();
    feedback_single_leg();
    debug_spi_data_single_leg();

    compute_torque_out();
    debug_torqueout_single_leg(0);

    // compute_current_out();
    
    // // Send command to ODrives
    send_torque_control();
    process_bytes = false;
    bytecount = -1;
    ++_iteration;

    float end = micros();

    float timeSpent = end - start;
    
    Serial.print(_iteration); Serial.print(" time spend (ms): "); Serial.print(timeSpent); Serial.println("");
  }

  //feedback_single_leg();
  //debug_spi_data_single_leg();
  //delay(1000);
}


/*!
 * SPI interrupt service routine
 */
void spi_isr() {
  // Shift out dummy bytes
  if (bytecount == -1) {         // Dummy bytes
    mySPI.pushr(tx_buf[0]);
    mySPI.popr();
    bytecount++;
  }

  while (mySPI.available()) {
    rx_buf[bytecount] = mySPI.popr();
          
    bytecount++;
    if(bytecount<TX_LEN) {
      mySPI.pushr(tx_buf[bytecount]);
    }
  }

  if (bytecount >= TX_LEN) {
    process_bytes = true;
  }
}


/*!
 * Convert rx_buf to spi_command
   TODO: Add limit_command()
 */
void rx_to_command() {
  // check checksum first!
  uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buf, 64);   // 64 = (15 control values * 4 legs) + 4 flag entries
  for(int i = 0; i < CMD_LEN; i++)
  {
      ((uint16_t*)(&spi_command))[i] = rx_buf[i];
  }

  // run control, which fills in tx_buf for the next iteration
  if (calc_checksum != spi_command.checksum) {
    spi_data.flags[1] = 0xdead; 
  }

  // limit_command();
}


/*!
 * Limit joints commands to be within bounds
 */
void limit_command() {
  for (int i = 0; i < 4; i++) {
    spi_command.q_des_abad[i] = fminf(fmaxf(P_MIN, spi_command.q_des_abad[i]), P_MAX); 
    spi_command.q_des_hip[i] = fminf(fmaxf(P_MIN, spi_command.q_des_hip[i]), P_MAX); 
    spi_command.q_des_knee[i] = fminf(fmaxf(P_MIN, spi_command.q_des_knee[i]), P_MAX);

    spi_command.qd_des_abad[i] = fminf(fmaxf(V_MIN, spi_command.qd_des_abad[i]), V_MAX);
    spi_command.qd_des_hip[i] = fminf(fmaxf(V_MIN, spi_command.qd_des_hip[i]), V_MAX);
    spi_command.qd_des_knee[i] = fminf(fmaxf(V_MIN, spi_command.qd_des_knee[i]), V_MAX);

    spi_command.kp_abad[i] = fminf(fmaxf(KP_MIN, spi_command.kp_abad[i]), KP_MAX);
    spi_command.kp_hip[i] = fminf(fmaxf(KP_MIN, spi_command.kp_hip[i]), KP_MAX);
    spi_command.kp_knee[i] = fminf(fmaxf(KP_MIN, spi_command.kp_knee[i]), KP_MAX);

    spi_command.kd_abad[i] = fminf(fmaxf(KD_MIN, spi_command.kd_abad[i]), KD_MAX);
    spi_command.kd_hip[i] = fminf(fmaxf(KD_MIN, spi_command.kd_hip[i]), KD_MAX);
    spi_command.kd_knee[i] = fminf(fmaxf(KD_MIN, spi_command.kd_knee[i]), KD_MAX);

    spi_command.tau_abad_ff[i] = fminf(fmaxf(T_MIN, spi_command.tau_abad_ff[i]), T_MAX);
    spi_command.tau_hip_ff[i] = fminf(fmaxf(T_MIN, spi_command.tau_hip_ff[i]), T_MAX);
    spi_command.tau_knee_ff[i] = fminf(fmaxf(T_MIN, spi_command.tau_knee_ff[i]), T_MAX);
  }
}


/*!
 * compute torque_out using joint impedance control
 */
void  compute_torque_out() {
  memset(&torque_out,0,sizeof(torque_t));     // zero torque command

  for (int i = 0; i < 4; i++) {
    torque_out.tau_abad[i] =
      (spi_command.kp_abad[i] *
          (spi_command.q_des_abad[i] - spi_data.q_abad[i]) +
      spi_command.kd_abad[i] *
          (spi_command.qd_des_abad[i] - spi_data.qd_abad[i]) +
      spi_command.tau_abad_ff[i]);

    torque_out.tau_hip[i] =
        (spi_command.kp_hip[i] *
            (spi_command.q_des_hip[i] - spi_data.q_hip[i]) +
        spi_command.kd_hip[i] *
            (spi_command.qd_des_hip[i] - spi_data.qd_hip[i]) +
        spi_command.tau_hip_ff[i]);

    torque_out.tau_knee[i] =
        (spi_command.kp_knee[i] *
            (spi_command.q_des_knee[i] - spi_data.q_knee[i]) +
        spi_command.kd_knee[i] *
            (spi_command.qd_des_knee[i] - spi_data.qd_knee[i]) +
        spi_command.tau_knee_ff[i]);
  }
}


/*!
 * compute current_out from torque_out
 */
void  compute_current_out() {
  memset(&current_out,0,sizeof(torque_t));      // zero current command

  for (int i = 0; i < 4; i++) {
    current_out.tau_abad[i] = torque_out.tau_abad[i] / MOTOR_KT;

    current_out.tau_hip[i] = torque_out.tau_hip[i] / MOTOR_KT;

    current_out.tau_knee[i] = torque_out.tau_knee[i] / MOTOR_KT;
  }
}


/*!
 * checksum method
 */
uint32_t xor_checksum(uint32_t* data, size_t len)
{
    uint32_t t = 0;
    for(int i = 0; i < len; i++)   
        t = t ^ data[i];
    return t;

}


/// *** ODRIVE FUNCTIONS ***///

/*!
 * init all odrives
 */
void init_odrives() {
  calibrate_all_odrives();
  control_mode_all_odrives();
}


/*!
 * init single odrive, 1 leg
 */
void init_odrive_single_leg() {
  calibrate_single_odrive();
  control_mode_single_odrive();
}


/*!
 * calibrate all odrives
 */
void calibrate_all_odrives() {

}


/*!
 * calibrate single odrive, to test single leg
 */
void calibrate_single_odrive() {
  int requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  Serial.print("Leg 1: Requesting state "); Serial.print(requested_state); Serial.println("");
  odrive0.run_state(1, requested_state, true);
  Serial.println("0-1 done");
  odrive5.run_state(0, requested_state, true);
  Serial.println("5-0 done");
  odrive5.run_state(1, requested_state, true);
  Serial.println("5-1 done");
}

/*!
 * enter control mode for all odrives
 */
void control_mode_all_odrives() {

}


/*!
 * enter control mode for single odrive, to test single leg
 */
void control_mode_single_odrive() {
  int requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial.print("Leg FR: Requesting state "); Serial.print(requested_state); Serial.println("");

  odrive0.run_state(1, requested_state, false);     // FR UL
  odrive5.run_state(0, requested_state, false);     // FR FE
  odrive5.run_state(1, requested_state, false);     // FR AA
}


/*!
 * send torque_out to ODrives
 */
void send_torque_control() {
  serial_FLFR_UL.print("c 1 ");
  serial_FLFR_UL.print(torque_out.tau_knee[0]); serial_FLFR_UL.println();

  serial_FRFE_AA.print("t "); serial_FRFE_AA.print(torque_out.tau_hip[0]); serial_FRFE_AA.print(" ");
  serial_FRFE_AA.print(torque_out.tau_abad[0]); serial_FRFE_AA.println();
}


/*!
 * read state from ODrives
 */
void feedback() {
  
}

/*!
 * read state for ODrives corresponding to FR leg
 */
void feedback_single_leg() {
  serial_FLFR_UL.print("f\n");
  serial_FRFE_AA.print("f\n");
  
  feedback_strs[0] = "";
  feedback_strs[5] = "";

  bool odr0_flag = true;
  bool odr5_flag = true;

  while (odr0_flag || odr5_flag)  {
    if (serial_FLFR_UL.available() && odr0_flag) {
      char c = serial_FLFR_UL.read();
      if (c == '\n')
          odr0_flag = false;
      feedback_strs[0] += c;
    }

    if (serial_FRFE_AA.available() && odr5_flag) {
      char c = serial_FRFE_AA.read();
      if (c == '\n')
          odr5_flag = false;
      feedback_strs[5] += c;
    }
  }
  
  feedback_str_to_data_leg();
}


void feedback_str_to_data() {
  // 6 strings in feedback_strs (6 ODrives)
//   for (int i = 0; i < 6; i++) {
//     std::istringstream iss(feedback_strs[i]);
//     std::vector<float> arr;

//     float value;
//     while (iss >> value) {
//       arr.push_back(value);
//     }

//     // Save arr contents into spi_data
//   }
}


/*!
 * convert ODrives data for FR leg to spi_data, 
 * all other data is set to 0
 */
void feedback_str_to_data_leg() {
  memset(&spi_data, 0, sizeof(spi_data_t));
  
  // ODrive0 string to spi_data
  std::istringstream iss(feedback_strs[0].c_str());
  std::vector<float> arr;

  float value;
  while (iss >> value) {
    arr.push_back(value);
  }

  // Save arr contents into spi_data
  spi_data.q_knee[0] = arr[2];
  spi_data.qd_knee[0] = arr[3];

  // ODrive5 string to spi_data
  std::istringstream iss2(feedback_strs[5].c_str());
  std::vector<float> arr2;

  float value2;
  while (iss2 >> value2) {
    arr2.push_back(value2);
  }

  // Save arr contents into spi_data
  spi_data.q_hip[0] = arr2[0];
  spi_data.qd_hip[0] = arr2[1];
  spi_data.q_abad[0] = arr2[2];
  spi_data.qd_abad[0] = arr2[3];
}


void debug_command() {
  Serial.print("spi_command.q_des_abad[0]: "); Serial.print(spi_command.q_des_abad[0]); Serial.println("");
  Serial.print("spi_command.q_des_abad[1]: "); Serial.print(spi_command.q_des_abad[1]); Serial.println("");
  Serial.print("spi_command.q_des_abad[2]: "); Serial.print(spi_command.q_des_abad[2]); Serial.println("");
  Serial.print("spi_command.q_des_abad[3]: "); Serial.print(spi_command.q_des_abad[3]); Serial.println("");

  Serial.print("spi_command.q_des_hip[0]: "); Serial.print(spi_command.q_des_hip[0]); Serial.println("");
  Serial.print("spi_command.q_des_hip[1]: "); Serial.print(spi_command.q_des_hip[1]); Serial.println("");
  Serial.print("spi_command.q_des_hip[2]: "); Serial.print(spi_command.q_des_hip[2]); Serial.println("");
  Serial.print("spi_command.q_des_hip[3]: "); Serial.print(spi_command.q_des_hip[3]); Serial.println("");

  Serial.print("spi_command.q_des_knee[0]: "); Serial.print(spi_command.q_des_knee[0]); Serial.println("");
  Serial.print("spi_command.q_des_knee[1]: "); Serial.print(spi_command.q_des_knee[1]); Serial.println("");
  Serial.print("spi_command.q_des_knee[2]: "); Serial.print(spi_command.q_des_knee[2]); Serial.println("");
  Serial.print("spi_command.q_des_knee[3]: "); Serial.print(spi_command.q_des_knee[3]); Serial.println("");

  Serial.print("spi_command.qd_des_abad[0]: "); Serial.print(spi_command.qd_des_abad[0]); Serial.println("");
  Serial.print("spi_command.qd_des_abad[1]: "); Serial.print(spi_command.qd_des_abad[1]); Serial.println("");
  Serial.print("spi_command.qd_des_abad[2]: "); Serial.print(spi_command.qd_des_abad[2]); Serial.println("");
  Serial.print("spi_command.qd_des_abad[3]: "); Serial.print(spi_command.qd_des_abad[3]); Serial.println("");

  Serial.print("spi_command.qd_des_hip[0]: "); Serial.print(spi_command.qd_des_hip[0]); Serial.println("");
  Serial.print("spi_command.qd_des_hip[1]: "); Serial.print(spi_command.qd_des_hip[1]); Serial.println("");
  Serial.print("spi_command.qd_des_hip[2]: "); Serial.print(spi_command.qd_des_hip[2]); Serial.println("");
  Serial.print("spi_command.qd_des_hip[3]: "); Serial.print(spi_command.qd_des_hip[3]); Serial.println("");

  Serial.print("spi_command.qd_des_knee[0]: "); Serial.print(spi_command.qd_des_knee[0]); Serial.println("");
  Serial.print("spi_command.qd_des_knee[1]: "); Serial.print(spi_command.qd_des_knee[1]); Serial.println("");
  Serial.print("spi_command.qd_des_knee[2]: "); Serial.print(spi_command.qd_des_knee[2]); Serial.println("");
  Serial.print("spi_command.qd_des_knee[3]: "); Serial.print(spi_command.qd_des_knee[3]); Serial.println("");

  Serial.print("spi_command.kp_abad[0]: "); Serial.print(spi_command.kp_abad[0]); Serial.println("");
  Serial.print("spi_command.kp_abad[1]: "); Serial.print(spi_command.kp_abad[1]); Serial.println("");
  Serial.print("spi_command.kp_abad[2]: "); Serial.print(spi_command.kp_abad[2]); Serial.println("");
  Serial.print("spi_command.kp_abad[3]: "); Serial.print(spi_command.kp_abad[3]); Serial.println("");

  Serial.print("spi_command.kp_hip[0]: "); Serial.print(spi_command.kp_hip[0]); Serial.println("");
  Serial.print("spi_command.kp_hip[1]: "); Serial.print(spi_command.kp_hip[1]); Serial.println("");
  Serial.print("spi_command.kp_hip[2]: "); Serial.print(spi_command.kp_hip[2]); Serial.println("");
  Serial.print("spi_command.kp_hip[3]: "); Serial.print(spi_command.kp_hip[3]); Serial.println("");

  Serial.print("spi_command.kp_knee[0]: "); Serial.print(spi_command.kp_knee[0]); Serial.println("");
  Serial.print("spi_command.kp_knee[1]: "); Serial.print(spi_command.kp_knee[1]); Serial.println("");
  Serial.print("spi_command.kp_knee[2]: "); Serial.print(spi_command.kp_knee[2]); Serial.println("");
  Serial.print("spi_command.kp_knee[3]: "); Serial.print(spi_command.kp_knee[3]); Serial.println("");

  Serial.print("spi_command.kd_abad[0]: "); Serial.print(spi_command.kd_abad[0]); Serial.println("");
  Serial.print("spi_command.kd_abad[1]: "); Serial.print(spi_command.kd_abad[1]); Serial.println("");
  Serial.print("spi_command.kd_abad[2]: "); Serial.print(spi_command.kd_abad[2]); Serial.println("");
  Serial.print("spi_command.kd_abad[3]: "); Serial.print(spi_command.kd_abad[3]); Serial.println("");

  Serial.print("spi_command.kd_hip[0]: "); Serial.print(spi_command.kd_hip[0]); Serial.println("");
  Serial.print("spi_command.kd_hip[1]: "); Serial.print(spi_command.kd_hip[1]); Serial.println("");
  Serial.print("spi_command.kd_hip[2]: "); Serial.print(spi_command.kd_hip[2]); Serial.println("");
  Serial.print("spi_command.kd_hip[3]: "); Serial.print(spi_command.kd_hip[3]); Serial.println("");

  Serial.print("spi_command.kd_knee[0]: "); Serial.print(spi_command.kd_knee[0]); Serial.println("");
  Serial.print("spi_command.kd_knee[1]: "); Serial.print(spi_command.kd_knee[1]); Serial.println("");
  Serial.print("spi_command.kd_knee[2]: "); Serial.print(spi_command.kd_knee[2]); Serial.println("");
  Serial.print("spi_command.kd_knee[3]: "); Serial.print(spi_command.kd_knee[3]); Serial.println("");

  Serial.print("spi_command.flags[0]: "); Serial.print(spi_command.flags[0]); Serial.println("");
  Serial.print("spi_command.flags[1]: "); Serial.print(spi_command.flags[1]); Serial.println("");
  Serial.print("spi_command.flags[2]: "); Serial.print(spi_command.flags[2]); Serial.println("");
  Serial.print("spi_command.flags[3]: "); Serial.print(spi_command.flags[3]); Serial.println("");

  Serial.print("spi_command.checksum: "); Serial.print(spi_command.checksum); Serial.println("");
}

void debug_command_single_leg(int leg) {
  Serial.print(_iteration); Serial.print(", ");
  Serial.print(spi_command.q_des_abad[leg], 9); Serial.print(", ");
  Serial.print(spi_command.q_des_hip[leg], 9); Serial.print(", ");
  Serial.print(spi_command.q_des_knee[leg], 9); Serial.print(", ");

  Serial.print(spi_command.qd_des_abad[leg], 9); Serial.print(", ");
  Serial.print(spi_command.qd_des_hip[leg], 9); Serial.print(", ");
  Serial.print(spi_command.qd_des_knee[leg], 9); Serial.print(", ");

  Serial.print(spi_command.kp_abad[leg], 3); Serial.print(", ");
  Serial.print(spi_command.kp_hip[leg], 3); Serial.print(", ");
  Serial.print(spi_command.kp_knee[leg], 3); Serial.print(", ");

  Serial.print(spi_command.kd_abad[leg], 3); Serial.print(", ");
  Serial.print(spi_command.kd_hip[leg], 3); Serial.print(", ");
  Serial.print(spi_command.kd_knee[leg], 3); Serial.print(", ");

  Serial.print(spi_command.tau_abad_ff[leg], 9); Serial.print(", ");
  Serial.print(spi_command.tau_hip_ff[leg], 9); Serial.print(", ");
  Serial.print(spi_command.tau_knee_ff[leg], 9); Serial.print(", ");

  Serial.print(spi_command.flags[leg], 2); Serial.println("");
}

void debug_torqueout_single_leg(int leg) {
  Serial.print(_iteration); Serial.print(", ");
  Serial.print(torque_out.tau_abad[leg], 9); Serial.print(", ");
  Serial.print(torque_out.tau_hip[leg], 9); Serial.print(", ");
  Serial.print(torque_out.tau_knee[leg], 9);
  Serial.println();
}

void debug_spi_data_single_leg() {
  Serial.print(spi_data.q_abad[0], 4); Serial.print(", ");
  Serial.print(spi_data.q_hip[0], 4); Serial.print(", ");
  Serial.print(spi_data.q_knee[0], 4); Serial.print(", ");

  Serial.print(spi_data.qd_abad[0], 4); Serial.print(", ");
  Serial.print(spi_data.qd_hip[0], 4); Serial.print(", ");
  Serial.print(spi_data.qd_knee[0], 4);
  Serial.println();
}
