#include "SPISlave_T4.h"
#include "leg_message.h"
#include "math_ops.h"

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

struct spi_data_t spi_data; // data from spine to up
spi_command_t spi_command; // data from up to spine
torque_t  torque_out;
current_t  current_out;

// spi buffers
uint16_t rx_buf[RX_LEN];
uint16_t tx_buf[TX_LEN];

leg_state l1_state, l2_state, l3_state, l4_state;
leg_control l1_control, l2_control, l3_control, l4_control;

bool spi_enabled = 0;

volatile int bytecount;
volatile bool dummy_bytes;

bool process_bytes = false;

SPISlave_T4 mySPI(0, SPI_16_BITS);    // Same as MiniCheetah firmware, MODE0, SPI_BITS=16
// spi->frequency(12000000); in MiniCheetah firmware
//  spi->reply(0x0);      // initially writes 0x0 to SPI peripheral
//  cs.fall(&spi_isr);    // sets spi ISR

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");

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

  // SPI doesn't work if enabled while the CS pin is pulled low
  // Wait for CS to not be low, then enable SPI
  // if(!spi_enabled) {   
  //   while((spi_enabled==0) && (cs.read() == 0)){ delay(10);}
  //   init_spi();
  //   spi_enabled = 1;
  // }
  mySPI.begin(MSBFIRST, SPI_MODE0);
  mySPI.onReceive(spi_isr);
  Serial.println("Setup completed");
}

void loop() {

  if (process_bytes) {
    // After reading, save rx_buf into spi_command
    rx_to_command();
    // debug_rx_buf();
    // debug_command();
    debug_command_single_leg(0);

    // // Read states from ODrives
    // feedback();

    // compute_torque_out();
    // debug_torqueout_single_leg(0);

    // compute_current_out();
    
    // // Send command to ODrives
    // send_current_control();
    process_bytes = false;
    bytecount = -1;
    ++_iteration;
  }
}


/*!
 * SPI interrupt service routine
 */
void spi_isr() {
  // Serial.print("Enter ISR: "); Serial.print(bytecount); Serial.println("");

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
      spi_command.kp_abad[i] *
          (spi_command.q_des_abad[i] - spi_data.q_abad[i]) +
      spi_command.kd_abad[i] *
          (spi_command.qd_des_abad[i] - spi_data.qd_abad[i]) +
      spi_command.tau_abad_ff[i];

    torque_out.tau_hip[i] =
        spi_command.kp_hip[i] *
            (spi_command.q_des_hip[i] - spi_data.q_hip[i]) +
        spi_command.kd_hip[i] *
            (spi_command.qd_des_hip[i] - spi_data.qd_hip[i]) +
        spi_command.tau_hip_ff[i];

    torque_out.tau_knee[i] =
        spi_command.kp_knee[i] *
            (spi_command.q_des_knee[i] - spi_data.q_knee[i]) +
        spi_command.kd_knee[i] *
            (spi_command.qd_des_knee[i] - spi_data.qd_knee[i]) +
        spi_command.tau_knee_ff[i];
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
 * send current_out to ODrives
 */
void send_current_control() {

}


/*!
 * read state from ODrives
 */
void feedback() {

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
  Serial.print(torque_out.tau_knee[leg], 9); Serial.print(", ");
}


void debug_rx_buf() {
  // Serial.print("rx_buf[0]: "); Serial.print(rx_buf[0], HEX); Serial.println("");
  // Serial.print("rx_buf[1]: "); Serial.print(rx_buf[1], HEX); Serial.println("");
  // Serial.print("rx_buf[2]: "); Serial.print(rx_buf[2], HEX); Serial.println("");
  // Serial.print("rx_buf[3]: "); Serial.print(rx_buf[3], HEX); Serial.println("");
  // Serial.print("rx_buf[4]: "); Serial.print(rx_buf[4], HEX); Serial.println("");
  // Serial.print("rx_buf[5]: "); Serial.print(rx_buf[5], HEX); Serial.println("");
  // Serial.print("rx_buf[6]: "); Serial.print(rx_buf[6], HEX); Serial.println("");
  // Serial.print("rx_buf[7]: "); Serial.print(rx_buf[7], HEX); Serial.println("");
  // Serial.print("rx_buf[8]: "); Serial.print(rx_buf[8], HEX); Serial.println("");
  // Serial.print("rx_buf[9]: "); Serial.print(rx_buf[9], HEX); Serial.println("");
  
  // Serial.print("rx_buf[10]: "); Serial.print(rx_buf[10], HEX); Serial.println("");
  // Serial.print("rx_buf[11]: "); Serial.print(rx_buf[11], HEX); Serial.println("");
  // Serial.print("rx_buf[12]: "); Serial.print(rx_buf[12], HEX); Serial.println("");
  // Serial.print("rx_buf[13]: "); Serial.print(rx_buf[13], HEX); Serial.println("");
  // Serial.print("rx_buf[14]: "); Serial.print(rx_buf[14], HEX); Serial.println("");
  // Serial.print("rx_buf[15]: "); Serial.print(rx_buf[15], HEX); Serial.println("");
  // Serial.print("rx_buf[16]: "); Serial.print(rx_buf[16], HEX); Serial.println("");
  // Serial.print("rx_buf[17]: "); Serial.print(rx_buf[17], HEX); Serial.println("");
  // Serial.print("rx_buf[18]: "); Serial.print(rx_buf[18], HEX); Serial.println("");
  // Serial.print("rx_buf[19]: "); Serial.print(rx_buf[19], HEX); Serial.println("");

  // Serial.print("rx_buf[20]: "); Serial.print(rx_buf[20], HEX); Serial.println("");
  // Serial.print("rx_buf[21]: "); Serial.print(rx_buf[21], HEX); Serial.println("");
  // Serial.print("rx_buf[22]: "); Serial.print(rx_buf[22], HEX); Serial.println("");
  // Serial.print("rx_buf[23]: "); Serial.print(rx_buf[23], HEX); Serial.println("");
  // Serial.print("rx_buf[24]: "); Serial.print(rx_buf[24], HEX); Serial.println("");
  // Serial.print("rx_buf[25]: "); Serial.print(rx_buf[25], HEX); Serial.println("");
  // Serial.print("rx_buf[26]: "); Serial.print(rx_buf[26], HEX); Serial.println("");
  // Serial.print("rx_buf[27]: "); Serial.print(rx_buf[27], HEX); Serial.println("");
  // Serial.print("rx_buf[28]: "); Serial.print(rx_buf[28], HEX); Serial.println("");
  // Serial.print("rx_buf[29]: "); Serial.print(rx_buf[29], HEX); Serial.println("");
  
  // Serial.print("rx_buf[30]: "); Serial.print(rx_buf[30], HEX); Serial.println("");
  // Serial.print("rx_buf[31]: "); Serial.print(rx_buf[31], HEX); Serial.println("");
  // Serial.print("rx_buf[32]: "); Serial.print(rx_buf[32], HEX); Serial.println("");
  // Serial.print("rx_buf[33]: "); Serial.print(rx_buf[33], HEX); Serial.println("");
  // Serial.print("rx_buf[34]: "); Serial.print(rx_buf[34], HEX); Serial.println("");
  // Serial.print("rx_buf[35]: "); Serial.print(rx_buf[35], HEX); Serial.println("");
  // Serial.print("rx_buf[36]: "); Serial.print(rx_buf[36], HEX); Serial.println("");
  // Serial.print("rx_buf[37]: "); Serial.print(rx_buf[37], HEX); Serial.println("");
  // Serial.print("rx_buf[38]: "); Serial.print(rx_buf[38], HEX); Serial.println("");
  // Serial.print("rx_buf[39]: "); Serial.print(rx_buf[39], HEX); Serial.println("");

  // Serial.print("rx_buf[40]: "); Serial.print(rx_buf[40], HEX); Serial.println("");
  // Serial.print("rx_buf[41]: "); Serial.print(rx_buf[41], HEX); Serial.println("");
  // Serial.print("rx_buf[42]: "); Serial.print(rx_buf[42], HEX); Serial.println("");
  // Serial.print("rx_buf[43]: "); Serial.print(rx_buf[43], HEX); Serial.println("");
  // Serial.print("rx_buf[44]: "); Serial.print(rx_buf[44], HEX); Serial.println("");
  // Serial.print("rx_buf[45]: "); Serial.print(rx_buf[45], HEX); Serial.println("");
  // Serial.print("rx_buf[46]: "); Serial.print(rx_buf[46], HEX); Serial.println("");
  // Serial.print("rx_buf[47]: "); Serial.print(rx_buf[47], HEX); Serial.println("");
  // Serial.print("rx_buf[48]: "); Serial.print(rx_buf[48], HEX); Serial.println("");
  // Serial.print("rx_buf[49]: "); Serial.print(rx_buf[49], HEX); Serial.println("");

  // Serial.print("rx_buf[50]: "); Serial.print(rx_buf[50], HEX); Serial.println("");
  // Serial.print("rx_buf[51]: "); Serial.print(rx_buf[51], HEX); Serial.println("");
  // Serial.print("rx_buf[52]: "); Serial.print(rx_buf[52], HEX); Serial.println("");
  // Serial.print("rx_buf[53]: "); Serial.print(rx_buf[53], HEX); Serial.println("");
  // Serial.print("rx_buf[54]: "); Serial.print(rx_buf[54], HEX); Serial.println("");
  // Serial.print("rx_buf[55]: "); Serial.print(rx_buf[55], HEX); Serial.println("");
  // Serial.print("rx_buf[56]: "); Serial.print(rx_buf[56], HEX); Serial.println("");
  // Serial.print("rx_buf[57]: "); Serial.print(rx_buf[57], HEX); Serial.println("");
  // Serial.print("rx_buf[58]: "); Serial.print(rx_buf[58], HEX); Serial.println("");
  // Serial.print("rx_buf[59]: "); Serial.print(rx_buf[59], HEX); Serial.println("");
  
  Serial.print("rx_buf[60]: "); Serial.print(rx_buf[60], HEX); Serial.println("");
  Serial.print("rx_buf[61]: "); Serial.print(rx_buf[61], HEX); Serial.println("");
  Serial.print("rx_buf[62]: "); Serial.print(rx_buf[62], HEX); Serial.println("");
  Serial.print("rx_buf[63]: "); Serial.print(rx_buf[63], HEX); Serial.println("");
  Serial.print("rx_buf[64]: "); Serial.print(rx_buf[64], HEX); Serial.println("");
  Serial.print("rx_buf[65]: "); Serial.print(rx_buf[65], HEX); Serial.println("");
  Serial.print("rx_buf[66]: "); Serial.print(rx_buf[66], HEX); Serial.println("");
  Serial.print("rx_buf[67]: "); Serial.print(rx_buf[67], HEX); Serial.println("");
  Serial.print("rx_buf[68]: "); Serial.print(rx_buf[68], HEX); Serial.println("");
  Serial.print("rx_buf[69]: "); Serial.print(rx_buf[69], HEX); Serial.println("");

  Serial.print("rx_buf[124]: "); Serial.print(rx_buf[124], HEX); Serial.println("");
  Serial.print("rx_buf[125]: "); Serial.print(rx_buf[125], HEX); Serial.println("");
  Serial.print("rx_buf[126]: "); Serial.print(rx_buf[126], HEX); Serial.println("");
  Serial.print("rx_buf[127]: "); Serial.print(rx_buf[127], HEX); Serial.println("");
  Serial.print("rx_buf[128]: "); Serial.print(rx_buf[128], HEX); Serial.println("");
  Serial.print("rx_buf[129]: "); Serial.print(rx_buf[129], HEX); Serial.println("");
}
