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

struct spi_data_t spi_data; // data from spine to up
spi_command_t spi_command; // data from up to spine
torque_t  torque_out;
current_t  current_out;

// spi buffers
uint16_t rx_buff[RX_LEN];
uint16_t tx_buff[TX_LEN];

leg_state l1_state, l2_state, l3_state, l4_state;
leg_control l1_control, l2_control, l3_control, l4_control;

bool spi_enabled = 0;

volatile int bytecount;

bool process_bytes = false;

SPISlave_T4 mySPI(0, SPI_16_BITS);    // Same as MiniCheetah firmware, MODE0, SPI_BITS=16
// spi->frequency(12000000); in MiniCheetah firmware
//  spi->reply(0x0);      // initially writes 0x0 to SPI peripheral
//  cs.fall(&spi_isr);    // sets spi ISR

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");

  memset(&tx_buff, 0, TX_LEN * sizeof(uint16_t));
  memset(&spi_data, 0, sizeof(spi_data_t));
  memset(&spi_command,0,sizeof(spi_command_t));
  memset(&torque_out,0,sizeof(torque_t));
  memset(&current_out,0,sizeof(current_t));

  spi_data = {{1.1, 1.2, 1.3, 1.4}, {2.1, 2.2, 2.3, 2.4}, {3.1, 3.2, 3.3, 3.4}, 
              {4.1, 4.2, 4.3, 4.4}, {5.1, 5.2, 5.3, 5.4}, {6.1, 6.2, 6.3, 6.4}, 
              {12, 13, 14, 15}, 20};

  for(int i = 0; i < DATA_LEN; i++)
    tx_buff[i] = ((uint16_t*)(&spi_data))[i];

  bytecount = -1;

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

    // // Read states from ODrives
    // feedback();

    // // command_to_leg_control();
    // compute_torque_out();
    // compute_current_out();
    
    // // Send command to ODrives
    // send_current_control();
    process_bytes = false;
    bytecount = -1;
  }
}


/*!
 * SPI interrupt service routine
 */
void spi_isr() {
  Serial.print("Enter ISR: "); Serial.print(bytecount); Serial.println("");

  // Shift out dummy bytes
  if (bytecount == -1) {
    mySPI.popr();             // Dummy bytes
    mySPI.pushr(tx_buff[0]);
  }

  while (mySPI.available()) {
    rx_buff[bytecount] = mySPI.popr();
          
    bytecount++;
    if(bytecount<TX_LEN) {
      mySPI.pushr(tx_buff[bytecount]);
    }
  }

  if (bytecount > TX_LEN) {
    process_bytes = true;
  }
}


/*!
 * Convert rx_buf to spi_command
   TODO: Add limit_command()
 */
void rx_to_command() {
  // check checksum first!
  uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff, 64);   // 64 = (15 control values * 4 legs) + 4 flag entries
  for(int i = 0; i < CMD_LEN; i++)
  {
      ((uint16_t*)(&spi_command))[i] = rx_buff[i];
  }

  // run control, which fills in tx_buff for the next iteration
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