#include "SPISlave_T4.h"
#include "leg_message.h"
#include "math_ops.h"

int jj= 0;
int interruptPin = 10;
int interruptPin_jumped = 9;

// length of receive/transmit buffers
#define RX_LEN 130
// #define TX_LEN 130
#define TX_LEN 130

// length of outgoing/incoming messages
#define DATA_LEN 58
#define CMD_LEN  130

// /// Value Limits ///
//  #define P_MIN -12.5f
//  #define P_MAX 12.5f
//  #define V_MIN -65.0f
//  #define V_MAX 65.0f
//  #define KP_MIN 0.0f
//  #define KP_MAX 500.0f
//  #define KD_MIN 0.0f
//  #define KD_MAX 5.0f
//  #define T_MIN -18.0f
//  #define T_MAX 18.0f
 
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

volatile int bytecount = 0;

bool process_bytes = false;

SPISlave_T4 mySPI(0, SPI_16_BITS);    // Same as MiniCheetah firmware, MODE0, SPI_BITS=16
// spi->frequency(12000000); in MiniCheetah firmware
//  spi->reply(0x0);      // initially writes 0x0 to SPI peripheral
//  cs.fall(&spi_isr);    // sets spi ISR

uint32_t xor_checksum(uint32_t* data, size_t len)
{
    uint32_t t = 0;
    for(int i = 0; i < len; i++)   
        t = t ^ data[i];
    return t;

}

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
              {4.1, 4.2, 4.3, 4.4}, {5.1, 5.2, 5.3, 5.4}, {6.1, 6.2, 6.3, 6.4}, 10};

  for(int i = 0; i < DATA_LEN; i++)
    tx_buff[i] = ((uint16_t*)(&spi_data))[i];

  // pinMode(interruptPin, INPUT_PULLUP);
  pinMode(interruptPin_jumped, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(interruptPin), spi_isr, FALLING);

  // SPI doesn't work if enabled while the CS pin is pulled low
  // Wait for CS to not be low, then enable SPI
  // if(!spi_enabled) {   
  //   while((spi_enabled==0) && (cs.read() == 0)){ delay(10);}
  //   init_spi();
  //   spi_enabled = 1;
  // }
  mySPI.begin(MSBFIRST, SPI_MODE0);
  mySPI.onReceive(spi_isr2);
  // mySPI.onReceive(spi_isr);
  Serial.println("Setup completed");
}

void loop() {
  // Serial.print("read : "); Serial.print(spi_command.q_des_abad[0]); Serial.println("");
  // Serial.print(tx_buff[0], HEX); Serial.print(" "); Serial.println(tx_buff[1], HEX);
  // Serial.println(digitalReadFast(interruptPin_jumped));
  // delay(1000);
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
    bytecount = 0;
  }
}

void spi_isr2() {
  Serial.print("Enter ISR: "); Serial.print(bytecount); Serial.println("");
  if (bytecount == 0) {
    mySPI.pushr(tx_buff[0]);
  }

  while (mySPI.available()) {
    rx_buff[bytecount] = mySPI.popr();
          
    bytecount++;
    if(bytecount<TX_LEN) {
      
      // SPI1->DR = tx_buff[bytecount];
      mySPI.pushr(tx_buff[bytecount]);
    }
  }

  if (bytecount > TX_LEN) {
    process_bytes = true;
  }
}

/*!
 * Convert rx_buf to spi_command
 */
void rx_to_command() {
  // should probably check checksum first!
  // uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff,32);
  for(int i = 0; i < CMD_LEN; i++)
  {
      ((uint16_t*)(&spi_command))[i] = rx_buff[i];
  }
}


// /*!
//  * Convert spi_command to leg_control
//  */
// void command_to_leg_control() {
//   memset(&l1_control, 0, sizeof(l1_control));
//   memset(&l2_control, 0, sizeof(l2_control));
//   memset(&l3_control, 0, sizeof(l3_control));
//   memset(&l4_control, 0, sizeof(l4_control));
  
//   l1_control.a.p_des = spi_command.q_des_abad[0];
//   l1_control.a.v_des  = spi_command.qd_des_abad[0];
//   l1_control.a.kp = spi_command.kp_abad[0];
//   l1_control.a.kd = spi_command.kd_abad[0];
//   l1_control.a.t_ff = spi_command.tau_abad_ff[0];
  
//   l1_control.h.p_des = spi_command.q_des_hip[0];
//   l1_control.h.v_des  = spi_command.qd_des_hip[0];
//   l1_control.h.kp = spi_command.kp_hip[0];
//   l1_control.h.kd = spi_command.kd_hip[0];
//   l1_control.h.t_ff = spi_command.tau_hip_ff[0];
  
//   l1_control.k.p_des = spi_command.q_des_knee[0];
//   l1_control.k.v_des  = spi_command.qd_des_knee[0];
//   l1_control.k.kp = spi_command.kp_knee[0];
//   l1_control.k.kd = spi_command.kd_knee[0];
//   l1_control.k.t_ff = spi_command.tau_knee_ff[0];
  
//   l2_control.a.p_des = spi_command.q_des_abad[1];
//   l2_control.a.v_des  = spi_command.qd_des_abad[1];
//   l2_control.a.kp = spi_command.kp_abad[1];
//   l2_control.a.kd = spi_command.kd_abad[1];
//   l2_control.a.t_ff = spi_command.tau_abad_ff[1];
  
//   l2_control.h.p_des = spi_command.q_des_hip[1];
//   l2_control.h.v_des  = spi_command.qd_des_hip[1];
//   l2_control.h.kp = spi_command.kp_hip[1];
//   l2_control.h.kd = spi_command.kd_hip[1];
//   l2_control.h.t_ff = spi_command.tau_hip_ff[1];
  
//   l2_control.k.p_des = spi_command.q_des_knee[1];
//   l2_control.k.v_des  = spi_command.qd_des_knee[1];
//   l2_control.k.kp = spi_command.kp_knee[1];
//   l2_control.k.kd = spi_command.kd_knee[1];
//   l2_control.k.t_ff = spi_command.tau_knee_ff[1];

//   l2_control.a.p_des = spi_command.q_des_abad[2];
//   l2_control.a.v_des  = spi_command.qd_des_abad[2];
//   l2_control.a.kp = spi_command.kp_abad[2];
//   l2_control.a.kd = spi_command.kd_abad[2];
//   l2_control.a.t_ff = spi_command.tau_abad_ff[2];
  
//   l3_control.h.p_des = spi_command.q_des_hip[2];
//   l3_control.h.v_des  = spi_command.qd_des_hip[2];
//   l3_control.h.kp = spi_command.kp_hip[2];
//   l3_control.h.kd = spi_command.kd_hip[2];
//   l3_control.h.t_ff = spi_command.tau_hip_ff[2];
  
//   l3_control.k.p_des = spi_command.q_des_knee[2];
//   l3_control.k.v_des  = spi_command.qd_des_knee[2];
//   l3_control.k.kp = spi_command.kp_knee[2];
//   l3_control.k.kd = spi_command.kd_knee[2];
//   l3_control.k.t_ff = spi_command.tau_knee_ff[2];

//   l4_control.a.p_des = spi_command.q_des_abad[3];
//   l4_control.a.v_des  = spi_command.qd_des_abad[3];
//   l4_control.a.kp = spi_command.kp_abad[3];
//   l4_control.a.kd = spi_command.kd_abad[3];
//   l4_control.a.t_ff = spi_command.tau_abad_ff[3];
  
//   l4_control.h.p_des = spi_command.q_des_hip[3];
//   l4_control.h.v_des  = spi_command.qd_des_hip[3];
//   l4_control.h.kp = spi_command.kp_hip[3];
//   l4_control.h.kd = spi_command.kd_hip[3];
//   l4_control.h.t_ff = spi_command.tau_hip_ff[3];
  
//   l4_control.k.p_des = spi_command.q_des_knee[3];
//   l4_control.k.v_des  = spi_command.qd_des_knee[3];
//   l4_control.k.kp = spi_command.kp_knee[3];
//   l4_control.k.kd = spi_command.kd_knee[3];
//   l4_control.k.t_ff = spi_command.tau_knee_ff[3];
// }


/*!
 * Convert spi_data to leg_state
 */
void data_to_leg_state() {

}


/*!
 * compute torque_out using joint impedance control
 */
void  compute_torque_out() {
  memset(&torque_out,0,sizeof(torque_t));
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
  memset(&current_out,0,sizeof(torque_t));
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


// void spi_isr() {
//   //detachInterrupt (digitalPinToInterrupt(interruptPin));
//   // Serial.print("Enter ISR: "); Serial.println(++jj);
//   // For testing, covert spi_data to rx_buf
//   int bytecount = 0;
//     // SPI1->DR = tx_buff[0];
//   mySPI.pushr(tx_buff[0]);
//   // Serial.print("ISR CS State: ");Serial.println(digitalReadFast(interruptPin_jumped));

//   // while(cs == 0) {
//   // Returns 1 if frame transfer not completed, i.e. cs assert
//   while (!digitalReadFast(interruptPin_jumped)) {

//       /* SPI1->SR   
//         BIT8 | BIT7 | BIT6 | BIT5 | BIT4   | BIT3 | BIT2   | BIT1 | BIT0
//         FRE  | BSY  | OVR  | MODF | CRCERR | UDR  | CHSIDE | TXE  | RXNE
//         Check whether Rx buffer not empty
//       */
//       // if(SPI1->SR&0x1) {
//       if (mySPI.available()) {
//           //Serial.println("Serial available: ");
//           // rx_buff[bytecount] = SPI1->DR;
//           rx_buff[bytecount] = mySPI.popr();
          
//           bytecount++;
//           if(bytecount<12) {
            
//             // SPI1->DR = tx_buff[bytecount];
//             mySPI.pushr(tx_buff[bytecount]);
//           }
//      }  

//   }
//   mySPI.clearSR();
//   //attachInterrupt(digitalPinToInterrupt(interruptPin), spi_isr, FALLING);
//   // Serial.print("read rxbuff: "); Serial.print(rx_buff[0]); Serial.println("");

//   // after reading, save into spi_command
//   // should probably check checksum first!
//   // uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff,32);
//   for(int i = 0; i < CMD_LEN; i++)
//   {
//       ((uint16_t*)(&spi_command))[i] = rx_buff[i];
//   }

//   // Serial.print("read : "); Serial.print(spi_command.q_des_abad[0]); Serial.println("");
//   Serial.println("Exit ISR");
// }
