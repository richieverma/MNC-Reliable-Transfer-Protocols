#include "../include/simulator.h"
#include <iostream>
#include <string>
#include <cstring>

using namespace std;
/* ******************************************************************
 ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.1  J.F.Kurose

   This code should be used for PA2, unidirectional data transfer 
   protocols (from A to B). Network properties:
   - one way network delay averages five time units (longer if there
     are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
     or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
     (although some can be lost).
**********************************************************************/

/********* STUDENTS WRITE THE NEXT SEVEN ROUTINES *********/
//Global Params
#define RTT 10
#define BASE_RTT 12

//Sender
static int send_seq = -1; //Seq no of packet sent to B
static int recv_ack = -1; //Ack num of last ACK received from B
static struct pkt sent_dataPkt; // Copy of the last data packet sent to B
static float start_time, end_time, timer_fin;

//Receiver
static int recv_seq = -1; //Seq no of last packet received from A
static int send_ack = -1; //Ack num of last ACK sent to A
static struct pkt sent_ackPkt; // Copy of last ACK sent to A

//Function to generate checksum
int generate_checksum(struct pkt p){
  int checksum = 0;

  for (int i = 0; i < 20; i++){
    checksum += p.payload[i];
  }
  checksum += p.seqnum;
  checksum += p.acknum;
  
  //cout<<"Generated Checksum: "<<~checksum<<endl;
  return ~checksum;
}

//Function to verify checksum
bool check_corrupt(struct pkt p){
  int check = 0;
  check += p.seqnum;
  check += p.acknum;
  check += p.checksum; 
  
  for (int i = 0; i < 20; i++){
    check += p.payload[i];
  }  
  
  //cout<<"Checking Checksum: "<<p.checksum<<endl; 
  if (check == -1){
    cout<<"NOT Corrupt"<<endl;    
    return false;
  }
  else{
    cout<<"CORRUPT:"<<check<<endl;
    return true;
  }
}

/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message)
{
  //Ignore if ACK has not been received for last sent packet
  if (send_seq != recv_ack){
    cout<<"A_output Packet in transit\n";    
    return;
  }
  
  //Create new pkt to send to layer 3
  struct pkt p_toLayer3;
  
  send_seq = (send_seq+1)%2;
  
  p_toLayer3.seqnum = send_seq;
  p_toLayer3.acknum = recv_ack;
  strncpy(p_toLayer3.payload, message.data, 20);
  //Generate checksum
  p_toLayer3.checksum = generate_checksum(p_toLayer3);  
  sent_dataPkt = p_toLayer3;
  start_time = get_sim_time();
  
  cout<<"A_output sent to layer 3, SEQ:"<<send_seq<<"Data:"<<message.data<<" Time:"<<get_sim_time()<<endl;
  starttimer(0, timer_fin);
  tolayer3(0, p_toLayer3);
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
  cout<<"A_input ACK:"<<packet.acknum<<" received at time:"<<get_sim_time(); 
  
  //Check if ACK is corrupt, or duplicate ACK is received, then do nothing
  if (check_corrupt(packet) || packet.acknum == recv_ack){
    cout<<"Inside A_input. ACK corrupt\n";    
    return;
  }
  
  //Check if  duplicate ACK is received, then do nothing  
  if (packet.acknum == recv_ack){
    cout<<"Inside A_input. Duplicate ACK\n";    
    return;
  }  
  
  //Check if expected ACK received and is not a duplicate
  if (packet.acknum == send_seq && packet.acknum != recv_ack){
    //Update last received ACK number
    recv_ack = send_seq; 
    stoptimer(0);
    
    //Update timer based on new_rtt only if new_rtt is more than base RTT - to ignore quick ACK's for retransmissions
    end_time = get_sim_time();
    float new_rtt = end_time - start_time;
    if (new_rtt > RTT){
      float new_timer = (0.875 * timer_fin) + (0.125 * new_rtt);
      if (new_timer > RTT && new_timer < 2*BASE_RTT){
        timer_fin = new_timer;
      }
      cout<<"Inside A_input. New RTT:"<<new_rtt<<" New timer set to:"<<timer_fin<<endl;
    }
  }
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
  timer_fin = BASE_RTT;
  cout<<"A_timerinterrupt retransmitted to layer 3, SEQ:"<<send_seq<<" Data:"<<sent_dataPkt.payload<<" Time:"<<get_sim_time()<<endl;
  starttimer(0, timer_fin);  
  tolayer3(0, sent_dataPkt);
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
  cout<<"Inside A_init\n";
  timer_fin = BASE_RTT;
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt p_toLayer3;
  char data_fromA[20];
  
  //Check if packet is corrupt, then send prev ACK
  if (check_corrupt(packet)){
    tolayer3(1, sent_ackPkt);
    return;    
  }
  
  //Resend ACK if duplicate packet is received
  if (packet.seqnum == send_ack){
    tolayer3(1, sent_ackPkt);
    return;
  }
  else{
    send_ack = packet.seqnum;
    recv_seq = (recv_seq+1)%2;    
  }
  
  //Send data from A to Layer 5
  strncpy(data_fromA, packet.payload, 20);
  tolayer5(1, data_fromA);
  cout<<"B_input data sent to layer 5\n";
  
  //Send ACK to A for packet received
  p_toLayer3.seqnum = recv_seq;
  p_toLayer3.acknum = send_ack;
  memset(p_toLayer3.payload,'\0', 20);
  p_toLayer3.checksum = generate_checksum(p_toLayer3);
  sent_ackPkt = p_toLayer3;
  
  tolayer3(1, p_toLayer3);
  cout<<"B_input ACK"<<send_ack<<" sent to layer 3\n";  
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
    cout<<"Inside B_init\n";
}
