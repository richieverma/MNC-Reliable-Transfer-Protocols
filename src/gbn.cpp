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
#define BASE_RTT 18

//Sender
static int send_base = 1; //Seq no of first packet in sender's window
static int nextseqnum = 1; //Seq num of next packet that will be sent
static int window = 0; //Window size of sender
static int buffer_pos = -1; // Position of buffer pointer

//static int recv_ack = 0; //Ack num of last ACK received from B
static struct pkt sent_dataPkt[1010]; // Buffer of the data packet sent to B
static float start_time, end_time, timer_fin = 0.0;

//Receiver
static int expectedseqnum = 1; //Expected Seq no of next packet received from A
//static int recv_seq = 0; //Ack num of last ACK sent to A
static struct pkt sent_ackPkt; // Copy of last ACK sent to A

//Function to generate checksum
int generate_checksum(struct pkt p, int DataOrAck){
  int checksum = 0;
  //Include payload only for Data Packet  
  if (DataOrAck == 0){
    for (int i = 0; i < 20; i++){
      checksum += p.payload[i];
    }
    cout<<"Data: ";
  }
  else{
    cout<<"ACK: ";
  }
  checksum += p.seqnum;
  checksum += p.acknum;
  
  cout<<"Generated Checksum: "<<~checksum<<endl;
  return ~checksum;
}

//Function to verify checksum
bool check_corrupt(struct pkt p, int DataOrAck){
  int check = 0;
  check += p.seqnum;
  check += p.acknum;
  check += p.checksum; 
  
  //Include payload only for Data Packet
  if (DataOrAck == 0){  
    for (int i = 0; i < 20; i++){
      check += p.payload[i];
    }  
    cout<<"Data: ";    
  }
  else{
    cout<<"ACK: ";
  }  
  cout<<"Checking Checksum: "<<p.checksum; 
  if (check == -1){
    cout<<" NOT Corrupt"<<endl;    
    return false;
  }
  else{
    cout<<" CORRUPT:"<<check<<endl;
    return true;
  }
}

/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message)
{
  cout<<"A_output Base:"<<send_base<<" nextseqnum:"<<nextseqnum<<" window:"<<window<<endl;

  //Create new pkt to send to layer 3
  struct pkt p_toLayer3;
  
  p_toLayer3.seqnum = nextseqnum;
  //p_toLayer3.acknum = recv_ack;
  p_toLayer3.acknum = nextseqnum;
  strncpy(p_toLayer3.payload, message.data, 20);
  //TODO Generate checksum
  p_toLayer3.checksum = generate_checksum(p_toLayer3, 0); 
  //Buffer Message
  sent_dataPkt[nextseqnum++] = p_toLayer3;
  if (buffer_pos == -1){
    buffer_pos = nextseqnum-1;
  }
  
  //Send when packet is within sender window
  if (nextseqnum < send_base+window){
    tolayer3(0, p_toLayer3);    
  
    if (send_base == nextseqnum-1){
      start_time = get_sim_time();      
      cout<<"A_output sent to layer 3, SEQ:"<<send_base<<" Data:"<<message.data<<" Time:"<<get_sim_time()<<endl;
      starttimer(0, timer_fin);
    }
  }
  
  //Buffer if packet seqnum is out of sender window  
  else{
    cout<<"A_output Message SEQ:"<<nextseqnum-1<<" buffered"<<endl;
  }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
  cout<<"A_input ACK:"<<packet.acknum<<" received at time:"<<get_sim_time()<<endl; 
  
  //Check if ACK is corrupt
  if (!check_corrupt(packet, 1)){
    if (send_base > packet.acknum){
      //Restart timer
      stoptimer(0);
      starttimer(0, timer_fin);      
      return;
    }
    send_base = packet.acknum + 1;
    
    if (send_base == nextseqnum){
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
      //Update last received ACK number
      //recv_ack = packet.acknum;       
    }
    
    else{
      //Restart timer
      stoptimer(0);
      starttimer(0, timer_fin);
      
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
      
      //Check and send any buffered messages that fall into the window
      if (buffer_pos != -1){
        for (int i = buffer_pos; (i < nextseqnum) && (i < send_base+window); i++){
          tolayer3(0, sent_dataPkt[i]);
          buffer_pos++;
        }
      }
      if (buffer_pos == send_base+window){
        buffer_pos = -1;
      }      
    }    
  }
  else{
    cout<<"Inside A_input. ACK corrupt\n";    
    return;
  }
  

}

/* called when A's timer goes off */
void A_timerinterrupt()
{
  cout<<"Inside A_timerinterrupt\n";
  timer_fin = BASE_RTT;
  starttimer(0, timer_fin); 
  //Check and send all messages that fall into the window
  for (int i = send_base; (i < nextseqnum) && (i < send_base+window); i++){
    tolayer3(0, sent_dataPkt[i]);
  }
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
  cout<<"Inside A_init\n";
  timer_fin = BASE_RTT;
  window = getwinsize();
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt p_toLayer3;
  char data_fromA[20];
  
  //Process if packet is not corrupt, and has expected seqnum
  if (!check_corrupt(packet, 0) && packet.seqnum == expectedseqnum){
    
    
    //Send data from A to Layer 5
    strncpy(data_fromA, packet.payload, 20);
    tolayer5(1, data_fromA);
    cout<<"B_input data sent to layer 5\n";
  
    //Send ACK to A for packet received
    //p_toLayer3.seqnum = recv_seq;
    p_toLayer3.seqnum = expectedseqnum;
    p_toLayer3.acknum = expectedseqnum;
    p_toLayer3.checksum = generate_checksum(p_toLayer3, 1);
    sent_ackPkt = p_toLayer3;
    
    tolayer3(1, p_toLayer3);
    expectedseqnum++;
    //recv_seq++;
    cout<<"B_input ACK"<<expectedseqnum-1<<" sent to layer 3\n";  
  }
  else{
    cout<<"Inside B_input. Data corrupt\n";    
  }
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
    cout<<"Inside B_init\n";
}
