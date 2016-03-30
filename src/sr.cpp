#include "../include/simulator.h"
#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <unordered_map>
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
struct packet_sent{
  struct pkt p;
  float sent_time;
  int ack_recv;
};

//Global Params
#define RTT 10
#define BASE_RTT 18

//Sender
static int send_base = 1; //Seq no of first packet in sender's window
static int nextseqnum = 1; //Seq num of next packet that will be sent
static int window = 0; //Window size of sender
static int send_buffer_pos = -1; // Position of sender buffer pointer

static struct pkt sent_dataPkt[1010]; // Buffer of the data packet sent to B
static vector <pkt> in_flight; //List of packets sent so far
static unordered_map <int, float> in_flight_timer;
static float start_time, end_time, timer_fin = 0.0;

//Receiver
static int recv_base = 1; //Seq no of first packet in receiver's window
static int expectedseqnum = 1; //Expected Seq no of next packet received from A
static struct pkt sent_ackPkt[1010]; // Copy of all ACK's sent to A
static struct pkt recv_dataPkt[1010]; // Buffer of the data packet received by B
static unordered_map <int, int> ack_pkts; //Keep track of seqnum for which ack has been sent

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

//Function to remove the packet with given seqnum from list of in-flight packets
void update_in_flight_packets(int seqnum){
  if (in_flight.empty()){
    cout<<"ERROR while updating in-flight. There are no elements\n"
  }
  for(auto it = in_flight.begin(); it != in_flight.end(); ++it){
    if ((*it).seqnum == seqnum){
      it = in_flight.erase(it);
      return;
    }
  }
  cout<<"ERROR while updating in-flight. There is no packet with seqnum:"<<seqnum<<endl;
}


/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message)
{
  cout<<"A_output Base:"<<send_base<<" nextseqnum:"<<nextseqnum<<" window:"<<window<<endl;
  
  //Create new pkt to send to layer 3
  struct pkt p_toLayer3;
  
  p_toLayer3.seqnum = nextseqnum;
  p_toLayer3.acknum = nextseqnum;
  strncpy(p_toLayer3.payload, message.data, 20);
  p_toLayer3.checksum = generate_checksum(p_toLayer3, 0); 
  
  //Store a copy of Message
  sent_dataPkt[nextseqnum++] = p_toLayer3;
  
  //Send when packet is within sender window
  if (p_toLayer3.seqnum < send_base+window){
    tolayer3(0, p_toLayer3);    
    
    if (send_base == nextseqnum-1){
      start_time = get_sim_time();      
      cout<<"A_output sent to layer 3, SEQ:"<<send_base<<" Data:"<<message.data<<" Time:"<<get_sim_time()<<endl;
      starttimer(0, timer_fin);
    }
    
    //Keep details of timers of packets in flight
    in_flight.push_back(p_toLayer3);
    in_flight_timer[p_toLayer3.seqnum] = get_sim_time();
  }
  
  //Buffer if packet seqnum is out of sender window  
  else{
    cout<<"A_output Message SEQ:"<<p_toLayer3.seqnum<<" buffered"<<endl;
    //Keep track of seq num of message where buffering starts
    if (send_buffer_pos == -1){
      send_buffer_pos = p_toLayer3.seqnum;
    }    
  }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
  cout<<"A_input ACK:"<<packet.acknum<<" received at time:"<<get_sim_time()<<endl; 
  
  //Check if ACK is corrupt  
  if (check_corrupt(packet, 1)){
    cout<<"Inside A_input. ACK corrupt\n";    
    return;
  }
  
  //ACK outside window range of sender
  if (packet.acknum < send_base || packet.acknum >= send_base+window){
    cout<<"Inside A_input. ACK outside sender window\n";    
    return;
  }  
  
  //Check if ACK is for the first packet in sender window. Then update send_base
  if (packet.acknum == send_base){
    stoptimer(0);     
    ++send_base;
    
    //Remove from list of in-flight packets
    update_in_flight_packets(packet.acknum);
    
    //Restart timer for next in-flight packet, if any
    if (!in_flight.empty()){
      end_time = get_sim_time();      
      float remaining_time_before_timer_expires = timer_fin - (end_time - in_flight_timer[packet.acknum]);
      float transmission_time_diff = in_flight_timer[in_flight[0].seqnum] - in_flight_timer[packet.acknum];
      
      starttimer(0, remaining_time_before_timer_expires + transmission_time_diff);    
    }
    
    //Update timer based on new_rtt only if new_rtt is more than base RTT - to ignore quick ACK's for retransmissions
    end_time = get_sim_time();
    float new_rtt = end_time - in_flight_timer[packet.acknum];
    if (new_rtt > RTT){
      float new_timer = (0.875 * timer_fin) + (0.125 * new_rtt);
      if (new_timer > RTT && new_timer < 2*BASE_RTT){
        timer_fin = new_timer;
      }
      cout<<"Inside A_input. New RTT:"<<new_rtt<<" New timer set to:"<<timer_fin<<endl;
    }  
    
    //Mark packet as acknowledged
    in_flight_timer[packet.acknum] = -1;
    
    //Update send_base if ACK had already been received for other packets
    while(in_flight_timer[send_base] == -1){
      ++send_base;
    }

    //Check and send any buffered messages to B that fall into the new sender window of A
    if (buffer_pos != -1){
      for (int i = buffer_pos; (i < nextseqnum) && (i < send_base+window); i++){
        tolayer3(0, sent_dataPkt[i]);
        
        //Add to the list of packets in flight, and record it sending time
        in_flight.push_back(sent_dataPkt[i]);
        in_flight_timer[sent_dataPkt[i].seqnum] = get_sim_time();
        
        //Need to start timer if it was not running
        if (in_flight.empty()){
          starttimer(0, timer_fin);
        }
        buffer_pos++;
      }
    }
    if (buffer_pos == send_base+window){
      buffer_pos = -1;
    }    
  }
  
  else{
    //Remove from list of in-flight packets
    update_in_flight_packets(packet.acknum);

    //Update timer based on new_rtt only if new_rtt is more than base RTT - to ignore quick ACK's for retransmissions
    end_time = get_sim_time();
    float new_rtt = end_time - in_flight_timer[packet.acknum];
    if (new_rtt > RTT){
      float new_timer = (0.875 * timer_fin) + (0.125 * new_rtt);
      if (new_timer > RTT && new_timer < 2*BASE_RTT){
        timer_fin = new_timer;
      }
      cout<<"Inside A_input. New RTT:"<<new_rtt<<" New timer set to:"<<timer_fin<<endl;
    }     
    
    //Mark packet as acknowledged
    in_flight_timer[packet.acknum] = -1;
  }    
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
  cout<<"Inside A_timerinterrupt\n";
  
  struct pkt packet = in_flight.front();
  
  //Remove from front of list of in-flight packets, since it's timer expired 
  in_flight.erase(in_flight.begin());

  //Reset timer value
  timer_fin = BASE_RTT;
  
  //Retransmit packet
  tolayer3(0, packet);  
  
  //Restart timer for next in-flight packet, if any
  if (!in_flight.empty()){
    end_time = get_sim_time();
    float remaining_time_before_timer_expires = timer_fin - (end_time - in_flight_timer[packet.acknum]);
    float transmission_time_diff = in_flight_timer[in_flight[0].seqnum] - in_flight_timer[packet.acknum];
    
    starttimer(0, remaining_time_before_timer_expires + transmission_time_diff);  
  }
  //Else start timer for this retransmitted packet
  else{
    starttimer(0, timer_fin);
  }
  
  //Add retransmitted packet to the end of the list of in-flight packets and Update its sent timer
  in_flight.push_back(packet);
  in_flight_timer[packet.seqnum] = get_sim_time();  
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
  
  //Check if packet is corrupt
  if (check_corrupt(packet, 0)){
    cout<<"B_input packet corrupt\n";    
    return;
  }
  
  //Process if packet is not corrupt, and has seqnum in receiver window
  if (packet.seqnum >= recv_base && packet.seqnum < recv_base+window){
    
    //Send ACK to A for packet received
    p_toLayer3.seqnum = 1;
    p_toLayer3.acknum = packet.seqnum;
    p_toLayer3.checksum = generate_checksum(p_toLayer3, 1);
    sent_ackPkt[packet.seqnum] = p_toLayer3;
    
    tolayer3(1, p_toLayer3);
    cout<<"B_input ACK"<<expectedseqnum-1<<" sent to layer 3\n"; 
    
    //Mark packet as received and ACKed
    ack_pkts[packet.seqnum] == 1;
    
    //Send data received from A to B's Layer 5 if seqnum is in order, else buffer
    if (packet.seqnum == recv_base){
      strncpy(data_fromA, packet.payload, 20);
      tolayer5(1, data_fromA);
      cout<<"B_input data sent to layer 5\n";
      
      //Deliver other buffered messages, if any
      while(ack_pkts[++recv_base] == 1){
        strncpy(data_fromA, recv_dataPkt[packet.seqnum].payload, 20);
        tolayer5(1, data_fromA);
        cout<<"B_input data sent to layer 5\n";        
      }
    }
    else{
      //Add out-of-order packet to buffer
      recv_dataPkt[packet.seqnum] = packet;      
    }
  }
  else if(packet.seqnum < recv_base){
    cout<<"Inside B_input. Resend ACK\n";    
    tolayer3(1, sent_ackPkt[packet.seqnum]);    
  }
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
    cout<<"Inside B_init\n";
}
