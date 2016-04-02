#include "../include/simulator.h"
#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <algorithm>

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
#define BASE_RTT 12
#define DELAY 2

//Sender
static int send_base = 1; //Seq no of first packet in sender's window
static int nextseqnum = 1; //Seq num of next packet that will be sent
static int sender_window = 0; //Window size of sender
static int send_buffer_pos = -1; // Position of sender buffer pointer
static int delay = 0; //Delay introduced for expiry timer for batch packet transmissions

static struct pkt sent_dataPkt[1010]; // Buffer of the data packet sent to B
static vector <pkt> in_flight; //List of packets sent so far
static float in_flight_timer[1010]; //Time of when the timer for corresponding packet should expire
static float pkt_sent_timer[1010]; //Time of sending the packet
static float start_time, end_time, timer_fin = 0.0;

//Receiver
static int recv_base = 1; //Seq no of first packet in receiver's window
static int expectedseqnum = 1; //Expected Seq no of next packet received from A
static int recv_window = 0; //Window size of sender
static struct pkt sent_ackPkt[1010]; // Copy of all ACK's sent to A
static struct pkt recv_dataPkt[1010]; // Buffer of the data packet received by B
static int ack_pkts[1010]; //Keep track of seqnum for which ack has been sent

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
  
  //cout<<"Checking Checksum: "<<p.checksum; 
  if (check == -1){
    //cout<<" NOT Corrupt"<<endl;    
    return false;
  }
  else{
    //cout<<" CORRUPT:"<<check<<endl;
    return true;
  }
}

//Function to remove the packet with given seqnum from list of in-flight packets
void update_in_flight_packets(int seqnum){
  if (in_flight.empty()){
    //cout<<"ERROR while updating in-flight. There are no elements\n";
  }
  for(vector <pkt>::iterator it = in_flight.begin(); it != in_flight.end(); ++it){
    if (it->seqnum == seqnum){
      it = in_flight.erase(it);
      return;
    }
  }
  //cout<<"ERROR while updating in-flight. There is no packet with seqnum:"<<seqnum<<endl;
}

//Function to sort vector of packets on the basis of timer expiry
struct expiry_timer_less_than
{
  inline bool operator() (const pkt& p1, const pkt& p2)
  {
    return (in_flight_timer[p1.seqnum] < in_flight_timer[p2.seqnum]);
  }
};


/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message)
{
  //cout<<"\nA_output Base:"<<send_base<<" nextseqnum:"<<nextseqnum<<" send_buffer_pos:"<<send_buffer_pos<<endl;
  
  //Create new pkt to send to layer 3
  struct pkt p_toLayer3;
  
  p_toLayer3.seqnum = nextseqnum;
  p_toLayer3.acknum = nextseqnum;
  strncpy(p_toLayer3.payload, message.data, 20);
  p_toLayer3.checksum = generate_checksum(p_toLayer3); 
  
  //Store a copy of Message
  sent_dataPkt[nextseqnum++] = p_toLayer3;
  
  //Send when packet is within sender window
  if (p_toLayer3.seqnum < send_base+sender_window){
    tolayer3(0, p_toLayer3);    
    //cout<<"A_output sent to layer 3, SEQ:"<<nextseqnum-1<<" Data:"<<message.data<<" Time:"<<get_sim_time()<<endl;
    
    //Start full timer if there are no in-flight packets
    if (send_base == nextseqnum-1){
      delay = 0;
      start_time = get_sim_time();      
      starttimer(0, timer_fin + delay);
    }
    
    //Keep details of timers of packets in flight
    in_flight.push_back(p_toLayer3);
    pkt_sent_timer[p_toLayer3.seqnum] = get_sim_time();
    in_flight_timer[p_toLayer3.seqnum] = pkt_sent_timer[p_toLayer3.seqnum] + timer_fin + delay;
    sort(in_flight.begin(), in_flight.end(), expiry_timer_less_than());    
    delay += DELAY;
    
  }
  
  //Buffer if packet seqnum is out of sender window  
  else{
    //cout<<"A_output Message SEQ:"<<p_toLayer3.seqnum<<" buffered"<<endl;
    //Keep track of seq num of message where buffering starts
    if (send_buffer_pos == -1){
      send_buffer_pos = p_toLayer3.seqnum;
    }    
  }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
  //cout<<"A_input ACK:"<<packet.acknum<<" received at time:"<<get_sim_time()<<endl; 
  
  //Check if ACK is corrupt  
  if (check_corrupt(packet)){
    //cout<<"Inside A_input. ACK corrupt\n";    
    return;
  }
  
  //ACK outside window range of sender
  if (packet.acknum < send_base || packet.acknum >= send_base+sender_window){
    //cout<<"Inside A_input. ACK outside sender window\n";    
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
      float remaining_time_before_timer_expires = in_flight_timer[packet.acknum] - end_time;
      if (remaining_time_before_timer_expires < 0) remaining_time_before_timer_expires = 0;      
      float transmission_time_diff = in_flight_timer[in_flight[0].seqnum] - end_time;
      if (transmission_time_diff <= 0) transmission_time_diff = DELAY;
      //cout<<"A_input: Relative Timer:"<<remaining_time_before_timer_expires+transmission_time_diff<<endl;
      starttimer(0, remaining_time_before_timer_expires + transmission_time_diff);    
    }
    
    //Update timer based on new_rtt only if new_rtt is more than base RTT - to ignore quick ACK's for retransmissions
    end_time = get_sim_time();
    float new_rtt = end_time - pkt_sent_timer[packet.acknum];
    if (new_rtt > RTT){
      float new_timer = (0.875 * timer_fin) + (0.125 * new_rtt);
      if (new_timer > RTT && new_timer < 2*BASE_RTT){
        timer_fin = new_timer;
      }
      //cout<<"Inside A_input. New RTT:"<<new_rtt<<" New timer set to:"<<timer_fin<<endl;
    }  
    
    //Mark packet as acknowledged
    in_flight_timer[packet.acknum] = -1;
    
    //Update send_base if ACK had already been received for other packets
    while(in_flight_timer[send_base] == -1){
      ++send_base;
    }

    //Check and send any buffered messages to B that fall into the new sender window of A
    if (send_buffer_pos != -1){
      for (int i = send_buffer_pos; (i < nextseqnum) && (i < send_base+sender_window); i++){
        tolayer3(0, sent_dataPkt[i]);
        //cout<<"A_input buffered message sent to layer 3, SEQ:"<<i<<" nextseqnum:"<<nextseqnum<<" send_base:"<<send_base<<" Time:"<<get_sim_time()<<endl;
        
        //Need to start timer if it was not running
        if (in_flight.empty()){
          delay = 0;
          starttimer(0, timer_fin);
        }
        
        //Add to the list of packets in flight, and record it sending time
        in_flight.push_back(sent_dataPkt[i]);
        pkt_sent_timer[sent_dataPkt[i].seqnum] = get_sim_time();
        in_flight_timer[sent_dataPkt[i].seqnum] = pkt_sent_timer[sent_dataPkt[i].seqnum] + timer_fin + delay; 
        sort(in_flight.begin(), in_flight.end(), expiry_timer_less_than());
        delay += DELAY;

        send_buffer_pos++;
      }
    }
    if (send_buffer_pos == nextseqnum){
      send_buffer_pos = -1;
    }    
    //cout<<"A_input. New send_buffer_pos:"<<send_buffer_pos<<endl;
  }
  
  else{
    //Remove from list of in-flight packets
    update_in_flight_packets(packet.acknum);  

    //Update timer based on new_rtt only if new_rtt is more than base RTT - to ignore quick ACK's for retransmissions
    end_time = get_sim_time();
    float new_rtt = end_time - pkt_sent_timer[packet.acknum];
    if (new_rtt > RTT){
      float new_timer = (0.875 * timer_fin) + (0.125 * new_rtt);
      if (new_timer > RTT && new_timer < 2*BASE_RTT){
        timer_fin = new_timer;
      }
      //cout<<"Inside A_input. New RTT:"<<new_rtt<<" New timer set to:"<<timer_fin<<endl;
    }     
    
    //Mark packet as acknowledged
    in_flight_timer[packet.acknum] = -1;
  }    
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
  struct pkt packet = in_flight.front();
  //cout<<"\nInside A_timerinterrupt for SEQ:"<<packet.seqnum<<" Time:"<<get_sim_time()<<endl;
  
  //Remove from front of list of in-flight packets, since it's timer expired 
  in_flight.erase(in_flight.begin());

  //Reset timer value
  timer_fin = BASE_RTT;
  
  //Retransmit packet
  tolayer3(0, packet);  
  //cout<<"A_timerinterrupt Retransmitted SEQ:"<<packet.seqnum<<endl;
  
  //Restart relative timer for next in-flight packet, if any
  if (!in_flight.empty()){
    end_time = get_sim_time();
    float remaining_time_before_timer_expires = in_flight_timer[packet.seqnum] - end_time;
    if (remaining_time_before_timer_expires < 0) remaining_time_before_timer_expires = 0;
    float transmission_time_diff = in_flight_timer[in_flight[0].seqnum] - end_time;
    if (transmission_time_diff <= 0) transmission_time_diff = DELAY;
    //cout<<"A_timerinterrupt: Relative Timer:"<<remaining_time_before_timer_expires+transmission_time_diff<<endl;
    starttimer(0, remaining_time_before_timer_expires + transmission_time_diff);  
  }
  //Else start full timer for this retransmitted packet
  else{
    //cout<<"A_timerInterrupt: Full Timer:"<<timer_fin<<endl;    
    starttimer(0, timer_fin);
  }
  
  //Add retransmitted packet to the end of the list of in-flight packets and Update its sent timer
  in_flight.push_back(packet);
  pkt_sent_timer[packet.seqnum] = get_sim_time(); 
  in_flight_timer[packet.seqnum] = pkt_sent_timer[packet.seqnum] + timer_fin; 
  sort(in_flight.begin(), in_flight.end(), expiry_timer_less_than());
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
  //cout<<"Inside A_init\n";
  timer_fin = BASE_RTT;
  sender_window = getwinsize();
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  
  struct pkt p_toLayer3;
  char data_fromA[20];
  
  //Check if packet is corrupt
  if (check_corrupt(packet)){
    //cout<<"B_input packet corrupt\n";    
    return;
  }
  
  //cout<<"B_input ACK"<<packet.seqnum<<" RecvBase:"<<recv_base<<"\n"; 
  
  //Process if packet is not corrupt, and has seqnum in receiver window
  if (packet.seqnum >= recv_base && packet.seqnum < recv_base+recv_window){

    //Send data received from A to B's Layer 5 if seqnum is in order, else buffer
    if (packet.seqnum == recv_base){
      strncpy(data_fromA, packet.payload, 20);
      tolayer5(1, data_fromA);
      //cout<<"B_input data SEQ:"<<packet.seqnum<<"sent to layer 5\n";
      ++recv_base;
      
      //Deliver other buffered messages, if any
      while(ack_pkts[recv_base] == 1){
        strncpy(data_fromA, recv_dataPkt[recv_base].payload, 20);
        tolayer5(1, data_fromA);
        //cout<<"B_input data SEQ:"<<recv_dataPkt[recv_base].seqnum<<"sent to layer 5\n";       
        ++recv_base;
      }
    }
    else{
      //Add out-of-order packet to buffer
      recv_dataPkt[packet.seqnum] = packet;  
      //Mark packet as received and ACKed
      ack_pkts[packet.seqnum] = 1;       
    }
    
    //Send ACK to A for packet received
    p_toLayer3.seqnum = packet.seqnum;
    p_toLayer3.acknum = packet.seqnum;
    memset(p_toLayer3.payload,'\0', 20);    
    p_toLayer3.checksum = generate_checksum(p_toLayer3);
    sent_ackPkt[packet.seqnum] = p_toLayer3;
    
    tolayer3(1, p_toLayer3);
    //cout<<"B_input ACK"<<packet.seqnum<<" sent to layer 3 Time:"<<get_sim_time()<<"\n"; 
    
   
  }
  else if(packet.seqnum < recv_base){
    //cout<<"Inside B_input. Resend ACK"<<packet.seqnum<<" Time:"<<get_sim_time()<<"\n";    
    tolayer3(1, sent_ackPkt[packet.seqnum]);    
  }
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
  //cout<<"Inside B_init\n";
  recv_window = getwinsize();  
}
