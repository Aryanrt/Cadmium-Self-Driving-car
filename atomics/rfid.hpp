/**
* Aryan Rashidi-Tabrizi - Carleton University
*
* RFID:
* Simple modele to scan the RFID tags
*/

#ifndef BOOST_SIMULATION_PDEVS_RFID_HPP
#define BOOST_SIMULATION_PDEVS_RFID_HPP

#include <stdio.h>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <limits>
#include <math.h> 
#include <assert.h>
#include <memory>
#include <iomanip> 
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <algorithm>
#include <limits>
#include <random>
#include "../drivers/MFRC522.cpp"
#include "../drivers/nRF24L01P.h"
#include <cadmium/real_time/arm_mbed/embedded_error.hpp>

using namespace cadmium;
using namespace std;

    
struct rfid_defs 
{
        //struct dataOut : public out_port<bool> { };
        struct dataOut : public out_port<double> { };
        //struct idOut : public out_port<float> { };
        struct in : public in_port<bool> { };
};


template<typename TIME>
class Rfid
{
    
  using defs=rfid_defs; // putting definitions in context
  public:

  TIME   slowToggleTime;

  Rfid(PinName sda, PinName scl, PinName scl2, PinName scl3, PinName scl4) 
  {
    slowToggleTime    = TIME("00:00:59:00");
    state.rfid = new drivers::MFRC522(sda,scl,scl2,scl3,scl4);
    state.newTag = false;
    state.isTag = false;
    state.light = false;
    state.lognWait = false;

  }
    
  // state definition
  struct state_type
  {
    uint8_t id[10];
    drivers::MFRC522* rifd;
    bool newTag;
    bool lognWait;
    bool isTag;
    bool light;
   // int counter;
  }; 
  state_type state;

  using input_ports=std::tuple<>;
  using output_ports=std::tuple<typename defs::dataOut>;

  // internal transition
  void internal_transition() {
    if(state.newTag)
    {
      state.newTag = false;
      state.isTag = true;  
    }
    else
      state.isTag = false;

      for(int i =0; i <1; i++)
      {
        //printf("anything????\n");
        if (state.rfid->PICC_IsNewCardPresent() ) 
        {
          state.rfid->PICC_ReadCardSerial();
          state.newTag = true;
          //state,counter = 10;
          state.id[0] = state.rfid->uid.uidByte[0];
          state.id[1] = state.rfid->uid.uidByte[1];
          state.id[2] = state.rfid->uid.uidByte[2];
          state.id[3] = state.rfid->uid.uidByte[3];

          //state.prop = true;
          break;
          //}
        }
      }
      if( state.newTag == true)
      {
          printf("new tag found");
          for (uint8_t i = 0; i < 4; i++)
          {
            printf(" %f", (double)state.id[i]);
          }
          printf("\n\r");
      }
    
   
  }


  // external transition
  void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {

  }
  // confluence transition
  void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
    internal_transition();
  }

  // output function
  typename make_message_bags<output_ports>::type output() const {
    typename make_message_bags<output_ports>::type bags;
    //bool out;              
    double out=0;

    if(state.newTag)
      out = state.id[0];
    else
      out = 0;

    
    printf("rfid: %f\n", out);
    //out = false;
    get_messages<typename defs::dataOut>(bags).push_back(out);                    
    return bags;
  }

  // time_advance function
  TIME time_advance() const {  
    if(state.newTag)
      return TIME("00:00:00:00");
    else if(state.isTag)
      return TIME("00:00:10:00");
    else
      return TIME("00:00:05:00");
    //return std::numeric_limits<TIME>::infinity();
  }

  friend std::ostringstream& operator<<(std::ostringstream& os, const typename Rfid<TIME>::state_type& i) {
    os << "Output: " << (i.newTag ? 1 : 0); 
    return os;
  }
};     
#endif // RFID_HPP
