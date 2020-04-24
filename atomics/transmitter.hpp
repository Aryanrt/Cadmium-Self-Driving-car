/**
* Aryan Rashidi-Tabrizi - Carleton University
*
* Transimtter:
* Simple model to transmit RFID tag IDs and Commands
*/

#ifndef BOOST_SIMULATION_PDEVS_transmitter_HPP
#define BOOST_SIMULATION_PDEVS_transmitter_HPP
#define TRANSFER_SIZE   4

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <stdio.h>
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
#include "../drivers/nRF24L01P.h"

using namespace cadmium;
using namespace std;

//Port definition
    struct transmitter_defs {
        struct dataOut : public out_port<double> { };
        //struct in : public in_port<bool> { };
        struct in : public in_port<double> { };
    };

    template<typename TIME>
    class transmitter {
        using defs=transmitter_defs; // putting definitions in context
        public:
            //Parameters to be overwriten when instantiating the atomic model
            TIME   slowToggleTime;
            TIME   fastToggleTime;
            // default constructor
            transmitter(PinName s, PinName t, PinName q, PinName w, PinName r, PinName a) {
              slowToggleTime  = TIME("00:00:10:00");
              fastToggleTime  = TIME("00:00:00:10");
              state.temp = new nRF24L01P(s,t,q,w,r,a);
              state.newTag = false;
              state.isCard = false;
              state.flag = 0;
              state.sending = false;
              state.counter = 0;
              state.temp->powerUp(); 
              state.commandAvailable = false;
      
            }

            // state definition
            struct state_type{
              char s[26] =  " ";
              char s2[26] = "" ;
              nRF24L01P* temp;
              bool newTag;
              bool isCard;
              int flag;
              bool sending;
              int counter;
              int recieveCounter;
              bool commandAvailable;
            };
            state_type state;
            // ports definition

            using input_ports=std::tuple<typename defs::in>;
            using output_ports=std::tuple<typename defs::dataOut>;
            int t = 0 ;



            // internal transition
            void internal_transition() {
              if(state.newTag && state.sending)
                state.sending = false;
              else if(state.newTag && ! state.sending)
                state.newTag = false;              

              state.commandAvailable = false;

              if(state.sending == false || state.newTag == false)
              {


                state.temp->setTransferSize(32);
                state.temp->setReceiveMode();
                state.temp->enable();
                //for(int a = 0; a <= 2 ; a = a + 1) {
                for (int b = 0 ; b <=200; b= b+1)
                {

                   if( state.temp->readable(0)) 
                   {
                       state.temp->read(0, state.s2,sizeof(state.s2)); // reading msg
                       printf( "Receiving data :%s  \n",state.s2);
                       printf("\n");
                       printf("\n");
                       printf("\n");
                       //state.recieveCounter--;
                       state.sending = true;
                       break;

                   }

                  }
              }

              if(state.sending == true && state.newTag == true)
              {
                    state.newTag = 0;
               
              }
              else if(state.sending == true && state.newTag == false)
              {
                state.sending =false;
              }

            }

            // external transition
            void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {

              for(const auto &x : get_messages<typename defs::in>(mbs)){
                  
                if(x == 0 )
                  break;

                state.newTag = true;
                
                if(x == 155)
                {
                  state.flag = 1;
                  state.s[0] = '0';
                  printf("230   \n");                                    
                }
                else if (x == 42)
                {
                  state.flag = 1;
                  state.s[0] = '1';                             
                }
                else if (x == 69)
                {
                  state.flag = 1;
                  state.s[0] = '2';                             
                }
                else if (x == 117)
                {
                  state.flag = 1;
                  state.s[0] = '3';                             
                }
                else if (x == 5)
                {
                  state.flag = 1;
                  state.s[0] = '4';                             
                }
                else if (x == 229)
                {
                  state.flag = 1;
                  state.s[0] = '5';                             
                }
                else if (x == 21)
                {
                  state.flag = 1;
                  state.s[0] = '6';                             
                }
                else if (x == 149)
                {
                  state.flag = 1;
                  state.s[0] = '7';                             
                }
                //else
                  //state.s[0] = '0';
                  //state.s = "x:-75.696371, y:45.385320";

                state.sending = true;
                printf("newtag: %d\n", state.newTag );
                printf("flag:  %d\n", state.flag );  

                
              }

              

            }
            // confluence transition
            void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
              internal_transition();
             // external_transition(TIME(), std::move(mbs));
            }

            // output function
            typename make_message_bags<output_ports>::type output() const {
              typename make_message_bags<output_ports>::type bags;
              printf("newTAg in msg: %d\n", state.newTag );

              if(state.newTag == true)
              {
                printf("newTAg in msg: %d\n", state.newTag );
                state.temp->setTransferSize(32);
                state.temp->setTransmitMode();
                state.temp->enable(); // enable ce pin
                printf("%i \n",sizeof(state.s));
                if(state.s[0] =='0')
                {
                  printf( "Sending data:%s \n","1 A");
                  state.temp->write(1, const_cast<char*>("1 A"),32); // writing 
                  printf("%i \n",sizeof(state.s));
                }
                else if(state.s[0] =='1')
                {
                  printf( "1 B");
                  state.temp->write(1, const_cast<char*>("1 B"),32); // writing 
                  printf("%i \n",sizeof(state.s));
                }
                else if(state.s[0] =='2')
                {
                  printf( "1 C");
                  state.temp->write(1, const_cast<char*>("1 C"),32); // writing 
                  printf("%i \n",sizeof(state.s));
                }
                else if(state.s[0] == '3')
                {
                  printf( "1 D");
                  state.temp->write(1, const_cast<char*>("1 D"),32); // writing 
                  printf("%i \n",sizeof(state.s));
                }
                else if(state.s[0] == '4')
                {
                  printf( "1 E");
                  state.temp->write(1, const_cast<char*>("1 E"),32); // writing 
                  printf("%i \n",sizeof(state.s));
                }
                else if(state.s[0] == '5')
                {
                  printf( "1 F");
                  state.temp->write(1, const_cast<char*>("1 F"),32); // writing 
                  printf("%i \n",sizeof(state.s));
                }
                else if(state.s[0] == '6')
                {
                  printf( "1 G");
                  state.temp->write(1, const_cast<char*>("1 G"),32); // writing 
                  printf("%i \n",sizeof(state.s));
                }
                else if(state.s[0] == '7')
                {
                  printf( "1 H");
                  state.temp->write(1, const_cast<char*>("1 H"),32); // writing 
                  printf("%i \n",sizeof(state.s));
                }
               
                state.temp->disable();
              }
                  
              double out = 0;
              if(state.commandAvailable == true && state.isCard == true)
              {
                out = 100;
              }
              else if(state.commandAvailable == true && state.isCard == false)
              {
                out = 200;
              }

              get_messages<typename defs::dataOut>(bags).push_back(out);

              return bags;
            }

            // time_advance function
            TIME time_advance() const {
              return TIME("00:00:10");
            }

            friend std::ostringstream& operator<<(std::ostringstream& os, const typename transmitter<TIME>::state_type& i) {
              os << "Output: " << (i.newTag ? 1 : 0);
              return os;
            }
        };


#endif // BOOST_SIMULATION_PDEVS_BLINKY_HPP
