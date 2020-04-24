/**
* Aryan Rashidi-Tarbrizi - Carleton University
*
* Seeed Bot Driver:
* This model will do simple line following using a Seed Bot Shield.
* Its purpose is to demonstrate how to use all of the port IO models in RT_ARM_MBED.
*
* Note: The 'SCARED_OF_THE_DARK' macro will read from a Grove light sensor on 
*   analog port A5 and stop the car if the reading is less then 0.3.
* It must be defined here and in main if being used.
*/
#ifndef BOOST_SIMULATION_PDEVS_BLINKY_HPP
#define BOOST_SIMULATION_PDEVS_BLINKY_HPP

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

 //#define SCARED_OF_THE_DARK

using namespace cadmium;
using namespace std;
enum DriveState {right = 0, straight = 1, left = 2, stop = 3, unknown = 4, slightRight = 5, slightLeft = 6 };
//Port definition
    struct seeedBotDriver_defs {
        //Output ports
        struct rightMotor1 : public out_port<float> { };
        struct rightMotor2 : public out_port<bool> { };
        struct leftMotor1 : public out_port<float> { };
        struct leftMotor2 : public out_port<bool> { };
        //Input ports
        struct command : public in_port<double> { };
        struct rightIR : public in_port<bool> { };
        struct middleRightIR : public in_port<bool> { };
        struct centerIR : public in_port<bool> { };
        struct leftIR : public in_port<bool> { };
        struct middleLeftIR : public in_port<bool> { };
        #ifdef SCARED_OF_THE_DARK
        struct lightSensor : public in_port<float> { };
        #endif
    };

    template<typename TIME>
    class SeeedBotDriver {
        using defs=seeedBotDriver_defs; // putting definitions in context
        public:
            //Parameters to be overwriten when instantiating the atomic model
            TIME   slowToggleTime;
            TIME   fastToggleTime;
            // default constructor
            SeeedBotDriver() noexcept{
              state.dir = unknown;
              state.direction = 0;
              state.tagType = 0;
              state.counter = 0;
              state.counter2 = 0;
            }
            
            // state definition
            struct state_type{
              bool leftIR;
              bool middleLeftIR;
              bool centerIR;
              bool rightIR;
              bool middleRightIR;
              DriveState dir;
              bool prop;
              int tagType;
              int counter;
              int counter2;
              double direction;
            }; 
            state_type state;
            // ports definition
            #ifdef SCARED_OF_THE_DARK
            using input_ports=std::tuple<typename defs::rightIR, typename defs::lightSensor, typename defs::centerIR, typename defs::leftIR, typename defs::command>;
            #else
            using input_ports=std::tuple<typename defs::rightIR, typename defs::middleRightIR, typename defs::centerIR, typename defs::leftIR, typename defs::middleLeftIR, typename defs::command>;
            #endif
            using output_ports=std::tuple<typename defs::rightMotor1, typename defs::rightMotor2, typename defs::leftMotor1, typename defs::leftMotor2>;

            // internal transition
            void internal_transition() {
              state.prop = false;
              //if(state.tagType ==1 )
                //state.tagType = 2;
              //Do nothing... 
            }

            // external transition
            void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) { 
              float light = 0;
              DriveState oldState = state.dir;
              // Note: This will search the message bags for each port and store only the LAST value in the state variable.
              // Saving the inputs in a state variable is required since not all sensors are update at the same time.
              // For example, if a new rightIR reading comes through we need to know the last center and left IR readings 
              for(const auto &x : get_messages<typename defs::command>(mbs)){
          

              // to make the drive direction decision.
              for(const auto &x : get_messages<typename defs::rightIR>(mbs)){
                state.rightIR = !x;
              }
              for(const auto &x : get_messages<typename defs::middleRightIR>(mbs)){
                state.middleRightIR = !x;
              }
              for(const auto &x : get_messages<typename defs::centerIR>(mbs)){
                state.centerIR = !x;
              }
              for(const auto &x : get_messages<typename defs::leftIR>(mbs)){
                state.leftIR = !x;
              }
              for(const auto &x : get_messages<typename defs::middleLeftIR>(mbs)){
                state.middleLeftIR = !x;
              }
              #ifdef SCARED_OF_THE_DARK
              for(const auto &x : get_messages<typename defs::lightSensor>(mbs)){
                light = x;
              }
              #endif
              
              if( (!state.rightIR && !state.leftIR && !state.centerIR && !state.middleRightIR && !state.middleLeftIR) || (state.rightIR && state.leftIR && state.middleRightIR && state.middleLeftIR && 			state.centerIR)) {
                // This happens when no IR sensors see the line and if two or more IR sensors see the line.
                state.dir = DriveState::stop;
              }else if (state.middleRightIR && state.centerIR) {
                state.dir = DriveState::slightLeft;
              } 
              else if (state.middleLeftIR && state.centerIR) {
                state.dir = DriveState::slightRight;
              } else if (state.centerIR) {
                state.dir = DriveState::straight;
              }
              else if (state.rightIR) {
                state.dir = DriveState::left;
              } else if (state.leftIR) {
                state.dir = DriveState::right;
              }
              else if (state.middleLeftIR) {
                state.dir = DriveState::slightRight;
              } 
              else if (state.middleRightIR) {
                state.dir = DriveState::slightLeft;
              } 
                else {
                state.dir = DriveState::straight;
              }
              #ifdef SCARED_OF_THE_DARK
              if (light < 0.3) {
                state.dir = DriveState::stop;
              }
              #endif
              // If the state changes, force an output.
              state.prop = true; //state.dir == oldState;
            }

            // confluence transition
            void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
              internal_transition();
              external_transition(TIME(), std::move(mbs));
            }

            // output function
            typename make_message_bags<output_ports>::type output() const {
              typename make_message_bags<output_ports>::type bags;
              float rightMotorOut1;
              bool rightMotorOut2;
              float leftMotorOut1;
              bool leftMotorOut2;  

              if(state.direction == 100 )
              {
                rightMotorOut1 = 0;
                  rightMotorOut2 = 0;
                  leftMotorOut1 = 0;
                  leftMotorOut2 = 0;
              }
              else if(state.direction == 200 )
              {
                rightMotorOut1 = 0.25;
                rightMotorOut2 = 0;
                leftMotorOut1 = 0;
                leftMotorOut2 = 0; 
              }
                 
              else
              {
                switch(state.dir){
                  case DriveState::right:
                    rightMotorOut1 = 0.3;
                    rightMotorOut2 = 0;
                    leftMotorOut1 = 0;
                    leftMotorOut2 = 0;                
                  break;
                  case DriveState::slightRight:
                    rightMotorOut1 = 0.2;
                    rightMotorOut2 = 0;
                    leftMotorOut1 = 0.1;
                    leftMotorOut2 = 0;                
                  break;

                  case DriveState::left:
                    rightMotorOut1 = 0;
                    rightMotorOut2 = 0;
                    leftMotorOut1 = 0.3;
                    leftMotorOut2 = 0;
                  break;
                  case DriveState::slightLeft:
                    rightMotorOut1 = 0.1;
                    rightMotorOut2 = 0;
                    leftMotorOut1 = 0.2;
                    leftMotorOut2 = 0;
                  break;

                  case DriveState::straight:
                    rightMotorOut1 = 0.2;
                    rightMotorOut2 = 0;
                    leftMotorOut1 = 0.2;
                    leftMotorOut2 = 0;
                  break;

                  case DriveState::stop:
                  default:
                    rightMotorOut1 = 0;
                    rightMotorOut2 = 0;
                    leftMotorOut1 = 0;
                    leftMotorOut2 = 0;
                  break;
                }
              }


              get_messages<typename defs::rightMotor1>(bags).push_back(rightMotorOut1);
              get_messages<typename defs::rightMotor2>(bags).push_back(rightMotorOut2);
              get_messages<typename defs::leftMotor1>(bags).push_back(leftMotorOut1);
              get_messages<typename defs::leftMotor2>(bags).push_back(leftMotorOut2);
                
              return bags;
            }

            // time_advance function
            TIME time_advance() const { 
              if(state.prop)
                return TIME("00:00:00");


              return std::numeric_limits<TIME>::infinity();
            }

            friend std::ostringstream& operator<<(std::ostringstream& os, const typename SeeedBotDriver<TIME>::state_type& i) {
              os << "Current state: " << i.dir; 
              return os;
            }
        };     


#endif // BOOST_SIMULATION_PDEVS_BLINKY_HPP
