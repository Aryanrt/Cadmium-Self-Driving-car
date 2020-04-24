/**
* Aryan Rashidi-Tarbrizi - Carleton University
*
* Analog Input:
* This main file constructs the do simple line following project, using a RFID, Tranmitter and Seed Bot Shield.
* Its purpose is to demonstrate how to use all of the port IO models in RT_ARM_MBED.
*/

#include <iostream>
#include <chrono>
#include <algorithm>
#include <string>

// Allow a 10ms slip in the real time clock if defined. 
// If the slip is more 10ms the device will enter a failed state.
// #define MISSED_DEADLINE_TOLERANCE 10000

#include <cadmium/modeling/coupling.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/concept/coupled_model_assert.hpp>
#include <cadmium/modeling/dynamic_coupled.hpp>
#include <cadmium/modeling/dynamic_atomic.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/tuple_to_ostream.hpp>
#include <cadmium/logger/common_loggers.hpp>
#include <cadmium/io/iestream.hpp>


#include <NDTime.hpp>

#include <cadmium/real_time/arm_mbed/io/digitalInput.hpp>
#include <cadmium/real_time/arm_mbed/io/analogInput.hpp>
#include <cadmium/real_time/arm_mbed/io/pwmOutput.hpp>
#include <cadmium/real_time/arm_mbed/io/digitalOutput.hpp>

#include "../atomics/seeedBotDriver.hpp"
#include "../atomics/rfid.hpp"
#include "../atomics/transmitter.hpp"

#ifdef RT_ARM_MBED
  #include "../mbed.h"
#else
  const char* A0  = "./inputs/A0_RightIR_In.txt";
  const char* A2  = "./inputs/A2_CenterIR_In.txt";
  const char* D4  = "./inputs/D4_LeftIR_In.txt";
  const char* A5  = "./inputs/A5_LightSensor_In.txt";
  const char* D8  = "./outputs/D8_RightMotor1_Out.txt";
  const char* D11 = "./outputs/D11_RightMotor2_Out.txt";
  const char* D12 = "./outputs/D12_LeftMotor1_Out.txt";
  const char* D13 = "./outputs/D13_LeftMotor2_Out.txt";
#endif

// SCARED OF THE DARK definition is for the analog sensor demo.
// If the analog sensor is not present then this definition should be commented out.
// #define SCARED_OF_THE_DARK

using namespace std;

using hclock=chrono::high_resolution_clock;
using TIME = NDTime;

int main(int argc, char ** argv) {

  //This will end the main thread and create a new one with more stack.
  #ifdef RT_ARM_MBED
    //Logging is done over cout in RT_ARM_MBED
    struct oss_sink_provider{
      static std::ostream& sink(){
        return cout;
      }
    };
  #else
    // all simulation timing and I/O streams are ommited when running embedded

    auto start = hclock::now(); //to measure simulation execution time

    /*************** Loggers *******************/

    static std::ofstream out_data("seeed_bot_test_output.txt");
    struct oss_sink_provider{
      static std::ostream& sink(){
        return out_data;
      }
    };
  #endif

  using info=cadmium::logger::logger<cadmium::logger::logger_info, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using debug=cadmium::logger::logger<cadmium::logger::logger_debug, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using state=cadmium::logger::logger<cadmium::logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using log_messages=cadmium::logger::logger<cadmium::logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using routing=cadmium::logger::logger<cadmium::logger::logger_message_routing, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using global_time=cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using local_time=cadmium::logger::logger<cadmium::logger::logger_local_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using log_all=cadmium::logger::multilogger<info, debug, state, log_messages, routing, global_time, local_time>;
  using logger_top=cadmium::logger::multilogger<log_messages, global_time>;



/********************************************/
/*********** APPLICATION GENERATOR **********/
/********************************************/
  using AtomicModelPtr=std::shared_ptr<cadmium::dynamic::modeling::model>;
  using CoupledModelPtr=std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>>;

/********************************************/
/********** SeedBotDriver *******************/
/********************************************/

  AtomicModelPtr seeedBotDriver = cadmium::dynamic::translate::make_dynamic_atomic_model<SeeedBotDriver, TIME>("seeedBotDriver");

/********************************************/
/********** RFID *******************/
/********************************************/
 AtomicModelPtr transmitter1 = cadmium::dynamic::translate::make_dynamic_atomic_model<transmitter, TIME>("transmitter1", PC_12, PC_11, PC_10, PC_9,D14,D9);
 AtomicModelPtr rfid1 = cadmium::dynamic::translate::make_dynamic_atomic_model<Rfid, TIME>("rfid1", PB_15, PB_14, D6, D7, D15);

/********************************************/
/****************** Input *******************/
/********************************************/
  AtomicModelPtr rightIR = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalInput, TIME>("rightIR" , A0, TIME("00:00:00:100"));
  AtomicModelPtr middleRightIR = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalInput, TIME>("middleRightIR" , A1, TIME("00:00:00:100"));
  AtomicModelPtr centerIR = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalInput, TIME>("centerIR", A2, TIME("00:00:00:100"));
  AtomicModelPtr middleLeftIR = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalInput, TIME>("middleLeftIR", A3, TIME("00:00:00:100"));
  AtomicModelPtr leftIR = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalInput, TIME>("leftIR", D4, TIME("00:00:00:100"));

  #ifdef SCARED_OF_THE_DARK
  AtomicModelPtr lightSensor = cadmium::dynamic::translate::make_dynamic_atomic_model<AnalogInput, TIME>("lightSensor", A5, TIME("00:00:00:100"));
  #endif
/********************************************/
/***************** Output *******************/
/********************************************/
  AtomicModelPtr rightMotor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<PwmOutput, TIME>("rightMotor1", D8);
  AtomicModelPtr rightMotor2 = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalOutput, TIME>("rightMotor2", D11);
  AtomicModelPtr leftMotor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<PwmOutput, TIME>("leftMotor1", D12);
  AtomicModelPtr leftMotor2 = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalOutput, TIME>("leftMotor2", D13);


/************************/
/*******TOP MODEL********/
/************************/
  cadmium::dynamic::modeling::Ports iports_TOP = {};
  cadmium::dynamic::modeling::Ports oports_TOP = {};
  #ifdef SCARED_OF_THE_DARK
  cadmium::dynamic::modeling::Models submodels_TOP =  {lightSensor, seeedBotDriver, rightIR, middleRightIR, centerIR, leftIR, middleLeftIR, rfid1, transmitter1, rightMotor1, rightMotor2, leftMotor1, leftMotor2};
  #else
  cadmium::dynamic::modeling::Models submodels_TOP =  {seeedBotDriver, rfid1, transmitter1, rightIR, middleRightIR, centerIR, leftIR, middleLeftIR, rightMotor1, rightMotor2, leftMotor1, leftMotor2};
  #endif
  cadmium::dynamic::modeling::EICs eics_TOP = {};
  cadmium::dynamic::modeling::EOCs eocs_TOP = {};
  cadmium::dynamic::modeling::ICs ics_TOP = {
     cadmium::dynamic::translate::make_IC<seeedBotDriver_defs::rightMotor1, pwmOutput_defs::in>("seeedBotDriver","rightMotor1"),
     cadmium::dynamic::translate::make_IC<seeedBotDriver_defs::rightMotor2, digitalOutput_defs::in>("seeedBotDriver","rightMotor2"),
     cadmium::dynamic::translate::make_IC<seeedBotDriver_defs::leftMotor1, pwmOutput_defs::in>("seeedBotDriver","leftMotor1"),
     cadmium::dynamic::translate::make_IC<seeedBotDriver_defs::leftMotor2, digitalOutput_defs::in>("seeedBotDriver","leftMotor2"),
     #ifdef SCARED_OF_THE_DARK
     cadmium::dynamic::translate::make_IC<analogInput_defs::out, seeedBotDriver_defs::lightSensor>("lightSensor", "seeedBotDriver"),
     #endif
     cadmium::dynamic::translate::make_IC<digitalInput_defs::out, seeedBotDriver_defs::rightIR>("rightIR", "seeedBotDriver"),
     cadmium::dynamic::translate::make_IC<digitalInput_defs::out, seeedBotDriver_defs::middleRightIR>("middleRightIR", "seeedBotDriver"),
     cadmium::dynamic::translate::make_IC<digitalInput_defs::out, seeedBotDriver_defs::leftIR>("leftIR", "seeedBotDriver"),
     cadmium::dynamic::translate::make_IC<digitalInput_defs::out, seeedBotDriver_defs::middleLeftIR>("middleLeftIR", "seeedBotDriver"),
     cadmium::dynamic::translate::make_IC<digitalInput_defs::out, seeedBotDriver_defs::centerIR>("centerIR", "seeedBotDriver"),
         cadmium::dynamic::translate::make_IC<rfid_defs::dataOut, transmitter_defs::in>("rfid1","transmitter1")//,
  };
  CoupledModelPtr TOP = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
   "TOP",
   submodels_TOP,
   iports_TOP,
   oports_TOP,
   eics_TOP,
   eocs_TOP,
   ics_TOP
   );

///****************////

  #ifdef RT_ARM_MBED
    //Enable the motors:
    DigitalOut rightMotorEn(D9);
    DigitalOut leftMotorEn(D10);
    rightMotorEn = 1;
    leftMotorEn = 1;
  #endif

  // Logs are currently blocking opperations. It is recommended to turn them off when embedding your application.
  // They can be used for testing; however, keep in mind they will add extra delay to your model.
  cadmium::dynamic::engine::runner<NDTime, cadmium::logger::not_logger> r(TOP, {0});
  //cadmium::dynamic::engine::runner<NDTime, log_all> r(TOP, {0});
  r.run_until(NDTime("100:00:00:000"));

  #ifndef RT_ARM_MBED
    return 0;
  #endif
}
