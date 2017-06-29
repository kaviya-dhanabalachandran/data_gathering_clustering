
/*
 * AUTHOR: Kaviya
 *
 * This controller lets a set of agents to move to interesting regions to collect necessary data
 *
 * This controller is meant to be used with the XML file:
 *    xml/footbot_taskAllocation_Clustering.xml
 */

#ifndef _FOOTBOTMISAGENT_H_
#define _FOOTBOTMISAGENT_H_

#include <iostream>
#include <fstream>
/* Definition of the CCI_Controller class. */
#include <argos2/common/control_interface/ci_controller.h>
#include <argos2/common/utility/logging/argos_log.h>

#include <argos2/common/utility/argos_random.h>
#include <argos2/common/utility/datatypes/color.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_beacon_actuator.h>

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <math.h>
#include <unistd.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <argos2/common/utility/datatypes/datatypes.h>
#include <navigation/client/nav_client.h>
#include <fstream>
#include <deque>


#define PI 3.14159265

using namespace argos;
using namespace std;


#define MAX_UDP_SOCKET_BUFFER_SIZE 1500


/*
 * This mission agent controller is  an implementation of the CCI_Controller class.
 */
class FootbotMissionAgents: public CCI_Controller
{
 
 public:
    
    struct Position
    {
        float x;
        float y;
    };

    struct dataWrite
    {
      ofstream data_file;
      string filename;
    };


    struct SStateData {
      /* Agent State */
      enum EState {
    	  /* state in which the agent collects data */
         STATE_RESTING = 0,
		 /* states in which the agent moves to interesting regions */
         STATE_EXPLORING
      } State;

      /* Identifier of received data */
      enum ReceivedDataType {
    	 /* Identifier of Hello message from relay */
         RELAY_HELLO_MESSAGE = 0,
		 /* Identifier of acceptance to send collected data from relay */
         RELAY_ACCEPTANCE_FOR_DATA
      } ReceivedData;

      /* Identifier of Sending data */
      enum SentDataType {
    	/* Identifier of Profile message to relay */
        PROFILE_DATA = 0,
		/* Identifier of Collected data to relay */
        COLLECTED_DATA
      } SentData;

      /* Structure Constructor */
      SStateData();

      /* Object of navigation state of the robot. To check if the agent has reached it's target position */
      RobotNavState target_state;
      float wait_time;

      /* Current goal position */
      Position goal_loc;
      /* Current position */
      Position current_loc;
      
      uint8_t id;
      /* Size of data available to be sent to relay */
      uint64_t size_of_data_available;
      uint32_t ttl;
      /* map with data generation time as key and size of data generated as value */
      map<uint64_t, uint64_t> data_generatedTimeSizeMap;
      vector<uint64_t> size_of_data_sent;

      std::vector< std::vector<int> > dataGenerationTime;
      std::vector< std::vector<float> > randomPos; 
    };

    struct SRelayData{
      /* Information of relay visiting the agent */

      /* Relay Id*/
      uint8_t id;
      /* Time profile data sent to relay */
      uint64_t time_profile_data_sent;
      /* Time collected data sent to relay */
      uint64_t time_gathered_data_sent;
      SRelayData();
    };


 private:
    UInt32 RandomSeed;
    std::string m_MyIdStr;
    UInt64 m_Steps;
    CARGoSRandom::CRNG* m_randomGen;
    UInt64 m_sendPackets;
    UInt8 m_myID;
    float speed;

    /* Pointer to navigation*/
    RVONavClient *m_navClient;

    /* Pointer to the wifi sensor */
    CCI_WiFiSensor* m_pcWifiSensor;
    /* Pointer to the wifi actuator */
    CCI_WiFiActuator* m_pcWifiActuator;

    CCI_FootBotLedsActuator* m_pcLEDs;
    

    SStateData stateData;
    SRelayData relayData;
    map<uint8_t, SRelayData> relayMap;

    int numberOfFutureTargetsToBeSent;
    vector<Position> targetPositions;

  public:

    /* Class constructor. */
    FootbotMissionAgents();

    /* Class destructor. */
    virtual ~FootbotMissionAgents() {
    }

    /*
     * This function initializes the controller.
     * The 't_tree' variable points to the <parameters> section in the XML
     * file in the controllers/footbot_mission_agent_controller section.
     */
    virtual void Init(TConfigurationNode& t_tree);

    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    virtual void ControlStep();
    virtual void Destroy();
    virtual bool IsControllerFinished() const;
    
    void updateState();

    /*
     * This function is called when the agent is collecting data
     */
    void Rest();

    /*
     * This function is called when the agent is moving to an
     * interesting region for data gathering
     */
    void Explore();

    CVector3 randomWaypoint();

    static std::string getTimeStr();
    UInt64 getTime();
    uint32_t getTimeLimit(float,float);

    void initialiseAgentData(TConfigurationNode& t_node, int numberOfAgents, int goalN);
    
    void generateData();

    /* this function is called when a data is sent to a relay
     * Variable type_of_message
     */
    void SendData(uint8_t type_of_message, uint8_t relay_id);
    void ParseMessage(uint8_t id, std::vector<char> &v);

    void ParseRelayMessage(vector<char> &incoming_agent_message);
    void ParseRelayAcceptance(vector<char> &incoming_agent_message);
    
    // This is used for caculating fairness in meeting agents and for how many agents were met by relay in 'X' timesteps
    dataWrite timeStepsWhenRelayMet;
    dataWrite agentPositions;

    //dataWrite dataInformation;
    // ParseNeighborData
    size_t SendProfileData(char* ptr, uint8_t id, uint8_t relay_id);
    size_t SendCollectedData(char* ptr, uint8_t id, uint8_t relay_id);
};
#endif
