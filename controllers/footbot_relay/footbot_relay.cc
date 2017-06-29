#include "footbot_relay.h"


#ifndef FOOTBOT_SIM
#else
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/simulator.h>
#endif


#define MAX_UDP_SOCKET_BUFFER_SIZE 1500

#define __USE_DEBUG_COMM 1
#if __USE_DEBUG_COMM
#define DEBUGCOMM(m, ...) \
{\
  fprintf(stderr, "%.2f DEBUGCOMM[%d]: " m,\
    (float) m_Steps,\
    (int) m_myID, \
          ## __VA_ARGS__);\
  fflush(stderr);\
}
#else
#define DEBUGCOMM(m, ...)
#endif


FootbotRelay::FootbotRelay():
  RandomSeed(12345),
  m_Steps(0),
  target_state(STATE_ARRIVED_AT_TARGET),
  search_time(0),
  clusteringAllowed(true),
  clusteringDone(false),
  timeInDataGather(0),
  datasizeToBaseStation(0),
  timeInSearch(0)
  {}



FootbotRelay::SStateData::SStateData()
{
  IsGoalSet = false;
  //IstargetAgentSet = false;
  IsAgentDetected = false; 
  collected_data_size = 0;
  State = SStateData::STATE_NOGOAL;
  SearchState = SStateData::To_AGENT;
  MovingToBaseStation = false;
  visited_baseStation = false;
  agentfound  = false;
  IsDataSentToBaseStation = false; // Data sent to base station initially false
  /****Calculate time limit and min data size ****/
  allCompleted = true;
}


FootbotRelay::SAgentData::SAgentData()
{
  IsGoalSet = true;
  IsDataReceived = false;
  delay_time = 0;
  delay_in_collection = 0;
  time_last_visited = 0;
  time_last_data_collected = 0;
  transmitted_data_time = 0;
  transmitted_data_size = 0;
}

void
FootbotRelay::SStateData::Init(TConfigurationNode& t_node)
{
  
  printf(" number of base station  %d \n", NUMBER_OF_BASESTATION);
  Position loc; 
  
  //NUMBER_OF_BASESTATION ;
  for(int i=0; i< NUMBER_OF_BASESTATION; i++)
  { 
    
    ostringstream temp_str; 

    temp_str << "base_station" <<  i+1;
  
    GetNodeAttribute(GetNode(t_node, temp_str.str()), "x", loc.x);
    //cout << loc.x << endl;
    GetNodeAttribute(GetNode(t_node, temp_str.str()), "y", loc.y);
    //cout << loc.y << endl;
    
    Position tempB;
    if(loc.x + 0.5 > 24)
    {
      tempB.x = loc.x -0.9;
      tempB.y = loc.y -0.9;
    }
    else
    {
      tempB.x = loc.x +0.9;
      tempB.y = loc.y +0.9;
    }
    base_station.emplace(i+1,loc);  
    
  }

}




void 
FootbotRelay::Init(TConfigurationNode& t_node) 
{ 
  

  /// The first thing to do, set my ID
#ifdef FOOTBOT_SIM
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(3).c_str());
#else
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(7).c_str());
#endif
  
  printf("MyID %d\n", m_myID);

  /// Random
  GetNodeAttributeOrDefault(t_node, "RandomSeed", RandomSeed, RandomSeed);

  string temp;
  GetNodeText(GetNode(t_node, "destinationAreaX"), temp);
  size_x = atof(temp.c_str());

  GetNodeText(GetNode(t_node, "destinationAreaY"),temp);
  size_y = atof(temp.c_str());

  GetNodeText(GetNode(t_node, "numberofrelay"), temp);
  stateData.NUMBER_OF_RELAY = atoi(temp.c_str());

  GetNodeText(GetNode(t_node, "numberofBS"), temp);
  stateData.NUMBER_OF_BASESTATION = atoi(temp.c_str());

  printf("Number of base_station: %d \n ", stateData.NUMBER_OF_BASESTATION);
  
  GetNodeText(GetNode(t_node, "numberofAgent"), temp);
  stateData.NUMBER_OF_AGENT = atoi(temp.c_str());

  GetNodeText(GetNode(t_node, "NumberOfGoalKnown"), temp);
  numberOfFutureTargetPositionsKnown = atoi(temp.c_str());
  
  //prob(Prob(size_x,size_y));

  string speed_string;
  if (NodeExists(t_node, "optimalSpeed")) 
  {
    GetNodeText(GetNode(t_node, "optimalSpeed"), speed_string);
    sscanf(speed_string.c_str(), "%f", &speed);
  }
  
      
    
  if( m_randomGen == NULL )
    {
      CARGoSRandom::CreateCategory("rwp",
           RandomSeed+m_myID);
      m_randomGen = CSimulator::GetInstance().GetRNG();
    }


  //////////////////////////////////////////////////////////////
  // Initialize things required for communications
  //////////////////////////////////////////////////////////////
  m_pcWifiSensor = dynamic_cast<CCI_WiFiSensor* >(GetRobot().GetSensor("wifi"));
  m_pcWifiActuator = dynamic_cast<CCI_WiFiActuator* >(GetRobot().GetActuator("wifi"));

  //  m_pcWifiSensorLongRange = dynamic_cast<CCI_WiFiSensor* >(GetRobot().GetSensor("wifilongrange"));
  //  m_pcWifiActuatorLongRange = dynamic_cast<CCI_WiFiActuator* >(GetRobot().GetActuator("wifilongrange"));

  //Led actuator
  m_pcLEDs   = dynamic_cast<CCI_FootBotLedsActuator*>(GetRobot().GetActuator("footbot_leds"));
   
  /// create the client and pass the configuration tree (XML) to it
  m_navClient = new RVONavClient(m_myID, GetRobot());
  m_navClient->init(t_node);
 
  /// start the navigation client
  m_navClient->start(); 
  m_pcLEDs->SetAllColors(CColor::MAGENTA); 
  
  int NumberOfAgents = stateData.NUMBER_OF_AGENT;
  
  printf("Initialising agent and base station positions \n");
  /****Initialising Agent and Base Station positions ****/
  stateData.Init(GetNode(t_node, "state"));

  initialiseAgentData(GetNode(t_node, "agent"),NumberOfAgents, numberOfFutureTargetPositionsKnown);

  printf("initialising files \n");
 
  // Records Positions of Agents
  relayPositions.filename = "outputFiles"+to_string(stateData.NUMBER_OF_RELAY)+"/position" + to_string(m_myID)+".csv";
  relayPositions.data_file.open(relayPositions.filename, ios::out | ios::ate);
  relayPositions.data_file << "x" << "," << "y" << "\n";

 // Records the time and position at which a relay meets the agent
  timeStepToMeet.filename = "outputFiles"+to_string(stateData.NUMBER_OF_RELAY)+"/timestep"+ to_string(m_myID)+".csv";
  timeStepToMeet.data_file.open(timeStepToMeet.filename, ios::out | ios::ate);
  timeStepToMeet.data_file << "TimeWhenMet" << "," << "x" << "," << "y" << "," << "AgentId" << "\n";

  delayTime.filename = "outputFiles"+to_string(stateData.NUMBER_OF_RELAY)+"/delayTime"+ to_string(m_myID)+".csv";
  delayTime.data_file.open(delayTime.filename, ios::out | ios::ate);
  delayTime.data_file << "Agents" << "," << "DelayTime" << "\n";

  clusterData.filename = "outputFiles"+to_string(stateData.NUMBER_OF_RELAY)+"/cluster" + to_string(m_myID)+".csv";
  clusterData.data_file.open(clusterData.filename, ios::out | ios::ate);

  timeForACycle.filename = "outputFiles"+to_string(stateData.NUMBER_OF_RELAY)+"/CycleTime"+ to_string(m_myID)+".csv";
  timeForACycle.data_file.open(timeForACycle.filename, ios::out | ios::ate);
  timeForACycle.data_file << "Start" << "," << "End" << "\n";
  timeForACycle.data_file << 5 << ",";

  uint8_t relayBeginningId = (uint8_t)(stateData.NUMBER_OF_AGENT +stateData.NUMBER_OF_BASESTATION +1);
  
  

  for(int i= 0; i< stateData.NUMBER_OF_RELAY; i++)
  {
    relayIdMap.emplace(relayBeginningId,i);

    if(relayBeginningId != m_myID)
    { 
      stateData.RelaysReachedBaseStation.insert(relayBeginningId);
      relayIdList.push_back(relayBeginningId);
    }
  
    relayBeginningId = relayBeginningId + 1;

  }

  for(int i= 0; i < (int)relayIdMap[m_myID]; i++)
  { 
    stateData.parents.push_back(relayIdList[i]);
  }
        
  

  std::sort(relayIdList.begin(), relayIdList.end());
  std::sort(stateData.parents.begin(), stateData.parents.end());

  std::set_difference(relayIdList.begin(), relayIdList.end(), stateData.parents.begin(), stateData.parents.end(), std::back_inserter(stateData.children));
  
  //if(stateData.children.size() > 0)
    //printf("children %d \n", (int)stateData.children[0]);
}




void
FootbotRelay::initialiseAgentData(TConfigurationNode& t_node, int NumberOfAgents, int NumberOfGoal)
{
  
  Position loc; 
  printf("number agetns %d\n", NumberOfAgents);
  //NUMBER_OF_BASESTATION ;
  for(int i=0; i< NumberOfAgents; i++)
  {  
    std::shared_ptr<SAgentData> agentTemp(new SAgentData());

    ostringstream temp_str;
    temp_str << "agent" << i+1;

    GetNodeAttribute(GetNode(t_node, temp_str.str()), "id",  agentTemp->id);
   
    GetNodeAttribute(GetNode(t_node, temp_str.str()), "x", agentTemp->current_location.x);
    GetNodeAttribute(GetNode(t_node, temp_str.str()), "y", agentTemp->current_location.y);
    
    for(int j= 1; j <= NumberOfGoal; j++)
    { 
      ostringstream posX;
      ostringstream posY;

      posX << "goal" << j << "x";
      posY << "goal" << j << "y";

      Position goal;
      GetNodeAttribute(GetNode(t_node, temp_str.str()), posX.str(), goal.x);
      GetNodeAttribute(GetNode(t_node, temp_str.str()), posY.str(), goal.y);

      agentTemp->goalLocationList.emplace_back(goal);
    }
    agentIdMap.emplace(i,agentTemp->id);
    agentIdList.push_back(agentTemp->id);

    agentTemp->goal_location  =  agentTemp->goalLocationList[0];
    
    agentMap.emplace(agentTemp->id,agentTemp);
    
  
    prob.initialise(agentTemp->id,agentTemp->current_location,agentTemp->goalLocationList);
    printf("Initialised all agents \n");
  }
  
 agentIdToConsider = agentIdList;
}



size_t 
FootbotRelay::HelloToAgent(uint8_t identifier, char* out_to_agent)
{  
  /*** This message is sent to detect agent in range ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);
  long unsigned int final_address;
    try
    {
      /// identifier - 0 relay to agent hello message 
      //uint8_t identifier = 0; 
      printf("Size of character %d\n", sizeof(identifier));
      memcpy(out_to_agent,&identifier,sizeof(identifier));
      out_to_agent = out_to_agent + sizeof(identifier);
      
      uint8_t id = (uint8_t)m_myID;
      memcpy(out_to_agent,&id,sizeof(id));
      out_to_agent = out_to_agent + sizeof(id);
      
      final_address = (long unsigned int)&(*out_to_agent);
      
    }
    catch(exception& e)
    {
      printf("Exception: %s\n",e.what());
    }
    printf("message created \n");
  return size_t(final_address-initial_address);
}

size_t 
FootbotRelay::AcceptanceToAgent(uint8_t identifier,char* out_to_agent)
{
  /*** This message is sent as acceptance to collect data from agent in range ***/
  printf("Creating message to request data from agent\n");
  long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);

  //uint8_t identifier = 1;
  memcpy(out_to_agent,&identifier,sizeof(identifier));
  out_to_agent += sizeof(identifier);

  uint8_t id_relay = m_myID;
  memcpy(out_to_agent,&id_relay,sizeof(id_relay));
  out_to_agent+= sizeof(id_relay);
    
  long unsigned int final_address =  (long unsigned int)&(*out_to_agent);

  return size_t(final_address-initial_address);

}

size_t
FootbotRelay::ToBaseStation(uint8_t identifier,char* data_to_basestation_ptr)
{
  long unsigned int initial_address =  (long unsigned int)&(*data_to_basestation_ptr);
  printf("Creating message to send data to base station\n");

  uint8_t messId = identifier;
  memcpy(data_to_basestation_ptr, &messId, sizeof(messId));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(messId);
  printf("Identifier data to base station %d\n", messId);


  uint8_t relay_id = m_myID;
  memcpy(data_to_basestation_ptr, &relay_id, sizeof(relay_id));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(relay_id);
  printf("Relay Id %d\n", (int)relay_id);

  uint8_t numberOfAgents = agentToVisit.size();
  memcpy(data_to_basestation_ptr, &numberOfAgents, sizeof(numberOfAgents));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(numberOfAgents);
  printf("number of agents %d\n", (int)numberOfAgents);

  for(auto &agentId:agentToVisit)
  {
      uint8_t id = agentMap[agentId]->id;
      memcpy(data_to_basestation_ptr, &id, sizeof(id));
      data_to_basestation_ptr = data_to_basestation_ptr + sizeof(id);
      printf("Agent Id:  %d\n", id);

      uint64_t time_last_data_collected = agentMap[agentId]->transmitted_data_time;
      memcpy(data_to_basestation_ptr, &time_last_data_collected, sizeof(time_last_data_collected));
      data_to_basestation_ptr = data_to_basestation_ptr + sizeof(time_last_data_collected);
      printf("Time Message Sent:  %d\n", time_last_data_collected);

      uint32_t agent_message_size = agentMap[agentId]->transmitted_data_size;
      memcpy(data_to_basestation_ptr, &agent_message_size, sizeof(agent_message_size));
      data_to_basestation_ptr = data_to_basestation_ptr + sizeof(agent_message_size);
      printf("Agent Message Size:  %d\n", agent_message_size);
      agentMap[agentId]->transmitted_data_size = 0;

      // delay time considers the time between the data received to the relay and delivered to base station
      printf("Agent ID %d delay in gathering data %d \n", (int)agentId,agentMap[agentId]->delay_in_collection);
      printf("Delay in deliverign data %d \n", (m_Steps-agentMap[agentId]->transmitted_data_time));

      agentMap[agentId]->delay_time  = 0;

      if(agent_message_size != 0)
      {
        agentMap[agentId]->delay_time = (m_Steps-agentMap[agentId]->transmitted_data_time) + agentMap[agentId]->delay_in_collection;
      }
      
      printf("Total Delay Time: %d\n", agentMap[agentId]->delay_time);
      
      delayTime.data_file << agentMap[agentId]->id << "," << agentMap[agentId]->delay_time << "\n";
  }

  
  
  long unsigned int final_address =  (long unsigned int)&(*data_to_basestation_ptr);
  uint32_t data_to_BS_size = (final_address-initial_address); 
  
  /// 8.Message size (4)
  memcpy(data_to_basestation_ptr, &data_to_BS_size,sizeof(data_to_BS_size));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(data_to_BS_size);
  data_to_BS_size = data_to_BS_size + sizeof(data_to_BS_size);
  
  printf("%d message_size \n", data_to_BS_size);
  stateData.IsDataSentToBaseStation = true;

  return size_t(data_to_BS_size);
}


size_t 
FootbotRelay::TaskAssignedToRelay(uint8_t identifier, char* out_to_relay)
{  
  /*** This message is sent to other relays to announce the agents assigned  ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    

  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  
  //number of agents assigned
  uint8_t numberAssigned = (uint8_t)agentToVisit.size();
  memcpy(out_to_relay,&numberAssigned,sizeof(numberAssigned));
  out_to_relay = out_to_relay + sizeof(numberAssigned);
  
  //Agents Assigned
  for(int i= 0 ; i < numberAssigned; i++)
  {
    uint8_t agId = (uint8_t)agentToVisit[i];
    memcpy(out_to_relay,&agId,sizeof(agId));
    out_to_relay = out_to_relay + sizeof(agId);
  }

  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}


size_t 
FootbotRelay::NewDataInformationToRelay(uint8_t identifier, char* out_to_relay)
{  
  /*** This message is sent  to other relays to announce agents met and their information ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    
  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  
  //Recently visited agent's info
  uint8_t agId = (uint8_t)stateData.recentlyProfileDataCollectedId;
  memcpy(out_to_relay,&agId,sizeof(agId));
  out_to_relay = out_to_relay + sizeof(agId);

  stateData.recentlyProfileDataCollectedId = 0;
  
  // current Location
  float curr_x = agentMap[agId]->current_location.x;
  memcpy(out_to_relay,&curr_x,sizeof(curr_x));
  out_to_relay = out_to_relay + sizeof(curr_x);

  float curr_y = agentMap[agId]->current_location.y;
  memcpy(out_to_relay,&curr_y,sizeof(curr_y));
  out_to_relay = out_to_relay + sizeof(curr_y);

  //current goal 
  float goal_x = agentMap[agId]->goal_location.x;
  memcpy(out_to_relay,&goal_x,sizeof(goal_x));
  out_to_relay = out_to_relay + sizeof(goal_x);

  float goal_y = agentMap[agId]->goal_location.y;
  memcpy(out_to_relay,&goal_y,sizeof(goal_y));
  out_to_relay = out_to_relay + sizeof(goal_y);
 
  // goal Location list
  for(int i=0; i < numberOfFutureTargetPositionsKnown; i++)
  {
      //future goal 
    float fgoal_x = agentMap[agId]->goalLocationList[i].x;
    memcpy(out_to_relay,&fgoal_x,sizeof(fgoal_x));
    out_to_relay = out_to_relay + sizeof(fgoal_x);

    float fgoal_y = agentMap[agId]->goalLocationList[i].y;
    memcpy(out_to_relay,&fgoal_y,sizeof(fgoal_y));
    out_to_relay = out_to_relay + sizeof(fgoal_y);
  }
  
  

  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}

size_t 
FootbotRelay::SendingTransmittedDataTimeToRelay(uint8_t identifier, char* out_to_relay)
{

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    
  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  
 //Recently visited agent's info
  uint8_t agId = (uint8_t)stateData.recentlyVisitedAgentId;
  memcpy(out_to_relay,&agId,sizeof(agId));
  out_to_relay = out_to_relay + sizeof(agId);

  stateData.recentlyVisitedAgentId = 0;

  // time when data gathered
  uint64_t dataReceivedTime = agentMap[agId]->transmitted_data_time;
  memcpy(out_to_relay,&dataReceivedTime,sizeof(dataReceivedTime));
  out_to_relay = out_to_relay + sizeof(dataReceivedTime);

  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}


size_t 
FootbotRelay::AnnouncingTaskcompletion(uint8_t identifier, char* out_to_relay)
{  
  /*** This message is sent  to other relays to announce agents met and their information ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    
  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  

  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}

void 
FootbotRelay::ParseAgentProfile(vector<char> &incoming_agent_message)
{   

    char* agent_mes_ptr = (char*)&incoming_agent_message[0];
    

    uint8_t mes_type = (uint8_t)agent_mes_ptr[0];
    agent_mes_ptr = agent_mes_ptr + sizeof(mes_type);

    //neighbour_count = neighbour_count + 1;
   

   printf("profile message from agent\n");
    
    // Agent id
    uint8_t id = (uint8_t)agent_mes_ptr[0];
    agent_mes_ptr = agent_mes_ptr + sizeof(id);
    agentMap[id]->id = id;
    printf("agent id %d\n",agentMap[id]->id);
    
    // Time when message is sent
   
    memcpy(&agentMap[id]->time_last_visited, agent_mes_ptr, sizeof(agentMap[id]->time_last_visited));
    //printf("time sent %u\n",agentMap[id]->time_last_visited);
    agent_mes_ptr+=sizeof(agentMap[id]->time_last_visited);

    // Agent pos
    memcpy(&agentMap[id]->current_location.x, agent_mes_ptr, sizeof(agentMap[id]->current_location.x));
    agent_mes_ptr+= sizeof(agentMap[id]->current_location.x);

    memcpy(&agentMap[id]->current_location.y, agent_mes_ptr, sizeof(agentMap[id]->current_location.y));
    agent_mes_ptr+= sizeof(agentMap[id]->current_location.y);
    
    //printf("Agent Position %f %f \n", agentMap[id]->current_location.x, agentMap[id]->current_location.y);


    // last time when hte data is transmitted
    memcpy(&agentMap[id]->time_last_data_collected,agent_mes_ptr,sizeof(agentMap[id]->time_last_data_collected));
    agent_mes_ptr+= sizeof(agentMap[id]->time_last_data_collected);
    //printf("LAST DATA transmitted %u\n",agentMap[id]->time_last_data_collected);

    // future position
    agentMap[id]->goal_location.x = 0.0;
    agentMap[id]->goal_location.y = 0.0;

    memcpy(&agentMap[id]->goal_location.x, agent_mes_ptr, sizeof(agentMap[id]->goal_location.x));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->goal_location.x);
    //printf("target X %f\n", agentMap[id]->goal_location.x);

    memcpy(&agentMap[id]->goal_location.y, agent_mes_ptr, sizeof(agentMap[id]->goal_location.y));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->goal_location.y);
    //printf("target Y %f\n", agentMap[id]->goal_location.y);
    agentMap[id]->IsGoalSet = true;

    vector <Util::Position> goalList;
    goalList.emplace_back(agentMap[id]->goal_location);

    //Future Goal Locations - 4 + 1 (current target)
    for(int i=1; i<= numberOfFutureTargetPositionsKnown; i++)
    {
        Position goal;

        memcpy(&goal.x, agent_mes_ptr, sizeof(goal.x));
        agent_mes_ptr = agent_mes_ptr + sizeof(goal.x);
        //printf("target X %f\n", goal.x);

        memcpy(&goal.y, agent_mes_ptr, sizeof(goal.y));
        agent_mes_ptr = agent_mes_ptr + sizeof(goal.y);
        //printf("target Y %f\n", goal.y);

        goalList.emplace_back(goal);
    }
    
      agentMap[id]->goalLocationList = goalList;
    
    
     // amount of data available from the agent
    memcpy(&agentMap[id]->data_available, agent_mes_ptr, sizeof(agentMap[id]->data_available));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->data_available);
    printf("Amount of data available %lu\n", agentMap[id]->data_available);
    
    uint32_t message_size;
    memcpy(&message_size, agent_mes_ptr, sizeof(message_size));
    agent_mes_ptr = agent_mes_ptr+sizeof(message_size);
    //printf("message_size %d\n",message_size);

    
       
       //stateData.detectedAgentId.push_back(id);
       vector<uint8_t>::iterator itr = std::find(stateData.InRange.begin(),stateData.InRange.end(),id);
       vector<uint8_t>::iterator itr2 = std::find(order_of_visit.begin(),order_of_visit.end(),id);

     
       
    
 
 if(agentMap[id]->data_available > 0 && find(agentToVisit.begin(),agentToVisit.end(),id) != agentToVisit.end() && (agentMap[id]->transmitted_data_size == 0 || find(order_of_visit.begin(),order_of_visit.end(),id) != order_of_visit.end()))
 {  
    if( itr == stateData.InRange.end() && itr2 != order_of_visit.end())
    {
          stateData.InRange.push_back(id);
          stateData.IsAgentDetected = true;
    }
    else if (itr == stateData.InRange.end())
    {
          stateData.InRange.push_back(id);
          stateData.IsAgentDetected = true;
    } 
    stateData.SentData = SStateData::RELAY_SENDING_ACCEPTANCE_TO_AGENT;
    SendData(stateData.SentData,id);

 }
 else if(agentMap[id]->data_available == 0 && stateData.targetAgentId == id)
 {
    order_of_visit.erase(std::remove(order_of_visit.begin(), order_of_visit.end(), id), order_of_visit.end());
    stateData.State = SStateData::STATE_NOGOAL;
 }
    if(stateData.targetAgentId == id)
      stateData.agentfound = true;
    //update agent locations in the probability map
    //if(not prob.getAgentState(id) == 0)
    
      //printf("Tell me what is happeningggg \n");
    prob.updateAgentPosition(id, agentMap[id]->current_location, agentMap[id]->goalLocationList);
    
    
    stateData.recentlyProfileDataCollectedId = id;
    if(stateData.NUMBER_OF_RELAY > 1)
    {
      stateData.SentData = SStateData::NEW_DATA;
      SendData(stateData.SentData, 0);
    }

    timeStepToMeet.data_file << m_Steps << "," << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY()<< "," << id << "\n" ;
    printf("done parsing agent message\n");
}



void 
FootbotRelay::ParseAgentCollectedData(vector<char> &incoming_agent_message)
{   
  char* agent_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)agent_mes_ptr[0];
  agent_mes_ptr = agent_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,agent_mes_ptr,sizeof(id));
  agent_mes_ptr = agent_mes_ptr + sizeof(id);


  memcpy(&agentMap[id]->transmitted_data_time, agent_mes_ptr, sizeof(agentMap[id]->transmitted_data_time));
  agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->transmitted_data_time);

  printf("Received data from agents\n");
    /// Receiving and storing data
   
  memcpy(&agentMap[id]->transmitted_data_size,agent_mes_ptr,sizeof(agentMap[id]->transmitted_data_size));
  agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->transmitted_data_size);

  datasizeToBaseStation = agentMap[id]->transmitted_data_size + datasizeToBaseStation; 
  
  memcpy(&agentMap[id]->delay_in_collection,agent_mes_ptr,sizeof(agentMap[id]->delay_in_collection));
  agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->delay_in_collection);


  agentMap[id]->IsDataReceived = true;
  

  stateData.InRange.erase(std::remove(stateData.InRange.begin(), stateData.InRange.end(), id), stateData.InRange.end());
  order_of_visit.erase(std::remove(order_of_visit.begin(), order_of_visit.end(), id), order_of_visit.end());
  
  agentMap[id]->data_available = 0;
  agentMap[id]->IsDataReceived = true;
  

  int i = id;

  printf("After collecting data from %d size of agent to visit %d \n", i, order_of_visit.size());
  visitedAgent.push_back(id);
  stateData.recentlyVisitedAgentId = id;
}

void 
FootbotRelay::ParseRelayTaskInfo(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  stateData.ParentMessages.insert(id);

  uint8_t numberOfAgentsAssigned;
  memcpy(&numberOfAgentsAssigned,relay_mes_ptr,sizeof(numberOfAgentsAssigned));
  relay_mes_ptr = relay_mes_ptr + sizeof(numberOfAgentsAssigned);

  for(int i=0 ; i < numberOfAgentsAssigned; i++)
  {
      uint8_t agentAssi;
      memcpy(&agentAssi,relay_mes_ptr,sizeof(agentAssi));
      relay_mes_ptr = relay_mes_ptr + sizeof(agentAssi);

      printf(" Removed Agent %d \n", (int)agentAssi);

      agentIdToConsider.erase(std::remove(agentIdToConsider.begin(), agentIdToConsider.end(), agentAssi), agentIdToConsider.end());
  
  }
  for(auto &rem: agentIdToConsider)
  {
     printf("%d Remaining agents %d \n", (int)m_myID, rem);
  }

}


void 
FootbotRelay::ParseRelayNewInformation(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  uint8_t agId;
  memcpy(&agId,relay_mes_ptr,sizeof(agId));
  relay_mes_ptr = relay_mes_ptr + sizeof(agId);

  float curr_agX;
  memcpy(&curr_agX,relay_mes_ptr,sizeof(curr_agX));
  relay_mes_ptr = relay_mes_ptr + sizeof(curr_agX);

  float curr_agY;
  memcpy(&curr_agY,relay_mes_ptr,sizeof(curr_agY));
  relay_mes_ptr = relay_mes_ptr + sizeof(curr_agY);

  float goal_agX;
  memcpy(&goal_agX,relay_mes_ptr,sizeof(goal_agX));
  relay_mes_ptr = relay_mes_ptr + sizeof(goal_agX);

  float goal_agY;
  memcpy(&goal_agY,relay_mes_ptr,sizeof(goal_agY));
  relay_mes_ptr = relay_mes_ptr + sizeof(goal_agY);

  agentMap[agId]->current_location.x = curr_agX;
  agentMap[agId]->current_location.y = curr_agY;

  agentMap[agId]->goal_location.x = goal_agX;
  agentMap[agId]->goal_location.y = goal_agY;

  for(int i= 0; i < numberOfFutureTargetPositionsKnown; i++)
  {
    float goalX;
    memcpy(&goalX,relay_mes_ptr,sizeof(goalX));
    relay_mes_ptr = relay_mes_ptr + sizeof(goalX);

    float goalY;
    memcpy(&goalY,relay_mes_ptr,sizeof(goalY));
    relay_mes_ptr = relay_mes_ptr + sizeof(goalY);

    agentMap[agId]->goalLocationList[i].x = goalX;
    agentMap[agId]->goalLocationList[i].y = goalY;
  }
  
  

  prob.updateAgentPosition(agId, agentMap[agId]->current_location, agentMap[agId]->goalLocationList);
}


void 
FootbotRelay::ParseRelayTransmittedDataTime(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  uint8_t agId;
  memcpy(&agId,relay_mes_ptr,sizeof(agId));
  relay_mes_ptr = relay_mes_ptr + sizeof(agId);

  uint64_t timeAgentDataGathered;
  memcpy(&timeAgentDataGathered,relay_mes_ptr,sizeof(timeAgentDataGathered));
  relay_mes_ptr = relay_mes_ptr + sizeof(timeAgentDataGathered);
  
  agentMap[agId]->transmitted_data_time = timeAgentDataGathered;

  
}



void 
FootbotRelay::ParseRelayFinalData(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  stateData.RelaysReachedBaseStation.insert(id);
}

void 
FootbotRelay::ParseMessage(vector<char> &incoming_agent_message, uint8_t received_data_id, uint8_t sender_id)
{
  switch(received_data_id) {
      case SStateData::AGENT_PROFILE_DATA: {
        printf("differ %d or initially %d \n", m_Steps - agentMap[sender_id]->time_last_visited, agentMap[sender_id]->time_last_visited);
        printf(" initial %s \n",agentMap[sender_id]->time_last_visited < 0 ? "true" : "false");
        ParseAgentProfile(incoming_agent_message);
        break;
      }
      case SStateData::AGENT_COLLECTED_DATA: {

        if((m_Steps - agentMap[sender_id]->time_last_data_collected) > 50 || agentMap[sender_id]->time_last_data_collected == 0)
        {
         ParseAgentCollectedData(incoming_agent_message); // explore is not necessary for one-one-one case

         if(stateData.NUMBER_OF_RELAY > 1)
          {
           stateData.SentData = SStateData::TRANSMITTED_DATA_TIME;
           SendData(stateData.SentData, 0);
          } 
        }
        else
        {
          printf(" %d recently data collected \n",(int)sender_id);
          stateData.InRange.erase(std::remove(stateData.InRange.begin(), stateData.InRange.end(), sender_id), stateData.InRange.end());
          order_of_visit.erase(std::remove(order_of_visit.begin(), order_of_visit.end(), sender_id), order_of_visit.end());
        }
         break;
      }
       case SStateData::RELAY_TASK_ASSIGNED_DATA: {
        printf("PArsing received Task Assigned message \n");
         ParseRelayTaskInfo(incoming_agent_message); 
         break;
      } 
      
      case SStateData::RELAY_NEW_DATA: {
        printf("PArsing received new data message \n");
         ParseRelayNewInformation(incoming_agent_message); 
         break;
      } 

      case SStateData::RELAY_TRANSMITTED_DATA_TIME: {
        printf("PArsing received TRANSMITTED_DATA_TIME message \n");
         ParseRelayTransmittedDataTime(incoming_agent_message); 
         break;
      } 

      case SStateData::RELAY_REACHED_BASESTATION: {
        printf("PArsing received reached BS message \n");
         ParseRelayFinalData(incoming_agent_message); 
         break;
      } 

      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
  } 
}

void
FootbotRelay::SendData(uint8_t send_data_id, uint8_t id)
{ 
  std::ostringstream str_tmp(ostringstream::out);
  str_tmp << "fb_" << id;
  string str_Dest = str_tmp.str();
  
  switch(send_data_id) {
      
      case SStateData::RELAY_HELLO_TO_AGENT: {
         printf("sending hello \n" );
         char agent_socket_msg[20];
         size_t mes_size =  HelloToAgent(send_data_id,agent_socket_msg);
         printf("size of hello message %d\n", (int)mes_size);

         // m_pcWifiActuator->BroadcastMessage_Extern(agent_socket_msg,mes_size);
         m_pcWifiActuator->SendBinaryMessageTo_Extern("-1",agent_socket_msg,mes_size);
       
         
         break;
      }
      
      case SStateData::RELAY_SENDING_ACCEPTANCE_TO_AGENT: {
         
         char agent_data_msg[20];
         size_t mes_size = AcceptanceToAgent(send_data_id,agent_data_msg); 

         m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),agent_data_msg,mes_size);
         break;
      }
      case SStateData::RELAY_TO_BASESTATION: {
         
         char base_socket_msg[MAX_UDP_SOCKET_BUFFER_SIZE*2];
         size_t collected_data_size = ToBaseStation(send_data_id,base_socket_msg); 

          printf("Base Station Target %s \n", str_Dest.c_str());
          m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),base_socket_msg,collected_data_size);
          break;
      }
      
      // Task Assigned info is sent only to agents which are not parents
      case SStateData::TASK_ASSIGNED: {

         char task_assigned_msg[200];
         size_t data_size = TaskAssignedToRelay(send_data_id,task_assigned_msg); 
        
         printf(" Sending message to children \n");
         for(auto &child:stateData.children)
         { 
            std::ostringstream str_tmpId(ostringstream::out);
            str_tmpId << "fb_" << child;
            string str_relayDest = str_tmpId.str();
            printf("Message from %d to %s \n", (int)m_myID, str_relayDest.c_str());
            m_pcWifiActuator->SendBinaryMessageTo_Local(str_relayDest.c_str(),task_assigned_msg,data_size);
         }
         
         break;
      }

      // new data info is sent to all other relays
      case SStateData::NEW_DATA: 
      {
        
          char new_data_msg[MAX_UDP_SOCKET_BUFFER_SIZE];
         size_t data_size = NewDataInformationToRelay(send_data_id,new_data_msg); 

         printf("NEW DATA SENDING \n");
         m_pcWifiActuator->SendBinaryMessageTo_Local("-1",new_data_msg,data_size);
         
         break;
      }

       case SStateData::TRANSMITTED_DATA_TIME: 
      {
         char new_data_msg[MAX_UDP_SOCKET_BUFFER_SIZE];
         size_t data_size = SendingTransmittedDataTimeToRelay(send_data_id,new_data_msg); 

         printf("TRANSMITTED_DATA_TIME DATA SENDING \n");
         m_pcWifiActuator->SendBinaryMessageTo_Local("-1",new_data_msg,data_size);
         
         break;
      }

      // new data info is sent to all other relays
      case SStateData::REACHED_BASESTATION: {

         char dest_data_msg[100];
         size_t data_size = AnnouncingTaskcompletion(send_data_id,dest_data_msg); 

         printf("SENDING REACHED BASE STATION \n");
         m_pcWifiActuator->SendBinaryMessageTo_Local("-1",dest_data_msg,data_size);
         
         break;
      }

      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
  }
}

bool
FootbotRelay::UpdateState()
{ 
  bool goalCalculated = false;
  printf("In update \n");
  
  // Sending hello message to agent
  if(m_Steps%10 == 0 && not(stateData.MovingToBaseStation))
  //if(m_Steps%10 == 0
  {   
    stateData.SentData = SStateData::RELAY_HELLO_TO_AGENT;
    try{
      SendData(stateData.SentData, 0);
    }
    catch(exception& e)
    {
      printf("%s\n",e.what());
    }
   
  }
  
  /**
  *  The code below updates probability matrix values and finds the final belief vector
  *
  */
  
  if(m_Steps % (10) == 0 || m_Steps == 1)  // converting seconds to timestep 1 second = 10 timestep
  { 
    printf("Here finding prob \n");
    
    for (auto const &i:agentMap) 
      {
         if(prob.getAgentState(i.first) != 1)
        {
          Util::Position agentProbablePos;
          agentMap[i.first]->current_location = prob.update(i.first);
        
          printf("update done position of agent id %d , x %f , y %f \n", (int)i.first, agentMap[i.first]->current_location.x , agentMap[i.first]->current_location.y);

         goalCalculated = true;
        }
      }
  }
  return goalCalculated;
}


void FootbotRelay::SetGoal(bool goalCalculated) 
{ 
  stateData.agentfound = false;
  for(int i =0 ; i < order_of_visit.size() ; i++)
  {
     printf("order to follow %d\n", (int)order_of_visit[i]);
  }
  //if(not stateData.visited_baseStation)
  {
    if(order_of_visit.size() > 0)
    { 
      //std::unordered_set<uint8_t>::iterator it = agentToVisit.begin();
      stateData.targetAgentId = order_of_visit[0];
      stateData.State =  SStateData::STATE_SEARCH;
      
      Position agentPos = agentMap[stateData.targetAgentId]->current_location;
      CVector3 targetPos(agentPos.x,agentPos.y,0.0);
      
      m_navClient->setTargetPosition(targetPos);
      printf("stateData.targetAgentId TARGET %d %f %f \n",(int)stateData.targetAgentId, agentPos.x, agentPos.y);
    }

    else if(datasizeToBaseStation == 0 && order_of_visit.size() == 0)
    {
       initialiseBeforeClustering();
       visitedAgent.clear();
       stateData.InRange.clear();
    }
    else if(order_of_visit.size() == 0)
    { 
      stateData.IsGoalSet = false;
      stateData.State = SStateData::STATE_RETURN_TO_BASESTATION;
    }
    
  }
}
float FootbotRelay::checkUpperBounds(float num, float max)
{
  num = max-num < 0.5 ? num-1.0 : 0.0;
  return num;
}

float FootbotRelay::checkLowerBounds(float num)
{ 
  num = num-0.0 < 0.5 ? num+1.0 : 0.0;
  return num;
}

float FootbotRelay::checkBounds(float num, float max)
{  
   float test = (checkUpperBounds(num,max) || checkLowerBounds(num));
   if(test)
   {
      num = test;
   }
   return num;
}

void 
FootbotRelay::Search(bool goalCalculated)
{ 
  Position currentPos;
  currentPos.x = m_navClient->currentPosition().GetX();
  currentPos.y = m_navClient->currentPosition().GetY();

  if(stateData.IsAgentDetected)
  { 
    
    for(auto &i : stateData.InRange)
    {   
       printf("AGENT DETECTEDDDDDDDD");
       printf("detected agent %d\n",(int)i);
    }
    if(stateData.InRange.size() > 0)
    {
      stateData.State = SStateData::STATE_DATA_GATHERING;
      stateData.IsGoalSet = false;
      stateData.IsAgentDetected = false;
    }
    
  }
  else
  {
    if(m_navClient->state() == target_state)
    { 
      printf("Agent not found but reached target\n");
      timeInSearch = timeInSearch + 1;
      if(timeInSearch > 35)
      {
        order_of_visit.erase(std::remove(order_of_visit.begin(), order_of_visit.end(), stateData.targetAgentId), order_of_visit.end());
        stateData.State = SStateData::STATE_NOGOAL;
        timeInSearch = 0;
      }
    }
    else
    {
      
      printf("Search State Target not reached \n");
    }
    Position target = agentMap[stateData.targetAgentId]->current_location;
    
    if(goalCalculated)
    {   
        target.x = checkBounds(target.x,size_x);
        target.y = checkBounds(target.y,size_x);
        //m_navClient->start();
        printf("Set a new target Position every 20 seconds\n");
        
        CVector3 targetPos(target.x,target.y,0.0);
        m_navClient->setTargetPosition(targetPos);
        printf("TARGET id and loc %d %f %f \n", (int)stateData.targetAgentId, targetPos[0], targetPos[1]);
    } 
  }
}

void
FootbotRelay::DataGather(uint8_t agentId)
{
  //send the acceptance to collect data after checking the available data size.
  // move towards agent's current location Potential field -- agent should attract until it sends data and once data is sent agent should repel

// collect data from agents assigned alone  
std::vector<uint8_t>::iterator itr = find(order_of_visit.begin(), order_of_visit.end(), agentId);

if(itr != order_of_visit.end())
{ 
  
    if(not stateData.IsGoalSet)
    {
      //unordered_set<uint8_t>::iterator itr = stateData.InRange.begin();
     
      printf("Agent Location set as target\n");
      CVector3 targetPos(agentMap[agentId]->current_location.x,agentMap[agentId]->current_location.y,0.0);
      m_navClient->setTargetPosition(targetPos);
      printf("TARGET %f %f \n", targetPos[0], targetPos[1]);
      timeInDataGather = 0;
      stateData.IsGoalSet = true;
     }
      
      
    
    //if(m_navClient->state() == target_state)
    else if(agentMap[agentId]->IsDataReceived)
    {  
       
       printf("Received gathered data \n");
       //stateData.collected_data_size = stateData.collected_data_size + agentData.data_available;
       //agentData.data_available = 0;
       
       stateData.IsGoalSet = false;
       stateData.IsAgentDetected = false;
       
       // if no agent in range 
      agentMap[agentId]->IsDataReceived = false;

    }
    
    else if(stateData.IsGoalSet)
    { 
      printf("Incrementing time in Data gather %d \n", timeInDataGather + 1);
      timeInDataGather = timeInDataGather + 1;
      if(timeInDataGather > 10)
      { 
        printf("Calling Data Gather Again %d \n", timeInDataGather);
        
        stateData.IsGoalSet = false;
        DataGather(agentId);
        
      }
    }

    else
    {
      printf("Moving to agent %d \n",(int)agentId);
    }
  }
  
  
  
}

void 
FootbotRelay::SetState()
{ 
  printf("in set state \n");
  printf("visitedAgent size, number of agents %d %d \n",visitedAgent.size(), stateData.NUMBER_OF_AGENT);
  //printf("target agent, target agent found %d \n", int(stateData.targetAgentId));
  printf("%s \n",stateData.agentfound ? "true" : "false");

  stateData.IsGoalSet = false;
  /*if(visitedAgent.size() == agentToVisit.size())
  {
    stateData.State = SStateData::STATE_RETURN_TO_BASESTATION;
  }*/
  if(not stateData.agentfound)
  {
    stateData.State =  SStateData::STATE_SEARCH;
    
  }
  else if(datasizeToBaseStation == 0 && order_of_visit.size() == 0)
  {
       initialiseBeforeClustering();
       visitedAgent.clear();
       stateData.InRange.clear();
  }
  else
  {
    stateData.State =  SStateData::STATE_NOGOAL;
  }

  printf("Current State %d \n", stateData.State );
}


void 
FootbotRelay::initialiseBeforeClustering()
{
  
 m_navClient->stop();
 stateData.IsGoalSet = false;

 clusteringDone = false;
 clusteringAllowed = true;
 stateData.allCompleted = true;
 agentIdToConsider = agentIdList;

 stateData.SentData = SStateData::REACHED_BASESTATION;
 SendData(stateData.SentData, 0);

 
  stateData.State = SStateData::STATE_NOGOAL;

}

uint8_t 
FootbotRelay::findNearestBaseStation()
{  
   Position current;
   current.x = m_navClient->currentPosition().GetX();
   current.y = m_navClient->currentPosition().GetY();

   float dist = 500;
   int index = 0;
   for(auto &pos:stateData.base_station)
   {  
      float t = util.findDistance(&current, &pos.second);
      
      if(dist > t)
      {
         dist = t;
         index = pos.first;
      }
   }
  
  return index;
  
}

void
FootbotRelay::Return()
{   
    
      
    if(not stateData.IsGoalSet && not stateData.IsDataSentToBaseStation) // If the goal is not set and data is not transferred to base station
    {
      
      idToSendData = findNearestBaseStation();
    
      printf("Target set to base location\n");
      CVector3 targetPos(stateData.base_station[idToSendData].x,stateData.base_station[idToSendData].y,0.0);
      m_navClient->setTargetPosition(targetPos);
      printf("TARGET %f %f \n", targetPos[0], targetPos[1]);
      stateData.IsGoalSet = true;
      stateData.MovingToBaseStation = true;
      datasizeToBaseStation = 0;
    }  
    
    else
    {
      Position currentL;
      currentL.x = m_navClient->currentPosition().GetX();
      currentL.y = m_navClient->currentPosition().GetY();

      double dist = util.findDistance(&stateData.base_station[idToSendData], &currentL);
      printf("Distance: %f \n", dist);
      printf("Navigation state of agent %d\n", m_navClient->state());

      if(stateData.IsGoalSet && dist <= 0.40) // if an object is stationary(Base Station) at that target location, it doesn't go beyond 0.28.0.35
      {
         // send data once reached base Station
       printf("Reached Base Station\n");
       stateData.SentData = SStateData::RELAY_TO_BASESTATION;
       SendData(stateData.SentData,idToSendData);
       
       m_navClient->stop();
       
       stateData.IsGoalSet = false;

       /*clusteringDone = false;
       clusteringAllowed = true;
       stateData.allCompleted = true;
       agentIdToConsider = agentIdList;*/

       
       stateData.MovingToBaseStation = false;
       timeForACycle.data_file << m_Steps << "\n";
       timeForACycle.data_file << m_Steps << ",";
      }
        
      
      else if(stateData.IsDataSentToBaseStation)
      {  
         //m_navClient->start();
         printf("Data Sent to Base Station\n");
         
         stateData.IsDataSentToBaseStation = false;
         stateData.visited_baseStation = true;
         
         clusteringAllowed = true;
         stateData.allCompleted = true;
         agentIdToConsider = agentIdList;

        
         stateData.State = SStateData::STATE_HOME;
         
         // copying visited to toVisit list
         //agentToVisit = visitedAgent;
         //std::copy(visitedAgent.begin(), visitedAgent.end(), inserter(agentToVisit, agentToVisit.begin()));
         visitedAgent.clear();
         stateData.InRange.clear();
         stateData.SentData = SStateData::REACHED_BASESTATION;
         SendData(stateData.SentData, 0);
      
      }
    }
    
    //if(not stateData.IsGoalSet)
}


void 
FootbotRelay::Home() 
{ 
  printf("GOING HOME STATE\n");
  if(not stateData.IsGoalSet)
  { 
    m_navClient->start();
    Position to_home;
    if(m_navClient->currentPosition().GetX() + 10 > 30)
    {
      to_home.x = m_navClient->currentPosition().GetX() - 0.15*m_myID - 1.0;
      to_home.y = m_navClient->currentPosition().GetY() - 0.15*m_myID - 1.0;
    }
    else
    {
      to_home.x = m_navClient->currentPosition().GetX() + 0.15*m_myID + 1.0;
      to_home.y = m_navClient->currentPosition().GetY() + 0.15*m_myID + 1.0;
    }
    
    CVector3 targetPos(to_home.x,to_home.y,0.0);
    m_navClient->setTargetPosition(targetPos);
    stateData.IsGoalSet = true;
  }
  else if(m_navClient->state() == target_state)
  {
    m_navClient->stop();
    clusteringDone = false;
  }
}

void 
FootbotRelay::ControlStep() 
{   
  m_Steps+=1;
  
  printf("Current State %d \n", stateData.State );
  printf(" ID %d stateData.RelaysReachedBaseStation.size() %d  relayIdList.size() %d stateData.ParentMessages.size() %d stateData.parents.size() %d \n" , (int)m_myID, stateData.RelaysReachedBaseStation.size(), relayIdList.size(), stateData.ParentMessages.size(),stateData.parents.size());

  printf("number of agents to visit %d\n", order_of_visit.size());
  if(m_navClient->currentPosition().GetX() > 0 &&  m_navClient->currentPosition().GetY() > 0)
  {

  if(stateData.RelaysReachedBaseStation.size() == relayIdList.size() && stateData.ParentMessages.size() == stateData.parents.size() && stateData.allCompleted)
  {  
     clusteringAllowed = false;
     clusteringDone = true;
     stateData.allCompleted = false;
     stateData.RelaysReachedBaseStation.clear();
     stateData.ParentMessages.clear();
     
     Position relayPos;
     vector<Position> agentPositionList;
     vector<uint64_t> timeLastDataCollectedList;



     printf("before sending data to clustering class \n");

     for(int i= 0 ; i < agentIdToConsider.size() ; i++)
     {
        printf("agent ids to consider %d \n", (uint8_t)agentIdToConsider[i]);
     }

     for(auto &Id : agentIdToConsider)
     {
        
      agentPositionList.push_back(agentMap[Id]->current_location);
      timeLastDataCollectedList.push_back(m_Steps-agentMap[Id]->transmitted_data_time);
     }

     relayPos.x = m_navClient->currentPosition().GetX();
     relayPos.y = m_navClient->currentPosition().GetY();
     printf("not initialised\n");

     clusterData.data_file <<  relayPos.x << "," << relayPos.y << ",";

     cluster.initialise(relayPos,agentPositionList, stateData.NUMBER_OF_RELAY-stateData.parents.size());

     if(stateData.children.size() > 0)
     {
       int centroid = cluster.findClusters();
       printf("centroid index %d \n", centroid);
       for(auto &index : cluster.getAgentOrder(timeLastDataCollectedList, centroid))
      {
        order_of_visit.push_back(agentIdToConsider[index]);
        clusterData.data_file << agentIdToConsider[index] << "," << agentMap[agentIdToConsider[index]]->current_location.x << "," << agentMap[agentIdToConsider[index]]->current_location.y << ",";
      }
     }
    else
     {
        for(auto &index : cluster.getAgentOrder(timeLastDataCollectedList))
      {
        order_of_visit.push_back(agentIdToConsider[index]);
        clusterData.data_file << agentIdToConsider[index] << "," << agentMap[agentIdToConsider[index]]->current_location.x << "," << agentMap[agentIdToConsider[index]]->current_location.y << ",";
      }
     }
     clusterData.data_file << "\n";
    
    printf("initialised\n");
     
    for(auto &agi: order_of_visit)
    {
      printf("agent order to visit %d \n", (int)agi);
    }
     
     agentToVisit.clear();
     agentToVisit = order_of_visit;
     
     m_navClient->start(); 

    if(stateData.children.size() > 0)
    {
      stateData.SentData = SStateData::TASK_ASSIGNED;
      SendData(stateData.SentData, 0);
    } 
  
  if(stateData.State != 3)
    stateData.State = SStateData::STATE_NOGOAL;
    
  }

  if(m_Steps % 5 == 0)
  {
    relayPositions.data_file << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY() << "\n";
  }
    
    bool goalCalculated = UpdateState();
    printf("target now %d\n ",(int)stateData.targetAgentId);
    printf("goal calculated %s \n", goalCalculated ? "true" : "false");
    //SetState();
if(clusteringDone)
{
  switch(stateData.State) {

      case SStateData::STATE_NOGOAL: {
         printf("IN no goal set state \n");
         SetGoal(goalCalculated);
         break;
      }
      case SStateData::STATE_SEARCH: {
         printf("IN SEARCH STATE \n");
         Search(goalCalculated);
         break;
      }
    
      case SStateData::STATE_DATA_GATHERING: {
         printf("IN DataGather STATE \n");
         
         
           for(auto &agentId:stateData.InRange)
          {
            printf("Agents in Range %d\n",(int)agentId);
          }
        
          
          if(stateData.InRange.size() == 0)
           {  
              stateData.IsAgentDetected = false;
              stateData.IsGoalSet = false;

              //stateData.State = SStateData::STATE_NOGOAL;
              printf("No Agent\n");
              SetState();
           }
           else if(stateData.InRange.size() > 0)
           { 
             printf("Current Agent %d\n",(int)stateData.InRange[0]);
             printf("Agents in Range size greater than zero \n");
             if(agentMap[stateData.InRange[0]]->data_available > 0)
             {
                DataGather(stateData.InRange[0]); // Check argos2-footbot to decide when the relay should communicate
                ///If the detected agent is the same as agent in pursuit then go to No Goal state or go to search state
                printf("visiting agent \n");
              }
              else
              {  
                 printf("Erasing agent in range as data available is zero \n");
                 stateData.InRange.erase(std::remove(stateData.InRange.begin(), stateData.InRange.end(), stateData.InRange[0]), stateData.InRange.end());
                 order_of_visit.erase(std::remove(order_of_visit.begin(), order_of_visit.end(), stateData.InRange[0]), order_of_visit.end());
              }
           }
          
          else
          {
             printf("Why am i here \n");
             SetState();
          }
           
         
         break;
      }
      case SStateData::STATE_RETURN_TO_BASESTATION: {
         printf("IN Return STATE \n");
         Return(); // Once Reached BS send collected data
         break;
      }

      case SStateData::STATE_HOME: {
         printf("going home state \n");
         Home();
         break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }

     }
   }

  

 }
  
  printf(" Relay Position %f %f \n",m_navClient->currentPosition().GetX(), m_navClient->currentPosition().GetY());
    
  
  

    /********  Receive messages from mission agents ********/
  
  TMessageList message_from_agents;
  m_pcWifiSensor->GetReceivedMessages_Extern(message_from_agents);
  for(TMessageList::iterator it = message_from_agents.begin(); it!=message_from_agents.end();it++)
  {   
    //send_message_to_relay = true;
    printf("Received %lu bytes to incoming buffer\n", it->Payload.size());

    vector<char> check_message = it->Payload;
    printf("Identifier of received message [extern], sender  %d %d \n",int(check_message[0]),int(check_message[1]));
    uint8_t senderId =  uint8_t(check_message[1]);
    
    std::vector<uint8_t>::iterator itr = find(agentIdList.begin(), agentIdList.end(), senderId);
    
    if(itr != agentIdList.end())
    {
      ParseMessage(it->Payload, uint8_t(check_message[0]),uint8_t(check_message[1]));
    }
    string sender = it->Sender;
    /*if((char)check_message[0] == 'a')
    {   
      //agents.push_back(atoi((it->Sender).c_str));
      meeting_data_file << sender << " ";
      //parse_agent_message(it->Payload);
      //uint8_t received_from = stoi(it->Sender)+1;
      received_message_file << m_Steps << "," << check_message[0] << "," << sender << "," << (m_navClient->currentPosition().GetX()) << "," << (m_navClient->currentPosition().GetY()) << "\n";
      //received_message_file << m_Steps << "," << check_message[0] << "," << it->Sender << "\n";
    }

    else if((char)check_message[0] == 'b')
    {
      meeting_data_file << "d_"+(sender) << " ";
      //data_exchange_agents.push_back(atoi((it->Sender).c_str));
      received_message_file << m_Steps << "," << check_message[0] << "," << sender << "," << (m_navClient->currentPosition().GetX()) << "," << (m_navClient->currentPosition().GetY()) << "\n";
    } */
  }


  /********  Receive messages from relays ********/
if(stateData.NUMBER_OF_RELAY > 1)
{
  TMessageList message_from_relays;
  m_pcWifiSensor->GetReceivedMessages_Local(message_from_relays);
  for(TMessageList::iterator it = message_from_relays.begin(); it!=message_from_relays.end();it++)
  {   
    //send_message_to_relay = true;
    printf("Received %lu bytes to incoming buffer from relay \n", it->Payload.size());

    vector<char> check_message = it->Payload;
    printf("Identifier of received message [local], sender  %c %c \n",uint8_t(check_message[0]),uint8_t(check_message[1]));

    uint8_t senderId =  uint8_t(check_message[1]);

    if(senderId != m_myID)
    {
      ParseMessage(it->Payload, uint8_t(check_message[0]),uint8_t(check_message[1]));
    }
    
    
  }
}
  m_pcLEDs->SetAllColors(CColor::MAGENTA);
  m_navClient->setTime(getTime());
  m_navClient->update();
  
  ////cout <<"MyId: " << m_myID << "target: " << counter << endl;
  
}




void 
FootbotRelay::Destroy() 
{
  DEBUG_CONTROLLER("FootbotRelay::Destroy (  )\n");
}

/**************************************/

bool 
FootbotRelay::IsControllerFinished() const 
{
  return false;
}


std::string
FootbotRelay::getTimeStr()
{
#ifndef FOOTBOT_SIM
  char buffer [80];
  timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;
  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));
  char currentTime[84] = "";
  sprintf(currentTime, "%s:%d", buffer, milli);
  std::string ctime_str(currentTime);
  return ctime_str;
#else
  return "mytime";
#endif
}


/// returns time in milliseconds
  UInt64 
FootbotRelay::getTime()
{
#ifndef FOOTBOT_SIM
  struct timeval timestamp;
  gettimeofday(&timestamp, NULL);

  UInt64 ms1 = (UInt64) timestamp.tv_sec;
  ms1*=1000;

  UInt64 ms2 = (UInt64) timestamp.tv_usec;
  ms2/=1000;

  return (ms1+ms2);
#else
  return m_Steps * CPhysicsEngine::GetSimulationClockTick() * 1000;
#endif
}


  
REGISTER_CONTROLLER(FootbotRelay, "footbot_relay_controller")