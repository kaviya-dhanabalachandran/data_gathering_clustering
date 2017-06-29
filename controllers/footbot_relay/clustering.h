#ifndef _CLUSTERING_H_
#define _CLUSTERING_H_

#include "dkm.hpp"
#include <vector>
#include <map>
#include <array>
#include <cstdint>
#include <algorithm>
#include <tuple>
#include <iostream>
#include <typeinfo>
#include <math.h>

#include "utils.h" 

using namespace std;


class Clustering
{
  
  public:
   typedef Util::Position Position;
   Util util;

   Position relayP;
   vector<Position> agentPosList;
   int numberOfClusters;
   vector<uint8_t> agentOrder;

   std::vector<std::array<float, 2> > centroid;
   std::vector<uint32_t> clustersAssigned;
   std::vector<float> distToCentroid;

   Clustering();
   void initialise(Position relayPos, vector<Position> agentPositionList,int NUMBER_OF_RELAY);

   virtual ~Clustering() {
		}

   int findClusters();
   
   vector<uint8_t> getAgentOrder(vector<uint64_t> timeLastDataCollected, int);
   vector<uint8_t> getAgentOrder(vector<uint64_t> timeLastDataCollected);


};

#endif