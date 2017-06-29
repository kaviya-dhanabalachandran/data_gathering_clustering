#include "clustering.h"

/****
Step:1  Find clusters 
Step:2  find the weights associated with agents
Step:3  Order them in descending order
****/

Clustering::Clustering()
{

}

void 
Clustering::initialise(Position relayPos, vector<Position> agentPositionList,int NUMBER_OF_RELAY)
{
	relayP = relayPos;
	agentPosList = agentPositionList;
	numberOfClusters = NUMBER_OF_RELAY;
}

template <typename T>
size_t nearestCentroidIndex(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx[0];
}

template<typename A, typename B>
std::pair<B,A> flip_pair(const std::pair<A,B> &p)
{
    return std::pair<B,A>(p.second, p.first);
}

int 
Clustering::findClusters()
{	
	std::vector<std::array<float, 2>> dataPoints;
	Position centroidP;

	for(auto &agentP: agentPosList)
	{   
		array<float,2> temp {agentP.x,agentP.y};
		dataPoints.push_back(temp);

	}
	std::tie(centroid, clustersAssigned) = dkm::kmeans_lloyd(dataPoints, numberOfClusters);

	for(auto &c : centroid)
	{
		centroidP.x = c[0];  centroidP.y = c[1];
		distToCentroid.push_back(util.findDistance(&relayP, &centroidP));
		printf("distance to centroids %f \n", util.findDistance(&relayP, &centroidP));
	}

	std::sort(distToCentroid.begin(), distToCentroid.end());
	//int nearestCentroid = nearestCentroidIndex(distToCentroid);
	int nearestCentroid = 0; // Always index 0 has the least value
	return nearestCentroid;
}

vector<uint8_t> 
Clustering::getAgentOrder(vector<uint64_t> timeLastDataCollected, int nearestCentroid)
{	
	
	std::map<int,float> agentsAssigned;
	
	std::vector<unsigned int>::iterator iter = clustersAssigned.begin(); 
	while ((iter = std::find(iter, clustersAssigned.end(), nearestCentroid)) != clustersAssigned.end()) 
		{   
			int id  = std::distance(clustersAssigned.begin(), iter);

			printf("Agent ińdex %d , time %ld, distance %f, weight %f \n", id, timeLastDataCollected[id],util.findDistance(&relayP,&agentPosList[id]),util.weightCalculation(timeLastDataCollected[id]));
			float w = util.findDistance(&relayP,&agentPosList[id]) * util.weightCalculation(timeLastDataCollected[id]);
			agentsAssigned.emplace(id,w); 
			printf("weight of agent %f \n", w);
			iter++; 
		} 

	std::multimap<float, int, std::greater<float>> flipMap;
    std::transform(agentsAssigned.begin(), agentsAssigned.end(), std::inserter(flipMap, flipMap.begin()), flip_pair<int,float>);
    agentOrder.clear();
    for(auto & i: flipMap)
    {
    	agentOrder.push_back(i.second); // This is because agent id starts at 2
    }
	return agentOrder;
}


vector<uint8_t> 
Clustering::getAgentOrder(vector<uint64_t> timeLastDataCollected)
{	
	
	std::map<int,float> agentsAssigned;
	
	std::vector<unsigned int>::iterator iter = clustersAssigned.begin(); 
	  
	  for(int index= 0; index < timeLastDataCollected.size(); index++)
	   {    
	   	    printf("Agent ińdex %d ,time %ld, distance %f, weight %f \n", index,timeLastDataCollected[index],util.findDistance(&relayP,&agentPosList[index]),util.weightCalculation(timeLastDataCollected[index]));
			float w = util.findDistance(&relayP,&agentPosList[index]) * util.weightCalculation(timeLastDataCollected[index]);
			agentsAssigned.emplace(index,w); 
			printf("weight of agent %f \n", w);
			iter++; 
		} ;

	//std::multimap<float, int, std::greater<float>> flipMap;

	std::multimap<float, int> flipMap;
    
    std::transform(agentsAssigned.begin(), agentsAssigned.end(), std::inserter(flipMap, flipMap.begin()), flip_pair<int,float>);
    agentOrder.clear();
    for(auto & i: flipMap)
    {
    	agentOrder.push_back(i.second); // This is because agent id starts at 2
    }
	return agentOrder;
}


