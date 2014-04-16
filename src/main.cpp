#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
#include <boost/cstdint.hpp>
#include <iostream>
#include <sstream>

#include "map.hpp"

int main(int argc, char *argv[]) {
	const char * directionName[] = {"", "SW", "S", "SE", "W", "C", "E", "NW", "N", "NW", "U", "D"};

	std::ifstream input("test.map");

	Map map;
	OriginAndGoal goal = map.loadFromStream(input);
	map.debugDisplay(goal);

	std::cout << " Origin == \n";
	std::vector<Portal> origins = map.linkPositionAndPortals(goal.origin);
	for (size_t i = 0; i < origins.size(); ++i)
	{
		std::cout << origins[i] << "\n";
	}

	std::cout << " Goal == \n";
	std::vector<Portal> goals = map.linkPositionAndPortals(goal.goal);
	for (size_t i = 0; i < goals.size(); ++i)
	{
		std::cout << goals[i] << "\n";
	}

	if (map.portalPathfind(origins, goals))
	{
		std::cout << "Path found!\n";
	}
	else
	{
		std::cout << "No Path\n";
	}
}
