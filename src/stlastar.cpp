////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <thread>
#include <list>
#include <vector>
#include "map.h"

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;

// Global data

// The world map 

// Warining. 9 is a wall. The astar implementation works with weights. 
// 0 being least and 9 being highets/impassable

int MAP_WIDTH;
int MAP_HEIGHT; 
std::vector<int> world_map;

//int MAP_WIDTH = 20; 
//int MAP_HEIGHT = 20;

// int world_map[ MAP_WIDTH * MAP_HEIGHT ] = 
// {

// // 0001020304050607080910111213141516171819
// 	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,   // 00
// 	0,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,0,   // 01
// 	0,9,9,0,0,9,9,9,0,9,0,9,0,9,0,9,9,9,0,0,   // 02
// 	0,9,9,0,0,9,9,9,0,9,0,9,0,9,0,9,9,9,0,0,   // 03
// 	0,9,0,0,0,0,9,9,0,9,0,9,0,0,0,0,9,9,0,0,   // 04
// 	0,9,0,0,9,0,0,0,0,9,0,0,0,0,9,0,0,0,0,0,   // 05
// 	0,9,9,9,9,0,0,0,0,0,0,9,9,9,9,0,0,0,0,0,   // 06
// 	0,9,9,9,9,9,9,9,9,0,0,0,9,9,9,9,9,9,9,0,   // 07
// 	0,9,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,   // 08
// 	0,9,0,9,9,9,9,9,9,9,0,0,9,9,9,9,9,9,9,0,   // 09
// 	0,9,0,0,0,0,9,0,0,9,0,0,0,0,0,0,0,0,0,0,   // 10
// 	0,9,9,9,9,9,0,9,0,9,0,9,9,9,9,9,0,0,0,0,   // 11
// 	0,9,0,9,0,9,9,9,0,9,0,9,0,9,0,9,9,9,0,0,   // 12
// 	0,9,0,9,0,9,9,9,0,9,0,9,0,9,0,9,9,9,0,0,   // 13
// 	0,9,0,0,0,0,9,9,0,9,0,9,0,0,0,0,9,9,0,0,   // 14
// 	0,9,0,0,9,0,0,0,0,9,0,0,0,0,9,0,0,0,0,0,   // 15
// 	0,9,9,9,9,0,0,0,0,0,0,9,9,9,9,0,0,0,0,0,   // 16
// 	0,0,9,9,9,9,9,9,9,0,0,9,9,9,9,9,9,9,9,9,   // 17
// 	0,9,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,   // 18
// 	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,   // 19

// };

// map helper functions

int GetMap( int x, int y )
{
	if( x < 0 ||
	    x >= MAP_WIDTH ||
		 y < 0 ||
		 y >= MAP_HEIGHT
	  )
	{
		return 9;	 
	}

	return world_map[(y*MAP_WIDTH)+x];
}



// Definitions

class MapSearchNode
{
public:
	int x;	 // the (x,y) positions of the node
	int y;	
	
	MapSearchNode() { x = y = 0; }
	MapSearchNode( int px, int py ) { x=px; y=py; }

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo(); 


};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

	// same state in a maze search is simply when (x,y) are the same
	if( (x == rhs.x) &&
		(y == rhs.y) )
	{
		return true;
	}
	else
	{
		return false;
	}

}

void MapSearchNode::PrintNodeInfo()
{
	char str[100];
	sprintf( str, "Node position : (%d,%d)\n", x,y );

	cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

	if( (x == nodeGoal.x) &&
		(y == nodeGoal.y) )
	{
		return true;
	}

	return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

	int parent_x = -1; 
	int parent_y = -1; 

	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}
	

	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if( (GetMap( x-1, y ) < 9) 
		&& !((parent_x == x-1) && (parent_y == y))
	  ) 
	{
		NewNode = MapSearchNode( x-1, y );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap( x, y-1 ) < 9) 
		&& !((parent_x == x) && (parent_y == y-1))
	  ) 
	{
		NewNode = MapSearchNode( x, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap( x+1, y ) < 9)
		&& !((parent_x == x+1) && (parent_y == y))
	  ) 
	{
		NewNode = MapSearchNode( x+1, y );
		astarsearch->AddSuccessor( NewNode );
	}	

		
	if( (GetMap( x, y+1 ) < 9) 
		&& !((parent_x == x) && (parent_y == y+1))
		)
	{
		NewNode = MapSearchNode( x, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
	return (float) GetMap( x, y );

}


// Main
void setWorldMap(Map& map) {
	for (int x = 0; x < map.width(); x++) {
    for (int y = 0; y < map.height(); y++) {
      if (!map.get(Point(x, y))) { //0 is a wall
        world_map.push_back(9);
      } else {            // Asuming the other value is a 1
        world_map.push_back(0);
      }
    }
  }
}

//run this
list<Node> search(Map& map, Node source, Node goal) {
	AStarSearch<MapSearchNode> astarsearch;
	unsigned int SearchCount = 0;

	const unsigned int NumSearches = 1;

	setWorldMap(map);

	while(SearchCount < NumSearches)
	{

		// Create a start state
		Point start = map.node_to_point(source);
		MapSearchNode nodeStart;
		nodeStart.x = start.x;
		nodeStart.y = start.y; 

		// Define the goal state
		Point end = map.node_to_point(goal);
		MapSearchNode nodeEnd;
		nodeEnd.x = end.x;						
		nodeEnd.y = end.y; 
		
		// Set Start and goal states
		
		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

		unsigned int SearchState;
		unsigned int SearchSteps = 0;
		
		do
		{
			SearchState = astarsearch.SearchStep();

			SearchSteps++;

		}
		while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
		
		std::list<Node> shortest_path;
		if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
		{
				//cout << "Found Goual state \n"
				MapSearchNode *node = astarsearch.GetSolutionStart();
				int steps = 0;

				// node->PrintNodeInfo();
				Point start;
				start.x = node->x;
				start.y = node->y;

				shortest_path.push_back(map.point_to_node(start));
				for( ;; )
				{
					node = astarsearch.GetSolutionNext();

					if( !node )
					{
						break;
					}

					Point next;
					next.x = node->x;
					next.y = node->y;

					shortest_path.push_back(map.point_to_node(next));
					//node->PrintNodeInfo();
					steps ++;
				};
				// Once you're done with the solution you can free the nodes up
				astarsearch.FreeSolutionNodes();
		}
		else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
		{
			//cout << "Search terminated. Did not find goal state\n";
			return { }; //Justin Case
		}
		SearchCount ++;

		astarsearch.EnsureMemoryFreed();
		return shortest_path;
	}
	return {};

}

#if 0
int main( int argc, char *argv[] )
{

	cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";

	// Our sample problem defines the world as a 2d array representing a terrain
	// Each element contains an integer from 0 to 5 which indicates the cost 
	// of travel across the terrain. Zero means the least possible difficulty 
	// in travelling (think ice rink if you can skate) whilst 5 represents the 
	// most difficult. 9 indicates that we cannot pass.

	// Create an instance of the search class...

	AStarSearch<MapSearchNode> astarsearch;

	unsigned int SearchCount = 0;

	const unsigned int NumSearches = 1;

	Timer timer;

	while(SearchCount < NumSearches)
	{
		// TODO Filter this from Gavin
		// Create a start state
		MapSearchNode nodeStart;
		nodeStart.x = 3;
		nodeStart.y = 2; 

		// Define the goal state
		MapSearchNode nodeEnd;
		nodeEnd.x = MAP_WIDTH-1;						
		nodeEnd.y = MAP_HEIGHT-1; 
		
		// Set Start and goal states
		
		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

		unsigned int SearchState;
		unsigned int SearchSteps = 0;
		
		timer.start();
		do
		{
			SearchState = astarsearch.SearchStep();

			SearchSteps++;

		}
		while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
		
		timer.stop();

		if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
		{
			cout << "Search found goal state\n";

				MapSearchNode *node = astarsearch.GetSolutionStart();

	#if DISPLAY_SOLUTION
				cout << "Displaying solution\n";
	#endif
				int steps = 0;

				node->PrintNodeInfo();
				// std::list<Node> shortest_path = node;
				for( ;; )
				{
					node = astarsearch.GetSolutionNext();

					if( !node )
					{
						break;
					}

					node->PrintNodeInfo();
					steps ++;
				
				};

				cout << "Solution steps " << steps << endl;

				// Once you're done with the solution you can free the nodes up
				astarsearch.FreeSolutionNodes();

	
		}
		else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
		{
			cout << "Search terminated. Did not find goal state\n";
		
		}

		// Display the number of loops the search went through
		cout << "SearchSteps : " << SearchSteps << "\n";
		cout << "Search Time: " << timer.get_milliseconds() << "ms" << std::endl;
		SearchCount ++;

		astarsearch.EnsureMemoryFreed();
	}
	
	return 0;
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
