#ifndef GRID_PATH_PLANNER_H
#define GRID_PATH_PLANNER_H

#include "PartiallyKnownGrid.h"
#include <queue>
#include <vector>

// State struct used to store more values such as f, g, h
struct State {
	xyLoc loc;
	xyLoc parent;
	int g;
	int f;
	int h;
	bool inClosedSet;
	bool inOpenSet;
	State(xyLoc loc_, xyLoc parent_, int g_, int h_, bool inOpenSet_, bool inClosedSet_) {
		loc = loc_;
		parent = parent_;
		g = g_;
		h = h_;
		f = g + h;
		inOpenSet = inOpenSet_;
		inClosedSet = inClosedSet_;
	}
};

// custom comparator for openSet (priority_queue) 
struct Comparator {
	bool larger_g;
	Comparator(bool larger_g_) { larger_g = larger_g_; }
	bool operator()(const State *lhs, const State *rhs) {
		if(lhs->f == rhs->f) {
			return larger_g ? lhs->g < rhs->g : lhs->g > rhs->g;
		}
		return lhs->f > rhs->f;
	}
};

class GridPathPlanner{
public:
	GridPathPlanner(PartiallyKnownGrid* grid_, xyLoc destination_, bool adaptive_, bool larger_g_);
	~GridPathPlanner();
	
	// Finds a path from the "start" to the target (set in constructor).
	// Fills the "path" vector with a sequence of xyLocs.
	void FindPath(xyLoc start, std::vector<xyLoc> & path);

	// Return the current heuristic distance to the target.
	int GetHValue(xyLoc l);

	// Return the number of expanded nodes in the most recent search.
	int GetNumExpansionsFromLastSearch();
		
private:
	PartiallyKnownGrid* grid;	// Partially known grid (updates automatically as the agent moves.
	bool adaptive;						// If set, the heuristic should be updated after each search.
	xyLoc destination;				// Fixed goal cell.
	bool larger_g;	// If set to true, your search should tie-break towards larger g-values.
									// If set to false, your search should tie-break towards smaller g-values.
	
	int expansions; // keeps track of the number of expansions
	std::vector<std::vector<State *> > states; // states grid to keep track of g, h and f values
	int destinationG; // stores the g value of the destination. if there is no path, then it's -1

	// helper functions
	// creates the path vector by using the parent of each state
	void FindPathInitializer();

	// resets all values and if it's adaptive A*, it updates the h values of expanded states
	void CreatePath(xyLoc start, std::vector<xyLoc>& path);
};

#endif

