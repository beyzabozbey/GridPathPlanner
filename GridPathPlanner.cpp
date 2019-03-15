#include "GridPathPlanner.h"
#include <cmath>
#include <algorithm>

GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid_, xyLoc destination_, bool adaptive_, bool larger_g_) {
	grid = grid_;
	destination = destination_;
	adaptive = adaptive_;
	larger_g = larger_g_;
	expansions = 0;

	// creates the states grid
	for(int i=0; i < grid->GetHeight(); i++) {
		std::vector<State *> v;
		for(int j=0; j < grid->GetWidth(); j++) {

			// initialize h values with the manhattan distance
			int h = abs(destination.x - j) + abs(destination.y - i);

			State *s = new State(xyLoc(j, i), xyLoc(), 0, h, false, false);
			v.push_back(s);
		}
		states.push_back(v);
	}
	destinationG = -1;
}

// frees the allocated memory
GridPathPlanner::~GridPathPlanner(){
	for(int i=0; i < grid->GetHeight(); i++) {
		for(int j=0; j < grid->GetWidth(); j++) {
			delete states[i][j];
		}
	}
}

// resets all values and if it's adaptive A*, it updates the h values of expanded states
void GridPathPlanner::FindPathInitializer() {
	for(int i=0; i < grid->GetHeight(); i++) {
		for(int j=0; j < grid->GetWidth(); j++) {
			if(adaptive && destinationG != -1 && states[i][j]->inClosedSet) {
				states[i][j]->h = destinationG - states[i][j]->g;
			}
			states[i][j]->g = 0;
			states[i][j]->inOpenSet = false;
			states[i][j]->inClosedSet = false;
			states[i][j]->parent = xyLoc();
		}
	}
	destinationG = -1;
}

// fills the path vector by using the parent of each state
void GridPathPlanner::CreatePath(xyLoc start, std::vector<xyLoc>& path) {
	State *current = states[destination.y][destination.x];

	// creates the path by traversing from destination to start state
	while(grid->IsValidLocation(current->parent)) {
		path.push_back(current->loc);
		current = states[current->parent.y][current->parent.x];
	}
	path.push_back(start);

	// fixes the order to make the path from start to destination
	reverse(path.begin(), path.end());
}

void GridPathPlanner::FindPath(xyLoc start, std::vector<xyLoc> & path) {
	// TODO
	// Possible flow:
	// - Initialize data structures / open list
  // - Search until goal is selected for expansion
  // - Extract path
  // - Update heuristic if adaptive

	// openSet is used for the potential states to be expanded
	std::priority_queue<State *, std::vector<State *>, Comparator> openSet(Comparator(larger_g == true));
	expansions = 0;

	// resets all values and if it's adaptive A*, it updates the h values
	FindPathInitializer();
	path.clear();

	// if start or destination is not valid, terminate the function
	if(!grid->IsValidLocation(start) || !grid->IsValidLocation(destination))
		return;

	// neighbors array to make it easier to create neighbor states' coordinates
	int neighbors[4][2] = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}};
	State *s = states[start.y][start.x];
	openSet.push(s);

	while(!openSet.empty()) {
		State *current = openSet.top();
		openSet.pop();

		// if the state is already expanded, move on
		if(current->inClosedSet)
			continue;

		current->inClosedSet = true;
		if(current->loc == destination) {
			destinationG = current->g;
			CreatePath(start, path);
			return;
		}
		expansions++;

		for(int i=0; i < 4; i++) {
			int x = current->loc.x + neighbors[i][0];
			int y = current->loc.y + neighbors[i][1];
			if(!grid->IsValidLocation(xyLoc(x, y)) || grid->IsBlocked(xyLoc(x, y)) || states[y][x]->inClosedSet)
				continue;

			State *n = states[y][x];
			if(!(n->inOpenSet) || current->g + 1 < n->g) {
				n->g = current->g + 1;
				n->f = n->g + n->h;
				n->parent = current->loc;
				n->inOpenSet = true;
				openSet.push(n);
			}
		}
	}
}


int GridPathPlanner::GetHValue(xyLoc l) {
	// TODO
	return states[l.y][l.x]->h;
}

int GridPathPlanner::GetNumExpansionsFromLastSearch() {
	// TODO
	return expansions;
}

