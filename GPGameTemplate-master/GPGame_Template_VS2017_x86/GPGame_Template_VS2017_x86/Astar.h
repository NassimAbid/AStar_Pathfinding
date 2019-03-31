#pragma once
#include <queue>
#include <limits>
#include <cmath>
#define ROW 20
#define COL 20

class Node
{
	// current position
	int xPos;
	int yPos;
	// total distance already travelled to reach the node
	int disTravelled;
	// priority=level+remaining distance estimate
	int Distance;  // smaller: higher priority

public:
	Node(int xp, int yp, int d, int p);

	int getxPosition() const;
	int getyPosition() const;
	int getLevel() const;

	int getDistance() const;

	void UpdateCost(const int & xDest, const int & yDest);


	// Estimation function for the remaining distance to the goal.
	const int & GetHeuristic(const int & xDest, const int & yDest) const;
};

int** createGrid(int n, int m);

std::string runAstar(int** grid);