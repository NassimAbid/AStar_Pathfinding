#include "Astar.h";

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>

// map of all the nodes that have been tried
static int ClosedMap[ROW][COL]; 
// map of all of the nodes yet to be tried
static int OpenMap[ROW][COL]; 
// map of directions
static int DirectionMap[ROW][COL]; 
// number of directions 
const int NumOfDir = 4; 

static int diffx[NumOfDir]={1, 0, -1, 0};
static int diffy[NumOfDir]={0, 1, 0, -1};


Node::Node(int xposition, int ypposition, int tavelled, int Total) {
	xPos = xposition;
	yPos = ypposition;
	disTravelled = tavelled;
	Distance = Total;
}

int Node::getxPosition() const {
	
	return xPos; 

	}

int Node::getyPosition() const {
	
	return yPos;
	
	}

int Node::getLevel() const { 
	
	return disTravelled;

	}

int Node::getDistance() const {
	
	return Distance; 
	
	}


void Node::UpdateCost(const int & xDest, const int & yDest) {
	// this calculates choice
	Distance = disTravelled + GetHeuristic(xDest, yDest) * 10;
}



// Estimation function for the remaining distance to the goal.
const int & Node::GetHeuristic(const int & xDest, const int & yDest) const
{
	
	static int xDist, yDist, heur;
	
	xDist = xDest - xPos;
	yDist = yDest - yPos;



	// Manhattan distance
	heur =abs(xDist)+abs(yDist);



	return(heur);
}

// Determine priority in the queue based on the cost 
bool operator<(const Node & a, const Node & b)
{
	return a.getDistance() > b.getDistance();
}

//A star algorithm which returns a string of the path the player will take
std::string AstarPath(int** grid, const int & xStart, const int & yStart, const int & xFinish, const int & yFinish){

	 // list of closed nodes
	static std::priority_queue<Node> Closed;
	// list of open nodes
	static std::priority_queue<Node> Open;
	static Node* CurrNode;
	static Node* ChildNode;
	static int goTo, xpos, ypos, Xmove, Ymove;
	static char c;



	// create the start node and push into list of open nodes
	CurrNode = new Node(xStart, yStart, 0, 0);

	//Gets heuristic to the final destination
	CurrNode->UpdateCost(xFinish, yFinish);

	//pushes the current node to the open list
	Open.push(*CurrNode);

	//puts into open map at position 0,0
	OpenMap[xpos][ypos] = CurrNode->getDistance(); 

	// A star search part
	while (!Open.empty())
	{
		
		//gets the current node on the top of the open queue
		CurrNode = new Node(Open.top().getxPosition(), Open.top().getyPosition(),
			Open.top().getLevel(), Open.top().getDistance());

		xpos = CurrNode->getxPosition();

		ypos = CurrNode->getyPosition();

		// remove the node from the open list with its details stored in CurrNode
		Open.pop(); 

		//mark as empty
		OpenMap[xpos][ypos] = 0;
		
		// mark as taken on the closed
		ClosedMap[xpos][ypos] = 1;

		
		if (xpos == xFinish && ypos == yFinish) {

			// generates a path from the finish point to the start point by using the directions
			std::string path = "";
			while (!(xpos == xStart && ypos == yStart)) {
				
				goTo = DirectionMap[xpos][ypos];
				c = '0' + (goTo + NumOfDir / 2) % NumOfDir;
				path = c + path;
				xpos += diffx[goTo];
				ypos += diffy[goTo];

			}

		
			delete CurrNode;

			return path;
		}

		// generate moves (child nodes) in all possible directions
		for (int i = 0; i < NumOfDir; i++) {
			Xmove = xpos + diffx[i];
			Ymove = ypos + diffy[i];

			if (!(Xmove<0 || Xmove>ROW - 1 || 
				Ymove<0 || Ymove>COL - 1 || 
				grid[Xmove][Ymove] == 1|| 
				ClosedMap[Xmove][Ymove] == 1)) {
				
				//this creates a child node
				ChildNode = new Node(Xmove, Ymove, CurrNode->getLevel(), CurrNode->getDistance());

				//get the cost of the child node
				ChildNode->UpdateCost(xFinish, yFinish);

				//if it doesnt exist in the open map it will be added

				if (OpenMap[Xmove][Ymove] == 0) {

					OpenMap[Xmove][Ymove] = ChildNode->getDistance();
					Open.push(*ChildNode);
					// mark its parent node direction
					DirectionMap[Xmove][Ymove] = (i + NumOfDir / 2) % NumOfDir;
				}
				else if (OpenMap[Xmove][Ymove] > ChildNode->getDistance()) {
					
					// update the cost on the open map
					OpenMap[Xmove][Ymove] = ChildNode->getDistance();
					
					// give a direction for the parent node
					DirectionMap[Xmove][Ymove] = (i + NumOfDir / 2) % NumOfDir;

				
					//replace the nodes
					while (!(Open.top().getxPosition() == Xmove &&
						Open.top().getyPosition() == Ymove)) {
						Closed.push(Open.top());
						Open.pop();
					}
					//get the node you want
					Open.pop();

					while (!Open.empty()) {
						Closed.push(Open.top());
						Open.pop();
					}
					// add the better node
					Closed.push(*ChildNode); 
				}
				else delete ChildNode; 
			}
		}
		delete CurrNode; 
	}
	return ""; 
}


// Create a grid of row by col
int** createGrid(int n, int m) {
	int** arr = new int*[n];
	for (int i = 0; i < n; ++i) {
		arr[i] = new int[m];
		for (int j = 0; j < m; ++j) {
			// make empty map
			arr[i][j] = 0; 
		}
	}
	return arr;
}

std::string runAstar(int** grid)
{
	srand(time(NULL));

	// create empty map
	for (int y = 0; y < COL; y++) {
		for (int x = 0; x < ROW; x++) 
			grid[x][y] = 0;
	}
	
	//fill with obstacles
	for (int i = 0; i < 50; i++){
		int blockx = rand() % ROW;
		int blocky = rand() % COL;
		grid[blockx][blocky] = 1;
		
	}




	// randomly select start and finish locations
	int xStar, yStar, xFin, yFin;

	int ranStart = rand() % 4;
	if (ranStart == 0) {
		xStar = 0;
		yStar = COL - 1; 
		xFin = ROW - 1;
		yFin = 0;
	}
	else if (ranStart == 1) {
		xStar = ROW / 2 - 1; 
		yStar = COL / 2 + 1; 
		xFin = ROW / 2 + 1; 
		yFin = COL / 2 - 1;
	}
	else if (ranStart == 2) {
		xStar = ROW / 2 - 1; 
		yStar = 0; 
		xFin = ROW / 2 + 1; 
		yFin = COL - 1;
	}
	else if (ranStart == 3) {
		xStar = 0; 
		yStar = COL / 2 - 1; 
		xFin = ROW - 1; 
		yFin = COL / 2 + 1;

	}


	std::string route = AstarPath(grid, xStar, yStar, xFin, yFin);
	
	if (route == "") 
		std::cout << "An empty route generated!" << std::endl;


	//Follows the wrote and writes which later gets used
	if (route.length() > 0)
	{
		int j; char Direction;
		int x = xStar;
		int y = yStar;
		grid[x][y] = 2;
		for (int i = 0;i < route.length();i++)
		{
			Direction = route.at(i);
			j = atoi(&Direction);
			x = x + diffx[j];
			y = y + diffy[j];
			grid[x][y] = 3;
		}
		grid[x][y] = 4;


	}

	return(route);
}
