/******************************************************************************
 * File: Trailblazer.cpp
 *
 * Implementation of the graph algorithms that comprise the Trailblazer
 * assignment.
 */

#include "Trailblazer.h"
#include "TrailblazerCosts.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"
#include "random.h"

struct Node
{
    double distanceFromStart;
    double height;
    Color visitColor;
    Loc location;
    Node* parent;
};

struct MazeNode
{
    int clusterNo;
    Loc location;
};

struct MazeEdge
{
    double weight = 0.0;
    MazeNode* first;
    MazeNode* second;
};


using namespace std;

//Shortest Path Functions
Grid<Node*> allNodesGray(Grid<double>& world);
void colorCellWithCost(Node* nextNode, double distance, Color color);
Node* getTheStartingNode(Grid<Node*>& searchWorld, Loc startingLocation);
Set<Node*> allNeighbours(Node* currentNode, Grid<Node*> searchWorld,Grid<double>& world);

//Maze Functions



//Auxilary Functions
void printSet(Set<Node*> aSet);
void printVector(Vector<Loc> vector);
Vector<Loc> reverseVector(Vector<Loc> vector);

/* Function: shortestPath
 * 
 * Finds the shortest path between the locations given by start and end in the
 * specified world.	 The cost of moving from one edge to the next is specified
 * by the given cost function.	The resulting path is then returned as a
 * Vector<Loc> containing the locations to visit in the order in which they
 * would be visited.	If no path is found, this function should report an
 * error.
 *
 * In Part Two of this assignment, you will need to add an additional parameter
 * to this function that represents the heuristic to use while performing the
 * search.  Make sure to update both this implementation prototype and the
 * function prototype in Trailblazer.h.
 */
Vector<Loc>
shortestPath(Loc start,
             Loc end,
             Grid<double>& world,
             double costFn(Loc from, Loc to, Grid<double>& world),
             double heuristic(Loc start, Loc end, Grid<double>& world))
{
    //cout << "New World Created !" << endl;

    //Marks all the nodes in the world as gray
    Grid<Node*> searchWorld = allNodesGray(world);
    
    //Create a priority queue for visit order
    TrailblazerPQueue<Node*> visitingOrder = *new TrailblazerPQueue<Node*>;
    Vector<Loc> path;
    
    
    //Mark the initial node as yellow
    Node* startingNode = getTheStartingNode(searchWorld, start);
    startingNode->parent = NULL;

    startingNode->visitColor = YELLOW;
    visitingOrder.enqueue(startingNode, heuristic(start, end, world));
    
    Node* currentNode = NULL;
    
    while(!visitingOrder.isEmpty())
    {
        currentNode = visitingOrder.dequeueMin();

        currentNode->visitColor = GREEN;
        colorCell(world, currentNode->location, GREEN); //Graphical coloring

        if(currentNode->location == end)
        {
            //Reached to destination
            break;
        }

        Set<Node*> connectedNodes = allNeighbours(currentNode, searchWorld, world);
        
        printSet(connectedNodes);
        
        foreach(Node* nextNode in connectedNodes)
        {
            double tCost = terrainCost(currentNode->location, nextNode->location, world);
            double hCost = heuristic(nextNode->location, end, world);
            
            //cout << counter << "-Neighbour" << endl;
            
            if(nextNode->visitColor == GRAY)
            {
                nextNode->visitColor = YELLOW;
                
                //Set the distance
                
                nextNode->distanceFromStart = currentNode->distanceFromStart + tCost;
                nextNode->parent = currentNode;
                visitingOrder.enqueue(nextNode, nextNode->distanceFromStart + hCost);
                colorCell(world, nextNode->location, nextNode->visitColor);
            }
            else if(nextNode->visitColor == YELLOW && nextNode->distanceFromStart > currentNode->distanceFromStart + tCost)
            {
                nextNode->distanceFromStart = currentNode->distanceFromStart + tCost;
                nextNode->parent = currentNode;
                visitingOrder.decreaseKey(nextNode, nextNode->distanceFromStart + hCost);
            }
        }
    }
    
    
    while (currentNode != NULL)
    {
        path.add(currentNode->location);
        currentNode = currentNode->parent;
    }

    return reverseVector(path);;
}

Grid<Node*> allNodesGray(Grid<double>& world)
{
    Grid<Node*> searchWorld = *new Grid<Node*>(world.nRows, world.nCols);
    
    for(int i = 0 ; i < world.nRows ; i++)
    {
        for(int j = 0 ; j < world.nCols ; j++)
        {
            Loc newLocation = *new Loc;
            newLocation.row = i;
            newLocation.col = j;
            
            Node* newNode = new Node;
            newNode->location = newLocation;
            newNode->distanceFromStart = 0.0;
            newNode->visitColor = GRAY;
            newNode->height = world.get(i, j);
            newNode->parent = NULL;
            
            searchWorld.set(i, j, newNode);
        }        
    }
    
    return searchWorld;
}

void colorCellWithCost(Node* nextNode, double distance, Color color)
{
    nextNode->visitColor = color;
    nextNode->distanceFromStart = distance;
}

Node* getTheStartingNode(Grid<Node*>& searchWorld, Loc startingLocation)
{
    return searchWorld.get(startingLocation.row, startingLocation.col);
}

Set<Node*> allNeighbours(Node* currentNode, Grid<Node*> searchWorld, Grid<double>& world)
{
    Set<Node*> neighbourSet;
    
    int currentRow = currentNode->location.row;
    int currentColumn = currentNode->location.col;

    if(searchWorld.inBounds(currentRow-1, currentColumn-1))
    {
        Node* eligableNeighbour = searchWorld.get(currentRow-1, currentColumn-1);
        
        if(eligableNeighbour->visitColor != GREEN)
        {
            neighbourSet.add(eligableNeighbour);
        }
    }
    
    if(searchWorld.inBounds(currentRow-1, currentColumn))
    {
        Node* eligableNeighbour = searchWorld.get(currentRow-1, currentColumn);
        
        if(eligableNeighbour->visitColor != GREEN)
        {
            neighbourSet.add(eligableNeighbour);
        }
    }
    
    if(searchWorld.inBounds(currentRow-1, currentColumn+1))
    {
        Node* eligableNeighbour = searchWorld.get(currentRow-1, currentColumn+1);
        
        if(eligableNeighbour->visitColor != GREEN)
        {
            neighbourSet.add(eligableNeighbour);
        }
    }
    
    if(searchWorld.inBounds(currentRow, currentColumn-1))
    {
        Node* eligableNeighbour = searchWorld.get(currentRow, currentColumn-1);
        
        if(eligableNeighbour->visitColor != GREEN)
        {
            neighbourSet.add(eligableNeighbour);
        }
    }
    
    if(searchWorld.inBounds(currentRow, currentColumn+1))
    {
        Node* eligableNeighbour = searchWorld.get(currentRow, currentColumn+1);
        
        if(eligableNeighbour->visitColor != GREEN)
        {
            neighbourSet.add(eligableNeighbour);
        }
    }
    
    if(searchWorld.inBounds(currentRow+1, currentColumn-1))
    {
        Node* eligableNeighbour = searchWorld.get(currentRow+1, currentColumn-1);
        
        if(eligableNeighbour->visitColor != GREEN)
        {
            neighbourSet.add(eligableNeighbour);
        }
    }
    
    if(searchWorld.inBounds(currentRow+1, currentColumn))
    {
        Node* eligableNeighbour = searchWorld.get(currentRow+1, currentColumn);
        
        if(eligableNeighbour->visitColor != GREEN)
        {
            neighbourSet.add(eligableNeighbour);
        }
    }
    
    if(searchWorld.inBounds(currentRow+1, currentColumn+1))
    {
        Node* eligableNeighbour = searchWorld.get(currentRow+1, currentColumn+1);
        
        if(eligableNeighbour->visitColor != GREEN)
        {
            neighbourSet.add(eligableNeighbour);
        }
    }
    
    return neighbourSet;
}


Vector<Loc> reverseVector(Vector<Loc> vector)
{
    Vector<Loc> reverseVector;
    
    for(int i = 0 ; i < vector.size() ; i++)
    {
        reverseVector.add(vector[i]);
    }
    
    return reverseVector;
    
}

void printSet(Set<Node*> aSet)
{    
    foreach(Node* node in aSet)
    {
        Loc loc = node->location;
        
        //cout << "Row: " << loc->row << " Column: " << loc->col << endl;
    }
}

void printVector(Vector<Loc> vector)
{
    cout << "Path:" << endl;
    for(int i = 0 ; i < vector.size() ; i++)
    {
        cout << "(" << vector[i].row << "-" << vector[i].col << ")" << endl;
    }
    
    cout << endl;
}

//MAZE

Set<Edge> createMaze(int numRows, int numCols)
{
    Grid<MazeEdge*> mazeGrid(numRows , numCols);
    TrailblazerPQueue<MazeEdge*> edgeQueue;
    Set<Edge> mazeEdges;
    
    int numberOfClusters = numCols * numRows;
    int clusterCount = 0 ;
    
    for (int i = 0 ; i < mazeGrid.nRows; i++)
    {
        for (int j = 0 ; j < mazeGrid.nCols; j++)
        {
            MazeEdge* nextEdge = new MazeEdge;
            
            Loc location;
            location.row = i;
            location.col = j;
            
            MazeNode* firstNode = new MazeNode;
            firstNode->clusterNo = clusterCount;
            nextEdge->first = firstNode;

            MazeNode* secondNode = new MazeNode;
            secondNode->clusterNo = clusterCount + 1;
            nextEdge->second = secondNode;

            
            nextEdge->first->location = location;
            Loc nextLocation = location;
            nextLocation.col++;
            
            nextEdge->first->location = nextLocation;
            nextEdge->weight = randomReal(0, 1);
            
            edgeQueue.enqueue(nextEdge, nextEdge->weight);
        }
    }

    while (numberOfClusters > 3)
    {
        MazeEdge* edge  = edgeQueue.dequeueMin();
        Edge resultEdge;
        
        if(edge->first->clusterNo != edge->second->clusterNo)
        {
            edge->first->clusterNo = edge->second->clusterNo;
            resultEdge.start = edge->first->location;
            resultEdge.end = edge->second->location;
            numberOfClusters--;
            
            mazeEdges = mazeEdges + resultEdge;
        }
    }
    
    return mazeEdges;
}



















