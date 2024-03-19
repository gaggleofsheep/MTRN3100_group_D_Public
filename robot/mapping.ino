#include "robot.hpp"

String headings = "NESW";

void changeHeadingLeft() {
    int currHeadIdx = headings.indexOf(head);
    int newHeadIdx = currHeadIdx - 1;
    if (currHeadIdx == 0) {
        int newHeadIdx = 3;
    }
    head = headings[newHeadIdx];
}

void changeHeadingRight() {
    int currHeadIdx = headings.indexOf(head);
    int newHeadIdx = currHeadIdx + 1;
    if (currHeadIdx == 3) {
        int newHeadIdx = 0;
    }
    head = headings[newHeadIdx];
}

void changeCoordinates() {
    switch (head)
    {
    case 'N':
        row--;
        break;
    case 'S':
        row++;
        break;
    case 'E':
        col++;
        break;
    case 'W':
        col--;
        break;
    }

}

// returns as {row, col, head} string, e.g. {2, 3, S}
String getPose() {
    return String("{" + String(row) + ", " + String(col) + ", " + head + "}");
}

auto pos2index = [](int const row, int const col) -> int {
    // Verify row/col is valid.
    if ((row < 0 || row >= numRows) || (col < 0 || col >= numCols)) {
        return -1;
    }
    return row * numCols + col + 1;
};

// Helper function to insert edges into maze.
// Graph is a directed graph so need to do both directions.
void insert_edge(int cell1, int cell2) {
    mazeGraph.insert_edge(cell1, cell2, true);
    mazeGraph.insert_edge(cell2, cell1, true);
}

// Helper funciton for updateGraph
void neighbourCell(char direction) {
    switch (direction)
    {
    case 'N':
        nRow = row - 1;
        nCol = col;
        break; 
    case 'S':
        nRow = row + 1;
        nCol = col;
        break;
    case 'E':
        nRow = row;
        nCol = col + 1;
        break;
    case 'W':
        nRow = row;
        nCol = col - 1;
        break;
    }
}

// Helper function for updateGraph
char getDirection(int direction) {
    int currHeadIdx = headings.indexOf(head);
    int newHeadIdx = currHeadIdx + direction;
    if (currHeadIdx == 3 && direction == 1) {
        newHeadIdx = 0;
    } else if (currHeadIdx == 0 && direction == -1) {
        newHeadIdx = 3;
    }
    return headings[newHeadIdx];
}

// Used for autonomous mapping
// is Called after every movement to a newly visited cell
// Maps maze during exploration

void updateGraph() {
    detectWalls();
    for (int i = 0; i <= 2; i++) {
        if (currentWallPosition[i]) { 
            // if walls are detected properly, means any type checking for 
            // cells out of bounds is unnecessary 
            continue;
        }
        char direction = getDirection(i - 1);
        neighbourCell(direction);
        int nCellIdx = pos2index(nRow, nCol);
        int currCellIdx = pos2index(row, col);
        insert_edge(nCellIdx, currCellIdx);
    }
}

void shortestPath() {
    mtrn3100::LinkedList<int> queue;
    int goal = destIdx;
    int beginning = pos2index(row, col);
    bool visited[numRows*numCols] = {false};
    int parents[numRows*numCols] = {-1};
    int counter = 0;

    queue.push_front(beginning);
    visited[beginning - 1] = true;
    while (!queue.empty()) {
        counter++;
        int front = queue.pop_front();
        auto neighbours = mazeGraph.nodes(front);
    
        // For all neighbours of front
        for (size_t i = 0; i < neighbours.size(); i++) {
            if (!visited[neighbours[i] - 1]) {
                // Enqueue and mark as visited
                queue.push_back(neighbours[i]);
                visited[neighbours[i] - 1] = true;
                parents[neighbours[i] - 1] = front;
            }
        }
    }
    int curr = goal;
    int arrSize = 0;

    while (curr != beginning) {
        curr = parents[curr - 1];
        arrSize++;
    }
    arrSize++;
    int pathCount = arrSize - 1;
    curr = goal;
    for (int i = 0; i < (numRows*numCols); i++) {
        path[i] = -1;
    }
    while (curr != beginning) {
        path[pathCount] = curr;  
        curr = parents[curr - 1];
        pathCount--;
    }
    path[0] = beginning;
}

String pathToMotionPlan() {
    shortestPath();
    int currNode = path[0];
    char currHead = head;
    String plan = "";
    for (int i = 1; i < (numCols*numRows); i++) {
        int nxtNode = path[i];
        if (nxtNode == -1) {
            return plan;
        }
        if (nxtNode == (currNode - 1)) {
            plan = plan + mtrn3100::turnMotionPlan(currHead, 'W');
            currHead = 'W';
        } else if (nxtNode == (currNode + 1)) {
            plan = plan + mtrn3100::turnMotionPlan(currHead, 'E');
            currHead = 'E';
        } else if (nxtNode == (currNode + numCols)) {
            plan = plan + mtrn3100::turnMotionPlan(currHead, 'S');
            currHead = 'S';
        } else if (nxtNode == (currNode - numCols)) {
            plan = plan + mtrn3100::turnMotionPlan(currHead, 'N');
            currHead = 'N';
        }
        plan = plan + "F";
        currNode = nxtNode;
    }
    return plan;
}

// return motion plan string e.g. "FRLLFRFF"
// May end up not being needed
String createMotionPlan() {
    return pathToMotionPlan();
}

// Function to get maze from CV and convert into Graph for maze solving
void getGraph() {
    Serial3.println("Please enter your maze: ");
    String asciiMaze = "";
    while(1) {
      if (Serial3.available()) {
        asciiMaze = Serial3.readStringUntil('t');
        asciiMaze.replace('n', '\n');
        Serial.println(asciiMaze); 
        Serial3.println(asciiMaze); 
        asciiMaze.replace('#', ' ');
        break;
      }
    }

    char* str_maze = asciiMaze.c_str(); 
    mazeGraph = mtrn3100::ascii2graph(str_maze);
 }

void getSrtAndDest() {
    Serial3.println("Enter your Starting and desired Destination (SSHDD): ");
    while (1) {
        if (Serial3.available()) {
            // should read in a string of format SSHDD: 
            // Start row, Start col, start Heading, Destination row, Destination Col
            String mazeInfo = Serial3.readString(); 
            row = mazeInfo.charAt(0) - '0';
            col = mazeInfo.charAt(1) - '0';
            head = mazeInfo.charAt(2);
            int destRow = mazeInfo.charAt(3) - '0';
            int destCol = mazeInfo.charAt(4) - '0';
            destIdx = pos2index(destRow, destCol);
            break;
        }
    }
}


void autoMapping() {
    mtrn3100::LinkedList<int> stack;
    bool visited[numRows*numCols];
    mazeGraph = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,
                17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
                32,33,34,35,36,37,38,39,40,41,42,43,44,45}; // initialise mazeGraph with nodes
    stack.push_front(1);
    visited[0] = true;
    while (!stack.empty()) {
        int node = stack.pop_front();
        destIdx = node;
        if (createMotionPlan().length() > 0) { // shortest path called inside createMotionPlan
            followMotionPlan(createMotionPlan());
        }
        updateGraph(); // detect walls called inside updateGraph

        mtrn3100::LinkedList<int> neighbourNodes = mazeGraph.nodes(node);
        while (!neighbourNodes.empty()) {
            int neighbourNode = neighbourNodes.pop_front();
            if (!visited[neighbourNode - 1]) {
                visited[neighbourNode - 1] = true;
                stack.push_front(neighbourNode);
            }

        }
    }
    char* maz = mtrn3100::graph2ascii(mazeGraph);
    String ma(maz);
    ma.replace(' ', '#');
    Serial3.println(ma);
}
