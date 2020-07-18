//You will only be turning in this file
//Your solution will be graded based on it's runtime (smaller is better), 
//the optimality of the path you return (shorter is better), and the
//number of collisions along the path (it should be 0 in all cases).

//You must provide a function with the following prototype:
// ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes);
// Where: 
//    -startPos and goalPos are 2D start and goal positions
//    -centers and radii are arrays specifying the center and radius
//    -numObstacles specifies the number of obstacles
//    -nodePos is an array specifying the 2D position of roadmap nodes
//    -numNodes specifies the number of obstacles
// The function should return an ArrayList of node IDs (indexes into the nodePos array).
// This should provide a collision-free chain of direct paths from the start position
// to the position of each node, and finally to the goal position.
// If there is no collision-free path between the start and goal, return an ArrayList with
// the 0'th element of "-1".

// Your code can safely make the following assumptions:
//   - The variable maxNumNodes has been defined as a large static int, and it will
//     always be bigger than the numNodes variable passed into planPath()
//   - None of position in the nodePos array will be inside an obstacle
//   - The start and the goal position will never be inside an obstacle

// There are many useful functions in CollisionLibrary.pde and Vec2.pde
// which you can draw on in your implementation. Please add any additional 
// functionality you need to this file.

// Here we provide a simple PRM implementation to get you started.
// Be warned, this version has several important limitations.
// For example, it use BFS which will not provide the shortest path
// Also, it (wrongly) assumes the nodes closest to the start and goal
// are the best nodes to start/end on your path on. Be sure to fix 
// these and other issues as you work on this assignment (don't assume 
// this example funcationality is correct and copy it's mistakes!).

import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Collections;

//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node
ArrayList<Integer> nodesTouchingStart = new ArrayList<Integer>();
ArrayList<Integer> nodesTouchingGoal = new ArrayList<Integer>();

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

//This is probably a bad idea and you shouldn't use it...
int closestNode(Vec2 point, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();

  if (startPos == goalPos) {
    return null;
  }

  int startID = numNodes;
  int goalID = numNodes + 1;

  // Add start and end node to graph
  numNodes+=2;
  nodePos[startID] = startPos;
  nodePos[goalID] = goalPos;

  neighbors[startID] = new ArrayList<Integer>();
  neighbors[goalID] = new ArrayList<Integer>();
  nodesTouchingGoal = new ArrayList<Integer>();
  nodesTouchingStart = new ArrayList<Integer>();
  // Set up neighbors
  if (pointInCircleList(centers, radii, numObstacles, startPos) || pointInCircleList(centers, radii, numObstacles, goalPos)) {
    path.add(0,-1);
    return path;
  }
  for (int i = 0; i < numNodes; i++) {
    Vec2 startToNode = nodePos[i].minus(startPos);
    if (!rayCircleListIntersect(centers, radii, numObstacles, startPos, startToNode.normalized(), startToNode.length()).hit) {
      neighbors[i].add(startID);
      nodesTouchingStart.add(i);
      neighbors[startID].add(i);
    }
    Vec2 nodeToGoal = goalPos.minus(nodePos[i]);
    if (!rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], nodeToGoal.normalized(), nodeToGoal.length()).hit) {
      neighbors[i].add(goalID);
      nodesTouchingGoal.add(i);
      neighbors[goalID].add(i);
    }
  }

  path = runAStar(nodePos, numNodes, startID, goalID);

  // Reset neighbors
  for (int idx : nodesTouchingStart) {
    neighbors[idx].remove(neighbors[idx].size() - 1);
  }
  for (int idx : nodesTouchingGoal) {
    neighbors[idx].remove(neighbors[idx].size() - 1);
  }
  
  return path;
}

class NodeHeuristic {
  float distanceFromStart;
  float heuristicToGoal;
  int uid;
  boolean visited;
  NodeHeuristic parent;
  Vec2 pos;
}

class SortByPathLength implements Comparator<NodeHeuristic> {
  public int compare(NodeHeuristic a, NodeHeuristic b) {
    return (int)((a.heuristicToGoal + a.distanceFromStart) - (b.heuristicToGoal + b.distanceFromStart));
  }
}

//A*
ArrayList<Integer> runAStar(Vec2[] nodePos, int numNodes, int startID, int goalID) {
  NodeHeuristic[] nodes = new NodeHeuristic[numNodes];
  Comparator<NodeHeuristic> comparator = new SortByPathLength();
  PriorityQueue<NodeHeuristic> fringeQueue = new PriorityQueue<NodeHeuristic>(10, comparator);
  ArrayList<Integer> path = new ArrayList(); // Will resolve to final path

  // 0.) Clear visit tags and parent pointers, and define all heuristics
  for (int i = 0; i < numNodes; i++) {
    visited[i] = false;
    parent[i] = -1;
    nodes[i] = new NodeHeuristic();
    nodes[i].distanceFromStart = Float.MAX_VALUE;
    nodes[i].heuristicToGoal = nodePos[i].minus(nodePos[goalID]).length();
    nodes[i].uid = i;
    nodes[i].visited = false;
    nodes[i].pos = nodePos[i];
  }

  // 0.1.) Special case is first pt.
  nodes[startID].distanceFromStart = 0;
  fringeQueue.add(nodes[startID]);

  NodeHeuristic currentNode = fringeQueue.peek();
  
  while (fringeQueue.size() > 0) {
    // 1.) Choose the path with shortest length + heuristic and update visited
    currentNode = fringeQueue.poll();

    if (currentNode.uid == goalID) {
      // We've found the goal, but we don't yet know if it is optimal. Regardless, we don't get neighbor
      return reconstructPath(currentNode);
    }

    for (int neighbor : neighbors[currentNode.uid]) {
      NodeHeuristic neighborNode = nodes[neighbor];
      // if (neighborNode.visited == false) {
        // neighborNode.visited = true;
        float newDistance = currentNode.distanceFromStart + currentNode.pos.distanceTo(neighborNode.pos);
        if (newDistance < neighborNode.distanceFromStart) {
          // We can safely ignore nodes that have a distance greater than the min path because the heuristic will always be a lower bound
          neighborNode.parent = currentNode;
          neighborNode.distanceFromStart = newDistance;
          fringeQueue.add(neighborNode);
        }
      // }
    }
  }

  path.add(0,-1);
  return path;
}

public ArrayList<Integer> reconstructPath(NodeHeuristic goal) {
  ArrayList<Integer> path = new ArrayList<Integer>();
  // Reconstruct path from current node
  NodeHeuristic curNode = goal;
  while (curNode != null) {
    path.add(curNode.uid);
    curNode = curNode.parent;
  }

  Collections.reverse(path);

  return path;
}

//BFS (Breadth First Search)
ArrayList<Integer> runBFS(Vec2[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }

  //println("\nBeginning Search");
  
  visited[startID] = true;
  fringe.add(startID);
  //println("Adding node", startID, "(start) to the fringe.");
  //println(" Current Fringe: ", fringe);
  
  while (fringe.size() > 0){
    int currentNode = fringe.get(0);
    fringe.remove(0);
    if (currentNode == goalID){
      //println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        fringe.add(neighborNode);
        //println("Added node", neighborNode, "to the fringe.");
        //println(" Current Fringe: ", fringe);
      }
    } 
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  
  return path;
}


void drawPRMGraph() {
    push();
    stroke(0,255,255);
    strokeWeight(5);
    for (int i = 0; i < numNodes; i++) {
        point(nodePos[i].x, -3, nodePos[i].y);
    }
    // stroke(255,0,0);
    // strokeWeight(0.3);
    // for (int i = 0; i < numNodes; i++) {
    //   Vec3 pos = new Vec3(nodePos[i].x, -3, nodePos[i].y);
    //   for (int neighbor : neighbors[i]) {
    //     Vec3 neighborPos = new Vec3(nodePos[neighbor].x, -3, nodePos[neighbor].y);
    //     line(pos.x, pos.y, pos.z, neighborPos.x, neighborPos.y, neighborPos.z);
    //   }
    // }
    pop();
}

// Vec2 proportionAlongPath(float t, int[] path) {
//   float totalLength = 0;
//   for (int i : path) {
//     totalLength += nodePos[i].length();
//   }
//   float distanceAlongPath = totalLength*t;
//   float tmpLength = 0;
//   for (int i = 0; i < path.length - 1; i++) {
//     int idx1 = path[i];
//     int idx2 = path[i+1];
//     Vec2 btwnNodes = nodePos[idx2].minus(nodePos[idx1]);
//     tmpLength += btwnNodes.length();
//     if (tmpLength > distanceAlongPath) {
//       float segmentT = (distanceAlongPath - (tmpLength - btwnNodes.length())) / btwnNodes.length();
//       return btwnNodes.normalized().times(segmentT);
//     }
//   }
//   return null;
// }

Vec2 positionAlongPath(float distanceTravelled, int[] path) {
  float totalLength = 0;
  for (int i = 0; i < path.length - 1; i++) {
    int idx1 = path[i];
    int idx2 = path[i+1];
    Vec2 btwnNodes = nodePos[idx2].minus(nodePos[idx1]);
    totalLength += btwnNodes.length();
    if (totalLength > distanceTravelled) {
      return nodePos[idx1].plus(btwnNodes.normalized().times(distanceTravelled - (totalLength - btwnNodes.length())));
    }
  }
  return null;
}