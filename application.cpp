#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue> // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "json.hpp"
using json = nlohmann::json;


using namespace std;

double INF = numeric_limits<double>::max();

void buildGraph(istream &input, graph<long long, double> &g,
                vector<BuildingInfo> &buildings,
                unordered_map<long long, Coordinates> &coords) {
  json jStream;
  input >> jStream;

  for(const auto& bng : jStream["buildings"]){
    BuildingInfo buildData;
    buildData.id = bng["id"];
    buildData.name = bng["name"];
    buildData.abbr = bng["abbr"];
    buildData.location.lat = bng["lat"];
    buildData.location.lon = bng["lon"];

    buildings.push_back(buildData);
    //coords[buildData.id] = buildData.location;
    g.addVertex(buildData.id);
  }

  for(const auto &wp : jStream["waypoints"]){
    long long id = wp["id"];
    Coordinates cd;
    cd.lat = wp["lat"];
    cd.lon = wp["lon"];

    coords[id] = cd;
    g.addVertex(id);
  }

  for(const auto &fw : jStream["footways"]){
    for(size_t i = 0; i + 1 < fw.size(); i++){
      long long la = fw[i];
      long long lb = fw[i + 1];

      Coordinates a = coords[la];
      Coordinates b = coords[lb];

      double distD = distBetween2Points(a, b);

      g.addEdge(la, lb, distD);
      g.addEdge(lb, la, distD);
    }
  }

  unordered_set<long long> bID;
  for(const auto & br : buildings){
    bID.insert(br.id);
  }

  const double maxDist = 0.036;

  for(const auto &br : buildings){
    long long idB = br.id;
    Coordinates bc = br.location;

    for(const auto &p : coords){
      long long vr = p.first;
      if(vr == idB){
        continue;
      }
      if(bID.count(vr)){
        continue;
      }

      double distTwo = distBetween2Points(bc, p.second);
      if(distTwo <= maxDist){
        g.addEdge(idB, vr, distTwo);
        g.addEdge(vr, idB, distTwo);
      }
    }
  }
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo> &buildings,
                             const string &query) {
  for (const BuildingInfo &building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo> &buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo &building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

class prioritize { 
  public:
    bool operator()(const pair <long long, double>& p1,
                    const pair <long long, double>& p2) const{
      return p1.second > p2.second;
    }
};

vector<long long> dijkstra(const graph<long long, double> &G, long long start,
                           long long target,
                           const set<long long> &ignoreNodes) {
  if(start == target){
    return vector<long long>{start};
  }
  unordered_map<long long, double> mapDist;
  unordered_map<long long, long long> temp;

  for(long long vert : G.getVertices()){
    mapDist[vert] = INF;
  }
  if(mapDist.find(start) == mapDist.end() || mapDist.find(target) == mapDist.end()){
    return {};
  }
  mapDist[start] = 0.0;

  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> queueList;
  queueList.push({start, 0.0});
  while(!queueList.empty()){
    long long firstV = queueList.top().first;
    double secV = queueList.top().second;
    queueList.pop();

    if(secV != mapDist[firstV]){
      continue;
    }

    if(firstV == target){
      break;
    }

    if(ignoreNodes.count(firstV) && firstV != start && firstV != target){
      continue;
    }
    for(long long neigh : G.neighbors(firstV)){
      if(ignoreNodes.count(neigh) && neigh != start && neigh != target){
        continue;
      }

      double var;
      bool weightBool = G.getWeight(firstV, neigh, var);
      if(!weightBool){
        continue;
      }

      double disD = mapDist[firstV] + var;
      if(disD < mapDist[neigh]){
        mapDist[neigh] = disD;
        temp[neigh] = firstV;
        queueList.push({neigh, disD});
      }
    }
  }
  if(mapDist[target] == INF){
    return {};
  }

  vector<long long> trailVec;
  long long curr = target;
  trailVec.push_back(curr);
  while(curr != start){
    auto varIt = temp.find(curr);
    if(varIt == temp.end()){
      return {};
    }
    curr = varIt->second;
    trailVec.push_back(curr);
  }
  reverse(trailVec.begin(), trailVec.end());
  return trailVec;
  
}

double pathLength(const graph<long long, double> &G,
                  const vector<long long> &path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long> &path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

// Honestly this function is just a holdover from an old version of the project
void application(const vector<BuildingInfo> &buildings,
                 const graph<long long, double> &G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto &building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
