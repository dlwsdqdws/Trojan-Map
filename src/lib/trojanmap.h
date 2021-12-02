#ifndef TROJAN_MAP_H
#define TROJAN_MAP_H
#define DOT_SIZE 5
#define LINE_WIDTH 3

using namespace std;

#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#define INT_MAX 1e18
// A Node is the location of one point in the map.
class Node {
  public:
    Node(){};
    Node(const Node &n){id = n.id; lat = n.lat; lon = n.lon; name = n.name; neighbors = n.neighbors;};
    std::string id;    // A unique id assign to each point
    double lat;        // Latitude
    double lon;        // Longitude
    std::string name;  // Name of the location. E.g. "Bank of America".
    std::vector<std::string> neighbors;  // List of the ids of all neighbor points.
};

struct hashfun{
  size_t operator() (const Node& rhs) const{
    return hash<string>()(rhs.id); 
  }
};

struct equalfun{
  bool operator() (const Node& a, const Node& b) const{
    return a.id == b.id;
  }
};

struct individual{
  std::string gnome;
  double fitness;
  bool operator< (const individual& t) const{
    return fitness < t.fitness;
  }
};



class TrojanMap {
 public:
  // A map of ids to Nodes.
  std::unordered_map<std::string, Node> data;
  
  //-----------------------------------------------------
  // TODO: You do not and should not change the following functions:

  // Create the menu.
  void PrintMenu();

  // Create the Dynamic menu.
  // void DynamicPrintMenu();

  // Read in the data
  void CreateGraphFromCSVFile();

  // Visualization
  // Given a location id, plot the point on the map.
  void PlotPoint(std::string id);

  // Given a lat and lon, plot the point on the map.
  void PlotPoint(double lat, double lon);

  // Given a vector of location ids draws the path (connects the points)
  void PlotPath(std::vector<std::string> &location_ids);

  // Given a vector of location ids draws the points on the map (no path).
  void PlotPoints(std::vector<std::string> &location_ids);

  // Given a vector of location ids draws the points on the map with path.
  void PlotPointsandEdges(std::vector<std::string> &location_ids, std::vector<double> &square);

  // Given a vector of location ids draws the points with their order on the map (no path).
  void PlotPointsOrder(std::vector<std::string> &location_ids);

  // Given a vector of location ids and origin, draws the points with their label.
  void PlotPointsLabel(std::vector<std::string> &location_ids, std::string origin);

  // Create the videos of the progress to get the path
  void CreateAnimation(std::vector<std::vector<std::string>>);

  // Transform the location to the position on the map
  std::pair<double, double> GetPlotLocation(double lat, double lon);
  //-----------------------------------------------------
  // TODO: Implement these functions and create unit tests for them:

  // Get the Latitude of a Node given its id.
  double GetLat(std::string id);

  // Get the Longitude of a Node given its id.
  double GetLon(std::string id);

  // Get the name of a Node given its id.
  std::string GetName(std::string id);

  // Get the id given its name.
  std::string GetID(std::string name);

  // Get the neighbor ids of a Node.
  std::vector<std::string> GetNeighborIDs(std::string id);

  // Get the distance between 2 nodes.
  double CalculateDistance(const std::string &a, const std::string &b);

  // Calculates the total path length for the locations inside the vector.
  double CalculatePathLength(const std::vector<std::string> &path);

  // Returns a vector of names given a partial name.
  std::vector<std::string> Autocomplete(std::string name);

  // Returns lat and long of the given the name.
  std::pair<double, double> GetPosition(std::string name);

  // Given the name of two locations, it should return the **ids** of the nodes
  // on the shortest path.
  std::vector<std::string> CalculateShortestPath_Dijkstra(std::string location1_name,
                                                 std::string location2_name);
  std::vector<std::string> CalculateShortestPath_Bellman_Ford(std::string location1_name,
                                                 std::string location2_name);

  // Given CSV filename, it read and parse locations data from CSV file,
  // and return locations vector for topological sort problem.
  std::vector<std::string> ReadLocationsFromCSVFile(std::string locations_filename);
  
  // Given CSV filenames, it read and parse dependencise data from CSV file,
  // and return dependencies vector for topological sort problem.
  std::vector<std::vector<std::string>> ReadDependenciesFromCSVFile(std::string dependencies_filename);

  // Given a vector of location names, it should return a sorting of nodes
  // that satisfies the given dependencies.
  std::vector<std::string> DeliveringTrojan(std::vector<std::string> &location_names,
                                            std::vector<std::vector<std::string>> &dependencies);

  void backtrack(int u, vector<std::string>& ids, double& curp, double& maxp, std::vector<std::string>& path, std::vector<std::vector<std::string>>& res);
  void opt2_solver(std::vector<std::string>& ids, std::map<std::vector<std::string>, double>& mp, double& bestp, std::vector<std::vector<std::string>>& res);
  void opt2SA_solver(double& t, std::vector<std::string>& ids, std::map<std::vector<std::string>, double>& mp, double& bestp, std::vector<std::vector<std::string>>& res);
  void opt3_solver(std::vector<std::string>& ids, std::map<std::vector<std::string>, double>& mp, double& bestp, std::vector<std::vector<std::string>>& res);

  // Given a vector of location ids, it should reorder them such that the path
  // that covers all these points has the minimum length.
  // The return value is a pair where the first member is the total_path,
  // and the second member is the reordered vector of points.
  // (Notice that we don't find the optimal answer. You can return an estimated
  // path.)
  std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan(
      std::vector<std::string> &location_ids);

  
  std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_2opt(
      std::vector<std::string> &location_ids);

  std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_3opt(
      std::vector<std::string> &location_ids);

 std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_2optSA(
      std::vector<std::string> &location_ids);

  void simulate_anneal(std::vector<std::string>& ids, double& bestp);

  std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_SA(
      std::vector<std::string> &location_ids);

  std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_GA(
      std::vector<std::string> &location_ids);

  void TSPUtil(std::vector<std::vector<double>>& map, std::pair<double, std::vector<std::vector<std::string>>>& prog, double& rs);


  // Given a subgraph specified by a square-shape area, determine whether there is a
  // cycle or not in this subgraph.
  // vector square has 4 elements: left/right/top/bottom bound in order.
  bool dfs_cycle(Node& p, std::unordered_set<Node, hashfun, equalfun> & st, std::vector<double> &square, std::string fa);
  bool CycleDetection(std::vector<double> &square);

  // Given a location id and k, find the k closest points on the map
  void kclose_solver(std::string name, std::string origin, std::set<std::pair<double, std::string>>& distance, int k, std::unordered_map<std::string, double>& hash, double dist);
  // void kclose_solver(std::string& name, std::map<double, std::string>& mp, int k, std::unordered_map<std::string, double>& hash, double dist);
  void kclose_solver(std::string& name, std::string& origin, std::unordered_set<std::string>& st, std::priority_queue<std::pair<double, std::string>>& pq, int k);
  void kclose_solver(std::string& origin, std::priority_queue<std::pair<double, std::string>>& pq, std::unordered_set<std::string>& st, int k);
  std::vector<std::string> FindKClosestPoints(std::string name, int k);


  private:

  // Function to return a random number
  // from start and end
  int rand_num(int start, int end){
      int r = end - start;
      int rnum = start + rand() % r;
      return rnum;
  }

  // Function to check if the character
  // has already occurred in the string
  bool repeat(std::string s, char ch){
      for (int i = 0; i < s.size(); i++) {
          if (s[i] == ch)
              return true;
      }
      return false;
  }

  // Function to return a mutated GNOME
  // Mutated GNOME is a string
  // with a random interchange
  // of two genes to create variation in species
  std::string mutatedGene(std::string gnome, int V)
  {
      while (true) {
          int r = rand_num(1, V);
          int r1 = rand_num(1, V);
          if (r1 != r) {
              swap(gnome[r], gnome[r1]);
              break;
          }
      }
      return gnome;
  }

  // Function to return a valid GNOME string
  // required to create the population
  std::string create_gnome(int V){
      std::string gnome = "0";
      while (true) {
          if (gnome.size() == V) {
              gnome += gnome[0];
              break;
          }
          int temp = rand_num(1, V);
          if (!repeat(gnome, (char)(temp + 48)))
              gnome += (char)(temp + 48);
      }
      return gnome;
  }

  // Function to return the fitness value of a gnome.
  // The fitness value is the path length
  // of the path represented by the GNOME.
  double cal_fitness(std::string gnome, std::vector<std::vector<double>>& map){
      double f = 0;
      for (int i = 0; i < gnome.size() - 1; i++) {
          if (map[gnome[i] - 48][gnome[i + 1] - 48] == INT_MAX)
              return INT_MAX;
          f += map[gnome[i] - 48][gnome[i + 1] - 48];
      }
      return f;
  }

  // Function to return the updated value
  // of the cooling element.
  int cooldown(int temp){
      return (90 * temp) / 100;
  }

  // Comparator for GNOME struct.
  bool lessthan(struct individual t1, struct individual t2){
      return t1.fitness < t2.fitness;
  }
  //----------------------------------------------------- User-defined functions
};

#endif