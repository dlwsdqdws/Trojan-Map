#include "trojanmap.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
// #include <bits/stdc++.h>
#include <cmath>
#include <iostream>
#include <cctype>
#include <unordered_set>
#include <stack>
#include <chrono>
#include <cstring>
#include <ctime>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

using namespace std;

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu() {

  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Cycle Detection                                          \n"
      "* 6. Topological Sort                                         \n"
      "* 7. Find K Closest Points                                    \n"
      "* 8. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = Autocomplete(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = GetPosition(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1) {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                         \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto start = std::chrono::high_resolution_clock::now();
    //auto results = CalculateShortestPath_Dijkstra(input1, input2);
    auto results = CalculateShortestPath_Bellman_Ford(input1, input2);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
      std::cout << "The distance of the path is:" << CalculatePathLength(results) << " miles" << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Traveling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    // for (int i = 0; i < num; i++)
    //   locations.push_back(keys[rand() % keys.size()]);
    locations = {"123120189", "4011837229", "4011837224", "2514542032", "2514541020", "1931345270", "4015477529", "214470792", "63068532", "6807909279"};
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    //auto results = TravellingTrojan(locations);
    //auto results = TravellingTrojan_2opt(locations);
    //auto results = TravellingTrojan_2optSA(locations);
    auto results = TravellingTrojan_3opt(locations);
    //auto results = TravellingTrojan_GA(locations);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    CreateAnimation(results.second);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << " miles" << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '5':
  {
    menu =
        "**************************************************************\n"
        "* 5. Cycle Detection                                          \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the left bound longitude(between -118.299 and -118.264):";
    std::cout << menu;
    getline(std::cin, input);
    std::vector<double> square;
    square.push_back(atof(input.c_str()));

    menu = "Please input the right bound longitude(between -118.299 and -118.264):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    menu = "Please input the upper bound latitude(between 34.011 and 34.032):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    menu = "Please input the lower bound latitude(between 34.011 and 34.032):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    auto start = std::chrono::high_resolution_clock::now();
    auto results = CycleDetection(square);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results == true)
      std::cout << "there exists cycle in the subgraph " << std::endl;
    else
      std::cout << "there exist no cycle in the subgraph " << std::endl;
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '6':
  {
    menu =
        "**************************************************************\n"
        "* 6. Topological Sort                                         \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the locations filename:";
    std::cout << menu;
    std::string locations_filename;
    getline(std::cin, locations_filename);
    menu = "Please input the dependencies filename:";
    std::cout << menu;
    std::string dependencies_filename;
    getline(std::cin, dependencies_filename);
    
    // Read location names from CSV file
    std::vector<std::string> location_names;
    if (locations_filename == "") 
      location_names = {"Cardinal Gardens", "Coffee Bean1","CVS"};
    else
      location_names = ReadLocationsFromCSVFile(locations_filename);
    
    // Read dependencies from CSV file
    std::vector<std::vector<std::string>> dependencies;
    if (dependencies_filename == "")
      dependencies = {{"Coffee Bean1","Cardinal Gardens"}, {"CVS","Cardinal Gardens"}, {"CVS","Coffee Bean1"}};
    else
      dependencies = ReadDependenciesFromCSVFile(dependencies_filename);

    // std::vector<std::string> location_names = {"Cardinal Gardens", "Coffee Bean1","CVS"};
    // std::vector<std::vector<std::string>> dependencies = {{"Coffee Bean1","Cardinal Gardens"}, {"CVS","Cardinal Gardens"}, {"CVS","Coffee Bean1"}};
    auto start = std::chrono::high_resolution_clock::now();
    auto result = DeliveringTrojan(location_names, dependencies);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************";
    std::cout << menu << std::endl;
    std::cout << "Topological Sorting Results:" << std::endl;
    for (auto x : result) std::cout << x << std::endl;
    std::vector<std::string> node_ids;
    for (auto x: result) {
      std::string id = GetID(x);
      node_ids.push_back(id);
    }
    PlotPointsOrder(node_ids);
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
    case '7':
  {
    menu =
        "**************************************************************\n"
        "* 7. Find K Closest Points                                    \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    
    menu = "Please input the locations:";
    std::cout << menu;
    std::string origin;
    getline(std::cin, origin);
    menu = "Please input k:";
    std::cout << menu;
    getline(std::cin, input);
    int k = std::stoi(input);
    
    auto start = std::chrono::high_resolution_clock::now();
    auto result = FindKClosestPoints(origin, k);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************";
    std::cout << menu << std::endl;
    std::cout << "Find K Closest Points Results:" << std::endl;
    int cnt = 1;
    for (auto x : result) { 
      std::cout << cnt << " " << data[x].name << std::endl;
      cnt++;
    }
    PlotPointsLabel(result, GetID(origin));
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '8':
    break;
  default:
  {
    std::cout << "Please select 1 - 8." << std::endl;
    PrintMenu();
    break;
  }
  }
}


/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points inside square
 * @param  {std::vector<double>} square : boundary
 */
void TrojanMap::PlotPointsandEdges(std::vector<std::string> &location_ids, std::vector<double> &square) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto upperleft = GetPlotLocation(square[2], square[0]);
  auto lowerright = GetPlotLocation(square[3], square[1]);
  cv::Point pt1(int(upperleft.first), int(upperleft.second));
  cv::Point pt2(int(lowerright.first), int(lowerright.second));
  cv::rectangle(img, pt2, pt1, cv::Scalar(0, 0, 255));
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    for(auto y : data[x].neighbors) {
      auto start = GetPlotLocation(data[x].lat, data[x].lon);
      auto end = GetPlotLocation(data[y].lat, data[y].lon);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPointsOrder: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPointsOrder(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::putText(img, data[x].name, cv::Point(result.first, result.second), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
  }
  // Plot dots and lines
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::arrowedLine(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPointsLabel(std::vector<std::string> &location_ids, std::string origin) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  int cnt = 1;
  auto result = GetPlotLocation(data[origin].lat, data[origin].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 255, 0), cv::FILLED);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::putText(img, std::to_string(cnt), cv::Point(result.first, result.second), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
    cnt++;
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress){
  cv::VideoWriter video("src/lib/output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1248,992));
  for(auto location_ids: path_progress) {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
              cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < location_ids.size(); i++) {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
	video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id) {
    return data[id].lat;
}


/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) { 
    return data[id].lon;
}

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id) { 
    return data[id].name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id) {
    return data[id].neighbors;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  // TODO: Use Haversine Formula:
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  // a = (sin(dlat / 2)) ^ 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2;
  // c = 2 * arcsin(min(1, sqrt(a)));
  // distances = 3961 * c;

  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < path.size()-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);
  for(auto& [k,v] : data){
    std::string str = v.name;
    if(str.empty()) continue;
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    if(equal(name.begin(), name.end(), str.begin())) results.push_back(v.name);
  }
  return results;
}

/**
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  for(auto& [k,v] : data){
    if(v.name == name){
      results.first = v.lat;
      results.second = v.lon;
      // PlotPoint(v.id);
    }
  }
  return results;
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(std::string name) {
  std::string res = "";
  for(auto& [k,v] : data){
    if(v.name == name) res = k;
  }
  return res;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  // std::vector<std::string> path;
  std::unordered_map<std::string, double> dist;
  std::string st, ed;
  std::unordered_map<std::string, std::vector<std::string>> path;
  for(auto& [k,v] : data){
    dist[k] = 1e18;
    if(v.name == location1_name) st = k;
    if(v.name == location2_name) ed = k;
  }

  dist[st] = 0;
  path[st].push_back(st);
  std::priority_queue<std::pair<double, std::string>, vector<std::pair<double, std::string>>, greater<std::pair<double, std::string>>> pq;

  pq.push({0, st});
  while(pq.size()){
    auto t = pq.top();
    pq.pop();
    auto idx = t.second;
    if(idx == ed) break;
    for(auto& idy : data[idx].neighbors){
      double d = CalculateDistance(idx, idy);
      if(dist[idy] > dist[idx] + d){
        dist[idy] = dist[idx] + d;
        path[idy] = path[idx];
        path[idy].push_back(idy);
        pq.push({dist[idy], idy});
      }
    }
  }
  return path[ed];
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
  // std::vector<std::string> path;
  std::unordered_map<std::string, double> dist;
  std::string st, ed;
  std::unordered_map<std::string, std::vector<std::string>> path;
  std::unordered_map<std::string, bool> state;
  for(auto& [k,v] : data){
    dist[k] = 1e18;
    if(v.name == location1_name) st = k;
    if(v.name == location2_name) ed = k;
  }

  dist[st] = 0;
  path[st].push_back(st);
  std::queue<std::string> q;
  q.push(st);
  state[st] = true;

  while(q.size()){
    auto t = q.front();
    q.pop();
    state[t] = false;

    for(auto& nbor : data[t].neighbors){
      double d = CalculateDistance(nbor, t);
      if(dist[nbor] > dist[t] + d){
        dist[nbor] = dist[t] + d;
        path[nbor] = path[t];
        path[nbor].push_back(nbor);
        if(!state[nbor]){
          state[nbor] = true;
          q.push(nbor);
        }
      }
    }
  }
  return path[ed];
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
void TrojanMap::backtrack(int u, vector<std::string>& ids, double& curp, double& maxp, std::vector<std::string>& path, std::vector<std::vector<std::string>>& res){
  if(u >= ids.size()){
    double d = CalculateDistance(ids[ids.size()-1], ids[0]);
    if(d + curp < maxp){
      maxp = curp + d;
      for(int i = 0; i < ids.size(); i++){
        path[i] = ids[i];
      }
      path.push_back(ids[0]);
      res.push_back(path);
      path.pop_back();
    }
  }
  else{
    for(int i = u; i < ids.size(); i++){
      double d = CalculateDistance(ids[u-1], ids[i]);
      if(d + curp < maxp){
        swap(ids[u], ids[i]);
        curp += d;
        backtrack(u+1, ids, curp, maxp, path, res);
        swap(ids[u], ids[i]);
        curp -= d;
      }
    }
  }
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(
                                    std::vector<std::string> &location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> results;
  int n = location_ids.size();
  std::vector<std::string> ids = location_ids;
  double bestp = 1e18;
  double curp = 0;
  std::vector<std::string> path(n);
  results.second.push_back(ids);
  results.second[0].push_back(ids[0]);
  backtrack(1, ids, curp, bestp, path, results.second);
  results.first = bestp;
    for(int i = 0; i < results.second.size(); i++){
      double dist = CalculatePathLength(results.second[i]);
      if((dist - bestp) < 1e-8){
        swap(results.second[results.second.size()-1], results.second[i]);
        break;
      }
  }

  return results;
}

void TrojanMap::opt2_solver(std::vector<std::string>& ids, std::map<std::vector<std::string>, double>& mp, double& bestp, std::vector<std::vector<std::string>>& res){
  double d1 = CalculatePathLength(ids);
  if(d1 >= bestp) return ;

  std::vector<std::string> tmp;
  int n = ids.size();
  for(int i = 1; i < n-1; i++){
    for(int j = i+1; j < n-1; j++){
      tmp = ids;
      std::reverse(tmp.begin()+i, tmp.begin()+j+1);
      if(mp.count(tmp)) continue;
      double d2 = CalculatePathLength(tmp);
      if(d2 < d1){
        mp[tmp] = d2;
        bestp = d2;
        res.push_back(tmp);
        opt2_solver(tmp, mp, bestp, res);
      }
    }
  }
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> &location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::string> ids = location_ids;
  
  ids.push_back(ids[0]);
  results.second.push_back(ids);
  double bestp = 1e18;
  
  std::map<std::vector<std::string>, double> st;
  opt2_solver(ids, st, bestp, results.second);

  results.first = bestp;
  for(int i = 0; i < results.second.size(); i++){
    if(fabs(st[results.second[i]] - bestp) < 1e-8){
      swap(results.second[results.second.size()-1], results.second[i]);
      break;
    }
  }

  return results;
}

void TrojanMap::opt2SA_solver(double& t, std::vector<std::string>& ids, std::map<std::vector<std::string>, double>& mp, double& bestp, std::vector<std::vector<std::string>>& res){
  double d1 = CalculatePathLength(ids);
  if(d1 >= bestp) return ;

  std::vector<std::string> tmp;
  int n = ids.size();
  for(int i = 1; i < n-1; i++){
    for(int j = i+1; j < n-1; j++){
      tmp = ids;
      std::reverse(tmp.begin()+i, tmp.begin()+j+1);
      if(mp.count(tmp)) continue;
      double d2 = CalculatePathLength(tmp);
      mp[tmp] = d2;
      if(d2 < d1){
        bestp = d2;
        res.push_back(tmp);
        t *= 0.9;
        opt2SA_solver(t, tmp, mp, bestp, res);
      }
      else if(t > 1e-4 && exp(-(d2 - bestp)/t) > rand() / RAND_MAX){
        res.push_back(tmp);
        t *= 0.9;
        opt2SA_solver(t, tmp, mp, bestp, res);
      }
    }
  }
}

 std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2optSA(
      std::vector<std::string> &location_ids){
      std::pair<double, std::vector<std::vector<std::string>>> results;
      std::vector<std::string> ids = location_ids;
      
      ids.push_back(ids[0]);
      results.second.push_back(ids);
      double bestp = 1e18;
      
      std::map<std::vector<std::string>, double> st;
      double temper = 10000;
      opt2SA_solver(temper, ids, st, bestp, results.second);

      results.first = bestp;
      for(int i = 0; i < results.second.size(); i++){
        if(fabs(st[results.second[i]] - bestp) < 1e-8){
          swap(results.second[results.second.size()-1], results.second[i]);
          break;
        }
      }

      return results;
      }

void TrojanMap::opt3_solver(std::vector<std::string>& ids, std::map<std::vector<std::string>, double>& mp, double& bestp, std::vector<std::vector<std::string>>& res){
  double d1 = CalculatePathLength(ids);  //a b c
  if(d1 >= bestp) return ;
  bestp = d1;

  std::vector<std::string> tmp;
  int n = ids.size();

  for(int i = 1; i < n-2; i++){
    for(int j = i+1; j < n-1; j++){
        std::vector<std::string> a,b,c,rb,rc;
        for(int k = 0; k < i; k++) a.push_back(ids[k]);
        for(int k = i; k < j; k++) b.push_back(ids[k]);
        for(int k = j; k < n-1; k++) c.push_back(ids[k]);
        rb = b, rc = c;
        std::reverse(rb.begin(), rb.end());
        std::reverse(rc.begin(), rc.end());
        //a rb c
        std::vector<std::string> tmp;
        for(int k = 0; k < a.size(); k++) tmp.push_back(a[k]);
        for(int k = 0; k < rb.size(); k++) tmp.push_back(rb[k]);
        for(int k = 0; k < c.size(); k++) tmp.push_back(c[k]);
        tmp.push_back(ids[n-1]);
        if(mp.count(tmp)) continue;
        double d2 = CalculatePathLength(tmp);
        if(d2 < bestp){
          mp[tmp] = d2;
          bestp = d2;
          res.push_back(tmp);
          opt3_solver(tmp, mp, bestp, res);
        }

        //a b rc
        tmp = std::vector<std::string>();
        for(int k = 0; k < a.size(); k++) tmp.push_back(a[k]);
        for(int k = 0; k < b.size(); k++) tmp.push_back(b[k]);
        for(int k = 0; k < rc.size(); k++) tmp.push_back(rc[k]);
        tmp.push_back(ids[n-1]);
        if(mp.count(tmp)) continue;
        d2 = CalculatePathLength(tmp);
        if(d2 < bestp){
          mp[tmp] = d2;
          bestp = d2;
          res.push_back(tmp);
          opt3_solver(tmp, mp, bestp, res);
        }

        //a rb rc
        tmp = std::vector<std::string>();
        for(int k = 0; k < a.size(); k++) tmp.push_back(a[k]);
        for(int k = 0; k < rb.size(); k++) tmp.push_back(rb[k]);
        for(int k = 0; k < rc.size(); k++) tmp.push_back(rc[k]);
        tmp.push_back(ids[n-1]);
        if(mp.count(tmp)) continue;
        d2 = CalculatePathLength(tmp);
        if(d2 < bestp){
          mp[tmp] = d2;
          bestp = d2;
          res.push_back(tmp);
          opt3_solver(tmp, mp, bestp, res);
        }

        //a c b
        tmp = std::vector<std::string>();
        for(int k = 0; k < a.size(); k++) tmp.push_back(a[k]);
        for(int k = 0; k < c.size(); k++) tmp.push_back(c[k]);
        for(int k = 0; k < b.size(); k++) tmp.push_back(b[k]);
        tmp.push_back(ids[n-1]);
        if(mp.count(tmp)) continue;
        d2 = CalculatePathLength(tmp);
        if(d2 < bestp){
          mp[tmp] = d2;
          bestp = d2;
          res.push_back(tmp);
          opt3_solver(tmp, mp, bestp, res);
        }

        //a rc b
        tmp = std::vector<std::string>();
        for(int k = 0; k < a.size(); k++) tmp.push_back(a[k]);
        for(int k = 0; k < rc.size(); k++) tmp.push_back(rc[k]);
        for(int k = 0; k < b.size(); k++) tmp.push_back(b[k]);
        tmp.push_back(ids[n-1]);
        if(mp.count(tmp)) continue;
        d2 = CalculatePathLength(tmp);
        if(d2 < bestp){
          mp[tmp] = d2;
          bestp = d2;
          res.push_back(tmp);
          opt3_solver(tmp, mp, bestp, res);
        }

        //a rc rb
        tmp = std::vector<std::string>();
        for(int k = 0; k < a.size(); k++) tmp.push_back(a[k]);
        for(int k = 0; k < rc.size(); k++) tmp.push_back(rc[k]);
        for(int k = 0; k < rb.size(); k++) tmp.push_back(rb[k]);
        tmp.push_back(ids[n-1]);
        if(mp.count(tmp)) continue;
        d2 = CalculatePathLength(tmp);
        if(d2 < bestp){
          mp[tmp] = d2;
          bestp = d2;
          res.push_back(tmp);
          opt3_solver(tmp, mp, bestp, res);
        }

        //a c rb
        tmp = std::vector<std::string>();
        for(int k = 0; k < a.size(); k++) tmp.push_back(a[k]);
        for(int k = 0; k < c.size(); k++) tmp.push_back(c[k]);
        for(int k = 0; k < rb.size(); k++) tmp.push_back(rb[k]);
        tmp.push_back(ids[n-1]);
        if(mp.count(tmp)) continue;
        d2 = CalculatePathLength(tmp);
        if(d2 < bestp){
          mp[tmp] = d2;
          bestp = d2;
          res.push_back(tmp);
          opt3_solver(tmp, mp, bestp, res);
        }
    }
  }
}


std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_3opt(
      std::vector<std::string> &location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::string> ids = location_ids;
  ids.push_back(ids[0]);
  results.second.push_back(ids);
  double bestp = 1e18;
  
  std::map<std::vector<std::string>, double> st;
  opt3_solver(ids, st, bestp, results.second);

  results.first = bestp;
  for(int i = 0; i < results.second.size(); i++){
    if(fabs(st[results.second[i]] - bestp) < 1e-8){
      swap(results.second[results.second.size()-1], results.second[i]);
      break;
    }
  }

  return results;
}

void TrojanMap::simulate_anneal(std::vector<std::string>& ids, double& bestp){
  int n = ids.size();
  srand((unsigned int)time(0));
  for(double t = 1e6; t > 1e-8; t *= 0.9997){
        int a = 1 + rand() % (n-3);
        int b = a + 1 + rand() % (n-2-a);
        // double x = CalculatePathLength(ids);
        std::reverse(ids.begin()+a, ids.begin()+b+1);
        double y = CalculatePathLength(ids);
        double dt = y - bestp;
        if(exp(-dt/t) < rand() / RAND_MAX)
            std::reverse(ids.begin()+a, ids.begin()+b+1);
    }
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_SA(
      std::vector<std::string> &location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::string> ids = location_ids;
  ids.push_back(ids[0]);
  double bestp = CalculatePathLength(ids);
  results.first = bestp;
  results.second.push_back(ids);
  while((double)clock() / CLOCKS_PER_SEC < 5){
    srand(time(NULL));
    simulate_anneal(ids, bestp);
    double dist = CalculatePathLength(ids);
    if(dist < results.first){
      results.first = bestp;
      results.second.push_back(ids);
    }
  }
  return results;
}

 void TrojanMap::TSPUtil(std::vector<std::vector<double>>& map, std::pair<double, std::vector<std::vector<std::string>>>& prog, double& rs){
    // Generation Number
    int gen = 1;
    // Number of Gene Iterations
    int gen_thres = 5;
    // Initial population size for the algorithm
    int POP_SIZE = 10;

    vector<struct individual> population;
    struct individual temp;

    // Populating the GNOME pool.
    int V = map.size();
    for (int i = 0; i < POP_SIZE; i++) {
        temp.gnome = create_gnome(V);
        temp.fitness = cal_fitness(temp.gnome, map);
        population.push_back(temp);
    }

    for (int i = 0; i < POP_SIZE; i++){ 
        rs += population[i].fitness; 
    } 
    rs = rs / population.size();

    bool found = false;
    int temperature = 10000;

    // Iteration to perform
    // population crossing and gene mutation.
    while (temperature > 1000 && gen <= gen_thres) {
        // std::sort(population.begin(), population.end(), lessthan);
        std::sort(population.begin(), population.end(), [](individual& a, individual& b){
          return a < b;
        });

        std::vector<struct individual> new_population;
 
        for (int i = 0; i < POP_SIZE; i++) {
            struct individual p1 = population[i];
 
            while (true) {
                std::string new_g = mutatedGene(p1.gnome, V);
                struct individual new_gnome;
                new_gnome.gnome = new_g;
                new_gnome.fitness = cal_fitness(new_gnome.gnome, map);
 
                if (new_gnome.fitness <= population[i].fitness) {
                    new_population.push_back(new_gnome);
                    break;
                }
                else {
 
                    // Accepting the rejected children at
                    // a possible probability above threshold.
                    float prob = pow(2.7,
                                     -1 * ((float)(new_gnome.fitness
                                                   - population[i].fitness)
                                           / temperature));
                    if (prob > 0.5) {
                        new_population.push_back(new_gnome);
                        break;
                    }
                }
            }
        }

      temperature = cooldown(temperature);
      population = new_population;

      //get the final result
      double minv = 1e18;
      int minidx = 0;
      for (int i = 0; i < POP_SIZE; i++){
        if(population[i].fitness < minv){
            minv = population[i].fitness;
            minidx = i;
          }
      }
      std::vector<std::string> s;
      s.push_back(population[minidx].gnome);
      prog.second.push_back(s);
      if(minv < prog.first) prog.first = minv;

      gen++;
    }
 }

 std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_GA(
      std::vector<std::string> &location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> prog;
  prog.first = 1e18;
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::string> ids;
  ids = location_ids;
  int n = ids.size();

  //build mapping between string and index
  std::unordered_map<std::string, int> mp;
  std::unordered_map<int, std::string> rmp;
  for(int i = 0; i < n; i++){
    mp[ids[i]] = i;
    rmp[i] = ids[i];
  }

  //build weight matrix
  std::vector<std::vector<double>> w(n, std::vector<double>(n, 0));
  for(int i = 0; i < n; i++){
    for(int j = i+1; j < n; j++){
      w[i][j] = w[j][i] = CalculateDistance(ids[i], ids[j]);
    }
  }


  double rs = 0;
  TSPUtil(w, prog, rs);

  results.first = prog.first;
  for(int i = 0; i < prog.second.size(); i++){
    std::vector<std::string> gs;
    for(int j = 0; j < prog.second[i].size(); j++){
      for(int k = 0; k < prog.second[i][j].size(); k++){
        int z = prog.second[i][j].at(k) - 48;
        gs.push_back(rmp[z]);
      }
    }
    results.second.push_back(gs);
  }

  return results;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  fstream fin;
  fin.open(locations_filename, ios::in);
  std::string line, word;
  std::vector<std::string> location_names_from_csv;

  getline(fin, line);
  while (getline(fin, line)) {
    stringstream s(line);
    std::string location;
    getline(s, location, ',');
    location_names_from_csv.push_back(location);
  }
  fin.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  fstream fin;
  fin.open(dependencies_filename, ios::in);
  std::string line, word;
  std::vector<std::vector<std::string>> dependencies_from_csv;

  getline(fin, line);
  while (getline(fin, line)) {
    stringstream s(line);
    std::vector<string> locations;
    std::string location;
    while (getline(s, location, ',')) {
      locations.push_back(location);
    }
    dependencies_from_csv.push_back(locations);
  }
  fin.close();

  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  int n = locations.size();
  if(n == 0) return result;
  if(dependencies.size() == 0) return locations;
  std::unordered_map<std::string, int> d;
  for(auto& l : locations) d[l] = 0;
  unordered_map<std::string, std::vector<std::string>> adj;
  for(auto& dp : dependencies){
    if(dp.size() != 2) continue;
    auto a = dp[0], b = dp[1];
    d[b]++;
    adj[a].push_back(b);
  }
  queue<std::string> q;
  for(auto& [k,v] : d){
    if(v == 0){
      q.push(k);
    }
  }

  while(q.size()){
    auto t = q.front();
    q.pop();
    result.push_back(t);
    for(auto& v : adj[t]){
      if(--d[v] == 0){
        q.push(v);
      }
    }
  }
  return result;                                                     
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */

bool TrojanMap::dfs_cycle(Node& v, std::unordered_set<Node, hashfun, equalfun> & st, std::vector<double> &square, std::string fa){
  double lonleft = square[0], lonright = square[1], latup = square[2], latdown = square[3];
  if(v.lon < lonleft || v.lon > lonright || v.lat < latdown || v.lat > latup) return false;
  if(st.count(v)) return true;
  st.insert(v);
  for(auto& p: v.neighbors){
    if(p == fa) continue;
    if(dfs_cycle(data[p], st, square, v.id)) return true;
  }
  return false;
}


bool TrojanMap::CycleDetection(std::vector<double> &square) {
  std::unordered_set<Node, hashfun, equalfun> st;
  double lonleft = square[0], lonright = square[1], latup = square[2], latdown = square[3];
  if(lonright <= lonleft || latdown >= latup) return false;
  for(auto& [k,v] : data){
    if(st.count(v)) continue;
    if(v.lon < lonleft || v.lon > lonright || v.lat < latdown || v.lat > latup) continue;
    if(dfs_cycle(v, st, square, "")) return true;
  }
  //PlotPointsandEdges();
  return false;
}

/**
 * FindKClosetPoints: Given a location id and k, find the k closest points on the map
 * 
 * @param {std::string} name: the name of the location
 * @param {int} k: number of closest points
 * @return {std::vector<std::string>}: k closest points
 */
  void TrojanMap::kclose_solver(std::string name, std::string origin, std::set<std::pair<double, std::string>>& distance, int k, std::unordered_map<std::string, double>& hash, double dist){
    for(auto& v : data[name].neighbors){
      if(v == origin) continue;
      double d = CalculateDistance(name, v) + dist;
      if(data[v].name.empty()){
        // kclose_solver(v, name, distance, k, hash, d);
        continue;
      }
      else if(hash.count(v) && hash[v] != -1){
        double od = hash[v];
        if(od > d) hash[v] = d;
        distance.erase(distance.find({od, v}));
        distance.insert({d, v});
      }
      else if(distance.size() < k){
        distance.insert({d, v});
        hash[v] = d;
        kclose_solver(v, name, distance, k, hash, d);
      }
      else if(distance.size() >= k && d < distance.rbegin()->first){
        hash[distance.rbegin()->second] = -1;
        hash[v] = d;
        distance.erase(--distance.end());
        distance.insert({d, v});
        kclose_solver(v, name, distance, k, hash, d);
      }
    }
  }


void TrojanMap::kclose_solver(std::string& name, std::string& origin, std::unordered_set<std::string>& st, std::priority_queue<std::pair<double, std::string>>& pq, int k){
  for(auto& v : data[name].neighbors){
    // auto rname = data[name].name;
    if(st.count(name)) continue;
    if(v.empty()) kclose_solver(v, origin, st, pq, k);
    else{
      double d = CalculateDistance(origin, v);
      if(pq.size() < k){
        if(v.size()){
          pq.push({d, v});
        }
        st.insert(v);
        kclose_solver(v, origin, st, pq, k);
      }
      else if(d < pq.top().first){
        if(v.size()){
          pq.pop();
          pq.push({d, v});
        }
        st.insert(v);
        kclose_solver(v, origin, st, pq, k);
      }
    }
  }
}

void TrojanMap::kclose_solver(std::string& origin, std::priority_queue<std::pair<double, std::string>>& pq, std::unordered_set<std::string>& st, int m){
  for(auto& [k,v] : data){
      if(st.count(k)) continue;
      if(v.name.empty()) continue;
      double d = CalculateDistance(origin, k);
      if(pq.size() < m){
        pq.push({d, k});
        st.insert(k);
      }
      else if(d < pq.top().first){
        pq.push({d, k});
        st.insert(k);
      }
  }
}

std::vector<std::string> TrojanMap::FindKClosestPoints(std::string name, int k) {
  std::vector<std::string> res;
  name = GetID(name);
  if(!data.count(name)) return res;
  // std::set<std::pair<double, std::string>> distance;
  // std::map<double, std::string> mp;
  // std::unordered_map<std::string, double> hash;
  // kclose_solver(name, mp, k, hash, 0);
  // kclose_solver(name, "", distance, k, hash, 0);
  std::unordered_set<std::string> st;
  st.insert(name);
  // st.insert("");
  std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, greater<std::pair<double, std::string>>> pq;
  // kclose_solver(name, name, st, pq, k);
  for(auto& [m,v] : data){
    // if(st.count(m)) continue;
    if(m == name) continue;
    if(v.name.empty()) continue;
    double d = CalculateDistance(m, name);
    pq.push({d, m});
  }

  while(k--){
    res.push_back(pq.top().second);
    pq.pop();
  }
  return res;
}