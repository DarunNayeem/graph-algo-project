#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <cmath>
#include <string>
#include <algorithm>
#include <climits>
#include <iomanip>

using namespace std;

const double INF = 1e18;
const double PI = 3.14159265358979323846;
const double EARTH_RADIUS = 6371.0; // km

// haversine distance between two lat-long points
inline double haversine(double lat1, double lon1, double lat2, double lon2) {
    double lat1r = lat1 * PI / 180.0;
    double lat2r = lat2 * PI / 180.0;
    double dlat = (lat2 - lat1) * PI / 180.0;
    double dlon = (lon2 - lon1) * PI / 180.0;
    
    double a = sin(dlat/2) * sin(dlat/2) + cos(lat1r) * cos(lat2r) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * asin(sqrt(a));
    return EARTH_RADIUS * c;
}

// round to 6 decimal places for node key
inline long long makeKey(double lat, double lon) {
    long long latKey = (long long)(lat * 1000000);
    long long lonKey = (long long)(lon * 1000000);
    return latKey * 10000000LL + lonKey;
}

struct Edge {
    int to;
    double dist;
    int mode; // 0=car, 1=metro, 2=bikalpa, 3=uttara
};

struct Node {
    double lat, lon;
    string name;
    bool isStop;
};

class Graph {
public:
    map<long long, int> nodeMap;
    vector<Node> nodes;
    vector<vector<Edge>> adj;
    int nodeCount = 0;
    
    int addNode(double lat, double lon, string name = "", bool isStop = false) {
        long long key = makeKey(lat, lon);
        if (nodeMap.find(key) == nodeMap.end()) {
            nodeMap[key] = nodeCount++;
            nodes.push_back({lat, lon, "", isStop});
            adj.push_back(vector<Edge>());
        }
        int id = nodeMap[key];
        if (!name.empty()) nodes[id].name = name;
        if (isStop) nodes[id].isStop = true;
        return id;
    }
    
    void addEdge(int u, int v, double dist, int mode) {
        adj[u].push_back({v, dist, mode});
        adj[v].push_back({u, dist, mode});
    }
    
    int getNearestNode(double lat, double lon, double &minDist) {
        minDist = INF;
        int nearest = -1;
        for (int i = 0; i < nodeCount; i++) {
            double d = haversine(lat, lon, nodes[i].lat, nodes[i].lon);
            if (d < minDist) {
                minDist = d;
                nearest = i;
            }
        }
        return nearest;
    }
};

// parse CSV line
inline vector<string> parseCSV(string line) {
    vector<string> result;
    stringstream ss(line);
    string item;
    while (getline(ss, item, ',')) {
        result.push_back(item);
    }
    return result;
}

// trim whitespace
inline string trim(string s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    if (start == string::npos) return "";
    return s.substr(start, end - start + 1);
}

// load road map
inline void loadRoads(Graph& graph, string filename) {
    ifstream file(filename);
    string line;
    while (getline(file, line)) {
        vector<string> parts = parseCSV(line);
        if (parts.size() < 6) continue;
        
        vector<pair<double,double>> coords;
        for (int i = 1; i < (int)parts.size() - 2; i += 2) {
            try {
                double lon = stod(parts[i]);
                double lat = stod(parts[i+1]);
                coords.push_back({lat, lon});
            } catch (...) {
                break;
            }
        }
        
        for (int i = 0; i < (int)coords.size() - 1; i++) {
            int n1 = graph.addNode(coords[i].first, coords[i].second);
            int n2 = graph.addNode(coords[i+1].first, coords[i+1].second);
            double dist = haversine(coords[i].first, coords[i].second, coords[i+1].first, coords[i+1].second);
            graph.addEdge(n1, n2, dist, 0); // mode 0 = car
        }
    }
    file.close();
}

// load transport routes
inline void loadTransport(Graph& graph, string filename, int mode) {
    ifstream file(filename);
    string line;
    while (getline(file, line)) {
        vector<string> parts = parseCSV(line);
        if (parts.size() < 4) continue;
        
        string startName = trim(parts[parts.size()-2]);
        string endName = trim(parts[parts.size()-1]);
        
        vector<pair<double,double>> coords;
        for (int i = 1; i < (int)parts.size() - 2; i += 2) {
            try {
                double lon = stod(parts[i]);
                double lat = stod(parts[i+1]);
                coords.push_back({lat, lon});
            } catch (...) {
                break;
            }
        }
        
        if (coords.size() < 2) continue;
        
        int startNode = graph.addNode(coords[0].first, coords[0].second, startName, true);
        int endNode = graph.addNode(coords.back().first, coords.back().second, endName, true);
        
        int prevNode = startNode;
        for (int i = 1; i < (int)coords.size() - 1; i++) {
            int currNode = graph.addNode(coords[i].first, coords[i].second);
            double dist = haversine(graph.nodes[prevNode].lat, graph.nodes[prevNode].lon, coords[i].first, coords[i].second);
            graph.addEdge(prevNode, currNode, dist, mode);
            prevNode = currNode;
        }
        
        double dist = haversine(graph.nodes[prevNode].lat, graph.nodes[prevNode].lon, coords.back().first, coords.back().second);
        graph.addEdge(prevNode, endNode, dist, mode);
    }
    file.close();
}

// save KML file
inline void saveKML(Graph& graph, vector<int>& path, string filename) {
    ofstream file(filename);
    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<kml xmlns=\"http://earth.google.com/kml/2.1\">\n";
    file << "<Document>\n";
    file << "<Placemark>\n";
    file << "<name>" << filename << "</name>\n";
    file << "<LineString>\n";
    file << "<tessellate>1</tessellate>\n";
    file << "<coordinates>\n";
    
    for (int id : path) {
        file << fixed << setprecision(6) << graph.nodes[id].lon << "," << graph.nodes[id].lat << ",0\n";
    }
    
    file << "</coordinates>\n";
    file << "</LineString>\n";
    file << "</Placemark>\n";
    file << "</Document>\n";
    file << "</kml>\n";
    file.close();
}

// get mode name
inline string getModeName(int mode) {
    if (mode == 0) return "Car";
    if (mode == 1) return "Metro";
    if (mode == 2) return "Bikalpa Bus";
    if (mode == 3) return "Uttara Bus";
    return "Unknown";
}

// get node name or coords
inline string getNodeName(Graph& graph, int id) {
    if (!graph.nodes[id].name.empty()) return graph.nodes[id].name;
    stringstream ss;
    ss << fixed << setprecision(6) << "(" << graph.nodes[id].lat << ", " << graph.nodes[id].lon << ")";
    return ss.str();
}

// time utilities
inline int timeToMins(string timeStr) {
    int h, m;
    timeStr = trim(timeStr);
    sscanf(timeStr.c_str(), "%d:%d", &h, &m);
    
    if (timeStr.find("PM") != string::npos || timeStr.find("pm") != string::npos) {
        if (h != 12) h += 12;
    } else if (timeStr.find("AM") != string::npos || timeStr.find("am") != string::npos) {
        if (h == 12) h = 0;
    }
    
    return h * 60 + m;
}

inline string minsToTime(int mins) {
    int h = (mins / 60) % 24;
    int m = mins % 60;
    string period = (h < 12) ? "AM" : "PM";
    h = h % 12;
    if (h == 0) h = 12;
    
    stringstream ss;
    ss << h << ":" << setfill('0') << setw(2) << m << " " << period;
    return ss.str();
}

inline int nextDeparture(int currMins, int interval, int startMins, int endMins) {
    if (currMins < startMins) return startMins;
    if (currMins > endMins) return -1;
    
    int passed = (currMins - startMins) / interval;
    int currDep = startMins + passed * interval;
    
    if (currDep == currMins) return currMins;
    
    int nextDep = startMins + (passed + 1) * interval;
    if (nextDep <= endMins) return nextDep;
    
    return currDep <= currMins && currMins <= endMins ? currMins : -1;
}

#endif // GRAPH_H
