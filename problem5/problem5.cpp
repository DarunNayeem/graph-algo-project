// Problem 5: Fastest Route
#include "../common/graph.h"

Graph graph;

struct TimeResult {
    vector<pair<int,int>> pathWithTime; // {node, arrival time}
    vector<int> modes;
    double cost;
    int arrivalTime;
};

// Dijkstra's algorithm for fastest route
TimeResult fastestRoute(int start, int end, int startMins, double costPerKm[4], double speeds[4],
                        int intervals[4], int schedStart[4], int schedEnd[4], bool allowed[4]) {
    map<int, int> bestTime;
    
    // state: {time, cost, node}
    priority_queue<tuple<int,double,int,vector<pair<int,int>>,vector<int>>,
                   vector<tuple<int,double,int,vector<pair<int,int>>,vector<int>>>,
                   greater<tuple<int,double,int,vector<pair<int,int>>,vector<int>>>> pq;
    
    pq.push({startMins, 0, start, {{start, startMins}}, {}});
    
    while (!pq.empty()) {
        auto [currTime, currCost, u, path, modes] = pq.top();
        pq.pop();
        
        if (bestTime.count(u) && bestTime[u] <= currTime) continue;
        bestTime[u] = currTime;
        
        if (u == end) {
            return {path, modes, currCost, currTime};
        }
        
        for (auto& e : graph.adj[u]) {
            if (!allowed[e.mode]) continue;
            
            double speed = speeds[e.mode];
            int travelTime = (int)((e.dist / speed) * 60);
            
            int departTime = currTime;
            if (e.mode != 0 && intervals[e.mode] > 0) {
                int dep = nextDeparture(currTime, intervals[e.mode], schedStart[e.mode], schedEnd[e.mode]);
                if (dep == -1) continue;
                departTime = dep;
            }
            
            int arriveTime = departTime + travelTime;
            if (travelTime == 0) arriveTime = departTime + 1;
            double edgeCost = e.dist * costPerKm[e.mode];
            double newCost = currCost + edgeCost;
            
            if (!bestTime.count(e.to) || arriveTime < bestTime[e.to]) {
                auto newPath = path;
                newPath.push_back({e.to, arriveTime});
                auto newModes = modes;
                newModes.push_back(e.mode);
                pq.push({arriveTime, newCost, e.to, newPath, newModes});
            }
        }
    }
    
    return {{}, {}, -1, -1};
}

void printTimedRoute(ofstream& outFile, TimeResult& res, double costPerKm[4]) {
    if (res.cost < 0) {
        outFile << "No route found!\n";
        return;
    }
    
    double totalCost = 0;
    
    int i = 0;
    while (i < (int)res.pathWithTime.size() - 1) {
        int startNode = res.pathWithTime[i].first;
        int startT = res.pathWithTime[i].second;
        int mode = res.modes[i];
        
        int j = i;
        double segDist = 0;
        while (j < (int)res.pathWithTime.size() - 1 && j < (int)res.modes.size() && res.modes[j] == mode) {
            double d = haversine(graph.nodes[res.pathWithTime[j].first].lat, graph.nodes[res.pathWithTime[j].first].lon,
                                graph.nodes[res.pathWithTime[j+1].first].lat, graph.nodes[res.pathWithTime[j+1].first].lon);
            segDist += d;
            j++;
        }
        
        int endNode = res.pathWithTime[j].first;
        int endT = res.pathWithTime[j].second;
        double segCost = segDist * costPerKm[mode];
        totalCost += segCost;
        
        string action;
        if (mode == 0) action = "Ride Car";
        else if (mode == 1) action = "Ride Metro";
        else if (mode == 2) action = "Ride Bikolpo Bus";
        else action = "Ride Uttara Bus";
        
        outFile << minsToTime(startT) << " - " << minsToTime(endT);
        outFile << ", Cost: Tk " << fixed << setprecision(2) << segCost << ": " << action;
        outFile << " from (" << fixed << setprecision(6) << graph.nodes[startNode].lon << ", " << graph.nodes[startNode].lat << ")";
        outFile << " to (" << graph.nodes[endNode].lon << ", " << graph.nodes[endNode].lat << ").\n";
        
        i = j;
    }
    
    outFile << "\n";
    outFile << fixed << setprecision(2);
    outFile << "Total Cost: Tk " << totalCost << "\n";
}

int main() {
    string basePath = "/media/nym/Nym_s Files/grph-project/";
    
    cout << "Loading data for Problem 5...\n";
    loadRoads(graph, basePath + "Roadmap-Dhaka.csv");
    loadTransport(graph, basePath + "Routemap-DhakaMetroRail.csv", 1);
    loadTransport(graph, basePath + "Routemap-BikolpoBus.csv", 2);
    loadTransport(graph, basePath + "Routemap-UttaraBus.csv", 3);
    cout << "Loaded " << graph.nodeCount << " nodes\n\n";
    
    // Test input: Farmgate to Matijheel at 9:00 AM
    double srcLon = 90.387604, srcLat = 23.757573;
    double dstLon = 90.418119, dstLat = 23.727553;
    string startTime = "9:00 AM";
    
    double walkDist;
    int startId = graph.getNearestNode(srcLat, srcLon, walkDist);
    int endId = graph.getNearestNode(dstLat, dstLon, walkDist);
    int startMins = timeToMins(startTime);
    
    // Parameters (slower speeds for fastest route problem)
    double costPerKm[4] = {20, 5, 7, 7};
    double speeds[4] = {10, 10, 10, 10};
    int intervals[4] = {0, 15, 15, 15};
    int schedStart[4] = {0, timeToMins("6:00 AM"), timeToMins("6:00 AM"), timeToMins("6:00 AM")};
    int schedEnd[4] = {0, timeToMins("11:00 PM"), timeToMins("11:00 PM"), timeToMins("11:00 PM")};
    bool allowed[4] = {true, true, true, true};
    
    TimeResult res = fastestRoute(startId, endId, startMins, costPerKm, speeds, intervals, schedStart, schedEnd, allowed);
    
    // Create output file for test case
    ofstream outFile(basePath + "problem5/output_test1.txt");
    
    outFile << fixed << setprecision(6);
    outFile << "Problem No: 5\n";
    outFile << "Source: (" << srcLon << ", " << srcLat << ")\n";
    outFile << "Destination: (" << dstLon << ", " << dstLat << ")\n";
    outFile << "Starting time at source: " << startTime << "\n\n";
    
    printTimedRoute(outFile, res, costPerKm);
    
    if (res.cost >= 0) {
        vector<int> nodes;
        for (auto& p : res.pathWithTime) nodes.push_back(p.first);
        saveKML(graph, nodes, basePath + "problem5/output_test1.kml");
        cout << "Arrival Time: " << minsToTime(res.arrivalTime) << ", Cost = Tk " << fixed << setprecision(2) << res.cost << "\n";
    }
    
    outFile.close();
    
    cout << "Output saved to problem5/output_test1.txt\n";
    
    return 0;
}
