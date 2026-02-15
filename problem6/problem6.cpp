// Problem 6: Cheapest Route with Deadline
#include "../common/graph.h"

Graph graph;

struct TimeResult {
    vector<pair<int,int>> pathWithTime; // {node, arrival time}
    vector<int> modes;
    double cost;
    int arrivalTime;
};

// Dijkstra's algorithm for cheapest route with deadline constraint
TimeResult cheapestWithDeadline(int start, int end, int startMins, int deadlineMins, 
                                double costPerKm[4], double speeds[4],
                                int intervals[4], int schedStart[4], int schedEnd[4], bool allowed[4]) {
    map<int, double> bestCost;
    
    priority_queue<tuple<double,int,int,vector<pair<int,int>>,vector<int>>,
                   vector<tuple<double,int,int,vector<pair<int,int>>,vector<int>>>,
                   greater<tuple<double,int,int,vector<pair<int,int>>,vector<int>>>> pq;
    
    pq.push({0, startMins, start, {{start, startMins}}, {}});
    
    while (!pq.empty()) {
        auto [currCost, currTime, u, path, modes] = pq.top();
        pq.pop();
        
        if (currTime > deadlineMins) continue;
        
        if (bestCost.count(u) && bestCost[u] <= currCost) continue;
        bestCost[u] = currCost;
        
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
            if (arriveTime > deadlineMins) continue;
            
            double edgeCost = e.dist * costPerKm[e.mode];
            double newCost = currCost + edgeCost;
            
            if (!bestCost.count(e.to) || newCost < bestCost[e.to]) {
                auto newPath = path;
                newPath.push_back({e.to, arriveTime});
                auto newModes = modes;
                newModes.push_back(e.mode);
                pq.push({newCost, arriveTime, e.to, newPath, newModes});
            }
        }
    }
    
    return {{}, {}, -1, -1};
}

void printTimedRoute(ofstream& outFile, TimeResult& res, string deadline, double costPerKm[4]) {
    if (res.cost < 0) {
        outFile << "No route found within deadline!\n";
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
    
    cout << "Loading data for Problem 6...\n";
    loadRoads(graph, basePath + "Roadmap-Dhaka.csv");
    loadTransport(graph, basePath + "Routemap-DhakaMetroRail.csv", 1);
    loadTransport(graph, basePath + "Routemap-BikolpoBus.csv", 2);
    loadTransport(graph, basePath + "Routemap-UttaraBus.csv", 3);
    cout << "Loaded " << graph.nodeCount << " nodes\n\n";
    
    // Test input: Uttara to Secretariat, 6:00 PM start, 8:30 PM deadline
    double srcLon = 90.400500, srcLat = 23.869560;
    double dstLon = 90.406845, dstLat = 23.729983;
    string startTime = "6:00 PM";
    string deadline = "8:30 PM";
    
    double walkDist;
    int startId = graph.getNearestNode(srcLat, srcLon, walkDist);
    int endId = graph.getNearestNode(dstLat, dstLon, walkDist);
    int startMins = timeToMins(startTime);
    int deadlineMins = timeToMins(deadline);
    
    // Parameters for Problem 6
    double costPerKm[4] = {20, 5, 7, 10};
    double speeds[4] = {20, 15, 10, 12};
    int intervals[4] = {0, 5, 20, 10};
    int schedStart[4] = {0, timeToMins("1:00 AM"), timeToMins("7:00 AM"), timeToMins("6:00 AM")};
    int schedEnd[4] = {0, timeToMins("11:00 PM"), timeToMins("10:00 PM"), timeToMins("11:00 PM")};
    bool allowed[4] = {true, true, true, true};
    
    TimeResult res = cheapestWithDeadline(startId, endId, startMins, deadlineMins, costPerKm, speeds, intervals, schedStart, schedEnd, allowed);
    
    // Create output file for test case
    ofstream outFile(basePath + "problem6/output_test1.txt");
    
    outFile << fixed << setprecision(6);
    outFile << "Problem No: 6\n";
    outFile << "Source: (" << srcLon << ", " << srcLat << ")\n";
    outFile << "Destination: (" << dstLon << ", " << dstLat << ")\n";
    outFile << "Starting time at source: " << startTime << "\n";
    outFile << "Destination reaching time: " << deadline << "\n\n";
    
    printTimedRoute(outFile, res, deadline, costPerKm);
    
    if (res.cost >= 0) {
        vector<int> nodes;
        for (auto& p : res.pathWithTime) nodes.push_back(p.first);
        saveKML(graph, nodes, basePath + "problem6/output_test1.kml");
        cout << "Cost = Tk " << fixed << setprecision(2) << res.cost << ", Arrival: " << minsToTime(res.arrivalTime) << "\n";
    } else {
        cout << "No route found within deadline\n";
    }
    
    outFile.close();
    
    cout << "Output saved to problem6/output_test1.txt\n";
    
    return 0;
}
