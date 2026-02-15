// Problem 4: Cheapest Route with Time Consideration
#include "../common/graph.h"

Graph graph;

struct TimeResult {
    vector<pair<int,int>> pathWithTime; // {node, arrival time}
    vector<int> modes;
    double cost;
    int arrivalTime;
};

// Dijkstra's algorithm for cheapest route with time
TimeResult cheapestWithTime(int start, int end, int startMins, double costPerKm[4], double speeds[4], 
                            int intervals[4], int schedStart[4], int schedEnd[4], bool allowed[4]) {
    map<int, double> bestCost;
    
    // state: {cost, time, node}
    priority_queue<tuple<double,int,int,vector<pair<int,int>>,vector<int>>, 
                   vector<tuple<double,int,int,vector<pair<int,int>>,vector<int>>>,
                   greater<tuple<double,int,int,vector<pair<int,int>>,vector<int>>>> pq;
    
    pq.push({0, startMins, start, {{start, startMins}}, {}});
    
    while (!pq.empty()) {
        auto [currCost, currTime, u, path, modes] = pq.top();
        pq.pop();
        
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
            if (e.mode != 0 && intervals[e.mode] > 0) { // public transport has schedule
                int dep = nextDeparture(currTime, intervals[e.mode], schedStart[e.mode], schedEnd[e.mode]);
                if (dep == -1) continue;
                departTime = dep;
            }
            
            int arriveTime = departTime + travelTime;
            if (travelTime == 0) arriveTime = departTime + 1; // at least 1 min
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

void printTimedRoute(ofstream& outFile, TimeResult& res, double srcLat, double srcLon, double dstLat, double dstLon, 
                     string startTime, double costPerKm[4]) {
    if (res.cost < 0) {
        outFile << "No route found!\n\n";
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
        if (mode == 0) action = "Drive Car";
        else if (mode == 1) action = "Ride Metro Rail";
        else if (mode == 2) action = "Ride Bikalpa Bus";
        else action = "Ride Uttara Bus";
        
        outFile << minsToTime(startT) << " - " << minsToTime(endT);
        outFile << fixed << setprecision(2);
        outFile << ", Cost: Tk " << segCost << ": " << action;
        outFile << " from (" << graph.nodes[startNode].lon << ", " << graph.nodes[startNode].lat << ")";
        outFile << " to (" << graph.nodes[endNode].lon << ", " << graph.nodes[endNode].lat << ").\n";
        
        i = j;
    }
    
    outFile << "\nTotal Cost: Tk " << totalCost << "\n";
    outFile << "Arrival Time: " << minsToTime(res.arrivalTime) << "\n\n";
}

int main() {
    string basePath = "/media/nym/Nym_s Files/grph-project/";
    
    cout << "Loading data for Problem 4...\n";
    loadRoads(graph, basePath + "Roadmap-Dhaka.csv");
    loadTransport(graph, basePath + "Routemap-DhakaMetroRail.csv", 1);
    loadTransport(graph, basePath + "Routemap-BikolpoBus.csv", 2);
    loadTransport(graph, basePath + "Routemap-UttaraBus.csv", 3);
    cout << "Loaded " << graph.nodeCount << " nodes\n\n";
    
    // Open output file
    ofstream outFile(basePath + "problem4/output.txt");
    
    outFile << "============================================================\n";
    outFile << "PROBLEM 4: Cheapest Route with Time Consideration\n";
    outFile << "Cost per km: Car = Tk 20, Metro = Tk 5, Bikalpa = Tk 7, Uttara = Tk 7\n";
    outFile << "Speed: All modes = 30 km/h\n";
    outFile << "Schedule: Metro/Bus every 15 mins, 6:00 AM - 11:00 PM\n";
    outFile << "============================================================\n\n";
    
    // Test input: Mirpur 10 to Shahbag at 5:30 PM
    double srcLon = 90.366249, srcLat = 23.815764;
    double dstLon = 90.396151, dstLat = 23.738265;
    string startTime = "5:30 PM";
    
    double walkDist;
    int startId = graph.getNearestNode(srcLat, srcLon, walkDist);
    int endId = graph.getNearestNode(dstLat, dstLon, walkDist);
    int startMins = timeToMins(startTime);
    
    // Parameters
    double costPerKm[4] = {20, 5, 7, 7};
    double speeds[4] = {30, 30, 30, 30};
    int intervals[4] = {0, 15, 15, 15};
    int schedStart[4] = {0, timeToMins("6:00 AM"), timeToMins("6:00 AM"), timeToMins("6:00 AM")};
    int schedEnd[4] = {0, timeToMins("11:00 PM"), timeToMins("11:00 PM"), timeToMins("11:00 PM")};
    bool allowed[4] = {true, true, true, true};
    
    TimeResult res = cheapestWithTime(startId, endId, startMins, costPerKm, speeds, intervals, schedStart, schedEnd, allowed);
    
    outFile << "------------------------------------------------------------\n";
    outFile << "Test Case:\n";
    outFile << "------------------------------------------------------------\n";
    outFile << fixed << setprecision(6);
    outFile << "Source: (" << srcLon << ", " << srcLat << ") [Mirpur 10]\n";
    outFile << "Destination: (" << dstLon << ", " << dstLat << ") [Shahbag]\n";
    outFile << "Starting Time: " << startTime << "\n\n";
    
    printTimedRoute(outFile, res, srcLat, srcLon, dstLat, dstLon, startTime, costPerKm);
    
    if (res.cost >= 0) {
        vector<int> nodes;
        for (auto& p : res.pathWithTime) nodes.push_back(p.first);
        saveKML(graph, nodes, basePath + "problem4/output.kml");
        cout << "Cost = Tk " << fixed << setprecision(2) << res.cost << ", Arrival: " << minsToTime(res.arrivalTime) << "\n";
    }
    
    outFile << "============================================================\n";
    outFile.close();
    
    cout << "Output saved to problem4/output.txt\n";
    
    return 0;
}
