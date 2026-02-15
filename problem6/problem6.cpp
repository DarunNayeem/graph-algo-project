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

void printTimedRoute(ofstream& outFile, TimeResult& res, double srcLat, double srcLon, double dstLat, double dstLon, 
                     string startTime, string deadline, double costPerKm[4]) {
    if (res.cost < 0) {
        outFile << "No route found within deadline!\n\n";
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
    outFile << "Arrival Time: " << minsToTime(res.arrivalTime) << "\n";
    outFile << "Deadline Met: " << (res.arrivalTime <= timeToMins(deadline) ? "YES" : "NO") << "\n\n";
}

int main() {
    string basePath = "/media/nym/Nym_s Files/grph-project/";
    
    cout << "Loading data for Problem 6...\n";
    loadRoads(graph, basePath + "Roadmap-Dhaka.csv");
    loadTransport(graph, basePath + "Routemap-DhakaMetroRail.csv", 1);
    loadTransport(graph, basePath + "Routemap-BikolpoBus.csv", 2);
    loadTransport(graph, basePath + "Routemap-UttaraBus.csv", 3);
    cout << "Loaded " << graph.nodeCount << " nodes\n\n";
    
    // Open output file
    ofstream outFile(basePath + "problem6/output.txt");
    
    outFile << "============================================================\n";
    outFile << "PROBLEM 6: Cheapest Route with Deadline Constraint\n";
    outFile << "Cost per km: Car = Tk 20, Metro = Tk 5, Bikalpa = Tk 7, Uttara = Tk 10\n";
    outFile << "Speed: Car = 20, Metro = 15, Bikalpa = 10, Uttara = 12 km/h\n";
    outFile << "Schedule: Metro 1 AM-11 PM (5 min), Bikalpa 7 AM-10 PM (20 min), Uttara 6 AM-11 PM (10 min)\n";
    outFile << "============================================================\n\n";
    
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
    
    outFile << "------------------------------------------------------------\n";
    outFile << "Test Case:\n";
    outFile << "------------------------------------------------------------\n";
    outFile << fixed << setprecision(6);
    outFile << "Source: (" << srcLon << ", " << srcLat << ") [Uttara]\n";
    outFile << "Destination: (" << dstLon << ", " << dstLat << ") [Secretariat]\n";
    outFile << "Starting Time: " << startTime << "\n";
    outFile << "Deadline: " << deadline << "\n\n";
    
    printTimedRoute(outFile, res, srcLat, srcLon, dstLat, dstLon, startTime, deadline, costPerKm);
    
    if (res.cost >= 0) {
        vector<int> nodes;
        for (auto& p : res.pathWithTime) nodes.push_back(p.first);
        saveKML(graph, nodes, basePath + "problem6/output.kml");
        cout << "Cost = Tk " << fixed << setprecision(2) << res.cost << ", Arrival: " << minsToTime(res.arrivalTime) << "\n";
    }
    
    outFile << "============================================================\n";
    outFile.close();
    
    cout << "Output saved to problem6/output.txt\n";
    
    return 0;
}
