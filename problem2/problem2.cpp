// Problem 2: Cheapest Cost (Car + Metro Only)
#include "../common/graph.h"

Graph graph;

struct CostResult {
    vector<int> path;
    vector<int> modes;
    double cost;
};

// Dijkstra's algorithm for cheapest route
CostResult cheapestRoute(int start, int end, double costPerKm[4], bool allowed[4]) {
    int n = graph.nodeCount;
    vector<double> cost(n, INF);
    vector<pair<int,int>> parent(n, {-1, -1}); // {prev node, mode}
    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;
    
    cost[start] = 0;
    pq.push({0, start});
    
    while (!pq.empty()) {
        auto [c, u] = pq.top();
        pq.pop();
        
        if (c > cost[u]) continue;
        if (u == end) break;
        
        for (auto& e : graph.adj[u]) {
            if (!allowed[e.mode]) continue;
            double edgeCost = e.dist * costPerKm[e.mode];
            double newCost = cost[u] + edgeCost;
            if (newCost < cost[e.to]) {
                cost[e.to] = newCost;
                parent[e.to] = {u, e.mode};
                pq.push({newCost, e.to});
            }
        }
    }
    
    if (cost[end] >= INF) return {{}, {}, -1};
    
    vector<int> path, modes;
    for (int v = end; v != -1; v = parent[v].first) {
        path.push_back(v);
        if (parent[v].first != -1) modes.push_back(parent[v].second);
    }
    reverse(path.begin(), path.end());
    reverse(modes.begin(), modes.end());
    return {path, modes, cost[end]};
}

void printRoute(ofstream& outFile, CostResult& res, double srcLat, double srcLon, double dstLat, double dstLon, double costPerKm[4]) {
    if (res.cost < 0) {
        outFile << "No route found!\n\n";
        return;
    }
    
    double totalDist = 0, totalCost = 0;
    
    int i = 0;
    while (i < (int)res.path.size() - 1) {
        int startNode = res.path[i];
        int mode = res.modes[i];
        
        int j = i;
        double segDist = 0;
        while (j < (int)res.path.size() - 1 && res.modes[j] == mode) {
            double d = haversine(graph.nodes[res.path[j]].lat, graph.nodes[res.path[j]].lon,
                                graph.nodes[res.path[j+1]].lat, graph.nodes[res.path[j+1]].lon);
            segDist += d;
            j++;
        }
        
        int endNode = res.path[j];
        double segCost = segDist * costPerKm[mode];
        totalDist += segDist;
        totalCost += segCost;
        
        string action = (mode == 0) ? "Drive" : "Ride";
        outFile << fixed << setprecision(2);
        outFile << "Cost: Tk " << segCost << " - " << action << " " << getModeName(mode);
        outFile << " from (" << graph.nodes[startNode].lon << ", " << graph.nodes[startNode].lat << ")";
        outFile << " to (" << graph.nodes[endNode].lon << ", " << graph.nodes[endNode].lat << ")";
        outFile << " (" << segDist << " km)\n";
        
        i = j;
    }
    
    outFile << "\nTotal Distance: " << totalDist << " km\n";
    outFile << "Total Cost: Tk " << totalCost << "\n\n";
}

int main() {
    string basePath = "/media/nym/Nym_s Files/grph-project/";
    
    cout << "Loading data for Problem 2...\n";
    loadRoads(graph, basePath + "Roadmap-Dhaka.csv");
    loadTransport(graph, basePath + "Routemap-DhakaMetroRail.csv", 1);
    cout << "Loaded " << graph.nodeCount << " nodes\n\n";
    
    // Open output file
    ofstream outFile(basePath + "problem2/output.txt");
    
    outFile << "============================================================\n";
    outFile << "PROBLEM 2: Cheapest Route (Car + Metro)\n";
    outFile << "Cost per km: Car = Tk 20, Metro = Tk 5\n";
    outFile << "============================================================\n\n";
    
    // Test inputs from dataset
    double testInputs[3][4] = {
        // Test 1: Mirpur 12 to Agargaon (Metro route)
        {90.363833, 23.834145, 90.380682, 23.776812},
        // Test 2: Cantonment to Shahbag
        {90.364255, 23.828335, 90.396151, 23.738265},
        // Test 3: Uttara to Matijheel
        {90.404772, 23.855136, 90.417671, 23.728911}
    };
    
    // Cost per km: Car=20, Metro=5
    double costPerKm[4] = {20, 5, 0, 0};
    bool allowed[4] = {true, true, false, false};
    
    for (int t = 0; t < 3; t++) {
        double srcLon = testInputs[t][0];
        double srcLat = testInputs[t][1];
        double dstLon = testInputs[t][2];
        double dstLat = testInputs[t][3];
        
        double walkDist;
        int startId = graph.getNearestNode(srcLat, srcLon, walkDist);
        int endId = graph.getNearestNode(dstLat, dstLon, walkDist);
        
        CostResult res = cheapestRoute(startId, endId, costPerKm, allowed);
        
        outFile << "------------------------------------------------------------\n";
        outFile << "Test Case " << (t+1) << ":\n";
        outFile << "------------------------------------------------------------\n";
        outFile << fixed << setprecision(6);
        outFile << "Source: (" << srcLon << ", " << srcLat << ")\n";
        outFile << "Destination: (" << dstLon << ", " << dstLat << ")\n\n";
        
        printRoute(outFile, res, srcLat, srcLon, dstLat, dstLon, costPerKm);
        
        if (!res.path.empty()) {
            saveKML(graph, res.path, basePath + "problem2/output_test" + to_string(t+1) + ".kml");
        }
        
        cout << "Test " << (t+1) << ": Cost = Tk " << fixed << setprecision(2) << res.cost << "\n";
    }
    
    outFile << "============================================================\n";
    outFile.close();
    
    cout << "\nOutput saved to problem2/output.txt\n";
    
    return 0;
}
