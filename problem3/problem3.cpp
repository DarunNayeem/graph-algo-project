// Problem 3: Cheapest Cost (All Transport Modes)
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

void printRoute(ofstream& outFile, CostResult& res, double costPerKm[4]) {
    if (res.cost < 0) {
        outFile << "No route found!\n";
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
        
        string action = (mode == 0) ? "Drive Car" : "Ride " + getModeName(mode);
        outFile << fixed << setprecision(2);
        outFile << "Cost: Tk " << segCost << ": " << action;
        outFile << " from (" << fixed << setprecision(6) << graph.nodes[startNode].lon << ", " << graph.nodes[startNode].lat << ")";
        outFile << " to (" << graph.nodes[endNode].lon << ", " << graph.nodes[endNode].lat << ").\n";
        
        i = j;
    }
    
    outFile << "\n";
    outFile << fixed << setprecision(2);
    outFile << "Total Distance: " << totalDist << " km\n";
    outFile << "Total Cost: Tk " << totalCost << "\n";
}

int main() {
    string basePath = "/media/nym/Nym_s Files/grph-project/";
    
    cout << "Loading data for Problem 3...\n";
    loadRoads(graph, basePath + "Roadmap-Dhaka.csv");
    loadTransport(graph, basePath + "Routemap-DhakaMetroRail.csv", 1);
    loadTransport(graph, basePath + "Routemap-BikolpoBus.csv", 2);
    loadTransport(graph, basePath + "Routemap-UttaraBus.csv", 3);
    cout << "Loaded " << graph.nodeCount << " nodes\n\n";
    
    // Test inputs from dataset
    double testInputs[3][4] = {
        // Test 1: Mirpur 12 to TSC (Metro + Bus available)
        {90.363833, 23.834145, 90.385008, 23.732862},
        // Test 2: Uttara (BRTC) to New Market
        {90.401026, 23.876867, 90.385008, 23.732862},
        // Test 3: Khilkhet to Dhanmondi 27
        {90.419952, 23.828786, 90.375289, 23.756477}
    };
    
    // Cost per km: Car=20, Metro=5, Bikalpa=7, Uttara=7
    double costPerKm[4] = {20, 5, 7, 7};
    bool allowed[4] = {true, true, true, true};
    
    for (int t = 0; t < 3; t++) {
        double srcLon = testInputs[t][0];
        double srcLat = testInputs[t][1];
        double dstLon = testInputs[t][2];
        double dstLat = testInputs[t][3];
        
        double walkDist;
        int startId = graph.getNearestNode(srcLat, srcLon, walkDist);
        int endId = graph.getNearestNode(dstLat, dstLon, walkDist);
        
        CostResult res = cheapestRoute(startId, endId, costPerKm, allowed);
        
        // Create separate output file for each test case
        ofstream outFile(basePath + "problem3/output_test" + to_string(t+1) + ".txt");
        
        outFile << fixed << setprecision(6);
        outFile << "Problem No: 3\n";
        outFile << "Source: (" << srcLon << ", " << srcLat << ")\n";
        outFile << "Destination: (" << dstLon << ", " << dstLat << ")\n\n";
        
        printRoute(outFile, res, costPerKm);
        
        if (!res.path.empty()) {
            saveKML(graph, res.path, basePath + "problem3/output_test" + to_string(t+1) + ".kml");
        }
        
        outFile.close();
        cout << "Test " << (t+1) << ": Cost = Tk " << fixed << setprecision(2) << res.cost << "\n";
    }
    
    cout << "\nOutput saved to problem3/output_test1.txt, output_test2.txt, output_test3.txt\n";
    
    return 0;
}
