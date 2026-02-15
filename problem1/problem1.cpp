// Problem 1: Shortest Distance (Car Only)
#include "../common/graph.h"

Graph graph;

// Dijkstra's algorithm for shortest car route
pair<vector<int>, double> shortestCarRoute(int start, int end) {
    int n = graph.nodeCount;
    vector<double> dist(n, INF);
    vector<int> parent(n, -1);
    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;
    
    dist[start] = 0;
    pq.push({0, start});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d > dist[u]) continue;
        if (u == end) break;
        
        for (auto& e : graph.adj[u]) {
            if (e.mode != 0) continue; // car only
            double newDist = dist[u] + e.dist;
            if (newDist < dist[e.to]) {
                dist[e.to] = newDist;
                parent[e.to] = u;
                pq.push({newDist, e.to});
            }
        }
    }
    
    if (dist[end] >= INF) return {{}, -1};
    
    vector<int> path;
    for (int v = end; v != -1; v = parent[v]) path.push_back(v);
    reverse(path.begin(), path.end());
    return {path, dist[end]};
}

int main() {
    string basePath = "/media/nym/Nym_s Files/grph-project/";
    
    cout << "Loading road data for Problem 1...\n";
    loadRoads(graph, basePath + "Roadmap-Dhaka.csv");
    cout << "Loaded " << graph.nodeCount << " nodes\n\n";
    
    // Open output file
    ofstream outFile(basePath + "problem1/output.txt");
    
    outFile << "============================================================\n";
    outFile << "PROBLEM 1: Shortest Car Route\n";
    outFile << "============================================================\n\n";
    
    // Test inputs from dataset (actual coordinates from road/transport data)
    double testInputs[3][4] = {
        // Test 1: Uttara to Mirpur 12 (Northern Dhaka)
        {90.404772, 23.855136, 90.363833, 23.834145},
        // Test 2: Farmgate to Shahbag (Central Dhaka)
        {90.390157, 23.758382, 90.396151, 23.738265},
        // Test 3: Banani to Matijheel (East-West route)
        {90.401034, 23.794465, 90.417671, 23.728911}
    };
    
    for (int t = 0; t < 3; t++) {
        double srcLon = testInputs[t][0];
        double srcLat = testInputs[t][1];
        double dstLon = testInputs[t][2];
        double dstLat = testInputs[t][3];
        
        double walkDist;
        int startId = graph.getNearestNode(srcLat, srcLon, walkDist);
        int endId = graph.getNearestNode(dstLat, dstLon, walkDist);
        
        auto [path, dist] = shortestCarRoute(startId, endId);
        
        outFile << "------------------------------------------------------------\n";
        outFile << "Test Case " << (t+1) << ":\n";
        outFile << "------------------------------------------------------------\n";
        outFile << fixed << setprecision(6);
        outFile << "Source: (" << srcLon << ", " << srcLat << ")\n";
        outFile << "Destination: (" << dstLon << ", " << dstLat << ")\n";
        
        if (dist >= 0) {
            outFile << fixed << setprecision(4);
            outFile << "Shortest Distance: " << dist << " km\n\n";
            saveKML(graph, path, basePath + "problem1/output_test" + to_string(t+1) + ".kml");
        } else {
            outFile << "Status: No route found!\n\n";
        }
        
        cout << "Test " << (t+1) << ": Distance = " << fixed << setprecision(4) << dist << " km\n";
    }
    
    outFile << "============================================================\n";
    outFile.close();
    
    cout << "\nOutput saved to problem1/output.txt\n";
    cout << "KML files saved to problem1/\n";
    
    return 0;
}
