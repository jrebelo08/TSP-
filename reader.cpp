#include "reader.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_set>
#include <complex>

std::unordered_map<int, Reader::Coordinates> Reader::readCoordinates() {
    std::ifstream file("../Data/Extra_Fully_Connected_Graphs/nodes.csv");
    std::unordered_map<int, Coordinates> nodeCoordinates;
    std::string line;
    if (file.is_open()) {
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;
            std::getline(iss, token, ',');
            int id = std::stoi(token);
            std::getline(iss, token, ',');
            double longitude = std::stod(token);
            std::getline(iss, token, ',');
            double latitude = std::stod(token);
            nodeCoordinates[id] = {latitude, longitude};
        }
        file.close();
    }
    return nodeCoordinates;
}



std::unordered_map<int, Reader::Coordinates> Reader::readCoordinatesRealWorldGraph(int type) {

    std::ifstream file;

    if(type==1){
         file.open("../Data/Real-world Graphs/graph1/nodes.csv");
    }
    else if(type==2){
        file.open("../Data/Real-world Graphs/graph2/nodes.csv");
    }

    else if(type==3){
       file.open("../Data/Real-world Graphs/graph3/nodes.csv");
    }

    std::unordered_map<int, Coordinates> nodeCoordinates;
    std::string line;
    if (file.is_open()) {
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;
            std::getline(iss, token, ',');
            int id = std::stoi(token);
            std::getline(iss, token, ',');
            double longitude = std::stod(token);
            std::getline(iss, token, ',');
            double latitude = std::stod(token);
            nodeCoordinates[id] = {latitude, longitude};
        }
        file.close();
    }
    return nodeCoordinates;
}

double Reader::convert_to_radians(double coord) {
    return coord * 3.14 / 180.0;
}

double Reader::Haversine(double lat1, double lon1, double lat2, double lon2) {

    double rad_lat1 = Reader::convert_to_radians(lat1);
    double rad_lon1 = Reader::convert_to_radians(lon1);
    double rad_lat2 = Reader::convert_to_radians(lat2);
    double rad_lon2 = Reader::convert_to_radians(lon2);


    double delta_lat = rad_lat2 - rad_lat1;
    double delta_lon = rad_lon2 - rad_lon1;

    double a = std::pow(std::sin(delta_lat / 2), 2) +
               std::cos(rad_lat1) * std::cos(rad_lat2) * std::pow(std::sin(delta_lon / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    const double earthRadius = 6371000;

    return earthRadius * c;
}

std::vector<std::vector<int>> Reader::kMeansClustering(const Graph<int>& graph, int k, const std::unordered_map<int, Coordinates>& coordinates) {
    std::vector<std::vector<int>> clusters(k);
    std::vector<int> vertexIds;

    for (const auto& vertex : graph.getVertexSet()) {
        vertexIds.push_back(vertex->getInfo());
    }

    std::vector<int> centroids;
    centroids.push_back(vertexIds[0]);
    std::unordered_set<int> visitedCentroids;
    visitedCentroids.insert(vertexIds[0]);

    while (centroids.size() < k) {
        double maxDist = -1.0;
        int farthestVertex = -1;
        for (int vertex : vertexIds) {
            if (visitedCentroids.find(vertex) == visitedCentroids.end()) {
                double minDist = INF;
                for (int centroid : centroids) {
                    double dist = Haversine(
                            coordinates.at(vertex).latitude, coordinates.at(vertex).longitude,
                            coordinates.at(centroid).latitude, coordinates.at(centroid).longitude
                    );
                    if (dist < minDist) {
                        minDist = dist;
                    }
                }
                if (minDist > maxDist) {
                    maxDist = minDist;
                    farthestVertex = vertex;
                }
            }
        }
        if (farthestVertex != -1) {
            centroids.push_back(farthestVertex);
            visitedCentroids.insert(farthestVertex);
        } else {
            break;
        }
    }

    bool changed = true;
    while (changed) {
        for (auto& cluster : clusters) {
            cluster.clear();
        }

        for (const auto& vertex : graph.getVertexSet()) {
            int nearestCentroid = -1;
            double minDistance = INF;
            for (int i = 0; i < k; ++i) {
                double dist = Haversine(
                        coordinates.at(vertex->getInfo()).latitude, coordinates.at(vertex->getInfo()).longitude,
                        coordinates.at(centroids[i]).latitude, coordinates.at(centroids[i]).longitude
                );
                if (dist < minDistance) {
                    minDistance = dist;
                    nearestCentroid = i;
                }
            }
            clusters[nearestCentroid].push_back(vertex->getInfo());
        }

        changed = false;
        for (int i = 0; i < k; ++i) {
            double avgLat = 0.0, avgLon = 0.0;
            for (int vertex : clusters[i]) {
                avgLat += coordinates.at(vertex).latitude;
                avgLon += coordinates.at(vertex).longitude;
            }
            avgLat /= clusters[i].size();
            avgLon /= clusters[i].size();

            double minDistance = INF;
            int newCentroid = centroids[i];
            for (int vertex : clusters[i]) {
                double dist = Haversine(avgLat, avgLon, coordinates.at(vertex).latitude, coordinates.at(vertex).longitude);
                if (dist < minDistance) {
                    minDistance = dist;
                    newCentroid = vertex;
                }
            }

            if (newCentroid != centroids[i]) {
                centroids[i] = newCentroid;
                changed = true;
            }
        }
    }

    return clusters;
}

Graph<int> Reader::readAndParseStadium() {
    std::ifstream file("../Data/Toy-Graphs/stadiums.csv");
    std::string line;

    Graph<int> graph;

    if (!file.is_open()) {
        std::cerr << "Failed to open file\n";
        return graph;
    }

    std::getline(file, line);

    while (std::getline(file, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');

        std::istringstream iss(line);
        int source, dest;
        double dist;

        if (!(iss >> source >> dest >> dist)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;
        }

        graph.addVertex(source);
        graph.addVertex(dest);
        graph.addEdge(source, dest, dist);
        graph.addEdge(dest, source, dist);
    }

    file.close();
    return graph;



}

Graph<int> Reader::readAndParseShipping() {
    std::ifstream file("../Data/Toy-Graphs/shipping.csv");
    std::string line;

    Graph<int> graph;

    if (!file.is_open()) {
        std::cerr << "Failed to open file\n";
        return graph;
    }

    std::getline(file, line);

    while (std::getline(file, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');

        std::istringstream iss(line);
        int source, dest;
        double dist;

        if (!(iss >> source >> dest >> dist)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;
        }

        graph.addVertex(source);
        graph.addVertex(dest);
        graph.addEdge(source, dest, dist);
        graph.addEdge(dest, source, dist);
    }

    file.close();
    return graph;
}


Graph<int> Reader::readAndParseTourism() {
    std::ifstream file("../Data/Toy-Graphs/tourism.csv");
    std::string line;

    Graph<int> graph;

    if (!file.is_open()) {
        std::cerr << "Failed to open file\n";
        return graph;
    }

    std::getline(file, line);

    while (std::getline(file, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');

        std::istringstream iss(line);
        int source, dest;
        double dist;
        std::string label_source_str, label_dest_str;

        if (!(iss >> source >> dest >> dist >> label_source_str >> label_dest_str)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;
        }

        graph.addVertex(source);
        graph.addVertex(dest);
        graph.addEdge(source, dest, dist);
        graph.addEdge(dest, source, dist);
    }

    file.close();
    return graph;
}

Graph<int> Reader::readAndParseRealWorld_Graphs(int graphNumber, std::unordered_map<int, Vertex<int>*> &vertexMap, std::unordered_map<std::string, Edge<int>*> &edgeMap)
{
    Graph<int> graph;
    std::string line;

    std::string filePath;
    switch (graphNumber) {
        case 1:
            filePath = "../Data/Real-world Graphs/graph1/edges.csv";
            break;
        case 2:
            filePath = "../Data/Real-world Graphs/graph2/edges.csv";
            break;
        case 3:
            filePath = "../Data/Real-world Graphs/graph3/edges.csv";
            break;
        default:
            std::cerr << "Invalid graph number!" << std::endl;
            return graph;
    }

    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return graph;
    }
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        if(line.empty()){
            return graph;
        }
        std::istringstream iss(line);
        int source, dest;
        double dist;

        if (!(iss >> source >> dest >> dist)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;
        }
        Vertex<int>* sourceVertex;
        Vertex<int>* destVertex;

        if (vertexMap.find(source) == vertexMap.end()) {
            sourceVertex = graph.addVertexNew(source);
            vertexMap[source] = sourceVertex;

        }else{
            sourceVertex = vertexMap[source];

        }

        if (vertexMap.find(dest) == vertexMap.end()) {
            destVertex = graph.addVertexNew(dest);
            vertexMap[dest] = destVertex;

        }else{
            destVertex = vertexMap[dest];
        }

        Edge<int>* edge = graph.addEdgeNew(sourceVertex, destVertex,dist);

        std::string nodes;
        nodes += std::to_string(sourceVertex->getInfo());
        nodes += "_";
        nodes += std::to_string(destVertex->getInfo());

        edgeMap[nodes] = edge;

        std::string nodes_otherWay;
        nodes_otherWay += std::to_string(destVertex->getInfo());
        nodes_otherWay += "_";
        nodes_otherWay += std::to_string(sourceVertex->getInfo());

        edgeMap[nodes_otherWay] = edge;


    }

    return graph;
}

Graph<int> Reader::readAndParseRealWorld_Graphs4_2(int graphNumber, std::unordered_map<int, Vertex<int>*> &vertexMap, std::unordered_map<std::string, Edge<int>*> &edgeMap)
{
    Graph<int> graph;
    std::string line;
    std::unordered_map<int, Coordinates> coordinates;

    std::string filePath;
    switch (graphNumber) {
        case 1:
            filePath = "../Data/Real-world Graphs/graph1/edges.csv";
            coordinates = readCoordinatesRealWorldGraph(1);
            break;
        case 2:
            filePath = "../Data/Real-world Graphs/graph2/edges.csv";
            coordinates = readCoordinatesRealWorldGraph(2);
            break;
        case 3:
            filePath = "../Data/Real-world Graphs/graph3/edges.csv";
            coordinates = readCoordinatesRealWorldGraph(3);
            break;
        default:
            std::cerr << "Invalid graph number!" << std::endl;
            return graph;
    }

    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return graph;
    }

    std::getline(file, line);
    while (std::getline(file, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        if (line.empty()) {
            continue;
        }

        std::istringstream iss(line);
        int source, dest;
        double dist;

        if (!(iss >> source >> dest >> dist)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;
        }

        Vertex<int>* sourceVertex;
        if (vertexMap.find(source) == vertexMap.end()) {
            sourceVertex = graph.addVertexNew(source);
            vertexMap[source] = sourceVertex;
        } else {
            sourceVertex = vertexMap[source];
        }

        Vertex<int>* destVertex;
        if (vertexMap.find(dest) == vertexMap.end()) {
            destVertex = graph.addVertexNew(dest);
            vertexMap[dest] = destVertex;
        } else {
            destVertex = vertexMap[dest];
        }

        Edge<int>* edge = graph.addEdgeNew(sourceVertex, destVertex, dist);
        edgeMap[std::to_string(source) + "_" + std::to_string(dest)] = edge;
        edgeMap[std::to_string(dest) + "_" + std::to_string(source)] = edge;
    }

    for (auto& sourceEntry : vertexMap) {
        for (auto& destEntry : vertexMap) {
            if (sourceEntry.first != destEntry.first) {
                double haversineDist = Haversine(
                        coordinates[sourceEntry.first].latitude, coordinates[sourceEntry.first].longitude,
                        coordinates[destEntry.first].latitude, coordinates[destEntry.first].longitude
                );

                std::string edgeId1 = std::to_string(sourceEntry.first) + "_" + std::to_string(destEntry.first);
                std::string edgeId2 = std::to_string(destEntry.first) + "_" + std::to_string(sourceEntry.first);

                if (edgeMap.find(edgeId1) == edgeMap.end() && edgeMap.find(edgeId2) == edgeMap.end()) {
                    Edge<int>* edge = graph.addEdgeNew(sourceEntry.second, destEntry.second, haversineDist);
                    edgeMap[edgeId1] = edge;
                    edgeMap[edgeId2] = edge;
                }
            }
        }
    }

    return graph;
}

Graph<int> Reader::readAndParse4_2Extra_Fully_Connected_Graphs(const std::string filename,std::unordered_map<int, Vertex<int>*> &vertexMap, std::unordered_map<std::string, Edge<int>*> &edgeMap) {
    std::ifstream file(filename);
    std::string line;

    Graph<int> graph;
    std::unordered_map<int, Coordinates> coordinates = readCoordinates();

    if (!file.is_open()) {
        std::cerr << "Failed to open file\n";
        return graph;
    }

    std::getline(file, line);

    while (std::getline(file, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        int source, dest;
        double dist;

        if (!(iss >> source >> dest >> dist)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;
        }
        Vertex<int>* sourceVertex;
        Vertex<int>* destVertex;

        if (vertexMap.find(source) == vertexMap.end()) {
            sourceVertex = graph.addVertexNew(source);
            vertexMap[source] = sourceVertex;

        }else{
            sourceVertex = vertexMap[source];

        }

        if (vertexMap.find(dest) == vertexMap.end()) {
            destVertex = graph.addVertexNew(dest);
            vertexMap[dest] = destVertex;

        }else{
            destVertex = vertexMap[dest];
        }

        Edge<int>* edge = graph.addEdgeNew(sourceVertex, destVertex,dist);

        std::string nodes;
        nodes += std::to_string(sourceVertex->getInfo());
        nodes += "_";
        nodes += std::to_string(destVertex->getInfo());

        edgeMap[nodes] = edge;

        std::string nodes_otherWay;
        nodes_otherWay += std::to_string(destVertex->getInfo());
        nodes_otherWay += "_";
        nodes_otherWay += std::to_string(sourceVertex->getInfo());

        edgeMap[nodes_otherWay] = edge;
    }

    auto vertices = graph.getVertexSet();
    for (auto& v : vertices) {
        int vInfo = v->getInfo();
        for (auto& w : vertices) {
            int wInfo = w->getInfo();
            if (vInfo != wInfo) {
                std::string edgeKey = std::to_string(vInfo) + "_" + std::to_string(wInfo);
                std::string reverseEdgeKey = std::to_string(wInfo) + "_" + std::to_string(vInfo);

                if (edgeMap.find(edgeKey) == edgeMap.end() && edgeMap.find(reverseEdgeKey) == edgeMap.end()) {
                    double dist = Haversine(
                            coordinates[vInfo].latitude, coordinates[vInfo].longitude,
                            coordinates[wInfo].latitude, coordinates[wInfo].longitude
                    );

                    Edge<int>* edge = graph.addEdgeNew(vertexMap[vInfo], vertexMap[wInfo], dist);
                    edgeMap[edgeKey] = edge;
                    edgeMap[reverseEdgeKey] = edge;

                    graph.addEdgeNew(vertexMap[wInfo], vertexMap[vInfo], dist);
                }
            }
        }
    }

    file.close();
    return graph;
}













