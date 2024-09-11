#ifndef READER_H
#define READER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "Classes/Graph.h"

/**
 * @brief A class for reading and parsing various types of graph data.
 *
 */
class Reader {
public:
    /**
     * @brief Reads and parses a graph representing stadium data.
     * @return A graph representing stadium data.
     */
    Graph<int> readAndParseStadium();

    /**
     * @brief Reads and parses a graph representing shipping data.
     * @return A graph representing shipping data.
     */
    Graph<int> readAndParseShipping();

    /**
     * @brief Reads and parses a graph representing tourism data.
     * @return A graph representing tourism data.
     */
    static Graph<int> readAndParseTourism();

    /**
     * @brief Reads and parses a graph and makes it fully connected.
     * @param filename The filename of the graph data.
     * @param vertexMap A map containing vertex IDs and pointers to vertex objects.
     * @param edgeMap A map containing edge keys and pointers to edge objects.
     * @return A fully connected graph.
     */
    Graph<int> readAndParse4_2Extra_Fully_Connected_Graphs(const std::string filename, std::unordered_map<int, Vertex<int>*> &vertexMap, std::unordered_map<std::string, Edge<int>*> &edgeMap);

    /**
     * @brief Reads and parses a real-world graph from a file based on the specified graph number.
     * In this specific parsing, we are using 2 unordered maps, one for vertex's and the other for edges, this prevent the exhaustive search that was being performed in the find vertex and addEgde, methods.
     * We also discard those functions to meet the performance requirements, creating new ones, that are more efficient, receiving the objects instead of searching them.
     * @param graphNumber The number of the graph to read and parse.
     * @param vertexMap A map containing vertex IDs and pointers to vertex objects.
     * @param edgeMap A map containing edge keys and pointers to edge objects.
     * @return A real-world graph.
     */
    static Graph<int> readAndParseRealWorld_Graphs(int graphNumber, std::unordered_map<int, Vertex<int>*> &vertexMap, std::unordered_map<std::string, Edge<int>*> &edgeMap);

    /**
     * @brief Reads and parses a real-world graph from a file based on the specified graph number.
     * In this specific parsing, we are using 2 unordered maps, one for vertex's and the other for edges, this prevent the exhaustive search that was being performed in the find vertex and addEgde, methods.
     * We also discard those functions to meet the performance requirements, creating new ones, that are more efficient, receiving the objects instead of searching them.
     * @param graphNumber The number of the graph to read and parse.
     * @param vertexMap A map containing vertex IDs and pointers to vertex objects.
     * @param edgeMap A map containing edge keys and pointers to edge objects.
     * @return A real-world graph.
     */
    Graph<int> readAndParseRealWorld_Graphs4_2(int graphNumber, std::unordered_map<int, Vertex<int>*> &vertexMap, std::unordered_map<std::string, Edge<int>*> &edgeMap);

    /**
     * @brief Structure to hold latitude and longitude coordinates.
     */
    struct Coordinates {
        double latitude; /**< Latitude coordinate */
        double longitude; /**< Longitude coordinate */
    };

    /**
     * @brief Converts degrees to radians.
     * @param coord The coordinate value in degrees.
     * @return The coordinate value in radians.
     */
    static double convert_to_radians(double coord);

    /**
     * @brief Computes the Haversine distance between two points given their coordinates.
     * @param lat1 Latitude of the first point.
     * @param lon1 Longitude of the first point.
     * @param lat2 Latitude of the second point.
     * @param lon2 Longitude of the second point.
     * @return The Haversine distance between the two points.
     */
    static double Haversine(double lat1, double lon1, double lat2, double lon2);

    /**
     * @brief Reads coordinates from a CSV file.
     * @return A map containing node IDs and their corresponding coordinates.
     */
    std::unordered_map<int, Coordinates> readCoordinates();

    /**
     * @brief Performs k-means clustering on a graph based on geographical coordinates.
     * @param graph The graph to cluster.
     * @param k The number of clusters.
     * @param coordinates A map containing node IDs and their coordinates.
     * @return A vector of clusters, where each cluster is represented by a vector of node IDs.
     */
    static std::vector<std::vector<int>> kMeansClustering(const Graph<int>& graph, int k, const std::unordered_map<int, Coordinates>& coordinates);


    static std::unordered_map<int, Reader::Coordinates> readCoordinatesRealWorldGraph(int type);
};

#endif /* READER_H */
