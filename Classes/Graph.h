#ifndef PROJ1_GRAPH_H
#define PROJ1_GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <unordered_set>

template <class T>
class Edge;

#define INF std::numeric_limits<double>::max()


template <class T>
class Vertex {
public:
    Vertex(T in);
    bool operator<(Vertex<T> & vertex) const;

    T getInfo() const;
    std::vector<Edge<T> *> getAdj() const;
    bool isVisited() const;
    void setVisited(bool visited);
    Edge<T> * addEdge(Vertex<T> *dest, double w);




protected:
    T info;
    std::vector<Edge<T> *> adj;

    bool visited = false;
    double dist = 0;

    std::vector<Edge<T> *> incoming;
};

template <class T>
class Edge {
public:
    Edge(Vertex<T> *orig, Vertex<T> *dest, double w);

    Vertex<T> * getDest() const;
    double getWeight() const;
    Vertex<T> * getOrig() const;

protected:
    Vertex<T> * dest;
    double weight;
    Vertex<T> *orig;

};

template <class T>
class Graph {
public:
    ~Graph();

    Vertex<T> *findVertex(const T &in) const;
    bool addVertex(const T &in);
    Vertex<T>* addVertexNew(const T &in);
    bool addEdge(const T &sourc, const T &dest, double w);
    Edge<T>* addEdgeNew(Vertex<T> *source, Vertex<T> *dest, double w);

    /**
     * @brief Finds the minimum spanning tree (MST) of the graph using Prim's algorithm.
     * @param startIdx The index of the starting vertex.
     * @return A vector of pointers to edges representing the MST.
     * @timecomplexity O(E * log(V)), where E is the number of edges and V is the number of vertices.
     */
    std::vector<Edge<T>*> primMST(int startIdx) {
        std::vector<Edge<T>*> mst;
        if (vertexSet.empty() || startIdx < 0 || startIdx >= vertexSet.size()) return mst;

        std::priority_queue<std::pair<double, Edge<T>*>, std::vector<std::pair<double, Edge<T>*>>, std::greater<>> pq;
        std::vector<bool> inMST(vertexSet.size(), false);

        inMST[startIdx] = true;
        for (auto edge : vertexSet[startIdx]->getAdj()) {
            pq.push({edge->getWeight(), edge});
        }

        while (!pq.empty()) {
            auto [weight, edge] = pq.top();
            pq.pop();
            Vertex<T>* dest = edge->getDest();

            int destIdx = findVertexIdx(dest->getInfo());
            if (destIdx != -1 && !inMST[destIdx]) {
                mst.push_back(edge);
                inMST[destIdx] = true;

                for (auto nextEdge : dest->getAdj()) {
                    if (!inMST[findVertexIdx(nextEdge->getDest()->getInfo())]) {
                        pq.push({nextEdge->getWeight(), nextEdge});
                    }
                }
            }
        }
        return mst;
    }

    /**
     * @brief Finds the minimum spanning tree (MST) of a graph using Prim's algorithm.
     * @param startIdx The index of the starting vertex.
     * @param vertexMap A map containing vertex IDs and pointers to vertex objects.
     * @param edgeMap A map containing edge keys and pointers to edge objects.
     * @return A vector of edges representing the MST.
     * @timecomplexity O(E * log(V)), where E is the number of edges and V is the number of vertices.
     */
    std::vector<Edge<T>*> primMSTMaps(int startIdx, std::unordered_map<int, Vertex<T>*> &vertexMap, std::unordered_map<std::string, Edge<T>*> &edgeMap) {
        std::vector<Edge<T>*> mst;
        if (vertexMap.empty() || vertexMap.find(startIdx) == vertexMap.end()) return mst;

        auto cmp = [](const std::pair<double, Edge<T>*>& left, const std::pair<double, Edge<T>*>& right) { return left.first > right.first; };
        std::priority_queue<std::pair<double, Edge<T>*>, std::vector<std::pair<double, Edge<T>*>>, decltype(cmp)> pq(cmp);

        std::unordered_map<int, bool> inMST;
        for (const auto& [vertexInfo, vertex] : vertexMap) {
            inMST[vertexInfo] = false;
        }

        inMST[startIdx] = true;
        Vertex<T>* startVertex = vertexMap[startIdx];
        for (auto edge : startVertex->getAdj()) {
            pq.push({edge->getWeight(), edge});
        }

        while (!pq.empty()) {
            auto [weight, edge] = pq.top();
            pq.pop();
            Vertex<T>* dest = edge->getDest();
            int destIdx = dest->getInfo();

            if (!inMST[destIdx]) {
                mst.push_back(edge);
                inMST[destIdx] = true;

                for (auto nextEdge : dest->getAdj()) {
                    int nextDestIdx = nextEdge->getDest()->getInfo();
                    if (!inMST[nextDestIdx]) {
                        pq.push({nextEdge->getWeight(), nextEdge});
                    }
                }
            }
        }

        return mst;
    }

    /**
     * @brief Constructs a tour using the nearest neighbor heuristic.
     * @param graph The graph on which to perform the nearest neighbor search.
     * @return A vector of vertices representing the tour.
     * @timecomplexity O(V^2), where V is the number of vertices.
     */
    std::vector<Vertex<T>*> nearestNeighbour(Graph<T>& graph) {
        std::vector<Vertex<T>*> tour;

        std::vector<Vertex<T>*> vertices = graph.getVertexSet();

        Vertex<int>* startVertex = graph.findVertex(0);

        if (startVertex == nullptr) {
            return tour;
        }

        tour.reserve(vertices.size() + 1);

        tour.push_back(startVertex);

        startVertex->setVisited(true);


        while (tour.size() < vertices.size()) {
            double minDistance = INF;
            Vertex<T>* nearestNeighbor = nullptr;
            for (auto vertex : vertices) {
                if (!vertex->isVisited()) {
                    double distance = graph.findEdge(tour.back()->getInfo(), vertex->getInfo())->getWeight();
                    if (distance < minDistance) {
                        minDistance = distance;
                        nearestNeighbor = vertex;
                    }
                }
            }
        }

        tour.push_back(startVertex);

        for (auto vertex : vertices) {
            vertex->setVisited(false);
        }

        return tour;
    }

    /**
     * @brief Constructs a tour using the nearest neighbor heuristic with medium complexity.
     * @param graph The graph on which to perform the nearest neighbor search.
     * @param vertexMap A map containing vertex IDs and pointers to vertex objects.
     * @param edgeMap A map containing edge keys and pointers to edge objects.
     * @return A vector of vertices representing the tour.
     * @timecomplexity O(V^2), where V is the number of vertices.
     */
    std::vector<Vertex<T>*> nearestNeighbourMedium(Graph<T>& graph, std::unordered_map<int, Vertex<int>*> vertexMap, std::unordered_map<std::string, Edge<int>*> edgeMap) {
        std::vector<Vertex<T>*> tour;

        auto startIt = vertexMap.find(0);
        if (startIt == vertexMap.end()) {
            return tour;
        }

        Vertex<int>* startVertex = startIt->second;

        tour.reserve(vertexMap.size() + 1);

        tour.push_back(startVertex);
        startVertex->setVisited(true);

        while (tour.size() < vertexMap.size()) {
            double minDistance = INF;
            Vertex<T>* nearestNeighbor = nullptr;

            Vertex<T>* currentVertex = tour.back();
            int currentVertexInfo = currentVertex->getInfo();

            for (auto& [vertexInfo, vertex] : vertexMap) {
                if (!vertex->isVisited()) {
                    std::string edgeKey = std::to_string(currentVertexInfo) + "_" + std::to_string(vertexInfo);
                    auto edgeIt = edgeMap.find(edgeKey);

                    if (edgeIt != edgeMap.end()) {
                        double distance = edgeIt->second->getWeight();
                        if (distance < minDistance) {
                            minDistance = distance;
                            nearestNeighbor = vertex;
                        }
                    }
                }
            }

            if (nearestNeighbor == nullptr) {
                break;
            }

            nearestNeighbor->setVisited(true);
            tour.push_back(nearestNeighbor);
        }

        tour.push_back(startVertex);

        for (auto& [vertexInfo, vertex] : vertexMap) {
            vertex->setVisited(false);
        }

        return tour;
    }

    /**
     * @brief Constructs a tour starting from a specified vertex using the nearest neighbor heuristic.
     * @param graph The graph on which to perform the nearest neighbor search.
     * @param startVertex The starting vertex of the tour.
     * @param vertexMap A map containing vertex IDs and pointers to vertex objects.
     * @param edgeMap A map containing edge keys and pointers to edge objects.
     * @return A vector of vertices representing the tour.
     * @timecomplexity O(V^2), where V is the number of vertices.
     */
    std::vector<Vertex<T>*> nearestNeighbourNode(Graph<T>& graph, Vertex<int>* startVertex,
                                                 std::unordered_map<int, Vertex<int>*> vertexMap,
                                                 std::unordered_map<std::string, Edge<int>*> edgeMap) {
        std::vector<Vertex<T>*> tour;
        std::vector<Vertex<T>*> vertices = graph.getVertexSet();

        if (startVertex == nullptr) {
            std::cout << "Start vertex is null." << std::endl;
            return tour;
        }

        tour.reserve(vertices.size() + 1);
        tour.push_back(startVertex);
        startVertex->setVisited(true);

        while (tour.size() < vertices.size()) {
            double minDistance = std::numeric_limits<double>::infinity();
            Vertex<T>* nearestNeighbor = nullptr;

            for (auto vertex : vertices) {
                if (!vertex->isVisited()) {
                    std::string nodes = std::to_string(tour.back()->getInfo()) + "_" + std::to_string(vertex->getInfo());
                    auto edgeIt = edgeMap.find(nodes);

                    if (edgeIt == edgeMap.end()) {
                        continue;
                    }

                    double distance = edgeIt->second->getWeight();
                    if (distance < minDistance) {
                        minDistance = distance;
                        nearestNeighbor = vertex;
                    }
                }
            }

            if (nearestNeighbor != nullptr) {
                tour.push_back(nearestNeighbor);
                nearestNeighbor->setVisited(true);
            } else {
                break;
            }
        }

        bool allVisited = true;
        for (auto vertex : vertices) {
            if (!vertex->isVisited()) {
                allVisited = false;
                std::cout << "Unvisited node: " << vertex->getInfo() << std::endl;
            }
        }

        std::string returnEdgeKey = std::to_string(tour.back()->getInfo()) + "_" + std::to_string(startVertex->getInfo());
        bool canReturnToStart = edgeMap.find(returnEdgeKey) != edgeMap.end();

        if (!canReturnToStart) {
            for (auto vertex : vertices) {
                vertex->setVisited(false);
            }
            std::cout << "Cannot return to start vertex." << std::endl;
            return std::vector<Vertex<T>*>{};
        }

        tour.push_back(startVertex);

        for (auto vertex : vertices) {
            vertex->setVisited(false);
        }

        std::cout << "Number of unvisited nodes: " << (allVisited ? 0 : vertices.size() - tour.size() - 1) << std::endl;

        return tour;
    }


    /**
     * @brief Constructs a tour for a cluster of vertices using the nearest neighbor heuristic.
     * @param graph The graph on which to perform the nearest neighbor search.
     * @param cluster A vector of vertex IDs representing the cluster of vertices.
     * @return A vector of vertex IDs representing the tour.
     * @timecomplexity O(V^2), where V is the number of vertices.
     */
    std::vector<int> nearestNeighborForCluster(Graph<T>& graph, const std::vector<int>& cluster) {
        std::vector<int> tour;
        if (cluster.empty()) return tour;

        std::unordered_set<int> clusterSet(cluster.begin(), cluster.end());
        Vertex<T>* startVertex = graph.findVertex(cluster[0]);
        if (!startVertex) return tour;

        std::unordered_set<int> visited;
        int currentNode = cluster[0];
        tour.push_back(currentNode);
        visited.insert(currentNode);

        while (visited.size() < cluster.size()) {
            double minDistance = INF;
            int nearestNeighbor = -1;
            for (int vertex : cluster) {
                if (visited.find(vertex) == visited.end()) {
                    Edge<T>* edge = graph.findEdge(currentNode, vertex);
                    if (edge && edge->getWeight() < minDistance) {
                        minDistance = edge->getWeight();
                        nearestNeighbor = vertex;
                    }
                }
            }
            if (nearestNeighbor != -1) {
                tour.push_back(nearestNeighbor);
                visited.insert(nearestNeighbor);
                currentNode = nearestNeighbor;
            } else {
                break;
            }
        }
        tour.push_back(cluster[0]);
        return tour;
    }

    /**
     * @brief Computes the total cost of a tour.
     * @param tour A vector of vertices representing the tour.
     * @param graph The graph containing the edges connecting the vertices.
     * @return The total cost of the tour.
     * @timecomplexity O(N), where N is the number of vertices in the tour.
     */
    double tourCost(const std::vector<Vertex<T>*>& tour, const Graph<T>& graph) {
        double cost = 0;
        for (size_t i = 0; i < tour.size() - 1; ++i) {
            cost += graph.findEdge(tour[i]->getInfo(), tour[i+1]->getInfo())->getWeight();
        }
        return cost;
    }

    /**
     * @brief Performs a 2-opt exchange on a tour to improve its quality.
     * @param tour A vector of vertices representing the tour.
     * @param i The index of the first vertex to exchange.
     * @param j The index of the second vertex to exchange.
     * @return A new tour resulting from the 2-opt exchange.
     * @timecomplexity O(n^2), where n is the number of vertices in the tour.
     */
    std::vector<Vertex<T>*> twoOptExchange(const std::vector<Vertex<T>*>& tour, size_t i, size_t j) {
        std::vector<Vertex<T>*> newTour = tour;
        std::reverse(newTour.begin() + i + 1, newTour.begin() + j + 1);
        return newTour;
    }

    /**
     * @brief Performs the Lin-Kernighan heuristic to improve the quality of a tour.
     * @param graph The graph on which to perform the Lin-Kernighan heuristic.
     * @param vertexMap A map containing vertex IDs and pointers to vertex objects.
     * @param edgeMap A map containing edge keys and pointers to edge objects.
     * @return A vector of vertices representing the improved tour.
     * @timecomplexity O(n^3), where n is the number of vertices in the graph.
     */
    std::vector<Vertex<T>*> linKernighan(Graph<T>& graph, std::unordered_map<int, Vertex<int>*> vertexMap, std::unordered_map<std::string, Edge<int>*> edgeMap) {
        std::vector<Vertex<T>*> tour = nearestNeighbourMedium(graph, vertexMap, edgeMap);
        std::vector<Vertex<T>*> bestTour = tour;
        double bestCost = tourCost(tour, graph);

        const int maxIterations = 2;
        int iter = 0;

        while (iter < maxIterations) {
            bool improvement = false;

            for (size_t i = 0; i < tour.size() - 2; ++i) {
                for (size_t j = i + 2; j < tour.size() - 1; ++j) {
                    std::vector<Vertex<T>*> newTour = twoOptExchange(tour, i, j);
                    double newCost = tourCost(newTour, graph);
                    if (newCost < bestCost) {
                        bestCost = newCost;
                        bestTour = newTour;
                        tour = newTour;
                        improvement = true;
                        break;
                    }
                }
                if (improvement) break;
            }

            if (!improvement) break;

            ++iter;
        }

        return bestTour;
    }

    /**
     * @brief Computes the length of a tour.
     * @param tour A vector of vertex IDs representing the tour.
     * @param graph The graph containing the edges connecting the vertices.
     * @return The length of the tour.
     * @timecomplexity O(N), where N is the number of vertices in the tour.
     */
    double computeTourLength(const std::vector<int>& tour, const Graph<int>& graph) {
        double length = 0.0;
        for (size_t i = 0; i < tour.size() - 1; ++i) {
            Edge<T>* edge = findEdge(tour[i], tour[i + 1]);
            if(edge) {
                double w = edge->getWeight();
                length += w;
            }
        }
        Edge<T>* edge = findEdge(tour.back(), tour.front());
        if(edge) {
        double w = edge->getWeight();
            length += w;
        }
        return length;
    }

    Edge<T>* findEdge(const T& source, const T& dest) const {
        Vertex<T>* srcVertex = findVertex(source);
        Vertex<T>* destVertex = findVertex(dest);
        if (srcVertex == nullptr || destVertex == nullptr) {
            return nullptr;
        }
        for (auto edge : srcVertex->getAdj()) {
            if (edge->getDest() == destVertex) {
                return edge;
            }
        }
        return nullptr;
    }

    int getNumVertex() const;
    std::vector<Vertex<T> *> getVertexSet() const;
    std:: vector<T> dfs() const;
    std:: vector<T> dfs(const T & source) const;
    void dfsVisit(Vertex<T> *v,  std::vector<T> & res) const;

protected:
    std::vector<Vertex<T> *> vertexSet;
    double ** distMatrix = nullptr;
    int **pathMatrix = nullptr;

    int findVertexIdx(const T &in) const;
};



void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

template <class T>
Vertex<T>::Vertex(T in): info(in) {}

template <class T>
Edge<T> * Vertex<T>::addEdge(Vertex<T> *d, double w) {
    auto newEdge = new Edge<T>(this, d, w);
    adj.push_back(newEdge);
    d->incoming.push_back(newEdge);
    return newEdge;
}

template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    return this->dist < vertex.dist;
}

template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
std::vector<Edge<T>*> Vertex<T>::getAdj() const {
    return this->adj;
}

template <class T>
bool Vertex<T>::isVisited() const {
    return this->visited;
}

template <class T>
void Vertex<T>::setVisited(bool visited) {
    this->visited = visited;
}

template <class T>
Edge<T>::Edge(Vertex<T> *orig, Vertex<T> *dest, double w): orig(orig), dest(dest), weight(w) {}

template <class T>
Vertex<T> * Edge<T>::getDest() const {
    return this->dest;
}

template <class T>
double Edge<T>::getWeight() const {
    return this->weight;
}

template <class T>
Vertex<T> * Edge<T>::getOrig() const {
    return this->orig;
}

template <class T>
int Graph<T>::getNumVertex() const {
    return vertexSet.size();
}

template <class T>
std::vector<Vertex<T> *> Graph<T>::getVertexSet() const {
    return vertexSet;
}

template <class T>
Vertex<T> * Graph<T>::findVertex(const T &in) const {
    for (auto v : vertexSet)
        if (v->getInfo() == in)
            return v;
    return nullptr;
}

template <class T>
int Graph<T>::findVertexIdx(const T &in) const {
    for (unsigned i = 0; i < vertexSet.size(); i++)
        if (vertexSet[i]->getInfo() == in)
            return i;
    return -1;
}

template <class T>
bool Graph<T>::addVertex(const T &in) {
    if (findVertex(in) != nullptr)
        return false;
    vertexSet.push_back(new Vertex<T>(in));
    return true;
}

template <class T>
Vertex<T>* Graph<T>::addVertexNew(const T &in) {
    if (findVertex(in) != nullptr) {
        return nullptr;
    }
    Vertex<T>* newV = new Vertex<T>(in);
    vertexSet.push_back(newV);

    return newV;
}

template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, w);
    return true;
}
template <class T>
 Edge<T> * Graph<T>::addEdgeNew(Vertex<T> *source, Vertex<T> *dest, double w) {
    if (source == nullptr || dest == nullptr)
        return nullptr;
    source->addEdge(dest, w);
    dest->addEdge(source,w);
    return new Edge<T>(source, dest, w);
}

template <class T>
std::vector<T> Graph<T>::dfs() const {
    std::vector<T> res;
    for (auto v : vertexSet)
        v->setVisited(false);
    for (auto v : vertexSet)
        if (!v->isVisited())
            dfsVisit(v, res);
    return res;
}

template <class T>
std::vector<T> Graph<T>::dfs(const T & source) const {
    std::vector<int> res;
    auto s = findVertex(source);
    if (s == nullptr) {
        return res;
    }
    for (auto v : vertexSet) {
        v->setVisited(false);
    }
    dfsVisit(s, res);

    return res;
}

template <class T>
void Graph<T>::dfsVisit(Vertex<T> *v, std::vector<T> & res) const {
    v->setVisited(true);
    res.push_back(v->getInfo());
    for (auto & e : v->getAdj()) {
        auto w = e->getDest();
        if (!w->isVisited()) {
            dfsVisit(w, res);
        }
    }
}

inline void deleteMatrix(int **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

inline void deleteMatrix(double **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

template <class T>
Graph<T>::~Graph() {
    deleteMatrix(distMatrix, vertexSet.size());
    deleteMatrix(pathMatrix, vertexSet.size());
}
#endif //PROJ1_GRAPH_H