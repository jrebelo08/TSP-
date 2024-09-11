#include "App.h"


#include <iostream>
#include <set>
#include <chrono>
#include "reader.h"

/**
 * Displays the menu for the backtracking algorithm and allows the user to choose the size of the graph.
 *
 * @remarks This function interacts with the user to obtain input for graph size selection.
 */
void display4_1menu();
/**
 * Displays the menu for the triangular approximation heuristic and allows the user to choose the size of the graph.
 *
 * @remarks This function interacts with the user to obtain input for graph size selection.
 */
void display4_2menu();

/**
 * Displays the menu for the backtracking algorithm with small graph options and applies the TSP backtracking algorithm.
 *
 * @param readAndParseFunc A function object that reads and parses the graph data.
 * @remarks This function interacts with the user to obtain input for graph selection and then applies the TSP backtracking algorithm.
 */
void display4_1menuSmallGraph();
/**
 * Displays the menu for the backtracking algorithm with the stadium graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and parses the stadium graph data before invoking the main menu for the backtracking algorithm.
 */
void display4_1menuSmallGraphStadium();
/**
 * Displays the menu for the backtracking algorithm with the shipping graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and parses the shipping graph data before invoking the main menu for the backtracking algorithm.
 */
void display4_1menuSmallGraphShipping();
/**
 * Displays the menu for the backtracking algorithm with the tourism graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and parses the tourism graph data before invoking the main menu for the backtracking algorithm.
 */
void display4_1menuSmallGraphTourism();
/**
 * Displays the menu for the backtracking algorithm with medium graph options and applies the TSP backtracking algorithm.
 *
 * @remarks This function interacts with the user to obtain input for selecting the size of the graph and then applies the TSP algorithm.
 */
void display4_1menuMediumGraph();
/**
 * Displays the menu for the backtracking algorithm with a specific medium graph size option and applies the TSP backtracking algorithm.
 *
 * @param filename The filename of the medium-sized graph data to be processed.
 * @remarks This function reads and processes the specified medium-sized graph data before invoking the TSP algorithm.
 * @timecomplexity O(n!).
 */
void display4_1menuMedium(const std::string &filename);
/**
 * Displays the menu for the triangular approximation heuristic with small graph options and applies the TSP Triangular approach algorithm.
 *
 * @remarks This function interacts with the user to obtain input for selecting the size of the graph and then applies the TSP algorithm.
 */
void display4_2menuSmallGraph();
/**
 * Displays the menu for the triangular approximation heuristic with medium graph options and applies the TSP algorithm.
 *
 * This function interacts with the user to obtain input for selecting the size of the graph and then applies the TSP algorithm.
 */
void display4_2menuMediumGraph();
/**
 * Displays the menu for the triangular approximation heuristic with large graph options and applies the TSP Triangular approach algorithm after fully connecting the graph.
 *
 * This function interacts with the user to obtain input for selecting the size of the large graph and the starting node, then applies the TSP algorithm.
 */
void display4_2menuLargeGraph();
/**
 * Displays the menu for the triangular approximation heuristic with the stadium graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and processes the stadium graph data before applying the TSP algorithm.
 */
void display4_2menuSmallGraphStadium();
/**
 * Displays the menu for the triangular approximation heuristic with the shipping graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and processes the shipping graph data before applying the TSP algorithm.
 */
void display4_2menuSmallGraphShipping();
/**
 * Displays the menu for the triangular approximation heuristic with the tourism graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and processes the tourism graph data before applying the TSP algorithm.
 */
void display4_2menuSmallGraphTourism();

/**
 * Displays the menu for the triangular approximation heuristic with a specific medium-sized graph option and applies the TSP Triangular approach algorithm.
 *
 * @param filename The filename of the medium-sized graph data.
 *
 * This function reads and processes the specified medium-sized graph data before applying the TSP algorithm.
 * @timecomplexity O(V^2(logV))
 */
void display4_2menuMedium(const std::string &filename);
/**
 * Displays the menu for the triangular approximation heuristic with the first large graph option and applies the TSP Triangular approach algorithm after fully connecting the graph.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(V^2(logV))
 * This function reads and parses the first large graph data, computes the TSP tour, and prints the results.
 */
void display4_2menuLarge1(int nodeID);
/**
 * Displays the menu for the triangular approximation heuristic with the second large graph option and applies the TSP algorithm after fully connecting the graph.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(V^2(logV))
 * This function reads and parses the second large graph data, computes the TSP tour, and prints the results.
 */
void display4_2menuLarge2(int nodeID);
/**
 * Displays the menu for the triangular approximation heuristic with the third large graph option and applies the TSP algorithm after fully connecting the graph.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(V^2(logV))
 * This function reads and parses the third large graph data, computes the TSP tour, and prints the results.
 */
void display4_2menuLarge3(int nodeID);
/**
 * Displays a menu for selecting other heuristics and algorithms to apply to the TSP.
 */
void display_OHmenu();
/**
 * Displays a menu for selecting small graph instances and applying the Nearest Neighbor algorithm.
 */
void display_NNmenu();
/**
 * Displays a menu for selecting small graph instances and applying the Nearest Neighbor algorithm.
 */
void display_NNmenuSmallGraph();
/**
 * Displays a menu for selecting medium graph instances and applying the Nearest Neighbor heuristic algorithm.
 * The user can choose the number of edges in the graph and apply the Nearest Neighbor algorithm accordingly.
 */
void display_NNmenuMediumGraph();
/**
 * Applies the Nearest Neighbor algorithm to the stadium graph instance.
 * Time Complexity: O(n^2), where 'n' is the number of vertices in the graph.
 */
void getValue_NNmenuSmallGraph(const std::function<Graph<int>(Reader&)>& readAndParseFunc);
/**
 * Applies the Nearest Neighbor algorithm to the shipping graph instance.
 */
void getValue_NNmenuSmallGraphStadium();
/**
 * Applies the Nearest Neighbor algorithm to the tourism graph instance.
 */
void getValue_NNmenuSmallGraphShipping();
/**
 * Applies the Nearest Neighbor algorithm to the tourism graph instance.
 */
void getValue_NNmenuSmallGraphTourism();
/**
 * Applies the Nearest Neighbor algorithm to a medium graph instance.
 *
 * @param filename The filename of the graph data to be read and parsed.
 *                 This should include the path to the file.
 * Time Complexity: O(n^2), where 'n' is the number of vertices in the graph.
 */
void getValue_NNmenuMediumGraph(const std::string &filename);

/**
 * Displays a menu for selecting the Lin-Kernighan Algorithm heuristic and the size of the graph.
 * Users can choose between small and medium graphs to apply the Lin-Kernighan Algorithm.
 */
void display_LINmenu();
/**
 * Displays a menu for selecting small graph instances and applying the Lin-Kernighan Algorithm heuristic.
 * Users can choose between different types of small graphs, such as stadium, shipping, and tourism graphs.
 */
void display_LINmenuSmallGraph();
/**
 * Displays a menu for selecting medium graph instances and applying the LinKernighan heuristic algorithm.
 * The user can choose the number of edges in the graph and apply the LinKernighan heuristic accordingly.
 */
void display_LINmenuMediumGraph();
/**
 * Applies the Lin-Kernighan algorithm to a small graph instance using the provided reader function.
 *
 * @param readAndParseFunc A function that reads and parses the graph data.
 *                         It should return a Graph<int> object.
 * @remarks The time complexity of the Lin-Kernighan algorithm O(n^3), where n is the number of vertices in the graph.
 * The algorithm has a better approximation than the NearestNeighbour heuristics but scales worse
 * @timecomplexity O(n^2log(n))
 */
void getValue_LINmenuSmallGraph(const std::function<Graph<int>(Reader&)>& readAndParseFunc);
/**
 * Applies the Lin-Kernighan Algorithm heuristic to a stadium graph instance.
 *
 * @param readAndParseStadium A lambda function that reads and parses the stadium graph data.
 *                            It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_LINmenuSmallGraphStadium();
/**
 * Applies the Lin-Kernighan Algorithm heuristic to a shipping graph instance.
 *
 * @param readAndParseShipping A lambda function that reads and parses the shipping graph data.
 *                             It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_LINmenuSmallGraphShipping();
/**
 * Applies the Lin-Kernighan Algorithm heuristic to a tourism graph instance.
 *
 * @param readAndParseTourism A lambda function that reads and parses the tourism graph data.
 *                            It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_LINmenuSmallGraphTourism();

/**
 * Applies the Lin-Kernighan Algorithm heuristic to a medium graph instance.
 *
 * @param filename The filename of the graph data to be read and parsed.
 *                 This should include the path to the file.
 * @remarks The time complexity of the Lin-Kernighan algorithm O(n^3), where n is the number of vertices in the graph.
 * The algorithm has a better approximation than the NearestNeighbour heuristics but scales worse
 *  * @timecomplexity O(n^2log(n))
 */

void getValue_LINmenuMediumGraph(const std::string &filename);

/**
 * @brief Displays the main menu for selecting the size of the graph for the Held-Karp algorithm.
 *
 * This function provides a menu interface for the user to select between a small graph, medium graph, or exit to the main menu.
 */
void display_HeldKarp_menu();

/**
 * @brief Displays the menu for selecting a specific small graph for the Held-Karp algorithm.
 *
 * This function provides a menu interface for the user to select between different predefined small graphs
 * or to exit back to the main Held-Karp menu.
 */
void display_HeldKarp_menuSmallGraph();
/**
 * Displays a menu for selecting medium graph instances and applying the Held-Karp Algorithm heuristic.
 * Users can choose the number of edges in the graph to apply the Held-Karp Algorithm.
 */
void display_HeldKarp_menuMediumGraph();
/**
 * Applies the Held-Karp algorithm to a small graph instance using the provided reader function.
 *
 * @param readAndParseFunc A function that reads and parses the graph data.
 *                         It should return a Graph<int> object.
 * @timecomplexity The time complexity is O(n^2 * 2^n), where 'n' is the number of vertices in the graph.
 *           The space complexity is O(n * 2^n).
 */
void getValue_HeldKarp_menuSmallGraph(const std::function<Graph<int>(Reader&)>& readAndParseFunc);
/**
 * Applies the Held-Karp algorithm to a stadium graph instance.
 *
 * @param readAndParseStadium A lambda function that reads and parses the stadium graph data.
 *                            It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_HeldKarp_menuSmallGraphStadium();
/**
 * Applies the Held-Karp algorithm to a shipping graph instance.
 *
 * @param readAndParseShipping A lambda function that reads and parses the shipping graph data.
 *                             It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_HeldKarp_menuSmallGraphShipping();
/**
 * Applies the Held-Karp algorithm to a tourism graph instance.
 *
 * @param readAndParseTourism A lambda function that reads and parses the tourism graph data.
 *                            It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_HeldKarp_menuSmallGraphTourism();
/**
 * Applies the Held-Karp algorithm to a medium graph instance.
 *
 * @param filename The filename of the graph data to be read and parsed.
 *                 This should include the path to the file.
 * @timecomplexity The time complexity is O(n^2 * 2^n), where 'n' is the number of vertices in the graph.
 *           The space complexity is O(n * 2^n).
 */
void getValue_HeldKarp_menuMediumGraph(const std::string &filename);

/**
 * Displays a menu for selecting graph instances and applying the K-means Clustering Nearest Neighbor Algorithm heuristic.
 * Users can choose between small and medium graph sizes or return to the main menu.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void display_CLUSTERmenu(int clusterOption);
/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small graph instance.
 *
 * @param readAndParseFunc A function to read and parse the graph data.
 * @param clusterOption The number of clusters for K-means clustering.
 * @timecomplexity O(n^2)
 */
void display_CLUSTERmenuSmallGraph(int clusterOption);
/**
 * Displays the menu for selecting a medium graph instance for applying the K-means Clustering Nearest Neighbor Algorithm heuristic.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void display_CLUSTERmenuMediumGraph(int clusterOption);
/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small graph instance.
 *
 * @param readAndParseFunc A function to read and parse the graph data.
 * @param clusterOption The number of clusters for K-means clustering.
 * @timecomplexity O(n^2)
 */
void getValue_CLUSTERmenuSmallGraph(const std::function<Graph<int>(Reader&)>& readAndParseFunc, int clusterOption);
/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small stadium graph instance.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void getValue_CLUSTERmenuSmallGraphStadium(int clusterOption);
/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small shipping graph instance.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void getValue_CLUSTERmenuSmallGraphShipping(int clusterOption);
/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small tourism graph instance.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void getValue_CLUSTERmenuSmallGraphTourism(int clusterOption);
/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a medium graph instance.
 *
 * @param filename The filename of the graph data.
 * @param clusterOption The number of clusters for K-means clustering.
 * @timecomplexity O(n^2)
 */
void getValue_CLUSTERmenuMediumGraph(const std::string &filename, int clusterOption);

/**
 * Displays the menu for selecting a real-world TSP instance to solve using various algorithms.
 *
 * This menu allows the user to choose between different graph instances and starting nodes.
 *
 * - Graph1('Small'): Small-sized graph instance.
 * - Graph2(Medium): Medium-sized graph instance.
 * - Graph3(Large): Large-sized graph instance.
 *
 * @note This function interacts with the user to obtain input for starting nodes and algorithm selection.
 */
void display_NNmenuLargeGraph();

/**
 * Solves the real-world TSP problem for a large-sized graph instance starting from the specified node.
 *
 * This function reads and parses the large-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 *
 * @remarks The time complexity of the nearest neighbor algorithm used in this function is O(n^2), where n is the number of vertices in the graph.
 */
void getValue_NNsmallGraph(int nodeID);
/**
 * Solves the real-world TSP problem for a large-sized graph instance starting from the specified node.
 *
 * This function reads and parses the large-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 *
 * @remarks The time complexity of the nearest neighbor algorithm used in this function is O(n^2), where n is the number of vertices in the graph.
 */
void getValue_NNmediumGraph(int nodeID);
/**
 * Solves the real-world TSP problem for a large-sized graph instance starting from the specified node.
 *
 * This function reads and parses the large-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 *
 * @remarks The time complexity of the nearest neighbor algorithm used in this function is O(n^2), where n is the number of vertices in the graph.
 */
void getValue_NNlargeGraph(int nodeID);

/**
 * Displays the menu for selecting a real-world TSP instance to solve using various algorithms.
 *
 * This menu allows the user to choose between different graph instances and starting nodes.
 *
 * - Graph1('Small'): Small-sized graph instance.
 * - Graph2(Medium): Medium-sized graph instance.
 * - Graph3(Large): Large-sized graph instance.
 *
 * @note This function interacts with the user to obtain input for starting nodes and algorithm selection.
 */
void display_RWmenu();

/**
 * Solves the real-world TSP problem for a small-sized graph instance starting from the specified node.
 *
 * This function reads and parses the small-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(n^2)
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 */
void getValue_RWsmallGraph(int nodeID);
/**
 * Solves the real-world TSP problem for a medium-sized graph instance starting from the specified node.
 *
 * This function reads and parses the medium-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(n^2)
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 */
void getValue_RWmediumGraph(int nodeID);
/**
 * Solves the real-world TSP problem for a large-sized graph instance starting from the specified node.
 *
 * This function reads and parses the large-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 *
 * @remarks The time complexity of the nearest neighbor algorithm used in this function is O(n^2), where n is the number of vertices in the graph.
 */
void getValue_RWlargeGraph(int nodeID);

/**
 * @brief Performs a pre-order traversal of a tree represented by an MST.
 * @param root The root vertex of the tree.
 * @param mst The minimum spanning tree (MST) represented by a vector of edges.
 * @return A vector containing the vertices visited in pre-order traversal.
 * @timecomplexity The time complexity of this function depends on the size of the MST and the efficiency of the pre-order traversal algorithm.
 * Typically, it has a time complexity of O(V + E), where V is the number of vertices and E is the number of edges in the MST.
 */
std::vector<int> preOrderTraversal(Vertex<int>* root, const std::vector<Edge<int>*>& mst);
/**
 * @brief Solves the Traveling Salesman Problem (TSP) using a backtracking algorithm.
 * @param currentNode The current node being visited.
 * @param path The current path being explored.
 * @param currentCost The current cost of the path.
 * @param level The level of recursion.
 * @param graph The graph representing the TSP problem.
 * @param bestPath The best path found so far.
 * @param bestCost The cost of the best path found so far.
 * @details This function recursively explores possible paths using a backtracking algorithm and updates the best path and cost found.
 * @timecomplexity O(n!).
 */
void tsp(int currentNode, std::vector<int>& path, double currentCost, int level, Graph<int>& graph, std::vector<int>& bestPath, double& bestCost);
/**
 * @brief Solves the Traveling Salesman Problem (TSP) using the Held-Karp algorithm.
 * @param graph The graph representing the TSP problem.
 * @param bestPath The best path found by the algorithm.
 * @param bestCost The cost of the best path found.
 * @details This function applies the Held-Karp algorithm to find the optimal solution for the TSP.
 * @timecomplexity The time complexity of this function is O(2^n * n^2), where n is the number of vertices in the graph.
 * @remarks Pros: Guarantees the optimal solution for the TSP. Suitable for small to medium-sized graphs.
 * Cons: High time complexity, impractical for large graphs due to exponential growth.
 */
void heldKarp(const Graph<int>& graph, std::vector<int>& bestPath, double& bestCost);


/**
 * Displays the main menu and handles user interaction to choose between different TSP algorithms and heuristics.
 *
 * @return 0 indicating successful execution.
 */
int mainMenu(){
    cout << "Loading...";

    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Main Menu       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option that suits your needs:\n";
        cout << "1. Backtracking Algorithm\n";
        cout << "2. Triangular Approximation Heuristic \n";
        cout <<"3. Other Heuristics / ALgorithms \n";
        cout << "4 Tsp in the Real World\n";
        cout << "e. Exit\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                display4_1menu();
                break;
            case '2':
                display4_2menu();
                break;
            case '3':
                display_OHmenu();
                break;
            case '4':
                display_RWmenu();
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
    return 0;
}

/**
 * Displays the menu for the backtracking algorithm and allows the user to choose the size of the graph.
 *
 * @remarks This function interacts with the user to obtain input for graph size selection.
 */
void display4_1menu() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Backtracing Menu       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option of the size of the graph you want:\n";
        cout << "1. Small Graph\n";
        cout << "2. Medium Graph \n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                display4_1menuSmallGraph();
                break;
            case '2':
                display4_1menuMediumGraph();
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Displays the menu for the triangular approximation heuristic and allows the user to choose the size of the graph.
 *
 * @remarks This function interacts with the user to obtain input for graph size selection.
 */
void display4_2menu() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Triangular Approximation Heuristic       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option of the size of the graph you want:\n";
        cout << "1. Small Graph\n";
        cout << "2. Medium Graph\n";
        cout << "3  Large Graph\n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                display4_2menuSmallGraph();
                break;
            case '2':
                display4_2menuMediumGraph();
                break;
            case '3':
                display4_2menuLargeGraph();
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Displays the menu for the backtracking algorithm with small graph options and applies the TSP backtracking algorithm.
 *
 * @param readAndParseFunc A function object that reads and parses the graph data.
 * @remarks This function interacts with the user to obtain input for graph selection and then applies the TSP backtracking algorithm.
 */
void display4_1menuSmallGraph(){
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Backtracing SmallGraph Menu       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the graph you want:\n";
        cout << "1. Stadium Graph\n";
        cout << "2. Shipping Graph \n";
        cout <<"3. Tourism Graph \n";
        cout << "e. Back to the backtracing Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                display4_1menuSmallGraphStadium();
                break;
            case '2':
                display4_1menuSmallGraphShipping();
                break;
            case '3':
                display4_1menuSmallGraphTourism();
            case 'e':
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Displays the menu for the backtracking algorithm with small graph options and applies the TSP backtracking algorithm.
 *
 * @param readAndParseFunc A function object that reads and parses the graph data.
 * @remarks This function interacts with the user to obtain input for graph selection and then applies the TSP algorithm.
 * @timecomplexity O(n!)
 */
void display4_1menuSmallGraph(const std::function<Graph<int>(Reader&)>& readAndParseFunc) {
    Reader reader;
    Graph<int> graph;
    graph = readAndParseFunc(reader);
    std::vector<int> bestPath;
    double bestCost = std::numeric_limits<double>::infinity();
    std::vector<int> path(1, 0);
    graph.findVertex(0)->setVisited(true);

    auto startTime = std::chrono::high_resolution_clock::now();
    tsp(0, path, 0.0, 1, graph, bestPath, bestCost);
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> execTime = endTime - startTime;

    std::cout << "Execution Time: " << execTime.count() << " seconds\n";
    std::cout << "Best Path Cost: " << bestCost << "\n";
    std::cout << "Best Path Size: " << bestPath.size() << "\n";
    std::cout << "Best Path: ";
    for (size_t i = 0; i < bestPath.size(); ++i) {
        std::cout << bestPath[i];
        if (i < bestPath.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << "\n";
}


/**
 * Displays the menu for the backtracking algorithm with the stadium graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and parses the stadium graph data before invoking the main menu for the backtracking algorithm.
 */
void display4_1menuSmallGraphStadium() {
    auto readAndParseStadium = [](Reader& reader) { return reader.readAndParseStadium(); };
    display4_1menuSmallGraph(readAndParseStadium);
}

/**
 * Displays the menu for the backtracking algorithm with the shipping graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and parses the shipping graph data before invoking the main menu for the backtracking algorithm.
 */
void display4_1menuSmallGraphShipping() {
    auto readAndParseShipping = [](Reader& reader) { return reader.readAndParseShipping(); };
    display4_1menuSmallGraph(readAndParseShipping);
}

/**
 * Displays the menu for the backtracking algorithm with the tourism graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and parses the tourism graph data before invoking the main menu for the backtracking algorithm.
 */
void display4_1menuSmallGraphTourism() {
    auto readAndParseTourism = [](Reader& reader) { return reader.readAndParseTourism(); };
    display4_1menuSmallGraph(readAndParseTourism);
}

/**
 * Displays the menu for the backtracking algorithm with medium graph options and applies the TSP backtracking algorithm.
 *
 * @remarks This function interacts with the user to obtain input for selecting the size of the graph and then applies the TSP algorithm.
 */
void display4_1menuMediumGraph() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Triangular Approximation Medium Graph Menu       \n";
        cout << "-----------------------------\n";
        cout << "How many edges does the graph have:\n";
        cout << "1. 25\n";
        cout << "2. 50\n";
        cout << "3. 75\n";
        cout << "4. 100\n";
        cout << "5. 200\n";
        cout << "6. 300\n";
        cout << "7. 400\n";
        cout << "8. 500\n";
        cout << "9. 600\n";
        cout << "10. 700\n";
        cout << "11. 800\n";
        cout << "12. 900\n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice == "e") {
            cout << "Exiting menu system...\n";
            exitMenu = true;
        } else {
            int choiceNum = stoi(choice);
            switch (choiceNum) {
                case 1:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_25.csv");
                    break;
                case 2:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_50.csv");
                    break;
                case 3:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_75.csv");
                    break;
                case 4:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_100.csv");
                    break;
                case 5:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_200.csv");
                    break;
                case 6:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_300.csv");
                    break;
                case 7:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_400.csv");
                    break;
                case 8:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_500.csv");
                    break;
                case 9:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_600.csv");
                    break;
                case 10:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_700.csv");
                    break;
                case 11:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_800.csv");
                    break;
                case 12:
                    display4_1menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_900.csv");
                    break;
                default:
                    cout << "Invalid input. Please choose a valid option.\n";
            }
        }
    }
}

/**
 * Displays the menu for the backtracking algorithm with a specific medium graph size option and applies the TSP backtracking algorithm.
 *
 * @param filename The filename of the medium-sized graph data to be processed.
 * @remarks This function reads and processes the specified medium-sized graph data before invoking the TSP algorithm.
 * @timecomplexity O(n!).
 */
void display4_1menuMedium(const std::string &filename){
    Reader reader;
    Graph<int> graph;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
    graph = reader.readAndParse4_2Extra_Fully_Connected_Graphs(filename,vertexMap,edgeMap);
    std::vector<int> bestPath;
    double bestCost = std::numeric_limits<double>::infinity();
    std::vector<int> path(1, 0);
    graph.findVertex(0)->setVisited(true);

    auto startTime = std::chrono::high_resolution_clock::now();
    tsp(0, path, 0.0, 1, graph, bestPath, bestCost);
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> execTime = endTime - startTime;

    std::cout << "Execution Time: " << execTime.count() << " seconds\n";
    std::cout << "Best Path Cost: " << bestCost << "\n";
    std::cout << "Best Path Size: " << bestPath.size() << "\n";
    std::cout << "Best Path: ";
    for (int node : bestPath) {
        std::cout << node << (node == 0 ? "\n" : " -> ");
    }

}

/**
 * Displays the menu for the triangular approximation heuristic with small graph options and applies the TSP Triangular approach algorithm.
 *
 * @remarks This function interacts with the user to obtain input for selecting the size of the graph and then applies the TSP algorithm.
 */
void display4_2menuSmallGraph(){
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to  Triangular Approximation Heuristic SmallGraph Menu       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the graph you want:\n";
        cout << "1. Stadium Graph\n";
        cout << "2. Shipping Graph \n";
        cout <<"3. Tourism Graph \n";
        cout << "e. Back to the triangular approximation heuristic  Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                display4_2menuSmallGraphStadium();
                break;
            case '2':
                display4_2menuSmallGraphShipping();
                break;
            case '3':
                display4_2menuSmallGraphTourism();
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Displays the menu for the triangular approximation heuristic with a specific small graph size option and applies the TSP algorithm.
 *
 * @param readAndParseFunc The function used to read and parse the small-sized graph data.
 * @remarks This function reads and processes the specified small-sized graph data before applying the TSP algorithm.
 * @timecomplexity O(V^2(logV))
 */
void display4_2menuSmall(const std::function<Graph<int>(Reader&)>& readAndParseFunc) {

    Reader reader;
    Graph<int> graph = readAndParseFunc(reader);
    Vertex<int>* startVertexPtr = graph.findVertex(0);

    auto start = std::chrono::high_resolution_clock::now();

    vector<Edge<int>*> mst = graph.primMST(startVertexPtr->getInfo());

    vector<int> preorderList = preOrderTraversal(startVertexPtr, mst);

    vector<int> tspTour;
    set<int> visited;
    for (int vertex : preorderList) {
        if (visited.insert(vertex).second) {
            tspTour.push_back(vertex);
        }
    }
    tspTour.push_back(tspTour.front());

    double totalDistance = 0;
    int previous = tspTour.front();
    for (size_t i = 1; i < tspTour.size(); i++) {
        Edge<int> *edge = graph.findEdge(previous, tspTour[i]);
        if (edge)
            totalDistance += edge->getWeight();
        previous = tspTour[i];
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> execTime = end - start;

    cout << "TSP Tour: ";
    for (size_t i = 0; i < tspTour.size(); ++i) {
        cout << tspTour[i] << (i < tspTour.size() - 1 ? " -> " : "");
    }
    cout << endl;
    cout << "Tour Size: " << tspTour.size() << "\n";
    cout << "Total Approximation Distance: " << totalDistance << "\n";
    cout << "Execution Time: " << execTime.count() << " seconds\n";
}

/**
 * Displays the menu for the triangular approximation heuristic with the stadium graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and processes the stadium graph data before applying the TSP algorithm.
 */
void display4_2menuSmallGraphStadium() {
    auto readAndParseStadium = [](Reader& reader) { return reader.readAndParseStadium(); };
    display4_2menuSmall(readAndParseStadium);
}

/**
 * Displays the menu for the triangular approximation heuristic with the shipping graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and processes the shipping graph data before applying the TSP algorithm.
 */
void display4_2menuSmallGraphShipping() {
    auto readAndParseShipping = [](Reader& reader) { return reader.readAndParseShipping(); };
    display4_2menuSmall(readAndParseShipping);
    cout << "Graph is not fully connected value isn´t a good approximation";
}

/**
 * Displays the menu for the triangular approximation heuristic with the tourism graph option and applies the TSP algorithm.
 *
 * @remarks This function reads and processes the tourism graph data before applying the TSP algorithm.
 */
void display4_2menuSmallGraphTourism() {
    auto readAndParseTourism = [](Reader& reader) { return reader.readAndParseTourism(); };
    display4_2menuSmall(readAndParseTourism);
}

/**
 * Displays the menu for the triangular approximation heuristic with medium graph options and applies the TSP algorithm.
 *
 * This function interacts with the user to obtain input for selecting the size of the graph and then applies the TSP algorithm.
 */
void display4_2menuMediumGraph() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Triangular Approximation Medium Graph Menu       \n";
        cout << "-----------------------------\n";
        cout << "How many edges does the graph have:\n";
        cout << "1. 25\n";
        cout << "2. 50\n";
        cout << "3. 75\n";
        cout << "4. 100\n";
        cout << "5. 200\n";
        cout << "6. 300\n";
        cout << "7. 400\n";
        cout << "8. 500\n";
        cout << "9. 600\n";
        cout << "10. 700\n";
        cout << "11. 800\n";
        cout << "12. 900\n";
        cout << "e. Back to the Triangular Approximation Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice == "e") {
            cout << "Exiting menu system...\n";
            exitMenu = true;
        } else {
            int choiceNum = stoi(choice);
            switch (choiceNum) {
                case 1:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_25.csv");
                    break;
                case 2:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_50.csv");
                    break;
                case 3:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_75.csv");
                    break;
                case 4:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_100.csv");
                    break;
                case 5:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_200.csv");
                    break;
                case 6:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_300.csv");
                    break;
                case 7:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_400.csv");
                    break;
                case 8:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_500.csv");
                    break;
                case 9:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_600.csv");
                    break;
                case 10:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_700.csv");
                    break;
                case 11:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_800.csv");
                    break;
                case 12:
                    display4_2menuMedium("../Data/Extra_Fully_Connected_Graphs/edges_900.csv");
                    break;
                default:
                    cout << "Invalid input. Please choose a valid option.\n";
            }
        }
    }
}

/**
 * Displays the menu for the triangular approximation heuristic with a specific medium-sized graph option and applies the TSP Triangular approach algorithm.
 *
 * @param filename The filename of the medium-sized graph data.
 *
 * This function reads and processes the specified medium-sized graph data before applying the TSP algorithm.
 * @timecomplexity O(V^2(logV))
 */
void display4_2menuMedium(const std::string &filename){
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
    Graph<int> graph = reader.readAndParse4_2Extra_Fully_Connected_Graphs(filename,vertexMap,edgeMap);
    Vertex<int>* startVertexPtr = nullptr;
    auto vertexIter = vertexMap.find(0);
    if (vertexIter != vertexMap.end()) {
        startVertexPtr = vertexIter->second;
    }

    auto start = std::chrono::high_resolution_clock::now();

    vector<Edge<int>*> mst = graph.primMST(startVertexPtr->getInfo());


    vector<int> preorderList = preOrderTraversal(startVertexPtr, mst);


    vector<int> tspTour;
    set<int> visited;
    for (int vertex : preorderList) {
        if (visited.insert(vertex).second) {
            tspTour.push_back(vertex);
        }
    }
    tspTour.push_back(tspTour.front());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> execTime = end - start;

    double totalDistance = 0;
    int previous = tspTour.front();
    for (size_t i = 1; i < tspTour.size(); i++) {
        Edge<int> *edge = graph.findEdge(previous, tspTour[i]);
        if (edge)
            totalDistance += edge->getWeight();
        previous = tspTour[i];

    }


    cout << "TSP Tour: ";
    for (size_t i = 0; i < tspTour.size(); ++i) {
        cout << tspTour[i] << (i < tspTour.size() - 1 ? " -> " : "");
    }
    cout << endl;
    cout << "Tour Size: " << tspTour.size() << "\n";
    cout << "Total Approximation Distance: " << totalDistance << "\n";
    cout << "Execution Time: " << execTime.count() << " seconds\n";

}

/**
 * Displays the menu for the triangular approximation heuristic with large graph options and applies the TSP Triangular approach algorithm after fully connecting the graph.
 *
 * This function interacts with the user to obtain input for selecting the size of the large graph and the starting node, then applies the TSP algorithm.
 */
void display4_2menuLargeGraph() {
    string choice_str;
    int choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Triangular Approximation for Large Graphs       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option of the size of the graph you want:\n";
        cout << "1. Graph1('Small')\n";
        cout << "2. Graph2(Medium) \n";
        cout << "3. Graph3(Large)  \n";
        cout << "e. Back to the Triangular Approximation Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice_str;

        if (choice_str == "e" || choice_str == "E") {
            cout << "Exiting menu system...\n";
            exitMenu = true;
            continue;
        }

        try {
            choice = stoi(choice_str);
        } catch (const invalid_argument&) {
            choice = 0;
        }

        int nodeID = 0;
        string choiceNode;

        switch (choice) {
            case 1:
                while (true) {
                    cout << "Input the starting node for the algorithm: ";
                    cin >> choiceNode;
                    try {
                        nodeID = stoi(choiceNode);
                        display4_2menuLarge1(nodeID);
                        break;
                    } catch (const invalid_argument&) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range&) {
                        cout << "Input out of range. Please enter a valid integer within the range of int.\n";
                    }
                }
                break;
            case 2:
                while (true) {
                    cout << "Input the starting node for the algorithm: ";
                    cin >> choiceNode;
                    try {
                        nodeID = stoi(choiceNode);
                        display4_2menuLarge2(nodeID);
                        break;
                    } catch (const invalid_argument&) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range&) {
                        cout << "Input out of range. Please enter a valid integer within the range of int.\n";
                    }
                }
                break;
            case 3:
                while (true) {
                    cout << "Input the starting node for the algorithm: ";
                    cin >> choiceNode;
                    try {
                        nodeID = stoi(choiceNode);
                        display4_2menuLarge3(nodeID);
                        break;
                    } catch (const invalid_argument&) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range&) {
                        cout << "Input out of range. Please enter a valid integer within the range of int.\n";
                    }
                }
                break;

            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}




/**
 * Displays the menu for the triangular approximation heuristic with the first large graph option and applies the TSP Triangular approach algorithm after fully connecting the graph.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(V^2(logV))
 * This function reads and parses the first large graph data, computes the TSP tour, and prints the results.
 */
void display4_2menuLarge1(int nodeID) {
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
   Graph<int> graph = reader.readAndParseRealWorld_Graphs4_2(1,vertexMap,edgeMap);
    Vertex<int>* startVertexPtr = nullptr;
    auto vertexIter = vertexMap.find(nodeID);
    if (vertexIter != vertexMap.end()) {
        startVertexPtr = vertexIter->second;
    } else {
        std::cerr << "Node ID " << nodeID << " not found in the graph.\n";
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();

    vector<Edge<int>*> mst = graph.primMSTMaps(startVertexPtr->getInfo(),vertexMap,edgeMap);

    vector<int> preorderList = preOrderTraversal(startVertexPtr, mst);

    vector<int> tspTour;
    set<int> visited;
    for (int vertex : preorderList) {
        if (visited.insert(vertex).second) {
            tspTour.push_back(vertex);
        }
    }
    tspTour.push_back(tspTour.front());

    double totalDistance = 0;
    int previous = tspTour.front();
    for (size_t i = 1; i < tspTour.size(); i++) {
        Edge<int> *edge = graph.findEdge(previous, tspTour[i]);
        if (edge)
            totalDistance += edge->getWeight();
        previous = tspTour[i];
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    totalDistance = graph.computeTourLength(tspTour, graph);
    std::cout << "TSP Tour: ";
    for (size_t i = 0; i < tspTour.size(); ++i) {
        std::cout << tspTour[i] << (i < tspTour.size() - 1 ? " -> " : "");
    }
    std::cout << std::endl;
    cout << "Tour Size: " << tspTour.size() << "\n";
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
}

/**
 * Displays the menu for the triangular approximation heuristic with the second large graph option and applies the TSP algorithm after fully connecting the graph.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(V^2(logV))
 * This function reads and parses the second large graph data, computes the TSP tour, and prints the results.
 */
void display4_2menuLarge2(int nodeID) {
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
    Graph<int> graph = reader.readAndParseRealWorld_Graphs4_2(2,vertexMap,edgeMap);
    Vertex<int>* startVertexPtr = nullptr;
    auto vertexIter = vertexMap.find(nodeID);
    if (vertexIter != vertexMap.end()) {
        startVertexPtr = vertexIter->second;
    } else {
        std::cerr << "Node ID " << nodeID << " not found in the graph.\n";
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();

    vector<Edge<int>*> mst = graph.primMSTMaps(startVertexPtr->getInfo(),vertexMap,edgeMap);

    vector<int> preorderList = preOrderTraversal(startVertexPtr, mst);

    vector<int> tspTour;
    set<int> visited;
    for (int vertex : preorderList) {
        if (visited.insert(vertex).second) {
            tspTour.push_back(vertex);
        }
    }
    tspTour.push_back(tspTour.front());

    double totalDistance = 0;
    int previous = tspTour.front();
    for (size_t i = 1; i < tspTour.size(); i++) {
        Edge<int> *edge = graph.findEdge(previous, tspTour[i]);
        if (edge)
            totalDistance += edge->getWeight();
        previous = tspTour[i];
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    totalDistance = graph.computeTourLength(tspTour, graph);
    std::cout << "TSP Tour: ";
    for (size_t i = 0; i < tspTour.size(); ++i) {
        std::cout << tspTour[i] << (i < tspTour.size() - 1 ? " -> " : "");
    }
    std::cout << std::endl;
    cout << "Tour Size: " << tspTour.size() << "\n";
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
}

/**
 * Displays the menu for the triangular approximation heuristic with the third large graph option and applies the TSP algorithm after fully connecting the graph.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(V^2(logV))
 * This function reads and parses the third large graph data, computes the TSP tour, and prints the results.
 */
void display4_2menuLarge3(int nodeID) {
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
    Graph<int> graph = reader.readAndParseRealWorld_Graphs4_2(3,vertexMap,edgeMap);
    Vertex<int>* startVertexPtr = nullptr;
    auto vertexIter = vertexMap.find(nodeID);
    if (vertexIter != vertexMap.end()) {
        startVertexPtr = vertexIter->second;
    } else {
        std::cerr << "Node ID " << nodeID << " not found in the graph.\n";
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();

    vector<Edge<int>*> mst = graph.primMSTMaps(startVertexPtr->getInfo(),vertexMap,edgeMap);

    vector<int> preorderList = preOrderTraversal(startVertexPtr, mst);

    vector<int> tspTour;
    set<int> visited;
    for (int vertex : preorderList) {
        if (visited.insert(vertex).second) {
            tspTour.push_back(vertex);
        }
    }
    tspTour.push_back(tspTour.front());

    double totalDistance = 0;
    int previous = tspTour.front();
    for (size_t i = 1; i < tspTour.size(); i++) {
        Edge<int> *edge = graph.findEdge(previous, tspTour[i]);
        if (edge)
            totalDistance += edge->getWeight();
        previous = tspTour[i];
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    totalDistance = graph.computeTourLength(tspTour, graph);
    std::cout << "TSP Tour: ";
    for (size_t i = 0; i < tspTour.size(); ++i) {
        std::cout << tspTour[i] << (i < tspTour.size() - 1 ? " -> " : "");
    }
    std::cout << std::endl;
    cout << "Tour Size: " << tspTour.size() << "\n";
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
}

/**
 * @brief Performs a pre-order traversal of a tree represented by an MST.
 * @param root The root vertex of the tree.
 * @param mst The minimum spanning tree (MST) represented by a vector of edges.
 * @return A vector containing the vertices visited in pre-order traversal.
 * @timecomplexity The time complexity of this function depends on the size of the MST and the efficiency of the pre-order traversal algorithm.
 * Typically, it has a time complexity of O(V + E), where V is the number of vertices and E is the number of edges in the MST.
 */
std::vector<int> preOrderTraversal(Vertex<int>* root, const std::vector<Edge<int>*>& mst) {
    std::vector<int> order;
    std::unordered_set<Vertex<int>*> visited;
    std::unordered_map<Vertex<int>*, std::vector<Vertex<int>*>> tree;

    for (auto edge : mst) {
        tree[edge->getOrig()].push_back(edge->getDest());
        tree[edge->getDest()].push_back(edge->getOrig());
    }

    std::function<void(Vertex<int>*)> dfs = [&](Vertex<int>* vertex) {
        if (!vertex || visited.count(vertex)) return;
        visited.insert(vertex);
        order.push_back(vertex->getInfo());

        for (auto next : tree[vertex]) {
            dfs(next);
        }
    };

    dfs(root);
    return order;
}

/**
 * @brief Solves the Traveling Salesman Problem (TSP) using a backtracking algorithm.
 * @param currentNode The current node being visited.
 * @param path The current path being explored.
 * @param currentCost The current cost of the path.
 * @param level The level of recursion.
 * @param graph The graph representing the TSP problem.
 * @param bestPath The best path found so far.
 * @param bestCost The cost of the best path found so far.
 * @details This function recursively explores possible paths using a backtracking algorithm and updates the best path and cost found.
 * @timecomplexity O(n!).
 */
void tsp(int currentNode, std::vector<int>& path, double currentCost, int level, Graph<int>& graph, std::vector<int>& bestPath, double& bestCost) {
    if (level == graph.getNumVertex()) {
        Edge<int>* edgeBack = graph.findEdge(currentNode, 0);
        if (edgeBack && currentCost + edgeBack->getWeight() < bestCost) {
            bestCost = currentCost + edgeBack->getWeight();
            bestPath = path;
            bestPath.push_back(0);
        }
        return;
    }

    if (currentCost >= bestCost)
        return;

    Vertex<int>* currentVertex = graph.findVertex(currentNode);
    for (Edge<int>* edge : currentVertex->getAdj()) {
        auto nextNode = edge->getDest();
        if (!nextNode->isVisited()) {
            nextNode->setVisited(true);
            path.push_back(nextNode->getInfo());
            tsp(nextNode->getInfo(), path, currentCost + edge->getWeight(), level + 1, graph, bestPath, bestCost);
            nextNode->setVisited(false);
            path.pop_back();
        }
    }
}

/**
 * @brief Solves the Traveling Salesman Problem (TSP) using the Held-Karp algorithm.
 * @param graph The graph representing the TSP problem.
 * @param bestPath The best path found by the algorithm.
 * @param bestCost The cost of the best path found.
 * @details This function applies the Held-Karp algorithm to find the optimal solution for the TSP.
 * @timecomplexity The time complexity of this function is O(2^n * n^2), where n is the number of vertices in the graph.
 * @remarks Pros: Guarantees the optimal solution for the TSP. Suitable for small to medium-sized graphs.
 * Cons: High time complexity, impractical for large graphs due to exponential growth.
 */
void heldKarp(const Graph<int>& graph, std::vector<int>& bestPath, double& bestCost) {
    int n = graph.getNumVertex();

    std::vector<std::vector<double>> dp(1 << n, std::vector<double>(n, INF));
    std::vector<std::vector<int>> parent(1 << n, std::vector<int>(n, -1));
    dp[1][0] = 0;

    for (int mask = 1; mask < (1 << n); ++mask) {
        for (int u = 0; u < n; ++u) {
            if (mask & (1 << u)) {
                for (int v = 0; v < n; ++v) {
                    if (!(mask & (1 << v))) {
                        Edge<int>* edge = graph.findEdge(u, v);
                        if (edge) {
                            double newCost = dp[mask][u] + edge->getWeight();
                            if (newCost < dp[mask | (1 << v)][v]) {
                                dp[mask | (1 << v)][v] = newCost;
                                parent[mask | (1 << v)][v] = u;
                            }
                        }
                    }
                }
            }
        }
    }

    bestCost = INF;
    int lastNode = -1;
    for (int u = 1; u < n; ++u) {
        Edge<int>* edge = graph.findEdge(u, 0);
        if (edge) {
            double currentCost = dp[(1 << n) - 1][u] + edge->getWeight();
            if (currentCost < bestCost) {
                bestCost = currentCost;
                lastNode = u;
            }
        }
    }

    if (lastNode != -1) {
        bestPath.clear();
        int mask = (1 << n) - 1;
        int currentNode = lastNode;
        while (currentNode != 0) {
            bestPath.push_back(currentNode);
            int temp = parent[mask][currentNode];
            mask ^= (1 << currentNode);
            currentNode = temp;
        }
        bestPath.push_back(0);
        std::reverse(bestPath.begin(), bestPath.end());
        bestPath.push_back(0);
    }
}

/**
 * Displays a menu for selecting other heuristics and algorithms to apply to the TSP.
 */
void display_OHmenu(){
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Other Heuristics Menu       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the approach you want:\n";
        cout << "1. NearestNeighbour ( The best balance of performance and efficiency !!! ) (The algo we tailored for the entire project and the one with the better results ) \n";
        cout << "2. K-means Clustering NearestNeighbour ( Extra !!! )\n";
        cout << "3. LinKernighan ( Gets the best results but only feasible on small and the 3 smallest medium graphs Another Extra !!! ) \n";
        cout << "4. HeldKarp ( This algo gets Optimal solution like the backtracking but is feasible only on toy graphs. Extra !!! )\n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                display_NNmenu();
                break;
            case '2': {
                string clusterChoice;
                int clusterOption;
                bool validInput = false;

                while (!validInput) {
                    cout << "Enter the number of clusters: ";
                    cin >> clusterChoice;

                    try {
                        clusterOption = stoi(clusterChoice);
                        validInput = true;
                    } catch (const invalid_argument& e) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range& e) {
                        cout << "Invalid input. The number is out of range.\n";
                    }
                }

                display_CLUSTERmenu(clusterOption);
                break;
            }
            case '3':
                display_LINmenu();
                break;
            case '4':
                display_HeldKarp_menu();
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }

}

/**
 * Displays a menu for selecting small graph instances and applying the Nearest Neighbor algorithm.
 */
void display_NNmenu() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Nearest Neighbour Algorithm Heuristic       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option of the size of the graph you want:\n";
        cout << "1. Small Graph\n";
        cout << "2. Medium Graph \n";
        cout << "3. Large Graph \n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                display_NNmenuSmallGraph();
                break;
            case '2':
                display_NNmenuMediumGraph();
                break;
            case '3':
                display_NNmenuLargeGraph();
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}


/**
 * Displays the menu for selecting a real-world TSP instance to solve using various algorithms.
 *
 * This menu allows the user to choose between different graph instances and starting nodes.
 *
 * - Graph1('Small'): Small-sized graph instance.
 * - Graph2(Medium): Medium-sized graph instance.
 * - Graph3(Large): Large-sized graph instance.
 *
 * @note This function interacts with the user to obtain input for starting nodes and algorithm selection.
 */
void display_NNmenuLargeGraph() {
    string choice_str;
    char choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to NN for Large Graphs ( Fully connecting nodes with haversine )       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option of the size of the graph you want:\n";
        cout << "1. Graph1('Small')\n";
        cout << "2. Graph2(Medium) \n";
        cout << "3. Graph3(Large)  \n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice_str;

        try {
            choice = stoi(choice_str);
        } catch (const invalid_argument&) {
            choice = 'e';
        }

        int nodeID = 0;
        string choiceNode;

        switch (choice) {
            case 1:
                while (true) {
                    cout << "Input the starting node for the algorithm: ";
                    cin >> choiceNode;
                    try {
                        nodeID = stoi(choiceNode);
                        getValue_NNsmallGraph(nodeID);
                        break;
                    } catch (const invalid_argument&) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range&) {
                        cout << "Input out of range. Please enter a valid integer within the range of int.\n";
                    }
                }
                break;
            case 2:
                while (true) {
                    cout << "Input the starting node for the algorithm: ";
                    cin >> choiceNode;
                    try {
                        nodeID = stoi(choiceNode);
                        getValue_NNmediumGraph(nodeID);
                        break;
                    } catch (const invalid_argument&) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range&) {
                        cout << "Input out of range. Please enter a valid integer within the range of int.\n";
                    }
                }
                break;
            case 3:
                while (true) {
                    cout << "Input the starting node for the algorithm: ";
                    cin >> choiceNode;
                    try {
                        nodeID = stoi(choiceNode);
                        getValue_NNlargeGraph(nodeID);
                        break;
                    } catch (const invalid_argument&) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range&) {
                        cout << "Input out of range. Please enter a valid integer within the range of int.\n";
                    }
                }
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Solves the real-world TSP problem for a large-sized graph instance starting from the specified node.
 *
 * This function reads and parses the large-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 *
 * @remarks The time complexity of the nearest neighbor algorithm used in this function is O(n^2), where n is the number of vertices in the graph.
 */
void getValue_NNsmallGraph(int nodeID) {
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;

    auto start1 = std::chrono::high_resolution_clock::now();
    Graph<int> graph = reader.readAndParseRealWorld_Graphs4_2(1, vertexMap, edgeMap);
    auto end1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration1 = end1 - start1;

    cout << "Parsing time: " << duration1.count() << "\n";
    Vertex<int>* startingVertex = vertexMap[nodeID];
    if(!startingVertex){
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.nearestNeighbourNode(graph,startingVertex, vertexMap, edgeMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;


    double totalDistance = 0;
    cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }

    std::cout << std::endl;
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
    std::cout << "Tour Size: " << tour.size() << "\n";
}


/**
 * Solves the real-world TSP problem for a large-sized graph instance starting from the specified node.
 *
 * This function reads and parses the large-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 *
 * @remarks The time complexity of the nearest neighbor algorithm used in this function is O(n^2), where n is the number of vertices in the graph.
 */
void getValue_NNmediumGraph(int nodeID) {
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;

    auto start1 = std::chrono::high_resolution_clock::now();
    Graph<int> graph = reader.readAndParseRealWorld_Graphs4_2(2, vertexMap, edgeMap);
    auto end1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration1 = end1 - start1;

    cout << "Parsing time: " << duration1.count() << "\n";
    Vertex<int>* startingVertex = vertexMap[nodeID];
    if(!startingVertex){
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.nearestNeighbourNode(graph,startingVertex, vertexMap, edgeMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;


    double totalDistance = 0;
    cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }

    std::cout << std::endl;
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
    std::cout << "Tour Size: " << tour.size() << "\n";
}


/**
 * Solves the real-world TSP problem for a large-sized graph instance starting from the specified node.
 *
 * This function reads and parses the large-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 *
 * @remarks The time complexity of the nearest neighbor algorithm used in this function is O(n^2), where n is the number of vertices in the graph.
 */
void getValue_NNlargeGraph(int nodeID) {
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;

    auto start1 = std::chrono::high_resolution_clock::now();
    Graph<int> graph = reader.readAndParseRealWorld_Graphs4_2(3, vertexMap, edgeMap);
    auto end1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration1 = end1 - start1;

    cout << "Parsing time: " << duration1.count() << "\n";
    Vertex<int>* startingVertex = vertexMap[nodeID];
    if(!startingVertex){
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.nearestNeighbourNode(graph,startingVertex, vertexMap, edgeMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;


    double totalDistance = 0;
    cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }

    std::cout << std::endl;
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
    std::cout << "Tour Size: " << tour.size() << "\n";
}

/**
 * Displays a menu for selecting small graph instances and applying the Nearest Neighbor algorithm.
 */
void display_NNmenuSmallGraph(){
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Nearest Neighbour Heuristic SmallGraph Menu       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the graph you want:\n";
        cout << "1. Stadium Graph\n";
        cout << "2. Shipping Graph \n";
        cout <<"3. Tourism Graph \n";
        cout << "e. Back to the other Heuristics Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                getValue_NNmenuSmallGraphStadium();
                break;
            case '2':
                getValue_NNmenuSmallGraphShipping();
                break;
            case '3':
                getValue_NNmenuSmallGraphTourism();
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Applies the Nearest Neighbor algorithm to the stadium graph instance.
 * Time Complexity: O(n^2), where 'n' is the number of vertices in the graph.
 */
void getValue_NNmenuSmallGraph(const std::function<Graph<int>(Reader&)>& readAndParseFunc) {
    Reader reader;
    Graph<int> graph;
    graph = readAndParseFunc(reader);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.nearestNeighbour(graph);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    double totalDistance = 0;
    std::cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        std::cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }
    std::cout << std::endl;
    cout << "Tour Size: " << tour.size() << "\n";
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";

}

/**
 * Applies the Nearest Neighbor algorithm to the shipping graph instance.
 */
void getValue_NNmenuSmallGraphStadium(){
    auto readAndParseStadium = [](Reader& reader) { return reader.readAndParseStadium(); };
    getValue_NNmenuSmallGraph(readAndParseStadium);
}

/**
 * Applies the Nearest Neighbor algorithm to the tourism graph instance.
 */
void getValue_NNmenuSmallGraphShipping() {
    auto readAndParseShipping = [](Reader& reader) { return reader.readAndParseShipping(); };
    getValue_NNmenuSmallGraph(readAndParseShipping);
    cout << "Graph is not fully connected value isn´t a good approximation";
}

/**
 * Applies the Nearest Neighbor algorithm to the tourism graph instance.
 */
void getValue_NNmenuSmallGraphTourism() {
    auto readAndParseTourism = [](Reader& reader) { return reader.readAndParseTourism(); };
    getValue_NNmenuSmallGraph(readAndParseTourism);
}

/**
 * Displays a menu for selecting medium graph instances and applying the Nearest Neighbor heuristic algorithm.
 * The user can choose the number of edges in the graph and apply the Nearest Neighbor algorithm accordingly.
 */
void display_NNmenuMediumGraph() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Nearest Neighbour Medium Graph Menu       \n";
        cout << "-----------------------------\n";
        cout << "How many edges does the graph have:\n";
        cout << "1. 25\n";
        cout << "2. 50\n";
        cout << "3. 75\n";
        cout << "4. 100\n";
        cout << "5. 200\n";
        cout << "6. 300\n";
        cout << "7. 400\n";
        cout << "8. 500\n";
        cout << "9. 600\n";
        cout << "10. 700\n";
        cout << "11. 800\n";
        cout << "12. 900\n";
        cout << "e. Back to the other heuristics Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice == "e") {
            cout << "Exiting menu system...\n";
            exitMenu = true;
        } else {
            int choiceNum = stoi(choice);
            switch (choiceNum) {
                case 1:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_25.csv");
                    break;
                case 2:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_50.csv");
                    break;
                case 3:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_75.csv");
                    break;
                case 4:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_100.csv");
                    break;
                case 5:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_200.csv");
                    break;
                case 6:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_300.csv");
                    break;
                case 7:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_400.csv");
                    break;
                case 8:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_500.csv");
                    break;
                case 9:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_600.csv");
                    break;
                case 10:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_700.csv");
                    break;
                case 11:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_800.csv");
                    break;
                case 12:
                    getValue_NNmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_900.csv");
                    break;
                default:
                    cout << "Invalid input. Please choose a valid option.\n";
            }
        }
    }
}

/**
 * Applies the Nearest Neighbor algorithm to a medium graph instance.
 *
 * @param filename The filename of the graph data to be read and parsed.
 *                 This should include the path to the file.
 * Time Complexity: O(n^2), where 'n' is the number of vertices in the graph.
 */
void getValue_NNmenuMediumGraph(const std::string &filename){
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
    Graph<int> graph = reader.readAndParse4_2Extra_Fully_Connected_Graphs(filename,vertexMap,edgeMap);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.nearestNeighbourMedium(graph,vertexMap,edgeMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    double totalDistance = 0;
    std::cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        std::cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }
    std::cout << std::endl;
    cout << "Tour Size: " << tour.size() << "\n";
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
}

/**
 * Displays a menu for selecting the Lin-Kernighan Algorithm heuristic and the size of the graph.
 * Users can choose between small and medium graphs to apply the Lin-Kernighan Algorithm.
 */
void display_LINmenu() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to LinKernighan Algorithm Heuristic       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option of the size of the graph you want:\n";
        cout << "1. Small Graph\n";
        cout << "2. Medium Graph \n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                display_LINmenuSmallGraph();
                break;
            case '2':
                display_LINmenuMediumGraph();
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Displays a menu for selecting small graph instances and applying the Lin-Kernighan Algorithm heuristic.
 * Users can choose between different types of small graphs, such as stadium, shipping, and tourism graphs.
 */
void display_LINmenuSmallGraph(){
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to LinKernighan Heuristic SmallGraph Menu       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the graph you want:\n";
        cout << "1. Stadium Graph\n";
        cout << "2. Shipping Graph \n";
        cout <<"3. Tourism Graph \n";
        cout << "e. Back to the Other Heuristics Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                getValue_LINmenuSmallGraphStadium();
                break;
            case '2':
                getValue_LINmenuSmallGraphShipping();
                break;
            case '3':
                getValue_LINmenuSmallGraphTourism();
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Applies the Lin-Kernighan algorithm to a small graph instance using the provided reader function.
 *
 * @param readAndParseFunc A function that reads and parses the graph data.
 *                         It should return a Graph<int> object.
 * @remarks The time complexity of the Lin-Kernighan algorithm O(n^3), where n is the number of vertices in the graph.
 * The algorithm has a better approximation than the NearestNeighbour heuristics but scales worse
 * @timecomplexity O(n^2log(n))
 */
void getValue_LINmenuSmallGraph(const std::function<Graph<int>(Reader&)>& readAndParseFunc) {
    Reader reader;
    Graph<int> graph;
    graph = readAndParseFunc(reader);
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.linKernighan(graph,vertexMap,edgeMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    double totalDistance = 0;
    std::cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        std::cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }
    std::cout << std::endl;
    cout << "Tour Size: " << tour.size() << "\n";
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";

}

/**
 * Applies the Lin-Kernighan Algorithm heuristic to a stadium graph instance.
 *
 * @param readAndParseStadium A lambda function that reads and parses the stadium graph data.
 *                            It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_LINmenuSmallGraphStadium(){
    auto readAndParseStadium = [](Reader& reader) { return reader.readAndParseStadium(); };
    getValue_LINmenuSmallGraph(readAndParseStadium);
}

/**
 * Applies the Lin-Kernighan Algorithm heuristic to a shipping graph instance.
 *
 * @param readAndParseShipping A lambda function that reads and parses the shipping graph data.
 *                             It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_LINmenuSmallGraphShipping() {
    auto readAndParseShipping = [](Reader& reader) { return reader.readAndParseShipping(); };
    getValue_LINmenuSmallGraph(readAndParseShipping);
    cout << "Graph is not fully connected value isn´t a good approximation";
}

/**
 * Applies the Lin-Kernighan Algorithm heuristic to a tourism graph instance.
 *
 * @param readAndParseTourism A lambda function that reads and parses the tourism graph data.
 *                            It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_LINmenuSmallGraphTourism() {
    auto readAndParseTourism = [](Reader& reader) { return reader.readAndParseTourism(); };
    getValue_LINmenuSmallGraph(readAndParseTourism);
}

/**
 * Displays a menu for selecting medium graph instances and applying the LinKernighan heuristic algorithm.
 * The user can choose the number of edges in the graph and apply the LinKernighan heuristic accordingly.
 */
void display_LINmenuMediumGraph() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to LinKernighan Medium Graph Menu       \n";
        cout << "-----------------------------\n";
        cout << "How many edges does the graph have:\n";
        cout << "1. 25\n";
        cout << "2. 50\n";
        cout << "3. 75\n";
        cout << "4. 100\n";
        cout << "5. 200\n";
        cout << "6. 300\n";
        cout << "7. 400\n";
        cout << "8. 500\n";
        cout << "9. 600\n";
        cout << "10. 700\n";
        cout << "11. 800\n";
        cout << "12. 900\n";
        cout << "e. Back to the other Heuristics Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice == "e") {
            cout << "Exiting menu system...\n";
            exitMenu = true;
        } else {
            int choiceNum = stoi(choice);
            switch (choiceNum) {
                case 1:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_25.csv");
                    break;
                case 2:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_50.csv");
                    break;
                case 3:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_75.csv");
                    break;
                case 4:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_100.csv");
                    break;
                case 5:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_200.csv");
                    break;
                case 6:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_300.csv");
                    break;
                case 7:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_400.csv");
                    break;
                case 8:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_500.csv");
                    break;
                case 9:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_600.csv");
                    break;
                case 10:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_700.csv");
                    break;
                case 11:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_800.csv");
                    break;
                case 12:
                    getValue_LINmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_900.csv");
                    break;
                default:
                    cout << "Invalid input. Please choose a valid option.\n";
            }
        }
    }
}

/**
 * Applies the Lin-Kernighan Algorithm heuristic to a medium graph instance.
 *
 * @param filename The filename of the graph data to be read and parsed.
 *                 This should include the path to the file.
 * @remarks The time complexity of the Lin-Kernighan algorithm O(n^3), where n is the number of vertices in the graph.
 * The algorithm has a better approximation than the NearestNeighbour heuristics but scales worse
 *  * @timecomplexity O(n^2log(n))
 */
void getValue_LINmenuMediumGraph(const std::string &filename){
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
    Graph<int> graph = reader.readAndParse4_2Extra_Fully_Connected_Graphs(filename,vertexMap,edgeMap);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.linKernighan(graph,vertexMap,edgeMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    double totalDistance = 0;
    std::cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        std::cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }
    std::cout << std::endl;
    cout << "Tour Size: " << tour.size() << "\n";
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
}

/**
 * @brief Displays the main menu for selecting the size of the graph for the Held-Karp algorithm.
 *
 * This function provides a menu interface for the user to select between a small graph, medium graph, or exit to the main menu.
 */
void display_HeldKarp_menu() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to HeldKarp Algorithm for Small Graphs optimal solution       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option of the size of the graph you want:\n";
        cout << "1. Small Graph\n";
        cout << "2. Medium Graph \n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                display_HeldKarp_menuSmallGraph();
                break;
            case '2':
                display_HeldKarp_menuMediumGraph();
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * @brief Displays the menu for selecting a specific small graph for the Held-Karp algorithm.
 *
 * This function provides a menu interface for the user to select between different predefined small graphs
 * or to exit back to the main Held-Karp menu.
 */
void display_HeldKarp_menuSmallGraph(){
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to HeldKarp Algorithm for Small Graphs optimal solution SmallGraph Menu       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the graph you want:\n";
        cout << "1. Stadium Graph\n";
        cout << "2. Shipping Graph \n";
        cout <<"3. Tourism Graph \n";
        cout << "e. Back to the other Heuristics Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                getValue_HeldKarp_menuSmallGraphStadium();
                break;
            case '2':
                getValue_HeldKarp_menuSmallGraphShipping();
                break;
            case '3':
                getValue_HeldKarp_menuSmallGraphTourism();
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Applies the Held-Karp algorithm to a small graph instance using the provided reader function.
 *
 * @param readAndParseFunc A function that reads and parses the graph data.
 *                         It should return a Graph<int> object.
 * @timecomplexity The time complexity is O(n^2 * 2^n), where 'n' is the number of vertices in the graph.
 *           The space complexity is O(n * 2^n).
 */
void getValue_HeldKarp_menuSmallGraph(const std::function<Graph<int>(Reader&)>& readAndParseFunc) {
    Reader reader;
    Graph<int> graph;
    graph = readAndParseFunc(reader);
    std::vector<int> bestPath;
    double bestCost = std::numeric_limits<double>::infinity();
    std::vector<int> path(1, 0);
    graph.findVertex(0)->setVisited(true);

    auto startTime = std::chrono::high_resolution_clock::now();
    heldKarp(graph,bestPath,bestCost);
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> execTime = endTime - startTime;

    std::cout << "Execution Time: " << execTime.count() << " seconds\n";
    cout << "Tour Size: " << bestPath.size() << "\n";
    std::cout << "Best Path Cost: " << bestCost << "\n";
    std::cout << "Best Path: ";
    for (size_t i = 0; i < bestPath.size(); ++i) {
        std::cout << bestPath[i];
        if (i < bestPath.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << "\n";
}

/**
 * Applies the Held-Karp algorithm to a stadium graph instance.
 *
 * @param readAndParseStadium A lambda function that reads and parses the stadium graph data.
 *                            It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_HeldKarp_menuSmallGraphStadium(){
    auto readAndParseStadium = [](Reader& reader) { return reader.readAndParseStadium(); };
    getValue_HeldKarp_menuSmallGraph(readAndParseStadium);
}

/**
 * Applies the Held-Karp algorithm to a shipping graph instance.
 *
 * @param readAndParseShipping A lambda function that reads and parses the shipping graph data.
 *                             It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_HeldKarp_menuSmallGraphShipping() {
    auto readAndParseShipping = [](Reader& reader) { return reader.readAndParseShipping(); };
    getValue_HeldKarp_menuSmallGraph(readAndParseShipping);
}

/**
 * Applies the Held-Karp algorithm to a tourism graph instance.
 *
 * @param readAndParseTourism A lambda function that reads and parses the tourism graph data.
 *                            It should take a reference to a Reader object and return a Graph<int> object.
 */
void getValue_HeldKarp_menuSmallGraphTourism() {
    auto readAndParseTourism = [](Reader& reader) { return reader.readAndParseTourism(); };
    getValue_HeldKarp_menuSmallGraph(readAndParseTourism);
}

/**
 * Applies the Held-Karp algorithm to a medium graph instance.
 *
 * @param filename The filename of the graph data to be read and parsed.
 *                 This should include the path to the file.
 * @timecomplexity The time complexity is O(n^2 * 2^n), where 'n' is the number of vertices in the graph.
 *           The space complexity is O(n * 2^n).
 */
void getValue_HeldKarp_menuMediumGraph(const std::string &filename){
    Reader reader;
    Graph<int> graph;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
    graph = reader.readAndParse4_2Extra_Fully_Connected_Graphs(filename,vertexMap,edgeMap);
    std::vector<int> bestPath;
    double bestCost = std::numeric_limits<double>::infinity();
    std::vector<int> path(1, 0);
    graph.findVertex(0)->setVisited(true);

    auto startTime = std::chrono::high_resolution_clock::now();
    tsp(0, path, 0.0, 1, graph, bestPath, bestCost);
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> execTime = endTime - startTime;

    std::cout << "Execution Time: " << execTime.count() << " seconds\n";
    std::cout << "Best Path Cost: " << bestCost << "\n";
    std::cout << "Best Path: ";
    for (int node : bestPath) {
        std::cout << node << (node == 0 ? "\n" : " -> ");
    }

}

/**
 * Displays a menu for selecting medium graph instances and applying the Held-Karp Algorithm heuristic.
 * Users can choose the number of edges in the graph to apply the Held-Karp Algorithm.
 */
void display_HeldKarp_menuMediumGraph() {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to HeldKarp Medium Graph Menu       \n";
        cout << "-----------------------------\n";
        cout << "How many edges does the graph have:\n";
        cout << "1. 25\n";
        cout << "2. 50\n";
        cout << "3. 75\n";
        cout << "4. 100\n";
        cout << "5. 200\n";
        cout << "6. 300\n";
        cout << "7. 400\n";
        cout << "8. 500\n";
        cout << "9. 600\n";
        cout << "10. 700\n";
        cout << "11. 800\n";
        cout << "12. 900\n";
        cout << "e. Back to the other heuristics Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice == "e") {
            cout << "Exiting menu system...\n";
            exitMenu = true;
        } else {
            int choiceNum = stoi(choice);
            switch (choiceNum) {
                case 1:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_25.csv");
                    break;
                case 2:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_50.csv");
                    break;
                case 3:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_75.csv");
                    break;
                case 4:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_100.csv");
                    break;
                case 5:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_200.csv");
                    break;
                case 6:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_300.csv");
                    break;
                case 7:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_400.csv");
                    break;
                case 8:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_500.csv");
                    break;
                case 9:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_600.csv");
                    break;
                case 10:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_700.csv");
                    break;
                case 11:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_800.csv");
                    break;
                case 12:
                    getValue_HeldKarp_menuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_900.csv");
                    break;
                default:
                    cout << "Invalid input. Please choose a valid option.\n";
            }
        }
    }
}

/**
 * Displays a menu for selecting graph instances and applying the K-means Clustering Nearest Neighbor Algorithm heuristic.
 * Users can choose between small and medium graph sizes or return to the main menu.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void display_CLUSTERmenu(int clusterOption) {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to K-means Clustering NearestNeighbour Algorithm Heuristic ( Mixes clustering and NN )      \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option of the size of the graph you want:\n";
        cout << "1. Small Graph\n";
        cout << "2. Medium Graph \n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':\
                display_CLUSTERmenuSmallGraph(clusterOption);
                break;
            case '2':
                display_CLUSTERmenuMediumGraph(clusterOption);
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small graph instance.
 *
 * @param readAndParseFunc A function to read and parse the graph data.
 * @param clusterOption The number of clusters for K-means clustering.
 * @timecomplexity O(n^2)
 */
void display_CLUSTERmenuSmallGraph(int clusterOption) {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to K-means Clustering NearestNeighbour Heuristic SmallGraph Menu       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the graph you want:\n";
        cout << "1. Stadium Graph\n";
        cout << "2. Shipping Graph \n";
        cout << "3. Tourism Graph \n";
        cout << "e. Back to the other Heuristics Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice.length() != 1) {
            choice = "0";
        }

        switch (choice[0]) {
            case '1':
                getValue_CLUSTERmenuSmallGraphStadium(clusterOption);
                break;
            case '2':
                getValue_CLUSTERmenuSmallGraphShipping(clusterOption);
                break;
            case '3':
                getValue_CLUSTERmenuSmallGraphTourism(clusterOption);
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small graph instance.
 *
 * @param readAndParseFunc A function to read and parse the graph data.
 * @param clusterOption The number of clusters for K-means clustering.
 * @timecomplexity O(n^2)
 */
void getValue_CLUSTERmenuSmallGraph(const std::function<Graph<int>(Reader&)>& readAndParseFunc, int clusterOption)  {
    auto start = std::chrono::high_resolution_clock::now();

    Reader reader;
    Graph<int> graph = readAndParseFunc(reader);
    auto coordinates = reader.readCoordinates();

    int numClusters = clusterOption;
    auto clusters = reader.kMeansClustering(graph, numClusters, coordinates);

    std::vector<int> combinedTour;
    std::unordered_set<int> visitedNodes;

    int currentNode = 0;
    combinedTour.push_back(currentNode);
    visitedNodes.insert(currentNode);

    for (const auto& cluster : clusters) {
        if (cluster.empty()) continue;

        std::vector<int> extendedCluster = cluster;
        if (std::find(cluster.begin(), cluster.end(), currentNode) == cluster.end()) {
            extendedCluster.push_back(currentNode);
        }

        auto clusterTour = graph.nearestNeighborForCluster(graph, extendedCluster);

        for (size_t i = 1; i < clusterTour.size(); ++i) {
            if (visitedNodes.insert(clusterTour[i]).second) {
                combinedTour.push_back(clusterTour[i]);
                currentNode = clusterTour[i];
            }
        }
    }

    combinedTour.push_back(0);

    double totalDistance = 0;
    int previous = combinedTour.front();
    for (size_t i = 1; i < combinedTour.size(); ++i) {
        Edge<int>* edge = graph.findEdge(previous, combinedTour[i]);
        if (edge) {
            totalDistance += edge->getWeight();
        }
        previous = combinedTour[i];
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> execTime = end - start;

    cout << "Tour Size: " << combinedTour.size() << "\n";
    std::cout << "TSP Tour: ";
    for (size_t i = 0; i < combinedTour.size(); ++i) {
        std::cout << combinedTour[i] << (i < combinedTour.size() - 1 ? " -> " : "");
    }
    std::cout << std::endl;
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Execution Time: " << execTime.count() << " seconds\n";
}

/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small stadium graph instance.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void getValue_CLUSTERmenuSmallGraphStadium(int clusterOption) {
    auto readAndParseStadium = [](Reader& reader) { return reader.readAndParseStadium(); };
    getValue_CLUSTERmenuSmallGraph(readAndParseStadium, clusterOption);
}
/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small shipping graph instance.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void getValue_CLUSTERmenuSmallGraphShipping(int clusterOption) {
    auto readAndParseShipping = [](Reader& reader) { return reader.readAndParseShipping(); };
    getValue_CLUSTERmenuSmallGraph(readAndParseShipping, clusterOption);
    cout << "Graph is not fully connected value isn´t a good approximation";
}
/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a small tourism graph instance.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void getValue_CLUSTERmenuSmallGraphTourism(int clusterOption) {
    auto readAndParseTourism = [](Reader& reader) { return reader.readAndParseTourism(); };
    getValue_CLUSTERmenuSmallGraph(readAndParseTourism, clusterOption);
}

/**
 * Applies the K-means Clustering Nearest Neighbor Algorithm heuristic to a medium graph instance.
 *
 * @param filename The filename of the graph data.
 * @param clusterOption The number of clusters for K-means clustering.
 * @timecomplexity O(n^2)
 */
void getValue_CLUSTERmenuMediumGraph(const std::string &filename, int clusterOption) {
    auto start = std::chrono::high_resolution_clock::now();

    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;
    Graph<int> graph = reader.readAndParse4_2Extra_Fully_Connected_Graphs(filename,vertexMap,edgeMap);
    auto coordinates = reader.readCoordinates();

    int numClusters = clusterOption;
    auto clusters = reader.kMeansClustering(graph, numClusters, coordinates);

    std::vector<int> combinedTour;
    std::unordered_set<int> visitedNodes;

    int currentNode = 0;
    combinedTour.push_back(currentNode);
    visitedNodes.insert(currentNode);

    for (const auto& cluster : clusters) {
        if (cluster.empty()) continue;

        std::vector<int> extendedCluster = cluster;
        if (std::find(cluster.begin(), cluster.end(), currentNode) == cluster.end()) {
            extendedCluster.push_back(currentNode);
        }

        auto clusterTour = graph.nearestNeighborForCluster(graph, extendedCluster);

        for (size_t i = 1; i < clusterTour.size(); ++i) {
            if (visitedNodes.insert(clusterTour[i]).second) {
                combinedTour.push_back(clusterTour[i]);
                currentNode = clusterTour[i];
            }
        }
    }

    combinedTour.push_back(0);

    double totalDistance = 0;
    int previous = combinedTour.front();
    for (size_t i = 1; i < combinedTour.size(); ++i) {
        Edge<int>* edge = graph.findEdge(previous, combinedTour[i]);
        if (edge) {
            totalDistance += edge->getWeight();
        }
        previous = combinedTour[i];
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> execTime = end - start;

    std::cout << "TSP Tour: ";
    for (size_t i = 0; i < combinedTour.size(); ++i) {
        std::cout << combinedTour[i] << (i < combinedTour.size() - 1 ? " -> " : "");
    }
    std::cout << std::endl;
    cout << "Tour Size: " << combinedTour.size() << "\n";
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Execution Time: " << execTime.count() << " seconds\n";

}

/**
 * Displays the menu for selecting a medium graph instance for applying the K-means Clustering Nearest Neighbor Algorithm heuristic.
 *
 * @param clusterOption The number of clusters for K-means clustering.
 */
void display_CLUSTERmenuMediumGraph(int clusterOption) {
    string choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Nearest Neighbour Medium Graph Menu       \n";
        cout << "-----------------------------\n";
        cout << "How many edges does the graph have:\n";
        cout << "1. 25\n";
        cout << "2. 50\n";
        cout << "3. 75\n";
        cout << "4. 100\n";
        cout << "5. 200\n";
        cout << "6. 300\n";
        cout << "7. 400\n";
        cout << "8. 500\n";
        cout << "9. 600\n";
        cout << "10. 700\n";
        cout << "11. 800\n";
        cout << "12. 900\n";
        cout << "e. Back to the other heuristics Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice;

        if (choice == "e") {
            cout << "Exiting menu system...\n";
            exitMenu = true;
        } else {
            int choiceNum = stoi(choice);
            switch (choiceNum) {
                case 1:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_25.csv", clusterOption);
                    break;
                case 2:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_50.csv", clusterOption);
                    break;
                case 3:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_75.csv", clusterOption);
                    break;
                case 4:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_100.csv", clusterOption);
                    break;
                case 5:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_200.csv", clusterOption);
                    break;
                case 6:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_300.csv", clusterOption);
                    break;
                case 7:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_400.csv", clusterOption);
                    break;
                case 8:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_500.csv", clusterOption);
                    break;
                case 9:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_600.csv", clusterOption);
                    break;
                case 10:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_700.csv", clusterOption);
                    break;
                case 11:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_800.csv", clusterOption);
                    break;
                case 12:
                    getValue_CLUSTERmenuMediumGraph("../Data/Extra_Fully_Connected_Graphs/edges_900.csv", clusterOption);
                    break;
                default:
                    cout << "Invalid input. Please choose a valid option.\n";
            }
        }
    }
}

/**
 * Displays the menu for selecting a real-world TSP instance to solve using various algorithms.
 *
 * This menu allows the user to choose between different graph instances and starting nodes.
 *
 * - Graph1('Small'): Small-sized graph instance.
 * - Graph2(Medium): Medium-sized graph instance.
 * - Graph3(Large): Large-sized graph instance.
 *
 * @note This function interacts with the user to obtain input for starting nodes and algorithm selection.
 */
void display_RWmenu() {
    string choice_str;
    char choice;
    bool exitMenu = false;
    while (!exitMenu) {
        cout << "\n-----------------------------\n";
        cout << "     Welcome to Real World TSP       \n";
        cout << "-----------------------------\n";
        cout << "Enter the number of the option of the size of the graph you want:\n";
        cout << "1. Graph1('Small')\n";
        cout << "2. Graph2(Medium) \n";
        cout << "3. Graph3(Large)  \n";
        cout << "e. Back to the main Menu\n";
        cout << "-----------------------------\n";
        cout << "Your choice: ";
        cin >> choice_str;

        try {
            choice = stoi(choice_str);
        } catch (const invalid_argument&) {
            choice = 'e';
        }

        int nodeID = 0;
        string choiceNode;

        switch (choice) {
            case 1:
                while (true) {
                    cout << "Input the starting node for the algorithm: ";
                    cin >> choiceNode;
                    try {
                        nodeID = stoi(choiceNode);
                        getValue_RWsmallGraph(nodeID);
                        break;
                    } catch (const invalid_argument&) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range&) {
                        cout << "Input out of range. Please enter a valid integer within the range of int.\n";
                    }
                }
                break;
            case 2:
                while (true) {
                    cout << "Input the starting node for the algorithm: ";
                    cin >> choiceNode;
                    try {
                        nodeID = stoi(choiceNode);
                        getValue_RWmediumGraph(nodeID);
                        break;
                    } catch (const invalid_argument&) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range&) {
                        cout << "Input out of range. Please enter a valid integer within the range of int.\n";
                    }
                }
                break;
            case 3:
                while (true) {
                    cout << "Input the starting node for the algorithm: ";
                    cin >> choiceNode;
                    try {
                        nodeID = stoi(choiceNode);
                        getValue_RWlargeGraph(nodeID);
                        break;
                    } catch (const invalid_argument&) {
                        cout << "Invalid input. Please enter a valid integer.\n";
                    } catch (const out_of_range&) {
                        cout << "Input out of range. Please enter a valid integer within the range of int.\n";
                    }
                }
                break;
            case 'e':
                cout << "Exiting menu system...\n";
                exitMenu = true;
                break;
            default:
                cout << "Invalid input. Please choose a valid option.\n";
        }
    }
}

/**
 * Solves the real-world TSP problem for a small-sized graph instance starting from the specified node.
 *
 * This function reads and parses the small-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(n^2)
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 */
void getValue_RWsmallGraph(int nodeID){

    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;

    auto start1 = std::chrono::high_resolution_clock::now();
    Graph<int> graph = reader.readAndParseRealWorld_Graphs(1, vertexMap, edgeMap);
    auto end1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration1 = end1 - start1;

    cout << "Parsing time: " << duration1.count() << "\n";

    Vertex<int>* startingVertex = vertexMap[nodeID];
    if(!startingVertex){
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.nearestNeighbourNode(graph,startingVertex, vertexMap, edgeMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;


    double totalDistance = 0;
    cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }

    std::cout << std::endl;
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Tour Size: " << tour.size() << "\n";
    std::cout << "Time: " << duration.count() << "\n";

}

/**
 * Solves the real-world TSP problem for a medium-sized graph instance starting from the specified node.
 *
 * This function reads and parses the medium-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @timecomplexity O(n^2)
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 */
void getValue_RWmediumGraph(int nodeID) {
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;

    auto start1 = std::chrono::high_resolution_clock::now();
    Graph<int> graph = reader.readAndParseRealWorld_Graphs(2, vertexMap, edgeMap);
    auto end1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration1 = end1 - start1;

    cout << "Parsing time: " << duration1.count() << "\n";

    Vertex<int>* startingVertex = vertexMap[nodeID];
    if(!startingVertex){
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.nearestNeighbourNode(graph,startingVertex, vertexMap, edgeMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;


    double totalDistance = 0;
    cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }

    std::cout << std::endl;
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
    std::cout << "Tour Size: " << tour.size() << "\n";
    std::cout << "!!! Unfeasible Path doesn´t cover all vertices !!!\n";
}

/**
 * Solves the real-world TSP problem for a large-sized graph instance starting from the specified node.
 *
 * This function reads and parses the large-sized graph instance, performs the nearest neighbor algorithm starting from the given node,
 * and prints the resulting tour along with its total distance and execution time.
 *
 * @param nodeID The ID of the starting node for the TSP algorithm.
 *
 * @note This function interacts with the user to obtain input for starting nodes.
 *
 * @remarks The time complexity of the nearest neighbor algorithm used in this function is O(n^2), where n is the number of vertices in the graph.
 */
void getValue_RWlargeGraph(int nodeID) {
    Reader reader;
    unordered_map<int, Vertex<int>*> vertexMap;
    unordered_map<std::string, Edge<int>*> edgeMap;

    auto start1 = std::chrono::high_resolution_clock::now();
    Graph<int> graph = reader.readAndParseRealWorld_Graphs(3, vertexMap, edgeMap);
    auto end1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration1 = end1 - start1;

    cout << "Parsing time: " << duration1.count() << "\n";
    Vertex<int>* startingVertex = vertexMap[nodeID];
    if(!startingVertex){
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Vertex<int>*> tour = graph.nearestNeighbourNode(graph,startingVertex, vertexMap, edgeMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;


    double totalDistance = 0;
    cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        cout << tour[i]->getInfo() << (i < tour.size() - 1 ? " -> " : "");
        if (i > 0) {
            Edge<int>* edge = graph.findEdge(tour[i - 1]->getInfo(), tour[i]->getInfo());
            if (edge)
                totalDistance += edge->getWeight();
        }
    }

    std::cout << std::endl;
    std::cout << "Total Approximation Distance: " << totalDistance << "\n";
    std::cout << "Time: " << duration.count() << "\n";
    std::cout << "Tour Size: " << tour.size() << "\n";
    std::cout << "!!! Unfeasible Path doesn´t cover all vertices !!!\n";
}

/**
 * @brief Runs the application by displaying the main menu.
 */
void App::run() {
    mainMenu();
}