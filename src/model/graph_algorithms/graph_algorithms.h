#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_ALGORITHMS_GRAPH_ALGORITHMS_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_ALGORITHMS_GRAPH_ALGORITHMS_H_

#include <list>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <vector>

#include "../graph/graph.h"
#include "matrix.h"

namespace graph_cb {

class GraphAlgorithms {
 public:
  struct TsmResult {
    std::vector<int> vertices;
    double distance;
  };

  enum GraphAlgoritmsError { kVertexNotFound = -1, kWayNotFound = -2 };
  enum SalesmanAlgorithms { kAntAlgorithm };

  using EdgeWeightType = Graph::EdgeWeightType;
  using Vertex = Graph::Vertex;
  using Edge = Graph::Edge;
  using Vertices = Graph::Vertexes;
  using WayBetweenTwo = TsmResult;

  GraphAlgorithms();
  virtual ~GraphAlgorithms();

  std::vector<Vertex> DepthFirstSearch(const Graph &graph,
                                       Vertex start_vertex) const;

  std::vector<Vertex> BreadthFirstSearch(const Graph &graph,
                                         Vertex start_vertex) const;

  /// @brief The search is implemented through Dijkstra's algorithm. Works only
  /// with non-negative weights
  /// @param graph
  /// @param vertex1
  /// @param vertex2
  /// @return Struct with path and visited_vertices. If there is no way return
  /// -1;
  WayBetweenTwo GetShortestPathBetweenVertices(const Graph &graph, int vertex1,
                                               int vertex2) const;

  mtlc::Matrix<EdgeWeightType> GetShortestPathsBetweenAllVertices(
      const Graph &graph) const;

  /// @brief Finds the minimum spanning tree using Prim's algorithm
  /// @return A 0 by 0 matrix if the graph is unconnected or empty
  mtlc::Matrix<EdgeWeightType> GetLeastSpanningTree(const Graph &graph) const;

  TsmResult SolveTravelingSalesmanProblem(const Graph &graph,
                                          SalesmanAlgorithms type,
                                          int rand = 0) const;

 private:
  struct WeightPheromone {
    EdgeWeightType weight_;
    float pheromone_;
  };

  struct Ant {
    int num_;
    std::vector<Vertex> visited_vertices_;
    std::set<Vertex> unvisited_vertices_;
    EdgeWeightType path_;
  };

  struct VertexProbility {
    Vertex vertex_;
    float probility_;
    EdgeWeightType weight_;
  };

  // --------------------------------------------------------------------------
  //                                  DFS
  // --------------------------------------------------------------------------

  void PushVertexToStack(std::stack<Vertex> &vertex_stack, Graph::Edges &edges,
                         const std::set<Vertex> &vertices) const;

  // --------------------------------------------------------------------------
  //                                  BFS
  // --------------------------------------------------------------------------

  void PushVertexToQueue(std::queue<Vertex> &vertex_queue, Graph::Edges &edges,
                         const std::set<Vertex> &vertices) const;

  // --------------------------------------------------------------------------
  //                         Shortest Path Between Two
  // --------------------------------------------------------------------------

  bool HasGraphVertices(const Graph &graph, int vertex1, int vertex2) const;

  void PushVertex(std::map<Vertex, EdgeWeightType> &vertices,
                  const Graph &graph, Vertex start) const;

  Vertex FindVertexWithMinWeight(
      const std::set<Vertex> &unvisited_vertices,
      const std::map<Vertex, EdgeWeightType> &vertices) const;

  WayBetweenTwo FindWay(const Graph &graph,
                        const std::map<Vertex, EdgeWeightType> &vertices,
                        int vertex1, int vertex2) const;

  Vertex FindPrevVertex(
      Vertex this_vertex, const std::map<Vertex, EdgeWeightType> &vertex_point,
      const Vertices &vertices,
      const mtlc::Matrix<EdgeWeightType> &adjacency_matrix) const;

  // --------------------------------------------------------------------------
  //                         Shortest Path Between All
  // --------------------------------------------------------------------------

  mtlc::Matrix<EdgeWeightType> CreateAdjacencyMatrix(const Graph &graph) const;

  std::map<Vertex, EdgeWeightType> NearVertices(
      const std::vector<Graph::Edge> &edges) const;

  // --------------------------------------------------------------------------
  //                              Spanning Tree
  // --------------------------------------------------------------------------

  EdgeWeightType FindVertexWithMinWeight(
      const Graph &graph, const Vertices &placed_vertices,
      const std::set<Vertex> &unplaced_vertices, Vertex &start,
      Vertex &end) const;

  void AddWeigthToSpanningTree(const Graph &graph,
                               mtlc::Matrix<EdgeWeightType> &spanning_tree,
                               const Vertices &vertices, Vertex start_vertex,
                               Vertex end_vertex,
                               EdgeWeightType end_weight) const;

  // --------------------------------------------------------------------------
  //                              Solve Salesman
  // --------------------------------------------------------------------------

  TsmResult SolveTravelingSalesmanProblemAnt(const Graph &graph,
                                             int rand) const;

  mtlc::Matrix<WeightPheromone> CreateAdjacencyMatrixWeightPheromone(
      const Graph &graph) const;

  std::vector<Ant> GetAnts(const Graph &graph) const;

  void MakeWay(const Graph &graph, std::vector<Ant>::iterator &ant,
               const mtlc::Matrix<WeightPheromone> &weight_pheromone,
               size_t count_vertices, int rand) const;

  VertexProbility FindNextVertex(
      const Graph &graph, Vertex start, std::vector<Ant>::iterator &ant,
      const mtlc::Matrix<WeightPheromone> &weight_pheromone, int rand) const;

  std::vector<VertexProbility> GetAntsWishes(
      const Graph &graph, Vertex start, std::vector<Ant>::iterator &ant,
      const mtlc::Matrix<WeightPheromone> &weight_pheromone) const;

  VertexProbility FindNearVertex(const std::vector<VertexProbility> ants_wishes,
                                 float next_vertex) const;

  TsmResult UpdatePheromoneIntensity(
      const Graph &graph, std::vector<Ant> &ants,
      mtlc::Matrix<WeightPheromone> &weight_pheromone) const;

  // --------------------------------------------------------------------------
  //                                Utils
  // --------------------------------------------------------------------------

  std::set<Vertex> GetUnvisitedVertices(const Graph &graph) const;

  std::set<Vertex> GetUnvisitedVertices(const Graph &graph,
                                        Vertex visited) const;

  bool HasGraphVertex(const std::set<Vertex> &vertices, Vertex vertex) const;

  /// @brief Determines whether a vertex has been visited
  /// @param vertices Takes a container of unvisited vertices
  /// @return Returns true if the vertex has not been visited
  bool IsItUnvisited(Vertex vertex, const std::set<Vertex> &vertices) const;

  bool IsThereNoWay(EdgeWeightType way_weigth) const;

  bool IsThereVertex(Vertex vertex) const;

  size_t FindPos(const std::vector<Vertex> &vertexes, Vertex vertex) const;

  const static inline float kStartPheromone_ = 2.0;
  const static inline float kAlpha_ = 2.0;
  const static inline float kBetta_ = 3.0;
  const static inline float kE_ = 0.2;
  const static inline float kK_ = 7.0;
};

}  // namespace graph_cb

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_ALGORITHMS_GRAPH_ALGORITHMS_H_
