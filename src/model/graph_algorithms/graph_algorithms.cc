// graph algorithms

#include "graph_algorithms.h"

#include <limits>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <vector>

#include "../graph/graph.h"

namespace graph {

GraphAlgorithms::GraphAlgorithms() {}

GraphAlgorithms::~GraphAlgorithms() {}

std::vector<GraphAlgorithms::Vertex> GraphAlgorithms::DepthFirstSearch(
    const Graph &graph, Vertex start_vertex) const {
  std::set<Vertex> unvisited_vertex = GetUnvisitedVertices(graph);

  std::vector<Vertex> visited_vertices;

  if (!HasGraphVertex(unvisited_vertex, start_vertex)) return visited_vertices;

  std::stack<Vertex> vertex_stack;
  vertex_stack.push(start_vertex);

  while (!vertex_stack.empty()) {
    Vertex new_vertex = vertex_stack.top();
    vertex_stack.pop();
    if (IsItUnvisited(new_vertex, unvisited_vertex)) {
      visited_vertices.push_back(new_vertex);
      unvisited_vertex.erase(unvisited_vertex.find(new_vertex));
      Graph::Edges edges = graph.GetEdges(new_vertex);
      PushVertexToStack(vertex_stack, edges, unvisited_vertex);
    }
  }

  return visited_vertices;
}

std::vector<GraphAlgorithms::Vertex> GraphAlgorithms::BreadthFirstSearch(
    const Graph &graph, Vertex start_vertex) const {
  std::set<Vertex> unvisited_vertex = GetUnvisitedVertices(graph);

  std::vector<Vertex> visited_vertices;

  if (!HasGraphVertex(unvisited_vertex, start_vertex)) return visited_vertices;

  std::queue<Vertex> vertex_queue;
  vertex_queue.push(start_vertex);

  while (!vertex_queue.empty()) {
    Vertex new_vertex = vertex_queue.front();
    vertex_queue.pop();
    if (IsItUnvisited(new_vertex, unvisited_vertex)) {
      visited_vertices.push_back(new_vertex);
      unvisited_vertex.erase(unvisited_vertex.find(new_vertex));
      Graph::Edges edges = graph.GetEdges(new_vertex);
      PushVertexToQueue(vertex_queue, edges, unvisited_vertex);
    }
  }

  return visited_vertices;
}

GraphAlgorithms::EdgeWeightType GraphAlgorithms::GetShortestPathBetweenVertices(
    const Graph &graph, int vertex1, int vertex2) const {
  if (!HasGraphVertices(graph, vertex1, vertex2))
    return static_cast<Vertex>(kVertexNotFound);

  std::map<Vertex, EdgeWeightType> vertices = {{vertex1, 0}};
  PushVertex(vertices, graph, vertex1);

  std::set<Vertex> unvisited_vertices = GetUnvisitedVertices(graph);

  Vertex min = vertex1;

  while (!unvisited_vertices.empty()) {
    auto iter = unvisited_vertices.find(min);
    unvisited_vertices.erase(iter);

    auto iter_to_min = vertices.find(min);
    Graph::EdgeWeightType edge_min = iter_to_min->second;
    Graph::Edges edges = graph.GetEdges(min);

    for (const auto &edge : edges) {
      Vertex v = edge.GetDistVertex();
      auto next_v = vertices.find(v);
      EdgeWeightType w = edge.GetWeight() + edge_min;
      if (w < next_v->second) next_v->second = w;
    }

    min = FindVertexWithMinWeight(unvisited_vertices, vertices);
    if (!IsThereVertex(min)) break;
  }

  if (IsThereNoWay(vertices.find(vertex2)->second))
    return static_cast<Vertex>(kWayNotFound);
  return vertices.find(vertex2)->second;
}

mtlc::Matrix<GraphAlgorithms::EdgeWeightType>
GraphAlgorithms::GetShortestPathsBetweenAllVertices(const Graph &graph) const {
  mtlc::Matrix<EdgeWeightType> adjacency_matrix = CreateAdjacencyMatrix(graph);
  if (adjacency_matrix.GetRows() == 0) return adjacency_matrix;

  for (size_t k = 0; k < adjacency_matrix.GetRows(); ++k) {
    for (size_t i = 0; i < adjacency_matrix.GetRows(); ++i) {
      for (size_t j = 0; j < adjacency_matrix.GetCols(); ++j) {
        if (i == j) continue;
        EdgeWeightType weight_way = adjacency_matrix(i, j);
        EdgeWeightType new_weight_way_1 = adjacency_matrix(i, k);
        EdgeWeightType new_weight_way_2 = adjacency_matrix(k, j);
        if (!IsThereNoWay(new_weight_way_1) &&
            !IsThereNoWay(new_weight_way_2)) {
          if (weight_way == 0)
            adjacency_matrix(i, j) = new_weight_way_1 + new_weight_way_2;
          else
            adjacency_matrix(i, j) =
                std::min(weight_way, new_weight_way_1 + new_weight_way_2);
          weight_way = adjacency_matrix(i, j);
        }
      }
    }
  }

  return adjacency_matrix;
}

mtlc::Matrix<GraphAlgorithms::EdgeWeightType>
GraphAlgorithms::GetLeastSpanningTree(const Graph &graph) const {
  Vertices vertices = graph.GetVertexes();
  if (vertices.empty()) return mtlc::Matrix<EdgeWeightType>(0, 0);

  std::set<Vertex> unplaced_vertices =
      GetUnvisitedVertices(graph, vertices.front());

  Vertices placed_vertices = {vertices.front()};
  mtlc::Matrix<EdgeWeightType> spanning_tree(vertices.size(), vertices.size());

  bool is_linked_graph = true;
  WayWithMinWeight way;

  while (!unplaced_vertices.empty()) {
    way = FindVertexWithMinWeight(graph, placed_vertices, unplaced_vertices);

    if (!IsThereVertex(way.start_)) {
      is_linked_graph = false;
      break;
    }

    unplaced_vertices.erase(unplaced_vertices.find(way.end_));

    placed_vertices.push_back(way.end_);

    AddWeigthToSpanningTree(graph, spanning_tree, vertices, way);
  }

  if (!is_linked_graph) return mtlc::Matrix<EdgeWeightType>(0, 0);
  return spanning_tree;
}

}  // namespace graph
