#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <vector>

#include "../graph/graph.h"
#include "graph_algorithms.h"

namespace graph {

std::set<GraphAlgorithms::Vertex> GraphAlgorithms::GetUnvisitedVertices(
    const Graph &graph) const {
  std::vector<Vertex> vertices = graph.GetVertexes();
  std::set<Vertex> unvisited_vertices;
  for (const auto &vertex : vertices) {
    unvisited_vertices.insert(vertex);
  }
  return unvisited_vertices;
}

std::set<GraphAlgorithms::Vertex> GraphAlgorithms::GetUnvisitedVertices(
    const Graph &graph, Vertex visited_vertex) const {
  std::vector<Vertex> vertices = graph.GetVertexes();
  std::set<Vertex> unvisited_vertices;
  for (const auto &vertex : vertices) {
    if (vertex == visited_vertex) continue;
    unvisited_vertices.insert(vertex);
  }
  return unvisited_vertices;
}

bool GraphAlgorithms::HasGraphVertex(const std::set<Vertex> &vertices,
                                     Vertex vertex) const {
  return vertices.find(vertex) != vertices.end();
}

bool GraphAlgorithms::IsItUnvisited(Vertex vertex,
                                    const std::set<Vertex> &vertices) const {
  return vertices.find(vertex) != vertices.end();
}

bool GraphAlgorithms::HasGraphVertices(const Graph &graph, int vertex1,
                                       int vertex2) const {
  Graph::Vertexes vertices = graph.GetVertexes();
  bool vertex_1_found = false;
  bool vertex_2_found = false;
  for (const auto &vertex : vertices) {
    if (vertex == vertex1) {
      vertex_1_found = true;
      if (vertex_2_found) return true;
    } else if (vertex == vertex2) {
      vertex_2_found = true;
      if (vertex_1_found) return true;
    }
  }
  return false;
}

void GraphAlgorithms::PushVertexToStack(
    std::stack<Vertex> &vertex_stack, Graph::Edges &edges,
    const std::set<Vertex> &unvisited_vertices) const {
  while (!edges.empty()) {
    Vertex vertex = edges.back().GetDistVertex();
    if (IsItUnvisited(vertex, unvisited_vertices)) vertex_stack.push(vertex);
    edges.pop_back();
  }
}

void GraphAlgorithms::PushVertexToQueue(
    std::queue<Vertex> &vertex_queue, Graph::Edges &edges,
    const std::set<Vertex> &unvisited_vertices) const {
  for (const auto &edge : edges) {
    Vertex vertex = edge.GetDistVertex();
    if (IsItUnvisited(vertex, unvisited_vertices)) vertex_queue.push(vertex);
  }
}

void GraphAlgorithms::PushVertex(
    std::map<Vertex, EdgeWeightType> &visited_vertices, const Graph &graph,
    Vertex start) const {
  Vertices vertices = graph.GetVertexes();
  for (const auto &vertex : vertices) {
    if (vertex != start)
      visited_vertices.insert(
          {vertex, std::numeric_limits<EdgeWeightType>::max()});
  }
}

bool GraphAlgorithms::IsThereNoWay(EdgeWeightType way_weigth) const {
  return way_weigth == 0 ||
         way_weigth == std::numeric_limits<EdgeWeightType>::max();
}

mtlc::Matrix<GraphAlgorithms::EdgeWeightType>
GraphAlgorithms::CreateAdjacencyMatrix(const Graph &graph) const {
  Vertices vertices = graph.GetVertexes();

  mtlc::Matrix<EdgeWeightType> adjacency_matrix(vertices.size(),
                                                vertices.size());

  for (size_t i = 0; i < adjacency_matrix.GetCols(); ++i) {
    std::vector<Graph::Edge> edges = graph.GetEdges(vertices[i]);
    std::map<Vertex, EdgeWeightType> near_vertices = NearVertices(edges);

    for (size_t j = 0; j < adjacency_matrix.GetRows(); ++j) {
      auto iter = near_vertices.find(vertices[j]);
      if (iter != near_vertices.end()) adjacency_matrix(i, j) = iter->second;
    }
  }

  return adjacency_matrix;
}

std::map<GraphAlgorithms::Vertex, GraphAlgorithms::EdgeWeightType>
GraphAlgorithms::NearVertices(const std::vector<Graph::Edge> &edges) const {
  std::map<Vertex, EdgeWeightType> near_vertexes;
  for (const auto &edge : edges) {
    auto iter = near_vertexes.find(edge.GetDistVertex());
    if (iter == near_vertexes.end())
      near_vertexes.insert({edge.GetDistVertex(), edge.GetWeight()});
    else {
      if (edge.GetWeight() < iter->second) iter->second = edge.GetWeight();
    }
  }
  return near_vertexes;
}

GraphAlgorithms::Vertex GraphAlgorithms::FindVertexWithMinWeight(
    const std::set<Vertex> &unvisited_vertices,
    const std::map<Vertex, EdgeWeightType> &vertices) const {
  Vertex vertex = *unvisited_vertices.begin();
  EdgeWeightType point = vertices.find(vertex)->second;

  for (const auto &unvis_vertex : unvisited_vertices) {
    if (point > vertices.find(unvis_vertex)->second) {
      vertex = unvis_vertex;
      point = vertices.find(unvis_vertex)->second;
    }
  }

  if (IsThereNoWay(point)) return kVertexNotFound;
  return vertex;
}

GraphAlgorithms::WayWithMinWeight GraphAlgorithms::FindVertexWithMinWeight(
    const Graph &graph, const Vertices &placed_vertices,
    const std::set<Vertex> &unplaced_vertex) const {
  WayWithMinWeight way;
  way.start_ = -1;
  way.edge_ = 0;
  for (const auto &placed_vertex : placed_vertices) {
    Graph::Edges edges = graph.GetEdges(placed_vertex);
    for (const auto &edge : edges) {
      Vertex new_vertex = edge.GetDistVertex();
      if (!IsItUnvisited(new_vertex, unplaced_vertex)) continue;
      EdgeWeightType new_e = edge.GetWeight();
      if (new_e < way.edge_ || way.start_ == -1) {
        way.start_ = placed_vertex;
        way.end_ = new_vertex;
        way.edge_ = new_e;
      }
    }
  }

  return way;
}

size_t GraphAlgorithms::FindPos(const std::vector<Vertex> &vertexes,
                                Vertex vertex) const {
  size_t pos = 0;
  for (; pos < vertexes.size(); ++pos)
    if (vertex == vertexes[pos]) break;
  return pos;
}

bool GraphAlgorithms::IsThereVertex(Vertex vertex) const {
  return vertex != GraphAlgoritmsError::kVertexNotFound;
}

void GraphAlgorithms::AddWeigthToSpanningTree(
    const Graph &graph, mtlc::Matrix<EdgeWeightType> &spanning_tree,
    const Vertices &vertices, const WayWithMinWeight &way) const {
  size_t start = FindPos(vertices, way.start_);
  size_t end = FindPos(vertices, way.end_);
  spanning_tree(start, end) = way.edge_;

  Graph::Edges edges = graph.GetEdges(way.end_);
  for (const auto &edge : edges) {
    if (edge.GetDistVertex() == way.start_) {
      spanning_tree(end, start) = way.edge_;
    }
  }
}

}  // namespace graph
