#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <vector>

#include "../graph/graph.h"
#include "graph_algorithms.h"

namespace graph_cb {

std::set<GraphAlgorithms::Vertex> GraphAlgorithms::GetUnvisitedVertices(
    const Graph &graph) const {
  std::vector<Vertex> vertices = graph.GetVertexes();
  std::set<Vertex> unvisited_vertices;
  for (auto iter = vertices.begin(); iter != vertices.end(); ++iter) {
    unvisited_vertices.insert(*iter);
  }
  return unvisited_vertices;
}

std::set<GraphAlgorithms::Vertex> GraphAlgorithms::GetUnvisitedVertices(
    const Graph &graph, Vertex visited) const {
  std::vector<Vertex> vertices = graph.GetVertexes();
  std::set<Vertex> unvisited_vertices;
  for (auto iter = vertices.begin(); iter != vertices.end(); ++iter) {
    if (*iter == visited) continue;
    unvisited_vertices.insert(*iter);
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
  Graph::Vertexes vertexes = graph.GetVertexes();
  bool vertex_1_found = false;
  bool vertex_2_found = false;
  for (auto iter = vertexes.begin(); iter != vertexes.end(); ++iter) {
    if (*iter == vertex1) {
      vertex_1_found = true;
      if (vertex_2_found) return true;
    } else if (*iter == vertex2) {
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
  for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
    Vertex vertex = iter->GetDistVertex();
    if (IsItUnvisited(vertex, unvisited_vertices)) vertex_queue.push(vertex);
  }
}

void GraphAlgorithms::PushVertex(
    std::map<Vertex, EdgeWeightType> &visited_vertexes, const Graph &graph,
    Vertex start) const {
  std::vector<Vertex> vertexes = graph.GetVertexes();
  while (!vertexes.empty()) {
    Vertex v = vertexes.back();
    if (v != start) {
      visited_vertexes.insert({v, std::numeric_limits<EdgeWeightType>::max()});
    }
    vertexes.pop_back();
  }
}

bool GraphAlgorithms::IsThereNoWay(EdgeWeightType way_weigth) const {
  return way_weigth == 0 ||
         way_weigth == std::numeric_limits<EdgeWeightType>::max();
}

GraphAlgorithms::WayBetweenTwo GraphAlgorithms::FindWay(
    const Graph &graph, const std::map<Vertex, EdgeWeightType> &vertex_point,
    int vertex1, int vertex2) const {
  if (IsThereNoWay(vertex_point.find(vertex2)->second))
    return {{}, kWayNotFound};

  std::stack<EdgeWeightType> return_trip;
  return_trip.push(vertex2);

  mtlc::Matrix<EdgeWeightType> adjacency_matrix = CreateAdjacencyMatrix(graph);

  Vertices vertices = graph.GetVertexes();

  Vertex prev_vertex = vertex2;

  while (prev_vertex != vertex1) {
    prev_vertex =
        FindPrevVertex(prev_vertex, vertex_point, vertices, adjacency_matrix);
    return_trip.push(prev_vertex);
  }

  std::vector<EdgeWeightType> res_way;
  while (!return_trip.empty()) {
    res_way.push_back(return_trip.top());
    return_trip.pop();
  }

  return {res_way, static_cast<double>(vertex_point.find(vertex2)->second)};
}

GraphAlgorithms::Vertex GraphAlgorithms::FindPrevVertex(
    Vertex this_vertex, const std::map<Vertex, EdgeWeightType> &vertex_point,
    const Vertices &vertices,
    const mtlc::Matrix<EdgeWeightType> &adjacency_matrix) const {
  size_t pos = FindPos(vertices, this_vertex);
  int point = vertex_point.find(this_vertex)->second;

  for (size_t j = 0; j < adjacency_matrix.GetCols(); ++j) {
    Vertex prev_vertex = vertices[j];
    EdgeWeightType prev_vertex_edge = adjacency_matrix(pos, j);

    if (prev_vertex_edge == 0) continue;

    int prev_vertex_point = vertex_point.find(prev_vertex)->second;

    if (point - prev_vertex_edge == prev_vertex_point) return prev_vertex;
  }

  return kVertexNotFound;
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
  for (size_t i = 0; i < edges.size(); ++i) {
    auto iter = near_vertexes.find(edges[i].GetDistVertex());
    if (iter == near_vertexes.end())
      near_vertexes.insert({edges[i].GetDistVertex(), edges[i].GetWeight()});
    else {
      if (edges[i].GetWeight() < iter->second)
        iter->second = edges[i].GetWeight();
    }
  }
  return near_vertexes;
}

GraphAlgorithms::Vertex GraphAlgorithms::FindVertexWithMinWeight(
    const std::set<Vertex> &unvisited_vertices,
    const std::map<Vertex, EdgeWeightType> &vertices) const {
  Vertex v = *unvisited_vertices.begin();
  EdgeWeightType point = vertices.find(v)->second;

  for (auto iter = unvisited_vertices.begin(); iter != unvisited_vertices.end();
       ++iter) {
    if (point > vertices.find(*iter)->second) {
      v = *iter;
      point = vertices.find(*iter)->second;
    }
  }

  if (IsThereNoWay(point)) return kVertexNotFound;
  return v;
}

GraphAlgorithms::EdgeWeightType GraphAlgorithms::FindVertexWithMinWeight(
    const Graph &graph, const Vertices &placed_vertices,
    const std::set<Vertex> &unplaced_vertex, Vertex &start, Vertex &end) const {
  start = -1;
  EdgeWeightType edge = 0;
  for (size_t i = 0; i < placed_vertices.size(); ++i) {
    Graph::Edges edges = graph.GetEdges(placed_vertices[i]);
    for (size_t j = 0; j < edges.size(); ++j) {
      Vertex new_vertex = edges[j].GetDistVertex();
      if (!IsItUnvisited(new_vertex, unplaced_vertex)) continue;
      EdgeWeightType new_e = edges[j].GetWeight();
      if (new_e < edge || start == -1) {
        start = placed_vertices[i];
        end = new_vertex;
        edge = new_e;
      }
    }
  }

  return edge;
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
    const Vertices &vertices, Vertex start_vertex, Vertex end_vertex,
    EdgeWeightType end_weight) const {
  size_t start = FindPos(vertices, start_vertex);
  size_t end = FindPos(vertices, end_vertex);
  spanning_tree(start, end) = end_weight;

  Graph::Edges edges = graph.GetEdges(end_vertex);
  for (size_t i = 0; i < edges.size(); ++i) {
    if (edges[i].GetDistVertex() == start_vertex) {
      spanning_tree(end, start) = end_weight;
    }
  }
}

}  // namespace graph_cb
