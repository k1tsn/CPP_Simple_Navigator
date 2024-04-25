#include <cmath>
#include <ctime>

#include "graph_algorithms.h"

namespace graph_cb {

GraphAlgorithms::TsmResult GraphAlgorithms::SolveTravelingSalesmanProblemAnt(
    const Graph &graph, int rand) const {
  mtlc::Matrix<WeightPheromone> weight_pheromone =
      CreateAdjacencyMatrixWeightPheromone(graph);

  TsmResult res = {{}, kWayNotFound};

  std::vector<Ant> ants = GetAnts(graph);
  size_t count_vertices = ants.size();

  for (size_t i = 0; i < count_vertices; ++i) {
    for (auto iter = ants.begin(); iter != ants.end(); ++iter) {
      MakeWay(graph, iter, weight_pheromone, count_vertices, rand);
    }
    TsmResult best_path =
        UpdatePheromoneIntensity(graph, ants, weight_pheromone);

    if (res.distance > best_path.distance || res.distance == kWayNotFound) {
      res.vertices = best_path.vertices;
      res.distance = best_path.distance;
    }

    ants = GetAnts(graph);
  }

  return res;
}

mtlc::Matrix<GraphAlgorithms::WeightPheromone>
GraphAlgorithms::CreateAdjacencyMatrixWeightPheromone(
    const Graph &graph) const {
  Vertices vertices = graph.GetVertexes();

  mtlc::Matrix<WeightPheromone> adjacency_matrix(
      vertices.size(), vertices.size(), WeightPheromone{});

  for (size_t i = 0; i < adjacency_matrix.GetCols(); ++i) {
    std::vector<Graph::Edge> edges = graph.GetEdges(vertices[i]);
    std::map<Vertex, EdgeWeightType> near_vertices = NearVertices(edges);

    for (size_t j = 0; j < adjacency_matrix.GetRows(); ++j) {
      auto iter = near_vertices.find(vertices[j]);
      if (iter != near_vertices.end())
        adjacency_matrix(i, j).weight_ = iter->second;
      adjacency_matrix(i, j).pheromone_ = kStartPheromone_;
    }
  }

  return adjacency_matrix;
}

std::vector<GraphAlgorithms::Ant> GraphAlgorithms::GetAnts(
    const Graph &graph) const {
  std::vector<Ant> ants;
  Vertices vertices = graph.GetVertexes();
  for (auto iter = vertices.begin(); iter != vertices.end(); ++iter) {
    ants.push_back({*iter, {*iter}, GetUnvisitedVertices(graph, *iter), 0});
  }
  return ants;
}

void GraphAlgorithms::MakeWay(
    const Graph &graph, std::vector<Ant>::iterator &ant,
    const mtlc::Matrix<WeightPheromone> &weight_pheromone,
    size_t count_vertices, int rand) const {
  for (size_t i = 0; i < count_vertices; ++i) {
    if (i == count_vertices - 1)
      ant->unvisited_vertices_.insert(ant->visited_vertices_.front());
    Vertex start = ant->visited_vertices_.back();
    VertexProbility end =
        FindNextVertex(graph, start, ant, weight_pheromone, rand);
    if (!IsThereVertex(end.vertex_)) break;
    ant->visited_vertices_.push_back(end.vertex_);
    ant->unvisited_vertices_.erase(ant->unvisited_vertices_.find(end.vertex_));
    ant->path_ += end.weight_;
  }
}

GraphAlgorithms::VertexProbility GraphAlgorithms::FindNextVertex(
    const Graph &graph, Vertex start, std::vector<Ant>::iterator &ant,
    const mtlc::Matrix<WeightPheromone> &weight_pheromone, int rand_num) const {
  std::vector<VertexProbility> ants_wishes =
      GetAntsWishes(graph, start, ant, weight_pheromone);

  if (ants_wishes.size() == 0)
    return {VertexProbility{GraphAlgoritmsError::kVertexNotFound, 0, 0}};

  float sum_wishes = 0;
  for (size_t i = 0; i < ants_wishes.size(); ++i)
    sum_wishes += ants_wishes[i].probility_;

  for (size_t i = 0; i < ants_wishes.size(); ++i) {
    ants_wishes[i].probility_ /= sum_wishes;
    if (i > 0) ants_wishes[i].probility_ += ants_wishes[i - 1].probility_;
  }

  std::srand(std::time(nullptr));
  if (rand_num != 0) std::srand(rand_num);

  float next_probility = rand() % 100 / 100.0;
  return FindNearVertex(ants_wishes, next_probility);
}

std::vector<GraphAlgorithms::VertexProbility> GraphAlgorithms::GetAntsWishes(
    const Graph &graph, Vertex start, std::vector<Ant>::iterator &ant,
    const mtlc::Matrix<WeightPheromone> &weight_pheromone) const {
  std::vector<VertexProbility> ants_wishes;
  Graph::Edges edges = graph.GetEdges(start);
  Vertices vertices = graph.GetVertexes();
  for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
    Vertex end = iter->GetDistVertex();
    if (IsItUnvisited(end, ant->unvisited_vertices_)) {
      int pos_start = FindPos(vertices, start);
      int pos_end = FindPos(vertices, end);
      EdgeWeightType weight = weight_pheromone(pos_start, pos_end).weight_;
      float pheromone = weight_pheromone(pos_start, pos_end).pheromone_;
      float wish =
          std::pow(pheromone, kAlpha_) + 1.0 / std::pow(weight, kBetta_);
      ants_wishes.push_back(VertexProbility{end, wish, iter->GetWeight()});
    }
  }
  return ants_wishes;
}

GraphAlgorithms::VertexProbility GraphAlgorithms::FindNearVertex(
    const std::vector<VertexProbility> ants_wishes,
    float next_probility) const {
  if (ants_wishes.size() == 1) return ants_wishes[0];

  int start_pos = ants_wishes.size() / 2;
  int end_pos = ants_wishes.size() - 1;

  while (start_pos != end_pos) {
    if (next_probility > ants_wishes[start_pos].probility_) {
      ++start_pos;
    } else {
      end_pos = start_pos;
      start_pos = end_pos / 2;
    }
  }

  if (start_pos == 0) return ants_wishes[0];
  return ants_wishes[start_pos - 1];
}

GraphAlgorithms::TsmResult GraphAlgorithms::UpdatePheromoneIntensity(
    const Graph &graph, std::vector<Ant> &ants,
    mtlc::Matrix<WeightPheromone> &weight_pheromone) const {
  TsmResult best_path = {{}, -1};

  Vertices vertices = graph.GetVertexes();
  for (size_t i = 0; i < weight_pheromone.GetRows(); ++i) {
    for (size_t j = 0; j < weight_pheromone.GetCols(); ++j) {
      weight_pheromone(i, j).pheromone_ *= kE_;
    }
  }

  for (size_t i = 0; i < ants.size(); ++i) {
    if (ants[i].visited_vertices_.front() == ants[i].visited_vertices_.back()) {
      float delayed_pheromone = kK_ / ants[i].path_;
      for (size_t j = 1; j < ants[i].visited_vertices_.size(); ++j) {
        int pos_start = FindPos(vertices, ants[i].visited_vertices_[j - 1]);
        int pos_end = FindPos(vertices, ants[i].visited_vertices_[j]);
        weight_pheromone(pos_start, pos_end).pheromone_ += delayed_pheromone;
      }
      if (best_path.distance > ants[i].path_ || best_path.distance == -1) {
        best_path.vertices = ants[i].visited_vertices_;
        best_path.distance = ants[i].path_;
      }
    }
  }

  return best_path;
}

}  // namespace graph_cb
