#include "model.h"

#include <string>

namespace graph_cb {

Model::Model() {}

Model::~Model() {}

Model::ReturnCode Model::LoadGraphFromFile(const std::string& filename) {
  Graph::ReturnCode code = graph_.LoadGraphFromFile(filename);

  switch (code) {
    case Graph::ReturnCode::kCodeFileDontOpen:
      return Model::ReturnCode::kCodeFileDontOpen;
      break;

    case Graph::ReturnCode::kCodeInvalidFile:
      return Model::ReturnCode::kCodeInvalidFile;
      break;

    default:
      break;
  }

  return Model::ReturnCode::kCodeOk;
}

std::vector<Model::Vertex> Model::DepthFirstSearch(int start_vertex) {
  return graph_algoritms_.DepthFirstSearch(graph_, start_vertex);
}

std::vector<Model::Vertex> Model::BreadthFirstSearch(int start_vertex) {
  return graph_algoritms_.BreadthFirstSearch(graph_, start_vertex);
}

Model::WayBetweenTwo Model::GetShortestPathBetweenVertices(int vertex1,
                                                           int vertex2) {
  return graph_algoritms_.GetShortestPathBetweenVertices(graph_, vertex1,
                                                         vertex2);
}

mtlc::Matrix<Graph::EdgeWeightType>
Model::GetShortestPathsBetweenAllVertices() {
  return graph_algoritms_.GetShortestPathsBetweenAllVertices(graph_);
}

mtlc::Matrix<Graph::EdgeWeightType> Model::GetLeastSpanningTree() {
  return graph_algoritms_.GetLeastSpanningTree(graph_);
}

GraphAlgorithms::TsmResult Model::SolveTravelingSalesmanProblem(
    SalesmanAlgorithms type, int rand) {
  return graph_algoritms_.SolveTravelingSalesmanProblem(graph_, type, rand);
}

}  // namespace graph_cb