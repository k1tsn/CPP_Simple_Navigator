#include "controller_default.h"

namespace graph_cb {

ControllerDefault::ControllerDefault(Model* model) { model_ = model; }

ControllerDefault::~ControllerDefault() {}

ControllerDefault::ReturnCode ControllerDefault::LoadGraphFromFile(
    const std::string& fileway) {
  return model_->LoadGraphFromFile(fileway);
}

std::vector<Controller::Vertex> ControllerDefault::DepthFirstSearch(
    int start_vertex) {
  return model_->DepthFirstSearch(start_vertex);
}

std::vector<Controller::Vertex> ControllerDefault::BreadthFirstSearch(
    int start_vertex) {
  return model_->BreadthFirstSearch(start_vertex);
}

Controller::WayBetweenTwo ControllerDefault::GetShortestPathBetweenVertices(
    int vertex1, int vertex2) {
  return model_->GetShortestPathBetweenVertices(vertex1, vertex2);
}

mtlc::Matrix<Graph::EdgeWeightType>
ControllerDefault::GetShortestPathsBetweenAllVertices() {
  return model_->GetShortestPathsBetweenAllVertices();
}

mtlc::Matrix<Graph::EdgeWeightType> ControllerDefault::GetLeastSpanningTree() {
  return model_->GetLeastSpanningTree();
}

Model::TsmResult ControllerDefault::SolveTravelingSalesmanProblem(
    SalesmanAlgorithms type) {
  return model_->SolveTravelingSalesmanProblem(type);
}

}  // namespace graph_cb
