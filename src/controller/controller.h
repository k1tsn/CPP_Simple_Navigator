#ifndef SIMPLE_NAVIGATOR_CONTROLLER_CONTROLLER_H_
#define SIMPLE_NAVIGATOR_CONTROLLER_CONTROLLER_H_

#include <string>

#include "../model/model.h"
#include "matrix.h"

namespace graph_cb {

class Controller {
 public:
  using Vertex = Model::Vertex;
  using ReturnCode = Model::ReturnCode;
  using EdgeWeightType = Model::EdgeWeightType;
  using TsmResult = Model::TsmResult;
  using WayBetweenTwo = Model::WayBetweenTwo;
  using SalesmanAlgorithms = Model::SalesmanAlgorithms;

  Controller() {}

  virtual ~Controller() {}

  virtual ReturnCode LoadGraphFromFile(const std::string& fileway) = 0;

  virtual std::vector<Vertex> DepthFirstSearch(int start_vertex) = 0;

  virtual std::vector<Vertex> BreadthFirstSearch(int start_vertex) = 0;

  virtual WayBetweenTwo GetShortestPathBetweenVertices(int vertex1,
                                                       int vertex2) = 0;

  virtual mtlc::Matrix<Graph::EdgeWeightType>
  GetShortestPathsBetweenAllVertices() = 0;

  virtual mtlc::Matrix<Graph::EdgeWeightType> GetLeastSpanningTree() = 0;

  virtual TsmResult SolveTravelingSalesmanProblem(SalesmanAlgorithms type) = 0;
};

}  // namespace graph_cb

#endif  // SIMPLE_NAVIGATOR_CONTROLLER_CONTROLLER_H_
