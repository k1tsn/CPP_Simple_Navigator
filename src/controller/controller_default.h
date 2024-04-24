#ifndef SIMPLE_NAVIGATOR_CONTROLLER_CONTROLLER_DEFAULT_H_
#define SIMPLE_NAVIGATOR_CONTROLLER_CONTROLLER_DEFAULT_H_

#include "controller.h"
#include "model/model.h"

namespace graph_cb {

class ControllerDefault : public Controller {
 public:
  ControllerDefault(Model* model);

  virtual ~ControllerDefault();

  ReturnCode LoadGraphFromFile(const std::string& fileway) override;

  std::vector<Vertex> DepthFirstSearch(int start_vertex) override;

  std::vector<Vertex> BreadthFirstSearch(int start_vertex) override;

  WayBetweenTwo GetShortestPathBetweenVertices(int vertex1,
                                               int vertex2) override;

  mtlc::Matrix<Graph::EdgeWeightType> GetShortestPathsBetweenAllVertices()
      override;

  mtlc::Matrix<Graph::EdgeWeightType> GetLeastSpanningTree() override;

  TsmResult SolveTravelingSalesmanProblem(SalesmanAlgorithms type) override;

 private:
  Model* model_;
};

}  // namespace graph_cb

#endif  // SIMPLE_NAVIGATOR_CONTROLLER_CONTROLLER_DEFAULT_H_
