#ifndef SIMPLE_NAVIGATOR_CONTROLLER_CONTROLLER_DEFAULT_H_
#define SIMPLE_NAVIGATOR_CONTROLLER_CONTROLLER_DEFAULT_H_

#include "controller.h"
#include "model/model.h"

namespace graph {

class ControllerDefault : public Controller {
 public:
  ControllerDefault(Model* model);

  virtual ~ControllerDefault();

  ReturnCode LoadGraphFromFile(const std::string& fileway) override;

  std::vector<Vertex> DepthFirstSearch(int start_vertex) override;

  std::vector<Vertex> BreadthFirstSearch(int start_vertex) override;

  EdgeWeightType GetShortestPathBetweenVertices(int vertex1,
                                                int vertex2) override;

  mtlc::Matrix<Graph::EdgeWeightType> GetShortestPathsBetweenAllVertices()
      override;

  mtlc::Matrix<Graph::EdgeWeightType> GetLeastSpanningTree() override;

  TsmResult SolveTravelingSalesmanProblem() override;

 private:
  Model* model_;
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_CONTROLLER_CONTROLLER_DEFAULT_H_
