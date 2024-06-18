#ifndef SIMPLE_NAVIGATOR_VIEW_VIEW_H_
#define SIMPLE_NAVIGATOR_VIEW_VIEW_H_

#include <cstdint>

#include "../controller/controller.h"
#include "matrix.h"

namespace graph {

class View {
 public:
  using Vertex = Controller::Vertex;
  using ReturnCode = Controller::ReturnCode;
  using TsmResult = Controller::TsmResult;

  View(Controller* controller);
  virtual ~View();
  void Show();

 private:
  enum LoadFileType { kAdjacencyMatrix, kDot };

  enum TaskType {
    kLoadFile = 1,
    kDepthSearch,
    kBreadthSearch,
    kSearchBetweenTwoPairs,
    kSearchAllPairs,
    kLeastSpanningTree,
    kSolveSalesman,
    kEnd
  };

  int ShowCallToAction();

  void LoadFile();
  void DepthFirstSearch();
  void BreadthFirstSearch();
  void GetShortestPathBetweenVertices();
  void GetShortestPathsBetweenAllVertices();
  void GetLeastSpanningTree();
  void SolveTravelingSalesmanProblem();

  void PrintMatrix(const mtlc::Matrix<Controller::EdgeWeightType>& matrix);

  bool is_end_;
  Controller* controller_;
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_VIEW_VIEW_H_
