#ifndef SIMPLE_NAVIGATOR_MODEL_MODEL_H_
#define SIMPLE_NAVIGATOR_MODEL_MODEL_H_

#include <string>
#include <vector>

#include "graph/graph.h"
#include "graph_algorithms/graph_algorithms.h"
#include "matrix.h"

namespace s21 {

class Model {
 public:
  using Vertex = GraphAlgorithms::Vertex;
  using EdgeWeightType = GraphEdge::WeightType;
  using TsmResult = GraphAlgorithms::TsmResult;
  using WayBetweenTwo = GraphAlgorithms::WayBetweenTwo;
  using SalesmanAlgorithms = GraphAlgorithms::SalesmanAlgorithms;

  enum ReturnCode {
    kCodeOk,
    kCodeFileDontSave,
    kCodeFileDontOpen,
    kCodeInvalidFile,
    kCodeVertexAlreadyExits,
    kCodeVertexNotExist,
    kCodeEdgeAlreadyExist,
    kCodeEdgeNotExist,
    kCodeInvalidEdge,
  };

  Model();
  virtual ~Model();

  ReturnCode LoadGraphFromFile(const std::string& fileway);

  std::vector<Vertex> DepthFirstSearch(int start_vertex);

  std::vector<Vertex> BreadthFirstSearch(int start_vertex);

  WayBetweenTwo GetShortestPathBetweenVertices(int vertex1, int vertex2);

  mtlc::Matrix<Graph::EdgeWeightType> GetShortestPathsBetweenAllVertices();

  mtlc::Matrix<Graph::EdgeWeightType> GetLeastSpanningTree();

  TsmResult SolveTravelingSalesmanProblem(SalesmanAlgorithms type);

 private:
  Graph graph_;
  GraphAlgorithms graph_algoritms_;
};

}  // namespace s21

#endif  // SIMPLE_NAVIGATOR_MODEL_MODEL_H_
