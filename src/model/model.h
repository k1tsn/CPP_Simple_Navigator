#ifndef SIMPLE_NAVIGATOR_MODEL_MODEL_H_
#define SIMPLE_NAVIGATOR_MODEL_MODEL_H_

#include <string>
#include <vector>

#include "graph/graph.h"
#include "graph_algorithms/graph_algorithms.h"
#include "matrix.h"

namespace graph {

class Model {
 public:
  using Vertex = GraphAlgorithms::Vertex;
  using EdgeWeightType = GraphEdge::WeightType;
  using TsmResult = GraphAlgorithms::TsmResult;

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

  EdgeWeightType GetShortestPathBetweenVertices(int vertex1, int vertex2);

  mtlc::Matrix<Graph::EdgeWeightType> GetShortestPathsBetweenAllVertices();

  mtlc::Matrix<Graph::EdgeWeightType> GetLeastSpanningTree();

  TsmResult SolveTravelingSalesmanProblem();

 private:
  Graph graph_;
  GraphAlgorithms graph_algoritms_;
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_MODEL_H_
