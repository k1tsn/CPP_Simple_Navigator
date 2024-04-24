// graph implementation adjacency matrix

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMP_ADJACENCY_MATRIX_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMP_ADJACENCY_MATRIX_H_

#include "graph_imp.h"
#include "matrix.h"

namespace graph_cb {

class GraphImpAdjacencyMatrix : public GraphImp {
 public:
  using ReturnCode = GraphImp::ReturnCode;
  using Edge = GraphImp::Edge;
  using Vertex = GraphImp::Vertex;
  using EdgeWeightType = GraphImp::EdgeWeightType;
  using Edges = GraphImp::Edges;
  using Vertexes = GraphImp::Vertexes;
  using AdjacencyMatrix = mtlc::Matrix<EdgeWeightType>;

  GraphImpAdjacencyMatrix();
  GraphImpAdjacencyMatrix(const GraphImpAdjacencyMatrix& other) = default;
  GraphImpAdjacencyMatrix(GraphImpAdjacencyMatrix&& other) noexcept = default;
  GraphImpAdjacencyMatrix& operator=(const GraphImpAdjacencyMatrix& other) =
      default;
  GraphImpAdjacencyMatrix& operator=(GraphImpAdjacencyMatrix&& other) noexcept =
      default;
  virtual ~GraphImpAdjacencyMatrix();

  /// @brief Add vertex.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexAlreadyExits if vertex already exist.
  ReturnCode AddVertex(Vertex vertex) override;

  /// @brief Remove vertex.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexNotExist if vertex not exist.
  ReturnCode RemoveVertex(Vertex vertex) override;

  /// @brief Add edge.
  /// @param vertex from witch vertex.
  /// @param edge edge.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexNotExist if vertex not exist.
  /// @return kCodeEdgeAlreadyExist if edge already exist.
  /// @return kCodeInvalidEdge if edge cant be added.
  ReturnCode AddEdge(Vertex vertex, const Edge& edge) override;

  /// @brief Remove edge.
  /// @param vertex from witch vertex.
  /// @param edge edge.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexNotExist if vertex not exist.
  /// @return kCodeEdgeNotExist if edge not exist.
  ReturnCode RemoveEdge(Vertex vertex, const Edge& edge) override;

  /// @brief Get edges of this vertex.
  /// @warning If vertex not exist, return Edges with size() override.
  Edges GetEdges(Vertex vertex) const override;

  /// @brief Get verteces of this graph.
  Vertexes GetVertexes() const override;

  /// @brief Clear graph.
  void Clear() override;

 private:
  Vertex IdxToVertex(AdjacencyMatrix::size_type idx) const;
  AdjacencyMatrix::size_type VertexToIdx(Vertex vertex) const;

  bool CheckVertexExists(Vertex vertex) const;
  bool CheckEdgeExists(Vertex vertex, const Edge& edge) const;

  AdjacencyMatrix adjacency_matrix_;
  std::vector<Vertex> vertex_idx_comparer_;
};

}  // namespace graph_cb

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMP_ADJACENCY_MATRIX_H_
