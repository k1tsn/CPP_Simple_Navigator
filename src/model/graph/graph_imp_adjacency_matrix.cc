// graph implementation adjacency matrix

#include "graph_imp_adjacency_matrix.h"

namespace graph {

GraphImpAdjacencyMatrix::GraphImpAdjacencyMatrix() {}

GraphImpAdjacencyMatrix::~GraphImpAdjacencyMatrix() {}

GraphImpAdjacencyMatrix::ReturnCode GraphImpAdjacencyMatrix::AddVertex(
    Vertex vertex) {
  if (CheckVertexExists(vertex)) return kCodeVertexAlreadyExits;

  vertex_idx_comparer_.push_back(vertex);
  adjacency_matrix_.SetSize(adjacency_matrix_.GetRows() + 1,
                            adjacency_matrix_.GetCols() + 1, EdgeWeightType(0));

  return kCodeOk;
}

GraphImpAdjacencyMatrix::ReturnCode GraphImpAdjacencyMatrix::RemoveVertex(
    Vertex vertex) {
  if (!CheckVertexExists(vertex)) return kCodeVertexNotExist;

  AdjacencyMatrix::size_type mi = VertexToIdx(vertex);
  std::vector<Vertex>::size_type i =
      static_cast<std::vector<Vertex>::size_type>(mi);
  for (; i != vertex_idx_comparer_.size() - 1; ++i) {
    vertex_idx_comparer_[i] = vertex_idx_comparer_[i + 1];
  }

  adjacency_matrix_.CutMatrix(mi, mi);

  return kCodeOk;
}

GraphImpAdjacencyMatrix::ReturnCode GraphImpAdjacencyMatrix::AddEdge(
    Vertex vertex, const Edge& edge) {
  if (!CheckVertexExists(vertex)) return kCodeVertexNotExist;
  if (!CheckVertexExists(edge.GetDistVertex())) return kCodeInvalidEdge;
  if (vertex == edge.GetDistVertex()) return kCodeInvalidEdge;
  if (CheckEdgeExists(vertex, edge)) return kCodeEdgeAlreadyExist;

  AdjacencyMatrix::size_type row = VertexToIdx(vertex);
  AdjacencyMatrix::size_type col = VertexToIdx(edge.GetDistVertex());
  adjacency_matrix_(row, col) = edge.GetWeight();

  return kCodeOk;
}

GraphImpAdjacencyMatrix::ReturnCode GraphImpAdjacencyMatrix::RemoveEdge(
    Vertex vertex, const Edge& edge) {
  if (!CheckVertexExists(vertex)) return kCodeVertexNotExist;
  if (!CheckVertexExists(edge.GetDistVertex())) return kCodeEdgeNotExist;
  if (!CheckEdgeExists(vertex, edge)) return kCodeEdgeNotExist;

  AdjacencyMatrix::size_type row = VertexToIdx(vertex);
  AdjacencyMatrix::size_type col = VertexToIdx(edge.GetDistVertex());
  adjacency_matrix_(row, col) = EdgeWeightType(0);

  return kCodeOk;
}

GraphImpAdjacencyMatrix::Edges GraphImpAdjacencyMatrix::GetEdges(
    Vertex vertex) const {
  Edges edges;

  if (!CheckVertexExists(vertex)) return edges;

  AdjacencyMatrix::size_type row = VertexToIdx(vertex);
  AdjacencyMatrix::size_type col = 0;
  for (; col != adjacency_matrix_.GetCols(); ++col) {
    if (adjacency_matrix_(row, col) != EdgeWeightType(0)) {
      Edge edge(adjacency_matrix_(row, col), IdxToVertex(col));
      edges.push_back(edge);
    }
  }

  return edges;
}

GraphImpAdjacencyMatrix::Vertexes GraphImpAdjacencyMatrix::GetVertexes() const {
  return vertex_idx_comparer_;
}

void GraphImpAdjacencyMatrix::Clear() {
  vertex_idx_comparer_.clear();
  adjacency_matrix_.SetSize(0, 0);
}

GraphImpAdjacencyMatrix::Vertex GraphImpAdjacencyMatrix::IdxToVertex(
    AdjacencyMatrix::size_type idx) const {
  return vertex_idx_comparer_[idx];
}

GraphImpAdjacencyMatrix::AdjacencyMatrix::size_type
GraphImpAdjacencyMatrix::VertexToIdx(Vertex vertex) const {
  std::vector<Vertex>::size_type i = 0;
  for (; i != vertex_idx_comparer_.size(); ++i)
    if (vertex_idx_comparer_[i] == vertex) break;
  return static_cast<AdjacencyMatrix::size_type>(i);
}

bool GraphImpAdjacencyMatrix::CheckVertexExists(Vertex vertex) const {
  std::vector<Vertex>::size_type i = 0;
  for (; i != vertex_idx_comparer_.size(); ++i)
    if (vertex_idx_comparer_[i] == vertex) return true;
  return false;
}

bool GraphImpAdjacencyMatrix::CheckEdgeExists(Vertex vertex,
                                              const Edge& edge) const {
  AdjacencyMatrix::size_type row = VertexToIdx(vertex);
  AdjacencyMatrix::size_type col = VertexToIdx(edge.GetDistVertex());
  if (row >= adjacency_matrix_.GetRows()) return false;
  if (col >= adjacency_matrix_.GetCols()) return false;
  if (adjacency_matrix_(row, col) == EdgeWeightType(0)) return false;
  return true;
}

}  // namespace graph
