// interface graph implementation

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMP_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMP_H_

#include <vector>

#include "graph_edge.h"

namespace graph_cb {

class GraphImp {
 public:
  enum ReturnCode {
    kCodeOk,
    kCodeVertexAlreadyExits,
    kCodeVertexNotExist,
    kCodeEdgeAlreadyExist,
    kCodeEdgeNotExist,
    kCodeInvalidEdge,
  };

  using Edge = GraphEdge;
  using Vertex = Edge::Vertex;
  using EdgeWeightType = Edge::WeightType;
  using Edges = std::vector<Edge>;
  using Vertexes = std::vector<Vertex>;

  GraphImp() {}
  virtual ~GraphImp() {}

  /// @brief Add vertex.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexAlreadyExits if vertex already exist.
  virtual ReturnCode AddVertex(Vertex vertex) = 0;

  /// @brief Remove vertex.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexNotExist if vertex not exist.
  virtual ReturnCode RemoveVertex(Vertex vertex) = 0;

  /// @brief Add edge.
  /// @param vertex from witch vertex.
  /// @param edge edge.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexNotExist if vertex not exist.
  /// @return kCodeEdgeAlreadyExist if edge already exist.
  /// @return kCodeInvalidEdge if edge cant be added.
  virtual ReturnCode AddEdge(Vertex vertex, const Edge& edge) = 0;

  /// @brief Remove edge.
  /// @param vertex from witch vertex.
  /// @param edge edge.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexNotExist if vertex not exist.
  /// @return kCodeEdgeNotExist if edge not exist.
  virtual ReturnCode RemoveEdge(Vertex vertex, const Edge& edge) = 0;

  /// @brief Get edges of this vertex.
  /// @warning If vertex not exist, return Edges with size() = 0.
  virtual Edges GetEdges(Vertex vertex) const = 0;

  /// @brief Get verteces of this graph.
  virtual Vertexes GetVertexes() const = 0;

  /// @brief Clear graph.
  virtual void Clear() = 0;
};

}  // namespace graph_cb

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMP_H_
