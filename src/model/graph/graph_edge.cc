// graph edge

#include "graph_edge.h"

namespace graph_cb {

GraphEdge::GraphEdge() {}

GraphEdge::GraphEdge(WeightType weight, Vertex dist_vertex)
    : weight_(weight), dist_vertex_(dist_vertex) {}

GraphEdge::~GraphEdge() {}

void GraphEdge::SetWeight(WeightType weight) { weight_ = weight; }

void GraphEdge::SetDistVertex(Vertex dist_vertex) {
  dist_vertex_ = dist_vertex;
}

GraphEdge::WeightType GraphEdge::GetWeight() const { return weight_; }

GraphEdge::Vertex GraphEdge::GetDistVertex() const { return dist_vertex_; }

}  // namespace graph_cb
