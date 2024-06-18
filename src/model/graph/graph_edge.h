// graph edge

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EDGE_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EDGE_H_

namespace graph {

class GraphEdge {
 public:
  using Vertex = int;
  using WeightType = int;

  GraphEdge();
  GraphEdge(WeightType weight, Vertex dist_vertex);
  GraphEdge(const GraphEdge& other) = default;
  GraphEdge(GraphEdge&& other) noexcept = default;
  GraphEdge& operator=(const GraphEdge& other) = default;
  GraphEdge& operator=(GraphEdge&& other) = default;
  ~GraphEdge();

  void SetWeight(WeightType weight);
  void SetDistVertex(Vertex dist_vertex);

  WeightType GetWeight() const;
  Vertex GetDistVertex() const;

 private:
  WeightType weight_;
  Vertex dist_vertex_;
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EDGE_H_