// graph exporter dot

#include "graph_exporter_dot.h"

#include <string>

#include "graph.h"
#include "graph_dot/exporter.h"
#include "graph_dot/graph.h"
#include "graph_exporter.h"

namespace graph {

GraphExporterDot::GraphExporterDot() : GraphExporter() {}

GraphExporterDot::~GraphExporterDot() {}

GraphExporterDot::ReturnCode GraphExporterDot::Export(
    const Graph& graph, const std::string& filename) {
  GraphDot::Graph graph_dot = ToGraphDot(graph);
  GraphDot::Exporter exporter;
  return GraphDotExporterCodeConvert(exporter.Export(graph_dot, filename));
}

GraphDot::Graph GraphExporterDot::ToGraphDot(const Graph& graph) {
  GraphDot::Graph graph_dot;
  GraphDot::Graph::Vertexes vertexes_dot;
  GraphDot::Graph::Edges edges_dot;

  Graph::Vertexes vertexes = graph.GetVertexes();
  for (auto& i : vertexes) {
    GraphDot::Graph::Vertex vertex_dot;
    vertexes_dot.insert({vertex_dot, GraphDot::Graph::Properties()});

    Graph::Edges edges = graph.GetEdges(i);
    for (auto& j : edges) edges_dot.push_back(EdgeToEdgeDor(i, j));
  }

  graph_dot.SetOrientType(GraphDot::Graph::OrientType::kTypeOrient);
  graph_dot.SetName("GraphDot");
  graph_dot.SetEdges(edges_dot);
  graph_dot.SetVertexes(vertexes_dot);
  return graph_dot;
}

GraphDot::Graph::Vertex GraphExporterDot::VertexToVertexDot(
    const Graph::Vertex& vertex) {
  return (std::string("v_") + std::to_string(vertex));
}

GraphDot::Graph::Edge GraphExporterDot::EdgeToEdgeDor(
    const Graph::Vertex& vertex, const Graph::Edge& edge) {
  GraphDot::Graph::Edge edge_dot;
  edge_dot.vertex_ = VertexToVertexDot(vertex);
  edge_dot.dist_vertex_ = VertexToVertexDot(edge.GetDistVertex());
  std::string str_weight = std::to_string(edge.GetWeight());
  edge_dot.properties_.insert({"label", "\"" + str_weight + "\""});
  edge_dot.properties_.insert({"weight", str_weight});
  return edge_dot;
}

GraphExporterDot::ReturnCode GraphExporterDot::GraphDotExporterCodeConvert(
    GraphDot::Exporter::ReturnCode code) {
  switch (code) {
    case GraphDot::Exporter::ReturnCode::kCodeFileDontSave:
      return ReturnCode::kCodeFileDontSave;
      break;
    default:
      break;
  }
  return ReturnCode::kCodeOk;
}

}  // namespace graph
