// graph exporter adjacency matrix

#include "graph_exporter_adjacency_matrix.h"

#include <fstream>
#include <string>

#include "graph_exporter.h"
#include "s21_graph.h"

namespace graph_cb {

GraphExporterAdjacencyMatrix::GraphExporterAdjacencyMatrix()
    : GraphExporter() {}

GraphExporterAdjacencyMatrix::~GraphExporterAdjacencyMatrix() {}

GraphExporterAdjacencyMatrix::ReturnCode GraphExporterAdjacencyMatrix::Export(
    const Graph& graph, const std::string& filename) {
  std::ofstream file(filename);
  if (file.fail()) return ReturnCode::kCodeFileDontSave;

  Graph::Vertexes vertexes = graph.GetVertexes();
  file << std::to_string(vertexes.size()) << "\n";

  for (auto i : vertexes) {
    Graph::Edges edges = graph.GetEdges(i);
    for (auto j : vertexes) {
      // Graph::EdgeWeightType weight = GetWeight(edges, j);
      file << GetWeight(edges, j) << " ";
    }
    file << "\n";
  }

  return kCodeOk;
}

Graph::EdgeWeightType GraphExporterAdjacencyMatrix::GetWeight(
    const Graph::Edges& edges, Graph::Vertex vertex) {
  Graph::EdgeWeightType weight = Graph::EdgeWeightType(0);
  for (auto i : edges) {
    if (i.GetDistVertex() == vertex) weight = i.GetWeight();
  }
  return weight;
}

}  // namespace graph_cb
