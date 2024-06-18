// graph exporter adjacency matrix

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_ADJACENCY_MATRIX_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_ADJACENCY_MATRIX_H_

#include <string>

#include "graph.h"
#include "graph_exporter.h"

namespace graph {

class GraphExporterAdjacencyMatrix : public GraphExporter {
 public:
  using ReturnCode = GraphExporter::ReturnCode;

  GraphExporterAdjacencyMatrix();
  virtual ~GraphExporterAdjacencyMatrix();

  /// @brief Export graph to file with type adjacency matrix.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontSave if file dont create.
  ReturnCode Export(const Graph& graph, const std::string& filename) override;

 private:
  Graph::EdgeWeightType GetWeight(const Graph::Edges& edges,
                                  Graph::Vertex vertex);
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_ADJACENCY_MATRIX_H_
