// graph exporter dot

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_DOT_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_DOT_H_

#include <string>

#include "graph.h"
#include "graph_dot/exporter.h"
#include "graph_dot/graph.h"
#include "graph_exporter.h"

namespace graph {

class GraphExporterDot : public GraphExporter {
 public:
  using ReturnCode = GraphExporter::ReturnCode;

  GraphExporterDot();
  virtual ~GraphExporterDot();

  /// @brief Export graph to file with type dot format.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontSave if file dont create.
  ReturnCode Export(const Graph& graph, const std::string& filename) override;

 private:
  GraphDot::Graph ToGraphDot(const Graph& graph);
  GraphDot::Graph::Vertex VertexToVertexDot(const Graph::Vertex& vertex);
  GraphDot::Graph::Edge EdgeToEdgeDor(const Graph::Vertex& vertex,
                                      const Graph::Edge& edge);
  ReturnCode GraphDotExporterCodeConvert(GraphDot::Exporter::ReturnCode code);
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_DOT_H_
