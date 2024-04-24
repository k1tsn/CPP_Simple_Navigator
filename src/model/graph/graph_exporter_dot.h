// graph exporter dot

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_DOT_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_DOT_H_

#include <string>

#include "graph_exporter.h"
#include "s21_graph.h"

namespace graph_cb {

class GraphExporterDot : public GraphExporter {
 public:
  using ReturnCode = GraphExporter::ReturnCode;

  GraphExporterDot();
  virtual ~GraphExporterDot();

  /// @brief Export graph to file with type dot format.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontSave if file dont create.
  ReturnCode Export(const Graph& graph, const std::string& filename) override;
};

}  // namespace graph_cb

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_DOT_H_
