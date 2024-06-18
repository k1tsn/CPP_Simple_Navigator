// interface graph exporter

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_H_

#include <string>

namespace graph {

class Graph;

class GraphExporter {
 public:
  enum ReturnCode {
    kCodeOk,
    kCodeFileDontSave,
  };

  GraphExporter() {}
  virtual ~GraphExporter() {}

  /// @brief Export graph to file.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontSave if file dont create.
  virtual ReturnCode Export(const Graph& graph,
                            const std::string& filename) = 0;
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_EXPORTER_H_
