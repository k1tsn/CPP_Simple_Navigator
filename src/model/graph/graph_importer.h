// interface graph importer

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMPORTER
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMPORTER

#include <string>

namespace graph {

class Graph;

class GraphImporter {
 public:
  enum ReturnCode {
    kCodeOk,
    kCodeFileDontOpen,
    kCodeInvalidFile,
  };

  GraphImporter() {}
  virtual ~GraphImporter() {}

  /// @brief Import graph from file.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontOpen if file dont open.
  /// @return kCodeInvalidFile if file is invalid.
  virtual ReturnCode Import(const std::string& filename, Graph& graph) = 0;
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMPORTER
