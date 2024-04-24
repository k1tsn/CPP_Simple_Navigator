// graph importer dot

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMPORTER_DOT
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMPORTER_DOT

#include <string>

#include "graph_importer.h"
#include "s21_graph.h"

namespace graph_cb {

class GraphImporterDot : public GraphImporter {
 public:
  using ReturnCode = GraphImporter::ReturnCode;

  GraphImporterDot();
  virtual ~GraphImporterDot();

  /// @brief Import graph from file with type dot format.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontOpen if file dont open.
  /// @return kCodeInvalidFile if file is invalid.
  ReturnCode Import(const std::string& filename, Graph& graph) override;
};

}  // namespace graph_cb

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMPORTER_DOT
