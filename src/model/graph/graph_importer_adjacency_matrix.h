// graph importer adjacency matrix

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMPORTER_ADJACENCY_MATRIX
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMPORTER_ADJACENCY_MATRIX

#include <string>

#include "graph.h"
#include "graph_importer.h"

namespace graph {

class GraphImporterAdjacencyMatrix : public GraphImporter {
 public:
  using ReturnCode = GraphImporter::ReturnCode;

  GraphImporterAdjacencyMatrix();
  virtual ~GraphImporterAdjacencyMatrix();

  /// @brief Import graph from file with type adjacency matrix.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontOpen if file dont open.
  /// @return kCodeInvalidFile if file is invalid.
  ReturnCode Import(const std::string& filename, Graph& graph) override;

 private:
  ReturnCode FileDontOpenCatch();
  ReturnCode InvalidFileCatch(Graph& graph);
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_IMPORTER_ADJACENCY_MATRIX
