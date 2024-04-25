// graph importer dot

#include "graph_importer_dot.h"

#include <string>

#include "graph.h"
#include "graph_importer.h"

namespace graph_cb {

GraphImporterDot::GraphImporterDot() : GraphImporter() {}

GraphImporterDot::~GraphImporterDot() {}

GraphImporterDot::ReturnCode GraphImporterDot::Import(
    const std::string& filename, Graph& graph) {
  (void)graph;
  (void)filename;
  return ReturnCode::kCodeOk;
}

}  // namespace graph_cb
