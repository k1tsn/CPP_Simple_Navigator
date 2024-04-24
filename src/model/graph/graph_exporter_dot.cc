// graph exporter dot

#include "graph_exporter_dot.h"

#include <string>

#include "graph_exporter.h"
#include "s21_graph.h"

namespace graph_cb {

GraphExporterDot::GraphExporterDot() : GraphExporter() {}

GraphExporterDot::~GraphExporterDot() {}

GraphExporterDot::ReturnCode GraphExporterDot::Export(
    const Graph& graph, const std::string& filename) {
  (void)graph;
  (void)filename;
  return ReturnCode::kCodeOk;
}

}  // namespace graph_cb
