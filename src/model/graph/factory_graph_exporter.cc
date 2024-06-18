// factory graph exporter

#include "factory_graph_exporter.h"

#include "graph_exporter.h"
#include "graph_exporter_adjacency_matrix.h"
#include "graph_exporter_dot.h"

namespace graph {

FactoryGraphExporter::FactoryGraphExporter() {}

FactoryGraphExporter::~FactoryGraphExporter() {}

GraphExporter* FactoryGraphExporter::Create(ExportType type) {
  switch (type) {
    case kTypeAdjacencyMatrix:
      return new GraphExporterAdjacencyMatrix();
      break;

    case kTypeDotFormat:
      return new GraphExporterDot();
      break;

    default:
      break;
  }

  return nullptr;
}

}  // namespace graph
