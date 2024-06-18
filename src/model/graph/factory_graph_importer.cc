// factory graph importer

#include "factory_graph_importer.h"

#include "graph_importer.h"
#include "graph_importer_adjacency_matrix.h"

namespace graph {

FactoryGraphImporter::FactoryGraphImporter() {}

FactoryGraphImporter::~FactoryGraphImporter() {}

GraphImporter* FactoryGraphImporter::Create(ImportType type) {
  switch (type) {
    case kTypeAdjacencyMatrix:
      return new GraphImporterAdjacencyMatrix();
      break;

    default:
      break;
  }

  return nullptr;
}

}  // namespace graph
