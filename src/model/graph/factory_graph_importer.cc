// factory graph importer

#include "factory_graph_importer.h"

#include "graph_importer.h"
#include "graph_importer_adjacency_matrix.h"
#include "graph_importer_dot.h"

namespace graph_cb {

FactoryGraphImporter::FactoryGraphImporter() {}

FactoryGraphImporter::~FactoryGraphImporter() {}

GraphImporter* FactoryGraphImporter::Create(ImportType type) {
  switch (type) {
    case kTypeAdjacencyMatrix:
      return new GraphImporterAdjacencyMatrix();
      break;

    case kTypeDotFormat:
      return new GraphImporterDot();
      break;

    default:
      break;
  }

  return nullptr;
}

}  // namespace graph_cb
