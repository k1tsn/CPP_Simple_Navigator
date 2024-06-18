// factory graph importer

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_IMPORTER
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_IMPORTER

#include "graph_importer.h"

namespace graph {

class FactoryGraphImporter {
 public:
  enum ImportType {
    kTypeAdjacencyMatrix,
  };

  FactoryGraphImporter();
  virtual ~FactoryGraphImporter();

  /// @brief Create new GraphImporter by type
  /// @return New allocated GraphImporter, ownership on memory on you
  virtual GraphImporter* Create(ImportType type);
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_IMPORTER
