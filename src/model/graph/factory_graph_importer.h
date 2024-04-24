// factory graph importer

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_IMPORTER
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_IMPORTER

#include "graph_importer.h"

namespace graph_cb {

class FactoryGraphImporter {
 public:
  enum ImportType {
    kTypeDotFormat,
    kTypeAdjacencyMatrix,
  };

  FactoryGraphImporter();
  virtual ~FactoryGraphImporter();

  /// @brief Create new GraphImporter by type
  /// @return New allocated GraphImporter, ownership on memory on you
  virtual GraphImporter* Create(ImportType type);
};

}  // namespace graph_cb

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_IMPORTER
