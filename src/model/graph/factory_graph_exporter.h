// factory graph exporter

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_EXPORTER
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_EXPORTER

#include "graph_exporter.h"

namespace graph {

class FactoryGraphExporter {
 public:
  enum ExportType {
    kTypeDotFormat,
    kTypeAdjacencyMatrix,
  };

  FactoryGraphExporter();
  virtual ~FactoryGraphExporter();

  /// @brief Create new GraphExporter by type
  /// @return New allocated GraphExporter, ownership on memory on you
  virtual GraphExporter* Create(ExportType type);
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_EXPORTER
