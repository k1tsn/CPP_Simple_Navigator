// factory graph imp

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_IMP_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_IMP_H_

#include "graph_imp.h"

namespace graph {

class FactoryGraphImp {
 public:
  enum GraphImpType {
    kTypeAdjacencyMatrix,
  };

  FactoryGraphImp();
  virtual ~FactoryGraphImp();

  /// @brief Create new GraphImp by type
  /// @return New allocated GraphImp, ownership on memory on you
  virtual GraphImp* Create(GraphImpType type);
};

}  // namespace graph

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_FACTORY_GRAPH_IMP_H_
