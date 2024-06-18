// factory graph imp

#include "factory_graph_imp.h"

#include "graph_imp.h"
#include "graph_imp_adjacency_matrix.h"

namespace graph {

FactoryGraphImp::FactoryGraphImp() {}

FactoryGraphImp::~FactoryGraphImp() {}

GraphImp* FactoryGraphImp::Create(GraphImpType type) {
  switch (type) {
    case kTypeAdjacencyMatrix:
      return new GraphImpAdjacencyMatrix();
      break;

    default:
      break;
  }

  return nullptr;
}

}  // namespace graph
