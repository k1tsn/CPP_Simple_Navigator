#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../../../src/model/graph/graph.h"
#include "../../../src/model/graph_algorithms/graph_algorithms.h"
#include "matrix.h"

struct ShortestPathBetweenVerticesDataTest {
  std::string filename_;
  graph::Graph::Vertex vertex_1_;
  graph::Graph::Vertex vertex_2_;
  graph::GraphAlgorithms::EdgeWeightType way_;
};

class ShortestPathBetweenVerticesTest : public testing::TestWithParam<int> {
 public:
  static int count_;
  static std::vector<ShortestPathBetweenVerticesDataTest> test_data_;
};

int ShortestPathBetweenVerticesTest::count_ = 11;
std::vector<ShortestPathBetweenVerticesDataTest>
    ShortestPathBetweenVerticesTest::test_data_ = {
        ShortestPathBetweenVerticesDataTest{"graphs/6.txt", 101, 57, 286},
        ShortestPathBetweenVerticesDataTest{"graphs/6.txt", 17, 70, 244},
        ShortestPathBetweenVerticesDataTest{"graphs/6.txt", 25, 84, 318},
        ShortestPathBetweenVerticesDataTest{"graphs/6.txt", 21, 60, 278},
        ShortestPathBetweenVerticesDataTest{"graphs/6.txt", 85, 62, 272},
        ShortestPathBetweenVerticesDataTest{"graphs/7.txt", 11, 40, 84},
        ShortestPathBetweenVerticesDataTest{"graphs/7.txt", 30, 3, 124},
        ShortestPathBetweenVerticesDataTest{
            "graphs/7.txt", 11, 47, graph::GraphAlgorithms::kWayNotFound},
        ShortestPathBetweenVerticesDataTest{"graphs/7.txt", 40, 24, 133},
        ShortestPathBetweenVerticesDataTest{"graphs/9.txt", 14, 2, 30},
        ShortestPathBetweenVerticesDataTest{"graphs/9.txt", 9, 12, 23}};

INSTANTIATE_TEST_SUITE_P(
    shortest_path_between_vertices, ShortestPathBetweenVerticesTest,
    testing::Range(0, ShortestPathBetweenVerticesTest::count_));

TEST_P(ShortestPathBetweenVerticesTest, Common) {
  int num_test = this->GetParam();
  ShortestPathBetweenVerticesDataTest& data =
      ShortestPathBetweenVerticesTest::test_data_[num_test];

  graph::Graph graph;
  graph.LoadGraphFromFile(data.filename_);

  graph::GraphAlgorithms alg;

  graph::GraphAlgorithms::EdgeWeightType way =
      alg.GetShortestPathBetweenVertices(graph, data.vertex_1_, data.vertex_2_);

  EXPECT_EQ(way, data.way_);
}
