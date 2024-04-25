#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../../../src/model/graph/graph.h"
#include "../../../src/model/graph_algorithms/graph_algorithms.h"
#include "matrix.h"

struct ShortestPathBetweenVerticesDataTest {
  std::string filename_;
  graph_cb::Graph::Vertex vertex_1_;
  graph_cb::Graph::Vertex vertex_2_;
  graph_cb::GraphAlgorithms::WayBetweenTwo way_;
};

class ShortestPathBetweenVerticesTest : public testing::TestWithParam<int> {
 public:
  static int count_;
  static std::vector<ShortestPathBetweenVerticesDataTest> test_data_;
};

int ShortestPathBetweenVerticesTest::count_ = 11;
std::vector<ShortestPathBetweenVerticesDataTest>
    ShortestPathBetweenVerticesTest::test_data_ = {
        ShortestPathBetweenVerticesDataTest{
            "graphs/6.txt", 101, 57, {{101, 78, 14, 4, 1, 3, 9, 49, 57}, 286}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/6.txt", 17, 70, {{17, 15, 5, 2, 1, 4, 12, 70}, 244}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/6.txt", 25, 84, {{25, 19, 6, 2, 1, 4, 11, 69, 84}, 318}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/6.txt", 21, 60, {{21, 18, 6, 2, 1, 3, 9, 50, 60}, 278}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/6.txt", 85, 62, {{85, 70, 12, 4, 1, 3, 10, 51, 62}, 272}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/7.txt", 11, 40, {{11, 4, 6, 7, 5, 18, 26, 40}, 84}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/7.txt",
            30,
            3,
            {{30, 28, 29, 33, 23, 21, 9, 10, 2, 19, 3}, 124}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/7.txt",
            11,
            47,
            {{}, graph_cb::GraphAlgorithms::kWayNotFound}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/7.txt",
            40,
            24,
            {{40, 26, 18, 5, 7, 6, 2, 10, 9, 21, 24}, 133}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/9.txt", 14, 2, {{14, 13, 10, 7, 4, 2}, 30}},
        ShortestPathBetweenVerticesDataTest{
            "graphs/9.txt", 9, 12, {{9, 11, 10, 12}, 23}}};

INSTANTIATE_TEST_SUITE_P(
    shortest_path_between_vertices, ShortestPathBetweenVerticesTest,
    testing::Range(0, ShortestPathBetweenVerticesTest::count_));

TEST_P(ShortestPathBetweenVerticesTest, Common) {
  int num_test = this->GetParam();
  ShortestPathBetweenVerticesDataTest& data =
      ShortestPathBetweenVerticesTest::test_data_[num_test];

  graph_cb::Graph graph;
  graph.LoadGraphFromFile(data.filename_);

  graph_cb::GraphAlgorithms alg;

  graph_cb::GraphAlgorithms::WayBetweenTwo way =
      alg.GetShortestPathBetweenVertices(graph, data.vertex_1_, data.vertex_2_);

  EXPECT_EQ(way.distance, data.way_.distance);

  EXPECT_EQ(way.vertices.size(), data.way_.vertices.size());

  for (size_t i = 0; i < way.vertices.size(); ++i) {
    EXPECT_EQ(way.vertices[i], data.way_.vertices[i]);
  }
}
