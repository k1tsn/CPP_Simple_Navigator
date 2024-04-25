#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../../../src/model/graph/graph.h"
#include "../../../src/model/graph_algorithms/graph_algorithms.h"
#include "matrix.h"

struct SolveSalesmanDataTest {
  std::string filename_;
  int path_;
  graph_cb::GraphAlgorithms::SalesmanAlgorithms type_;
  int rand_num_;
};

class SolveSalesmanTest : public testing::TestWithParam<int> {
 public:
  static int count_;
  static std::vector<SolveSalesmanDataTest> test_data_;
};

int SolveSalesmanTest::count_ = 3;
std::vector<SolveSalesmanDataTest> SolveSalesmanTest::test_data_ = {
    SolveSalesmanDataTest{
        "graphs/10_salesman.txt", 49,
        graph_cb::GraphAlgorithms::SalesmanAlgorithms::kAntAlgorithm, 1},
    SolveSalesmanDataTest{
        "graphs/11_salesman.txt", 45,
        graph_cb::GraphAlgorithms::SalesmanAlgorithms::kAntAlgorithm, 1},
    SolveSalesmanDataTest{
        "graphs/12.txt", 24,
        graph_cb::GraphAlgorithms::SalesmanAlgorithms::kAntAlgorithm, 1}};

INSTANTIATE_TEST_SUITE_P(solve_salesman, SolveSalesmanTest,
                         testing::Range(0, SolveSalesmanTest::count_));

TEST_P(SolveSalesmanTest, Common) {
  int num_test = this->GetParam();
  SolveSalesmanDataTest& data = SolveSalesmanTest::test_data_[num_test];

  graph_cb::Graph graph;
  graph.LoadGraphFromFile(data.filename_);

  graph_cb::GraphAlgorithms alg;

  graph_cb::GraphAlgorithms::TsmResult res =
      alg.SolveTravelingSalesmanProblem(graph, data.type_, data.rand_num_);

  EXPECT_EQ(res.distance, data.path_);
}
