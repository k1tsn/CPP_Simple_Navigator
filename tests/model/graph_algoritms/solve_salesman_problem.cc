#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../../../src/model/graph/graph.h"
#include "../../../src/model/graph_algorithms/graph_algorithms.h"
#include "matrix.h"

struct SolveSalesmanDataTest {
  std::string filename_;
  int path_;
};

class SolveSalesmanTest : public testing::TestWithParam<int> {
 public:
  static int count_;
  static std::vector<SolveSalesmanDataTest> test_data_;
};

int SolveSalesmanTest::count_ = 3;
std::vector<SolveSalesmanDataTest> SolveSalesmanTest::test_data_ = {
    SolveSalesmanDataTest{"graphs/10_salesman.txt", 49},
    SolveSalesmanDataTest{"graphs/11_salesman.txt", 37},
    SolveSalesmanDataTest{"graphs/12.txt", 24}};

INSTANTIATE_TEST_SUITE_P(solve_salesman, SolveSalesmanTest,
                         testing::Range(0, SolveSalesmanTest::count_));

TEST_P(SolveSalesmanTest, Common) {
  int num_test = this->GetParam();
  SolveSalesmanDataTest& data = SolveSalesmanTest::test_data_[num_test];

  graph::Graph graph;
  graph.LoadGraphFromFile(data.filename_);

  graph::GraphAlgorithms alg;

  graph::GraphAlgorithms::TsmResult res;
  graph::GraphAlgorithms::TsmResult best_res = {{}, -1};

  for (int i = 0; i < 30; ++i) {
    res = alg.SolveTravelingSalesmanProblem(graph);
    if (res.distance < best_res.distance || best_res.distance == -1)
      best_res = res;
  }

  EXPECT_EQ(best_res.distance, data.path_);
}
