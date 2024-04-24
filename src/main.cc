#include "controller/controller.h"
#include "controller/controller_default.h"
#include "view/view.h"

int main() {
  graph_cb::Model* model = new graph_cb::Model();
  graph_cb::Controller* controller = new graph_cb::ControllerDefault(model);
  graph_cb::View v(controller);
  v.Show();

  delete model;
  delete controller;
  return 0;
}