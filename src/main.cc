#include "controller/controller.h"
#include "controller/controller_default.h"
#include "view/view.h"

int main() {
  graph::Model* model = new graph::Model();
  graph::Controller* controller = new graph::ControllerDefault(model);
  graph::View view(controller);
  view.Show();

  delete model;
  delete controller;
  return 0;
}