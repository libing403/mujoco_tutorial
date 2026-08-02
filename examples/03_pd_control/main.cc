#include <cstdio>
#include <cstdlib>
#include <cstring>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  bool view = argc == 3 && std::strcmp(argv[2], "--view") == 0;
  if (argc < 2 || argc > 3 || (argc == 3 && !view)) {
    std::fprintf(stderr, "用法: %s model.xml [--view]\n", argv[0]);
    return EXIT_FAILURE;
  }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) {
    std::fprintf(stderr, "无法加载 %s:\n%s\n", argv[1], error);
    return EXIT_FAILURE;
  }
  mjData* d = mj_makeData(m);
  const mjtNum target[2] = {0.8, -1.1};
  const mjtNum kp = 80.0, kd = 8.0;

  GLFWwindow* window = nullptr;
  mjvCamera cam; mjvOption opt; mjvScene scene; mjrContext con;
  if (view) {
    if (!glfwInit()) return EXIT_FAILURE;
    window = glfwCreateWindow(900, 700, "03 PD control", nullptr, nullptr);
    if (!window) { glfwTerminate(); return EXIT_FAILURE; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjv_defaultCamera(&cam); mjv_defaultFreeCamera(m, &cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scene); mjv_makeScene(m, &scene, 1000);
    mjr_defaultContext(&con); mjr_makeContext(m, &con, mjFONTSCALE_150);
  }

  while (d->time < 3.0 && (!view || !glfwWindowShouldClose(window))) {
    mjtNum frame_start = d->time;
    do {
    for (int i = 0; i < 2; ++i) {
      d->ctrl[i] = kp * (target[i] - d->qpos[i]) - kd * d->qvel[i];
    }
    mj_step(m, d);
    } while (view && d->time-frame_start < 1.0/60.0);
    if (view) {
      int width, height; glfwGetFramebufferSize(window, &width, &height);
      mjrRect viewport = {0, 0, width, height};
      mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scene);
      mjr_render(viewport, &scene, &con);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                  "PD control", "target: (0.8, -1.1)", &con);
      glfwSwapBuffers(window); glfwPollEvents();
    }
  }
  std::printf("target=(%.3f %.3f) final=(%.3f %.3f) error=(%.4f %.4f)\n",
              target[0], target[1], d->qpos[0], d->qpos[1],
              target[0]-d->qpos[0], target[1]-d->qpos[1]);
  while (view && !glfwWindowShouldClose(window)) {
    int width, height; glfwGetFramebufferSize(window, &width, &height);
    mjrRect viewport = {0, 0, width, height};
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &con);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                "PD control: finished", "close the window to exit", &con);
    glfwSwapBuffers(window); glfwPollEvents();
  }
  if (view) {
    mjr_freeContext(&con); mjv_freeScene(&scene);
    glfwDestroyWindow(window); glfwTerminate();
  }
  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
