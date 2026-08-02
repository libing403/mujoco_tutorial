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
  const mjtNum target[2] = {0.5, -0.8};

  while (d->time < 1.0) {
    mj_step1(m, d);  // 此时最新的运动学和速度派生量可供控制器读取
    for (int i = 0; i < m->nu && i < 2; ++i) {
      d->ctrl[i] = 60*(target[i]-d->qpos[i]) - 6*d->qvel[i];
    }
    mj_step2(m, d);
  }
  std::printf("split-step final q=(%.5f %.5f), time=%.3f\n",
              d->qpos[0], d->qpos[1], d->time);
  if (view) {
    if (!glfwInit()) return EXIT_FAILURE;
    GLFWwindow* window=glfwCreateWindow(900,700,"08 split step",NULL,NULL);
    if (!window) { glfwTerminate(); return EXIT_FAILURE; }
    glfwMakeContextCurrent(window);
    mjvCamera cam; mjv_defaultCamera(&cam); mjv_defaultFreeCamera(m,&cam);
    mjvOption opt; mjv_defaultOption(&opt);
    mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(m,&scene,1000);
    mjrContext con; mjr_defaultContext(&con); mjr_makeContext(m,&con,mjFONTSCALE_150);
    while (!glfwWindowShouldClose(window)) {
      int width,height; glfwGetFramebufferSize(window,&width,&height);
      mjrRect viewport={0,0,width,height};
      mjv_updateScene(m,d,&opt,NULL,&cam,mjCAT_ALL,&scene);
      mjr_render(viewport,&scene,&con);
      mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,viewport,
                  "mj_step1 / controller / mj_step2","final controlled pose",&con);
      glfwSwapBuffers(window); glfwPollEvents();
    }
    mjr_freeContext(&con); mjv_freeScene(&scene);
    glfwDestroyWindow(window); glfwTerminate();
  }
  mj_deleteData(d); mj_deleteModel(m);
  return EXIT_SUCCESS;
}
