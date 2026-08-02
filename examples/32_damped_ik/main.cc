#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  bool view=argc==3 && std::strcmp(argv[2],"--view")==0;
  if (argc<2 || argc>3 || (argc==3 && !view)) {
    std::fprintf(stderr,"\u7528\u6cd5: %s model.xml [--view]\n",argv[0]); return 1;
  }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  mjData* d = mj_makeData(m);
  int tcp = mj_name2id(m, mjOBJ_SITE, "tcp");
  int target = mj_name2id(m, mjOBJ_SITE, "target");
  const double lambda2 = 0.02*0.02;

  for (int iter = 0; iter < 100; ++iter) {
    mj_forward(m, d);
    double e[2] = {d->site_xpos[3*target]-d->site_xpos[3*tcp],
                   d->site_xpos[3*target+1]-d->site_xpos[3*tcp+1]};
    if (std::hypot(e[0], e[1]) < 1e-9) break;
    mjtNum jp[6], jr[6];
    mj_jacSite(m, d, jp, jr, tcp);
    double a = jp[0]*jp[0] + jp[1]*jp[1] + lambda2;
    double b = jp[0]*jp[2] + jp[1]*jp[3];
    double c = jp[2]*jp[2] + jp[3]*jp[3] + lambda2;
    double det = a*c-b*b;
    double y[2] = {(c*e[0]-b*e[1])/det, (-b*e[0]+a*e[1])/det};
    mjtNum dq[2] = {jp[0]*y[0]+jp[2]*y[1], jp[1]*y[0]+jp[3]*y[1]};
    double norm = std::hypot(dq[0], dq[1]);
    double step = norm > 0.2 ? 0.2/norm : 1.0;
    mj_integratePos(m, d->qpos, dq, step);
  }
  mj_forward(m, d);
  double ex = d->site_xpos[3*target]-d->site_xpos[3*tcp];
  double ey = d->site_xpos[3*target+1]-d->site_xpos[3*tcp+1];
  std::printf("q = [% .9f, % .9f]\n", d->qpos[0], d->qpos[1]);
  std::printf("tcp = [%.9f, %.9f], target = [%.9f, %.9f]\n",
              d->site_xpos[3*tcp], d->site_xpos[3*tcp+1],
              d->site_xpos[3*target], d->site_xpos[3*target+1]);
  std::printf("position error = %.3g m\n", std::hypot(ex, ey));
  if (view) {
    if (!glfwInit()) return 1;
    GLFWwindow* window=glfwCreateWindow(900,700,"32 damped IK",NULL,NULL);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    mjvCamera cam; mjv_defaultCamera(&cam); mjv_defaultFreeCamera(m,&cam);
    mjvOption opt; mjv_defaultOption(&opt); opt.flags[mjVIS_JOINT]=1;
    mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(m,&scene,1000);
    mjrContext con; mjr_defaultContext(&con); mjr_makeContext(m,&con,mjFONTSCALE_150);
    while (!glfwWindowShouldClose(window)) {
      int width,height; glfwGetFramebufferSize(window,&width,&height);
      mjrRect viewport={0,0,width,height};
      mjv_updateScene(m,d,&opt,NULL,&cam,mjCAT_ALL,&scene);
      mjr_render(viewport,&scene,&con);
      mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,viewport,
                  "Damped least-squares IK","TCP reaches the red target",&con);
      glfwSwapBuffers(window); glfwPollEvents();
    }
    mjr_freeContext(&con); mjv_freeScene(&scene);
    glfwDestroyWindow(window); glfwTerminate();
  }
  mj_deleteData(d); mj_deleteModel(m); return 0;
}
