#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  bool view=argc==3 && std::strcmp(argv[2],"--view")==0;
  if (argc<2 || argc>3 || (argc==3 && !view)) {
    std::fprintf(stderr,"用法: %s model.xml [--view]\n",argv[0]); return 1;
  }
  char error[1024] = {0}; mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  mjData* truth = mj_makeData(m); mjData* est = mj_makeData(m);
  truth->qpos[0] = 0.8; est->qpos[0] = 0.65; mj_forward(m, truth); mj_forward(m, est);
  double P[4] = {.04, 0, 0, .25};
  const double Q[2] = {1e-7, 2e-5}, R = .03*.03;
  std::mt19937 rng(7); std::normal_distribution<double> noise(0, .03);
  double raw2=0, pos2=0, vel2=0;

  for (int k = 0; k < 2000; ++k) {
    double u = 0.35*std::sin(1.3*truth->time);
    truth->ctrl[0] = u; mj_step(m, truth);
    double z = truth->sensordata[0] + noise(rng);

    est->ctrl[0] = u;
    mjtNum A[4]; mjd_transitionFD(m, est, 1e-6, 0, A, NULL, NULL, NULL);
    double AP[4] = {A[0]*P[0]+A[1]*P[2], A[0]*P[1]+A[1]*P[3],
                    A[2]*P[0]+A[3]*P[2], A[2]*P[1]+A[3]*P[3]};
    double nextP[4] = {AP[0]*A[0]+AP[1]*A[1]+Q[0], AP[0]*A[2]+AP[1]*A[3],
                       AP[2]*A[0]+AP[3]*A[1], AP[2]*A[2]+AP[3]*A[3]+Q[1]};
    mj_step(m, est);

    double innovation = z-est->qpos[0], S = nextP[0]+R;
    double K[2] = {nextP[0]/S, nextP[2]/S};
    est->qpos[0] += K[0]*innovation; est->qvel[0] += K[1]*innovation;
    P[0] = (1-K[0])*nextP[0]; P[1] = (1-K[0])*nextP[1];
    P[2] = nextP[2]-K[1]*nextP[0]; P[3] = nextP[3]-K[1]*nextP[1];
    double offdiag = 0.5*(P[1]+P[2]); P[1]=P[2]=offdiag;
    mj_forward(m, est);

    raw2 += (z-truth->qpos[0])*(z-truth->qpos[0]);
    pos2 += (est->qpos[0]-truth->qpos[0])*(est->qpos[0]-truth->qpos[0]);
    vel2 += (est->qvel[0]-truth->qvel[0])*(est->qvel[0]-truth->qvel[0]);
  }
  std::printf("raw position RMS       = %.6f rad\n", std::sqrt(raw2/2000));
  std::printf("estimated position RMS = %.6f rad\n", std::sqrt(pos2/2000));
  std::printf("estimated velocity RMS = %.6f rad/s\n", std::sqrt(vel2/2000));
  if (view) {
    if (!glfwInit()) return 1;
    GLFWwindow* window=glfwCreateWindow(900,700,"37 EKF pendulum",NULL,NULL);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    mjvCamera cam; mjv_defaultCamera(&cam); mjv_defaultFreeCamera(m,&cam);
    mjvOption opt; mjv_defaultOption(&opt);
    mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(m,&scene,1000);
    mjrContext con; mjr_defaultContext(&con); mjr_makeContext(m,&con,mjFONTSCALE_150);
    while (!glfwWindowShouldClose(window)) {
      int width,height; glfwGetFramebufferSize(window,&width,&height);
      mjrRect viewport={0,0,width,height};
      mjv_updateScene(m,truth,&opt,NULL,&cam,mjCAT_ALL,&scene);
      mjr_render(viewport,&scene,&con);
      char status[120]; std::snprintf(status,sizeof(status),
          "truth q: %.4f   EKF q: %.4f",truth->qpos[0],est->qpos[0]);
      mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,viewport,
                  "Extended Kalman filter",status,&con);
      glfwSwapBuffers(window); glfwPollEvents();
    }
    mjr_freeContext(&con); mjv_freeScene(&scene);
    glfwDestroyWindow(window); glfwTerminate();
  }
  mj_deleteData(est); mj_deleteData(truth); mj_deleteModel(m); return 0;
}
