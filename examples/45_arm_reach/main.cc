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
    std::fprintf(stderr,"用法: %s model.xml [--view]\n",argv[0]); return 1;
  }
  char error[1024]={0}; mjModel* m=mj_loadXML(argv[1],NULL,error,sizeof(error));
  if (!m) { std::fprintf(stderr,"%s\n",error); return 1; }
  if (m->nv!=7 || m->nu!=7) { std::fprintf(stderr,"审计失败: 需要 nv=nu=7\n"); return 1; }
  mjData* d=mj_makeData(m); int tcp=mj_name2id(m,mjOBJ_SITE,"tcp");
  int target=mj_name2id(m,mjOBJ_SITE,"target"); mj_forward(m,d);
  double initial[3]={d->site_xpos[3*tcp],d->site_xpos[3*tcp+1],d->site_xpos[3*tcp+2]};
  double peak=0;
  for (int step=0;step<5000;++step) {
    mjtNum J[21], Jr[21]; mj_jacSite(m,d,J,Jr,tcp);
    mjtNum e[3], task[3];
    for (int i=0;i<3;++i) {
      e[i]=d->site_xpos[3*target+i]-d->site_xpos[3*tcp+i];
      task[i]=1.5*e[i];
    }
    mjtNum normal[9]={0};
    for (int r=0;r<3;++r) for (int c=0;c<3;++c) {
      for (int j=0;j<7;++j) normal[3*r+c]+=J[7*r+j]*J[7*c+j];
      if (r==c) normal[3*r+c]+=0.03*0.03;
    }
    mjtNum y[3];
    if (mju_cholFactor(normal,3,1e-12)<3) { std::fprintf(stderr,"DLS 分解失败\n"); break; }
    mju_cholSolve(y,normal,task,3);
    mjtNum vdes[7]={0};
    for (int j=0;j<7;++j) for (int i=0;i<3;++i) vdes[j]+=J[7*i+j]*y[i];
    double vmax=0; for (double v:vdes) vmax=mju_max(vmax,std::fabs(v));
    double scale=vmax>1.5 ? 1.5/vmax : 1.0;
    mjtNum ades[7],tau[7];
    for (int j=0;j<7;++j) ades[j]=6.0*(scale*vdes[j]-d->qvel[j]);
    mj_mulM(m,d,tau,ades);
    for (int j=0;j<7;++j) {
      tau[j]+=d->qfrc_bias[j]; d->ctrl[j]=mju_clip(tau[j],-35.0,35.0);
      peak=mju_max(peak,std::fabs(d->ctrl[j]));
    }
    mj_step(m,d);
  }
  double final[3],err2=0;
  for (int i=0;i<3;++i) { final[i]=d->site_xpos[3*tcp+i];
    double e=d->site_xpos[3*target+i]-final[i]; err2+=e*e; }
  double err=std::sqrt(err2);
  std::printf("initial TCP = [%.4f %.4f %.4f]\n",initial[0],initial[1],initial[2]);
  std::printf("final TCP   = [%.4f %.4f %.4f]\n",final[0],final[1],final[2]);
  std::printf("position error=%.6f m, peak torque=%.3f Nm, %s\n",err,peak,err<.005?"PASS":"FAIL");
  if (view) {
    if (!glfwInit()) return 1;
    GLFWwindow* window=glfwCreateWindow(1000,750,"45 arm reach",NULL,NULL);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    mjvCamera cam; mjv_defaultCamera(&cam); mjv_defaultFreeCamera(m,&cam);
    mjvOption opt; mjv_defaultOption(&opt); opt.flags[mjVIS_JOINT]=1;
    mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(m,&scene,2000);
    mjrContext con; mjr_defaultContext(&con); mjr_makeContext(m,&con,mjFONTSCALE_150);
    while (!glfwWindowShouldClose(window)) {
      int width,height; glfwGetFramebufferSize(window,&width,&height);
      mjrRect viewport={0,0,width,height};
      mjv_updateScene(m,d,&opt,NULL,&cam,mjCAT_ALL,&scene);
      mjr_render(viewport,&scene,&con);
      char status[100]; std::snprintf(status,sizeof(status),"TCP error: %.4f m",err);
      mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,viewport,
                  "7-DoF arm reach",status,&con);
      glfwSwapBuffers(window); glfwPollEvents();
    }
    mjr_freeContext(&con); mjv_freeScene(&scene);
    glfwDestroyWindow(window); glfwTerminate();
  }
  mj_deleteData(d); mj_deleteModel(m); return err<.005?0:2;
}
