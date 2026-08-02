#include <cmath>
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
  mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!m) {
    std::fprintf(stderr, "模型加载失败:\n%s\n", error);
    return EXIT_FAILURE;
  }
  mjData* d = mj_makeData(m);

  int stand = mj_name2id(m, mjOBJ_KEY, "stand");
  int pelvis = mj_name2id(m, mjOBJ_BODY, "pelvis");
  int torso = mj_name2id(m, mjOBJ_BODY, "torso_link");
  int left_ankle = mj_name2id(m, mjOBJ_BODY, "left_ankle_roll_link");
  int right_ankle = mj_name2id(m, mjOBJ_BODY, "right_ankle_roll_link");
  if (stand < 0 || pelvis < 0 || torso < 0 || left_ankle < 0 || right_ankle < 0) {
    std::fprintf(stderr, "模型接口审计失败：缺少关键 keyframe/body\n");
    return EXIT_FAILURE;
  }

  mj_resetDataKeyframe(m, d, stand);
  mj_forward(m, d);
  std::printf("Unitree G1: nq=%lld nv=%lld nu=%lld bodies=%lld joints=%lld\n",
              static_cast<long long>(m->nq), static_cast<long long>(m->nv),
              static_cast<long long>(m->nu), static_cast<long long>(m->nbody),
              static_cast<long long>(m->njnt));
  std::printf("mass=%.3f kg, timestep=%.4f s, stand keyframe=%d\n",
              m->body_subtreemass[pelvis], m->opt.timestep, stand);

  while (d->time < 2.0) {
    // stand keyframe 已把 29 个位置执行器的目标写入 ctrl。
    mj_step(m, d);
  }

  double max_joint_error = 0;
  for (int actuator = 0; actuator < m->nu; ++actuator) {
    int joint = m->actuator_trnid[2*actuator];
    int qadr = m->jnt_qposadr[joint];
    max_joint_error = mju_max(max_joint_error,
                              std::fabs(d->ctrl[actuator] - d->qpos[qadr]));
  }

  double foot_force[2] = {0, 0};
  int foot_contacts[2] = {0, 0};
  for (int contact = 0; contact < d->ncon; ++contact) {
    const mjContact& c = d->contact[contact];
    int body0 = m->geom_bodyid[c.geom[0]];
    int body1 = m->geom_bodyid[c.geom[1]];
    int side = body0 == left_ankle || body1 == left_ankle ? 0 :
               body0 == right_ankle || body1 == right_ankle ? 1 : -1;
    if (side < 0) continue;
    mjtNum wrench[6];
    mj_contactForce(m, d, contact, wrench);
    double world_z = c.frame[2]*wrench[0] +
                     c.frame[5]*wrench[1] + c.frame[8]*wrench[2];
    foot_force[side] += world_z;
    ++foot_contacts[side];
  }

  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
  for (int geom = 0; geom < m->ngeom; ++geom) {
    int body = m->geom_bodyid[geom];
    if (body != left_ankle && body != right_ankle) continue;
    double radius = m->geom_size[3*geom];
    xmin = mju_min(xmin, d->geom_xpos[3*geom] - radius);
    xmax = mju_max(xmax, d->geom_xpos[3*geom] + radius);
    ymin = mju_min(ymin, d->geom_xpos[3*geom+1] - radius);
    ymax = mju_max(ymax, d->geom_xpos[3*geom+1] + radius);
  }

  const mjtNum* com = d->subtree_com + 3*pelvis;
  double weight = m->body_subtreemass[pelvis] * std::fabs(m->opt.gravity[2]);
  double support_error = std::fabs(foot_force[0] + foot_force[1] - weight) / weight;
  bool com_inside = com[0] >= xmin && com[0] <= xmax &&
                    com[1] >= ymin && com[1] <= ymax;
  double torso_up = d->xmat[9*torso + 8];
  bool pass = foot_contacts[0] > 0 && foot_contacts[1] > 0 &&
              support_error < 0.08 && com_inside && torso_up > 0.98 &&
              max_joint_error < 0.08;

  std::printf("pelvis z=%.4f m, torso up dot world-z=%.6f\n",
              d->xpos[3*pelvis+2], torso_up);
  std::printf("CoM xy=[%.4f %.4f], support x=[%.3f %.3f] y=[%.3f %.3f], inside=%s\n",
              com[0], com[1], xmin, xmax, ymin, ymax, com_inside ? "yes" : "no");
  std::printf("left Fz=%.2f N (%d), right Fz=%.2f N (%d), weight=%.2f N\n",
              foot_force[0], foot_contacts[0], foot_force[1], foot_contacts[1], weight);
  std::printf("support error=%.3f%%, max joint error=%.5f rad, %s\n",
              100*support_error, max_joint_error, pass ? "PASS" : "FAIL");

  if (view) {
    if (!glfwInit()) return EXIT_FAILURE;
    GLFWwindow* window = glfwCreateWindow(1100, 800, "46 Unitree G1 standing", nullptr, nullptr);
    if (!window) { glfwTerminate(); return EXIT_FAILURE; }
    glfwMakeContextCurrent(window);
    mjvCamera cam; mjv_defaultCamera(&cam); mjv_defaultFreeCamera(m, &cam);
    cam.azimuth = 140; cam.elevation = -18;
    mjvOption opt; mjv_defaultOption(&opt);
    opt.flags[mjVIS_CONTACTPOINT] = 1;
    opt.flags[mjVIS_CONTACTFORCE] = 1;
    opt.frame = mjFRAME_SITE;
    mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(m, &scene, 3000);
    mjrContext con; mjr_defaultContext(&con); mjr_makeContext(m, &con, mjFONTSCALE_150);
    while (!glfwWindowShouldClose(window)) {
      int width, height; glfwGetFramebufferSize(window, &width, &height);
      mjrRect viewport = {0, 0, width, height};
      mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scene);
      mjr_render(viewport, &scene, &con);
      char status[160];
      std::snprintf(status, sizeof(status),
                    "CoM inside: %s   left/right Fz: %.1f / %.1f N",
                    com_inside ? "yes" : "no", foot_force[0], foot_force[1]);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                  "Unitree G1 | 29-DoF standing audit", status, &con);
      glfwSwapBuffers(window); glfwPollEvents();
    }
    mjr_freeContext(&con); mjv_freeScene(&scene);
    glfwDestroyWindow(window); glfwTerminate();
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return pass ? EXIT_SUCCESS : EXIT_FAILURE;
}
