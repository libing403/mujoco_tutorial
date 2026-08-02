#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "用法: %s model.xml\n", argv[0]);
    return EXIT_FAILURE;
  }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) {
    std::fprintf(stderr, "无法加载 %s:\n%s\n", argv[1], error);
    return EXIT_FAILURE;
  }
  mjData* d = mj_makeData(m);
  const char* name[] = {"plain", "damped", "dry", "spring", "armature"};
  int qadr[5], dadr[5];
  for (int i = 0; i < 5; ++i) {
    int joint = mj_name2id(m, mjOBJ_JOINT, name[i]);
    qadr[i] = m->jnt_qposadr[joint];
    dadr[i] = m->jnt_dofadr[joint];
  }

  for (int i = 0; i < 4; ++i) {
    d->qpos[qadr[i]] = 0.5;
    d->qvel[dadr[i]] = 1.0;
  }
  mj_forward(m, d);
  const mjtNum sample_time[] = {0.02, 0.2, 2.0};
  std::printf("free decay velocities:\n");
  std::printf("  %6s %10s %10s %10s %10s\n",
              "time", "plain", "damped", "dry", "spring");
  for (int s = 0; s < 3; ++s) {
    while (d->time < sample_time[s]) mj_step(m, d);
    std::printf("  %6.2f %10.6f %10.6f %10.6f %10.6f\n",
                d->time, d->qvel[dadr[0]], d->qvel[dadr[1]],
                d->qvel[dadr[2]], d->qvel[dadr[3]]);
  }

  mj_resetData(m, d);
  while (d->time < 0.2) {
    d->qfrc_applied[dadr[0]] = 1.0;
    d->qfrc_applied[dadr[4]] = 1.0;
    mj_step(m, d);
  }
  std::printf("same 1 N.m torque for 0.2 s:\n");
  std::printf("  plain    v=% .6f\n", d->qvel[dadr[0]]);
  std::printf("  armature v=% .6f\n", d->qvel[dadr[4]]);

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
