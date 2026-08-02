#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

struct Result { double final_error, rms_error, peak_error, saturation; };

Result run(const mjModel* m, int control_steps, bool one_cycle_delay) {
  mjData* d = mj_makeData(m);
  double command = 0, delayed = 0, sum2 = 0, peak = 0;
  int saturated = 0, steps = 3000;
  for (int k = 0; k < steps; ++k) {
    double target = k < 500 ? 0.0 : 1.0;
    if (k % control_steps == 0) {
      double raw = 8.0*(target-d->qpos[0]) - 0.7*d->qvel[0];
      double next = mju_clip(raw, -3.0, 3.0);
      if (std::fabs(raw) > 3.0) ++saturated;
      if (one_cycle_delay) { command = delayed; delayed = next; }
      else command = next;
    }
    d->ctrl[0] = command;
    mj_step(m, d);
    double e = target-d->qpos[0];
    sum2 += e*e;
    peak = mju_max(peak, std::fabs(e));
  }
  Result r{std::fabs(1.0-d->qpos[0]), std::sqrt(sum2/steps), peak,
           double(saturated)/(steps/control_steps)};
  mj_deleteData(d);
  return r;
}

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  const char* names[3] = {"1 ms", "20 ms", "20 ms + one-cycle delay"};
  Result r[3] = {run(m, 1, false), run(m, 20, false), run(m, 20, true)};
  std::puts("case                    final_error   rms_error   peak_error   saturation");
  for (int i = 0; i < 3; ++i)
    std::printf("%-24s %.6f      %.6f    %.6f     %.1f%%\n", names[i],
                r[i].final_error, r[i].rms_error, r[i].peak_error, 100*r[i].saturation);
  mj_deleteModel(m);
  return 0;
}
