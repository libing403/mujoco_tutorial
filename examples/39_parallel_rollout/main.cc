#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>
#include <mujoco/mujoco.h>

double run(const mjModel* m, int nthread, std::vector<double>& output) {
  std::atomic<int> next(0);
  auto worker = [&]() {
    mjData* d = mj_makeData(m);
    while (true) {
      int batch = next.fetch_add(1);
      if (batch >= static_cast<int>(output.size())) break;
      mj_resetData(m, d);
      d->qpos[0] = 0.7;
      d->qvel[0] = -2.0 + 4.0*batch/(output.size()-1);
      mj_forward(m, d);
      for (int k = 0; k < 4000; ++k) mj_step(m, d);
      output[batch] = d->qpos[0];
    }
    mj_deleteData(d);
  };
  auto begin = std::chrono::steady_clock::now();
  std::vector<std::thread> threads;
  for (int i = 0; i < nthread; ++i) threads.emplace_back(worker);
  for (auto& thread : threads) thread.join();
  auto end = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(end-begin).count();
}

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0}; mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  std::vector<double> serial(128), parallel(128);
  double t1 = run(m, 1, serial), t4 = run(m, 4, parallel);
  double checksum1=0, checksum4=0, maxdiff=0;
  for (size_t i = 0; i < serial.size(); ++i) {
    checksum1 += serial[i]; checksum4 += parallel[i];
    maxdiff = mju_max(maxdiff, std::abs(serial[i]-parallel[i]));
  }
  std::printf("1 worker: %.3f ms, checksum=%.12f\n", t1, checksum1);
  std::printf("4 workers: %.3f ms, checksum=%.12f, speedup=%.2fx\n", t4, checksum4, t1/t4);
  std::printf("max trajectory result difference = %.3g\n", maxdiff);
  mj_deleteModel(m); return 0;
}
