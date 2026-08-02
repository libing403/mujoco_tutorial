#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

#include <cstring>
#include <vector>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "用法: %s model.xml\n", argv[0]);
    return EXIT_FAILURE;
  }
  char error[1024] = {0};
  mjModel* xml = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!xml) {
    std::fprintf(stderr, "无法加载 %s:\n%s\n", argv[1], error);
    return EXIT_FAILURE;
  }
  int bytes = mj_sizeModel(xml);
  std::vector<unsigned char> buffer(bytes);
  mj_saveModel(xml, NULL, buffer.data(), bytes);
  mjModel* mjb = mj_loadModelBuffer(buffer.data(), bytes);
  if (!mjb) { std::fprintf(stderr, "无法从内存 MJB 加载\n"); return EXIT_FAILURE; }
  std::printf("MJB bytes=%d nq=%lld nv=%lld same_model_size=%s\n", bytes,
              (long long)mjb->nq, (long long)mjb->nv,
              mj_sizeModel(mjb) == bytes ? "yes" : "no");
  mj_deleteModel(mjb); mj_deleteModel(xml);
  return EXIT_SUCCESS;
}
