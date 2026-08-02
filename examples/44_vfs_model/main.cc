#include <cstdio>
#include <cstring>
#include <mujoco/mujoco.h>

int main() {
  const char xml[] = R"(<mujoco model="memory model">
    <worldbody><body><freejoint/><geom type="sphere" size=".1" mass="1"/></body></worldbody>
  </mujoco>)";
  mjVFS vfs; mj_defaultVFS(&vfs);
  if (mj_addBufferVFS(&vfs, "embedded.xml", xml, std::strlen(xml)) != 0) {
    std::fprintf(stderr, "无法向 VFS 添加内存模型\n"); mj_deleteVFS(&vfs); return 1;
  }
  char error[1024] = {0};
  mjModel* m=mj_loadXML("embedded.xml", &vfs, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); mj_deleteVFS(&vfs); return 1; }
  mjData* d=mj_makeData(m);
  for (int k=0;k<100;++k) mj_step(m,d);
  std::printf("loaded entirely from VFS: model=%s nq=%lld z=%.6f\n",
              m->names, static_cast<long long>(m->nq), d->qpos[2]);
  mj_deleteData(d); mj_deleteModel(m); mj_deleteVFS(&vfs); return 0;
}
