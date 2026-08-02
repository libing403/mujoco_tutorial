#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

void register_sensor() {
  mjpPlugin plugin; mjp_defaultPlugin(&plugin);
  plugin.name = "book.sensor.state";
  plugin.capabilityflags = mjPLUGIN_SENSOR;
  plugin.needstage = mjSTAGE_POS;
  plugin.nstate = +[](const mjModel*, int) { return 0; };
  plugin.nsensordata = +[](const mjModel*, int, int) { return 2; };
  plugin.init = +[](const mjModel*, mjData*, int) { return 0; };
  plugin.reset = +[](const mjModel*, mjtNum*, void*, int) {};
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability) {
    if (!(capability & mjPLUGIN_SENSOR)) return;
    for (int sensor=0; sensor<m->nsensor; ++sensor) {
      if (m->sensor_plugin[sensor] != instance) continue;
      int adr=m->sensor_adr[sensor];
      d->sensordata[adr] = d->time;
      d->sensordata[adr+1] = m->nq ? d->qpos[0] : 0;
    }
  };
  mjp_registerPlugin(&plugin);
}

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  register_sensor();
  char error[1024] = {0}; mjModel* m=mj_loadXML(argv[1],NULL,error,sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  mjData* d=mj_makeData(m); d->qpos[0]=0.7; mj_forward(m,d);
  for (int k=0;k<100;++k) mj_step(m,d);
  int id=mj_name2id(m,mjOBJ_SENSOR,"time_and_angle"), adr=m->sensor_adr[id];
  std::printf("plugin count=%d, sensor dim=%d, time=%.6f, angle=%.6f\n",
              mjp_pluginCount(), m->sensor_dim[id], d->sensordata[adr], d->sensordata[adr+1]);
  mj_deleteData(d); mj_deleteModel(m); return 0;
}
