#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

#ifdef BUILD_PLUGIN

struct Calibration {
  mjtNum gain;
  mjtNum offset;
};

void register_sensor() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "book.sensor.calibrated_joint";
  static const char* attributes[] = {"gain", "offset"};
  plugin.nattribute = 2;
  plugin.attributes = attributes;
  plugin.capabilityflags = mjPLUGIN_SENSOR;
  plugin.needstage = mjSTAGE_POS;
  plugin.nstate = +[](const mjModel*, int) { return 0; };
  plugin.nsensordata = +[](const mjModel*, int, int) { return 3; };
  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto* calibration = new Calibration{
        std::strtod(mj_getPluginConfig(m, instance, "gain"), nullptr),
        std::strtod(mj_getPluginConfig(m, instance, "offset"), nullptr)};
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(calibration);
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Calibration*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.copy = +[](mjData* dest, const mjModel*, const mjData* src, int instance) {
    auto* source = reinterpret_cast<Calibration*>(src->plugin_data[instance]);
    dest->plugin_data[instance] = reinterpret_cast<uintptr_t>(new Calibration(*source));
  };
  plugin.reset = +[](const mjModel*, mjtNum*, void*, int) {};
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability) {
    if (!(capability & mjPLUGIN_SENSOR)) return;
    auto* calibration = reinterpret_cast<Calibration*>(d->plugin_data[instance]);
    for (int sensor = 0; sensor < m->nsensor; ++sensor) {
      if (m->sensor_plugin[sensor] != instance) continue;
      mjtNum angle = d->qpos[0];
      mjtNum* output = d->sensordata + m->sensor_adr[sensor];
      output[0] = d->time;
      output[1] = angle;
      output[2] = calibration->gain * angle + calibration->offset;
    }
  };
  mjp_registerPlugin(&plugin);
}

mjPLUGIN_LIB_INIT(book_sensor_plugin) { register_sensor(); }

#else

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "用法: %s model.xml plugin.so\n", argv[0]);
    return 1;
  }

  mj_loadPluginLibrary(argv[2]);
  int slot = -1;
  if (!mjp_getPlugin("book.sensor.calibrated_joint", &slot)) {
    std::fprintf(stderr, "动态库未注册预期的 plugin\n");
    return 1;
  }

  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!m) {
    std::fprintf(stderr, "%s\n", error);
    return 1;
  }
  mjData* d = mj_makeData(m);
  d->qpos[0] = 0.7;
  for (int i = 0; i < 100; ++i) mj_step(m, d);

  int sensor = mj_name2id(m, mjOBJ_SENSOR, "joint_encoder");
  const mjtNum* value = d->sensordata + m->sensor_adr[sensor];
  std::printf("slot=%d, dim=%d, time=%.3f s, angle=%.3f rad, calibrated=%.3f deg\n",
              slot, m->sensor_dim[sensor], value[0], value[1], value[2]);

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}

#endif
