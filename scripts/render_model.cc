#define SDL_USE_BUILTIN_OPENGL_DEFINITIONS
#include <SDL2/SDL_egl.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "用法: %s model.xml output.ppm\n", argv[0]);
    return EXIT_FAILURE;
  }

  EGLDisplay display = eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA,
                                              EGL_DEFAULT_DISPLAY, nullptr);
  EGLint major = 0, minor = 0;
  if (display == EGL_NO_DISPLAY || !eglInitialize(display, &major, &minor)) {
    display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display == EGL_NO_DISPLAY || !eglInitialize(display, &major, &minor)) {
      std::fprintf(stderr, "无法初始化 EGL: 0x%x\n", eglGetError());
      return EXIT_FAILURE;
    }
  }

  const EGLint attributes[] = {
      EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
      EGL_DEPTH_SIZE, 24, EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
      EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
  EGLConfig config;
  EGLint count = 0;
  if (!eglChooseConfig(display, attributes, &config, 1, &count) || !count ||
      !eglBindAPI(EGL_OPENGL_API)) {
    std::fprintf(stderr, "无法选择 EGL OpenGL 配置\n");
    eglTerminate(display);
    return EXIT_FAILURE;
  }
  EGLContext egl = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
  if (egl == EGL_NO_CONTEXT ||
      !eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, egl)) {
    std::fprintf(stderr, "无法创建 EGL context: 0x%x\n", eglGetError());
    eglTerminate(display);
    return EXIT_FAILURE;
  }

  char error[1024] = {0};
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "无法加载 %s:\n%s\n", argv[1], error);
    return EXIT_FAILURE;
  }
  mjData* data = mj_makeData(model);
  mj_forward(model, data);
  for (int step = 0; step < 100; ++step) mj_step(model, data);

  mjvCamera camera;
  mjv_defaultCamera(&camera);
  mjv_defaultFreeCamera(model, &camera);
  camera.azimuth = 135;
  camera.elevation = -22;

  mjvOption option;
  mjv_defaultOption(&option);
  option.flags[mjVIS_CONTACTPOINT] = 1;
  option.flags[mjVIS_CONTACTFORCE] = 1;

  mjvScene scene;
  mjv_defaultScene(&scene);
  mjv_makeScene(model, &scene, 2000);
  mjrContext context;
  mjr_defaultContext(&context);
  mjr_makeContext(model, &context, mjFONTSCALE_150);
  mjr_setBuffer(mjFB_OFFSCREEN, &context);

  constexpr int width = 800;
  constexpr int height = 600;
  mjrRect viewport = {0, 0, width, height};
  mjv_updateScene(model, data, &option, nullptr, &camera, mjCAT_ALL, &scene);
  mjr_render(viewport, &scene, &context);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
              "MuJoCo 3.11.0", model->names, &context);

  std::vector<unsigned char> rgb(3 * width * height);
  mjr_readPixels(rgb.data(), nullptr, viewport, &context);
  FILE* output = std::fopen(argv[2], "wb");
  if (!output) {
    std::perror(argv[2]);
    return EXIT_FAILURE;
  }
  std::fprintf(output, "P6\n%d %d\n255\n", width, height);
  for (int row = height - 1; row >= 0; --row) {
    std::fwrite(rgb.data() + 3 * row * width, 1, 3 * width, output);
  }
  std::fclose(output);

  mjr_freeContext(&context);
  mjv_freeScene(&scene);
  mj_deleteData(data);
  mj_deleteModel(model);
  eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
  eglDestroyContext(display, egl);
  eglTerminate(display);
  return EXIT_SUCCESS;
}
