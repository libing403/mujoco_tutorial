#define SDL_USE_BUILTIN_OPENGL_DEFINITIONS
#include <SDL2/SDL_egl.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  EGLDisplay display = eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA,
                                              EGL_DEFAULT_DISPLAY, NULL);
  EGLint major, minor;
  if (display == EGL_NO_DISPLAY || !eglInitialize(display, &major, &minor)) {
    display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display == EGL_NO_DISPLAY || !eglInitialize(display, &major, &minor)) {
      std::fprintf(stderr, "无法初始化 EGL，错误 0x%x\n", eglGetError()); return 1;
    }
  }
  const EGLint attrs[] = {EGL_RED_SIZE,8,EGL_GREEN_SIZE,8,EGL_BLUE_SIZE,8,
                          EGL_DEPTH_SIZE,24,EGL_SURFACE_TYPE,EGL_PBUFFER_BIT,
                          EGL_RENDERABLE_TYPE,EGL_OPENGL_BIT,EGL_NONE};
  EGLConfig config; EGLint count;
  if (!eglChooseConfig(display, attrs, &config, 1, &count) || !count ||
      !eglBindAPI(EGL_OPENGL_API)) {
    std::fprintf(stderr, "无法选择 EGL OpenGL 配置\n"); eglTerminate(display); return 1;
  }
  EGLContext egl = eglCreateContext(display, config, EGL_NO_CONTEXT, NULL);
  if (egl == EGL_NO_CONTEXT || !eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, egl)) {
    std::fprintf(stderr, "无法创建 EGL context，错误 0x%x\n", eglGetError());
    eglTerminate(display); return 1;
  }

  char error[1024] = {0}; mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  mjData* d = mj_makeData(m); mj_forward(m, d);
  mjvCamera cam; mjv_defaultCamera(&cam); cam.type=mjCAMERA_FIXED;
  cam.fixedcamid=mj_name2id(m, mjOBJ_CAMERA, "view");
  mjvOption opt; mjv_defaultOption(&opt);
  mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(m, &scene, 1000);
  mjrContext context; mjr_defaultContext(&context); mjr_makeContext(m, &context, mjFONTSCALE_100);
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scene);
  mjr_setBuffer(mjFB_OFFSCREEN, &context);
  const int width=320, height=240; mjrRect viewport={0,0,width,height};
  mjr_render(viewport, &scene, &context);
  std::vector<unsigned char> rgb(3*width*height);
  std::vector<float> depth(width*height);
  mjr_readPixels(rgb.data(), depth.data(), viewport, &context);

  FILE* file=std::fopen("frame.ppm", "wb");
  if (file) {
    std::fprintf(file, "P6\n%d %d\n255\n", width, height);
    for (int y=height-1; y>=0; --y) std::fwrite(rgb.data()+3*y*width, 1, 3*width, file);
    std::fclose(file);
  }
  auto range=std::minmax_element(depth.begin(), depth.end());
  int center=(height/2)*width+width/2;
  std::printf("EGL %d.%d, RGB center=[%u %u %u], raw depth range=[%.6f, %.6f]\n",
              major, minor, rgb[3*center], rgb[3*center+1], rgb[3*center+2],
              *range.first, *range.second);
  std::printf("wrote frame.ppm (%d x %d)\n", width, height);

  mjr_freeContext(&context); mjv_freeScene(&scene); mj_deleteData(d); mj_deleteModel(m);
  eglMakeCurrent(display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
  eglDestroyContext(display,egl); eglTerminate(display); return 0;
}
