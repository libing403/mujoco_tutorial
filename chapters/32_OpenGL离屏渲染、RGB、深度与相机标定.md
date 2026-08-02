# 第 32 章　OpenGL 离屏渲染、RGB、深度与相机标定

> 本书示例代码仓库：[libing403/mujoco_tutorial](https://github.com/libing403/mujoco_tutorial)

上一章得到 `mjvScene`；本章创建图形 context，把 scene rasterize 到 offscreen framebuffer，并读回 RGB/depth。重点不是写一个 viewer，而是为机器人视觉数据集、相机仿真和 CI 录制建立可解释、可复现的渲染管线。

## 32.1 学习目标

- 理解平台 OpenGL context 与 `mjrContext` 的不同职责；
- 使用 EGL/OSMesa/隐藏窗口实现 headless rendering；
- 正确配置 offscreen buffer、viewport 和 pixel readback；
- 将非线性 depth buffer 转为 metric distance；
- 从模型相机参数建立 intrinsics/extrinsics，并避免视觉真值泄漏。

## 32.2 两个 context

```text
EGL / GLFW / OSMesa context
  └─ 操作系统/驱动资源，必须在当前线程 current
       └─ mjrContext
            └─ MuJoCo 上传的 mesh、texture、font、FBO 等 GPU 资源
```

`mjr_defaultContext` 只把 C struct 清为默认值，没有创建 GPU 资源。必须先让平台 OpenGL context current，再调用 `mjr_makeContext(m,&con,fontscale)`。

切换到结构或 assets 不同的新 model 时，应重新 `mjr_makeContext`。销毁顺序是：在 platform context 仍有效且 current 时 `mjr_freeContext`，然后销毁 EGL/GLFW/OSMesa context。反过来会让 GPU resource cleanup 访问已失效 context。

OpenGL context 通常线程绑定。在哪个线程 make current，就在该线程调用 `mjr_*`；跨线程迁移必须用平台 API 显式解绑/绑定并同步。

## 32.3 三种 Linux headless 路径

- **EGL**：无窗口硬件渲染，适合 NVIDIA/AMD/Intel server；需要正确 EGL vendor/driver；
- **OSMesa**：CPU software rendering，不依赖 display/GPU，速度较低但 CI 可移植；
- **隐藏 GLFW window**：简单且跨平台，但通常仍需要 X11/Wayland display。

官方 `record.cc` 可按 `MJ_EGL`、`MJ_OSMESA` 或 GLFW 编译。本书示例使用 EGL surfaceless display，不依赖 MuJoCo 源码；系统必须提供 `libEGL.so.1` 和可用 driver。预编译 SDK 已提供 MuJoCo headers/library，但 OS 图形 driver 无法合理地打包进仓库。

## 32.4 Offscreen framebuffer

模型控制 FBO 初始容量：

```xml
<visual><global offwidth="320" offheight="240"/></visual>
```

然后：

```cpp
mjr_makeContext(m, &con, mjFONTSCALE_100);
mjr_setBuffer(mjFB_OFFSCREEN, &con);
mjrRect viewport = {0, 0, width, height};
mjr_render(viewport, &scene, &con);
```

请求尺寸不能超过 `con.offWidth/offHeight`。运行时可更新 model visual offscreen 尺寸并调用 `mjr_resizeOffscreen`，但这会重新分配 GPU resources，不应每帧改变。

窗口 size 与 framebuffer pixel size 在 HiDPI display 上不同；window rendering 应使用 GLFW framebuffer size，而非逻辑 window size。

## 32.5 RGB readback

```cpp
std::vector<unsigned char> rgb(3*w*h);
mjr_readPixels(rgb.data(), NULL, viewport, &con);
```

OpenGL 原点在左下，常见图像文件原点在左上，保存 PPM/PNG 时需垂直翻转。RGB 是 tightly packed 的三通道 byte；若接入 OpenCV，要确认 RGB/BGR 顺序。

GPU→CPU readback 会同步 pipeline，是高帧率数据生成的常见瓶颈。大量相机可使用 pixel buffer objects、异步队列或保持后续视觉算法在 GPU；先用 profiler 证明瓶颈再增加复杂度。

## 32.6 Depth buffer

`mjr_readPixels(NULL, depth, ...)` 返回通常位于 `[0,1]` 的非线性 depth。它不是米，也不是相机光轴 z。透视投影的线性视线深度可由 near/far 转换：

\[
z=\frac{n f}{f-d(f-n)},
\]

其中 \(d\) 是 depth buffer value，\(n/f\) 是实际裁剪面距离。MuJoCo 按

\[
n=znear\cdot extent,\qquad f=zfar\cdot extent
\]

从 `vis.map` 与 `stat.extent` 得到裁剪面。不同 renderer/backend 或 reversed-Z 设置应以当前版本文档和投影矩阵验证；最可靠校准是渲染已知距离的正对平面。

视线深度转 point cloud 还需 intrinsics，并区分 optical-axis z 与 Euclidean ray range。rangefinder sensor 和 depth camera 不是同一个量。

## 32.7 相机内参

只给 vertical field of view \(\theta_y\) 和图像高 \(H\) 时：

\[
f_y=\frac{H/2}{\tan(\theta_y/2)},
\qquad f_x=f_y
\]

这里 `fx=fy` 假设 square pixels；horizontal FOV 由 aspect ratio 推出。principal point 通常取像素中心，但 `(W-1)/2` 与 `W/2` convention 要与下游投影统一。

MJCF camera 也可提供 `focal/sensorsize/resolution/principal` 等物理标定属性。若同时指定多组投影参数，应查 XML Reference 的 precedence/互斥规则，不要自行混合。

## 32.8 Extrinsics 与坐标 convention

MuJoCo 世界系通常 z-up；计算机视觉 camera frame 常采用 x-right、y-down、z-forward；OpenGL view convention 又不同。不要仅复制 camera rotation matrix就声称得到 CV extrinsics。

建议用三点测试：在 camera 前方、右方、上方放置可识别 marker，渲染并验证 pixel 移动方向；再输出 `mjv_cameraInModel` 的 head/forward/up 建立显式 basis conversion。

## 32.9 Segmentation 与多模态同步

MuJoCo renderer 支持 scene flags/segmentation coloring路径，可获得 object/geom ID 图。RGB、depth、segmentation 必须来自相同 state、camera 和 viewport；若每种模态间 physics 又 step 一次，就不是同一帧。

数据集应保存：simulation time、camera name/pose、intrinsics、near/far、model/version、随机化参数和 ID→name mapping。单独一张 PNG 无法复现实验。

## 32.10 独立实验：EGL 生成 PPM

`examples/42_offscreen_render/` 创建 surfaceless EGL context，构建 scene，渲染 320×240 RGB/depth，打印中心像素和深度范围，并在当前目录写出 `frame.ppm`。

```bash
cd examples/42_offscreen_render
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

若运行时报 EGL initialize/context 错误，说明目标机器缺少可用 EGL driver，而不是 MuJoCo physics 配置失败。服务器部署应在镜像构建阶段运行该 smoke test。

<!-- EMBEDDED_EXAMPLE_BEGIN: 42_offscreen_render -->
### 可视化运行与效果

```bash
../../mujoco-3.11.0/bin/simulate model.xml
```

该命令使用发布包自带的官方 `simulate` 界面，可暂停、单步、施加扰动并开启接触等可视化标志。

![42_offscreen_render 实验运行效果](../assets/experiments/42_offscreen_render.png)

*42_offscreen_render 的真实 MuJoCo 原生渲染结果。*

### 实验完整源码

以下文件与 `examples/42_offscreen_render/` 中可直接编译的版本一致。

#### 模型文件：`model.xml`

```xml
<mujoco model="offscreen render">
  <visual><global offwidth="320" offheight="240"/></visual>
  <worldbody>
    <light pos="0 -2 3" dir="0 1 -1"/>
    <geom type="plane" size="2 2 .1" rgba=".7 .7 .7 1"/>
    <body pos="0 0 .25">
      <geom type="box" size=".25 .18 .25" rgba=".1 .4 .9 1"/>
    </body>
    <camera name="view" pos="1.5 -2 1.2" xyaxes=".8 .6 0 -.25 .33 .91" fovy="45"/>
  </worldbody>
</mujoco>
```

#### 程序源码：`main.cc`

```cpp
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
```

#### 构建文件：`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(42_offscreen_render LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
add_executable(demo main.cc)

set(MUJOCO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../mujoco-3.11.0)
target_include_directories(demo PRIVATE ${MUJOCO_ROOT}/include)
target_link_directories(demo PRIVATE ${MUJOCO_ROOT}/lib)
target_link_libraries(demo PRIVATE mujoco "-l:libEGL.so.1")
set_target_properties(demo PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)
```
<!-- EMBEDDED_EXAMPLE_END: 42_offscreen_render -->

## 32.11 生产架构

physics 可以 1 kHz、camera 30 Hz、viewer 60 Hz。render thread 在 camera deadline 获取一致 state snapshot；同一 snapshot 可生成多模态。不要为了 30 Hz 图像把 physics 降到 30 Hz，也不要无条件每 physics step read pixels。

多 camera 数据生成可在一个 context 中依次更新 camera/render，复用 scene/model resources。多个 context/GPU 并行需评估 resource duplication、driver thread safety 和 GPU memory。

## 32.12 常见误区

- 没有 current platform context 就调用 `mjr_makeContext`；
- 把 `mjvScene` 当图像；
- 请求 viewport 大于 offscreen FBO；
- 保存图像上下颠倒或 RGB/BGR 互换；
- 把 raw depth 当米；
- 销毁 EGL 后才 free `mjrContext`；
- RGB/depth/segmentation 在不同 physics state 渲染；
- benchmark 包含首次 shader/driver initialization 却称稳态 FPS；
- 假设 headless 主机有 EGL library 就一定有可工作的 vendor driver。

## 32.13 习题与答案

1. `mjrContext` 能否自行创建 OpenGL context？  
   **答案：**不能；平台 context 必须先由 EGL/GLFW/OSMesa 创建并设为 current。

2. raw depth=0.5 是否表示 0.5 m？  
   **答案：**不是，它是非线性 depth buffer value，需结合 near/far 和投影 convention 转换。

3. 为什么图像保存需要垂直翻转？  
   **答案：**OpenGL framebuffer 原点通常左下，而常见图像存储/显示 convention 原点左上。

4. camera 30 Hz、physics 1 kHz 应多久渲染？  
   **答案：**约每 33.3 ms 取一次一致 snapshot 渲染，physics 仍按 1 ms 推进。

5. readback 慢时第一步做什么？  
   **答案：**分阶段 profile 确认 GPU→CPU synchronization/readback 确实主导，再考虑异步 PBO 或 GPU-resident processing。
