# 第 18 章　碰撞、摩擦锥与接触力

上一章回答“接触有多硬”，本章回答另外三个工程问题：**谁会碰谁、摩擦约束有几维、脚底承受的力究竟怎样读出**。这三件事如果混在一起，最常见的结果是机器人能站住，却无法解释力传感器、滑移或倾覆。

## 18.1 学习目标

完成本章后，读者应能：

- 区分 broad phase、near phase、接触约束求解三个阶段；
- 用 `contype/conaffinity`、`pair`、`exclude` 控制碰撞集合；
- 解释 `condim=1/3/4/6` 和滑动、扭转、滚动摩擦；
- 正确读取 `mjContact` 与 `mj_contactForce`，并把接触坐标系中的 wrench 变换到世界系；
- 用摩擦利用率、接触点分布和合力矩诊断人形机器人脚底接触。

先修内容：第 7 章的碰撞几何、第 11 章的坐标系、第 17 章的软约束。

## 18.2 从几何重叠到约束力

一次接触不是“两个三角形碰到了”这么简单。MuJoCo 的处理链可概括为：

```text
geom 集合
   │ broad phase：快速剔除不可能相交的包围体
   ▼
候选 geom pair
   │ filter + near phase：计算距离、法向、接触点
   ▼
mjData.contact[0 ... ncon-1]
   │ constraint construction：按 condim 展开约束
   ▼
efc_* 标量约束空间
   │ convex solver
   ▼
qfrc_constraint 与接触 wrench
```

`d->ncon` 是检测到的接触数量，而不是标量约束数量。一个 `condim="3"` 接触具有法向和两个切向自由度；使用金字塔摩擦锥时，内部标量表示还可能不同。因此不要手工解释 `efc_force` 的若干连续元素，优先调用 `mj_contactForce`。

## 18.3 碰撞筛选：自动配对与显式配对

两个 geom 自动成为候选对的核心位运算条件是：

\[
(c_1 \mathbin{\&} a_2)\ne0
\quad\lor\quad
(c_2 \mathbin{\&} a_1)\ne0,
\]

其中 \(c\) 是 `contype`，\(a\) 是 `conaffinity`。它们是位掩码，不是“碰撞组编号”。例如让机器人自身和环境分别占不同 bit，可以精确组织自碰撞。

还必须注意：

- 同一刚体上的 geom 默认不互碰；父子刚体通常也被过滤；
- `<contact><exclude body1=... body2=.../></contact>` 排除一对 body；
- 显式 `<pair geom1=... geom2=.../>` 可直接指定候选及其摩擦、margin、`solref` 等参数；
- `group` 主要服务于可视化选择，不负责碰撞过滤；
- visual geom 最稳妥的配置是 `contype="0" conaffinity="0" mass="0"`。

显式 pair 很适合脚掌—地面、夹爪—工件等关键界面，但大型模型逐对枚举会失去可维护性。工程上通常采用“掩码做大范围策略，pair 覆盖少量关键界面”。

## 18.4 `mjContact` 中有什么

位置阶段完成后，`d->contact[i]` 保存：

- `geom[0:2]`：接触的两个 geom ID；
- `pos[3]`：世界系接触点；
- `dist`：有符号接触距离；负值通常表示穿入；
- `frame[9]`：接触坐标系；
- `dim`：实际约束维数；
- `exclude`：该接触是否不进入求解；
- `efc_address`：它在活动标量约束数组中的起始位置。

接触 frame 是一个容易踩坑的特例：三个轴按**行**连续存储。`frame[0..2]` 是法向轴，`frame[3..5]` 与 `frame[6..8]` 是两条切向轴。它和 `mjData` 中通常按旋转矩阵理解的存储方式看起来不同。

## 18.5 摩擦锥与 `condim`

设接触坐标系 wrench 为

\[
\mathbf w_c=[f_n,f_{t1},f_{t2},\tau_n,\tau_{t1},\tau_{t2}]^T.
\]

`condim` 决定参与求解的分量：

| `condim` | 约束内容 | 典型用途 |
|---:|---|---|
| 1 | 仅法向 | 无摩擦滚珠、简化碰撞 |
| 3 | 法向 + 两向滑动摩擦 | 绝大多数刚体接触 |
| 4 | 再加绕法向扭转摩擦 | 脚掌、旋钮 |
| 6 | 再加两向滚动摩擦 | 球体、轮子 |

库仑滑动摩擦的理想圆锥写成

\[
\sqrt{f_{t1}^2+f_{t2}^2}\le \mu f_n,\qquad f_n\ge0.
\]

MuJoCo 支持椭圆锥和金字塔锥。用 `mj_isPyramidal(m)` 判断当前表示；无论内部表示是哪一种，`mj_contactForce` 都返回直观的六维接触系 wrench。

`friction` 最多有五项：两个滑动系数、一个扭转系数、两个滚动系数。它不是越大越好：过大摩擦会提高条件数、掩盖控制器问题；软接触还允许渐进滑移，不能仅靠把 \(\mu\) 调到极大来“焊死”脚底。

## 18.6 从局部接触力到世界系合力

调用：

```cpp
mjtNum wrench[6];
mj_contactForce(m, d, contact_id, wrench);
```

得到的力和力矩表达在接触系。若接触 frame 三个行向量组成 \(R_{cw}\)，则局部坐标到世界坐标为

\[
\mathbf f_w=R_{cw}^T\mathbf f_c.
\]

对足底所有接触求关于参考点 \(\mathbf p_0\) 的合 wrench：

\[
\mathbf F=\sum_i\mathbf f_i,
\qquad
\boldsymbol\tau_{p_0}=\sum_i
\left[(\mathbf p_i-\mathbf p_0)\times\mathbf f_i+\boldsymbol\tau_i\right].
\]

这正是把多个离散接触点聚合成六维足底力传感器读数的方法。注意 `mj_contactForce` 给出作用方向与 contact 中 geom 顺序有关；做左右脚统计时必须统一“地面对脚”的符号，可通过 geom ID 顺序检查并在实验中用静止重力校准。

## 18.7 独立实验：摩擦利用率与世界系支撑力

实验目录：`examples/28_contact_wrench/`。一个方块落地稳定后受到水平外力。程序逐接触读取 wrench，完成坐标变换，并报告

\[
\rho=\frac{\sqrt{f_{t1}^2+f_{t2}^2}}{\mu f_n}.
\]

`rho` 接近 1 表示摩擦裕量耗尽；大于 1 不应被机械地视作 bug，因为软约束、椭圆参数和瞬态都会影响简单估计，但持续越界通常提示参数解释或坐标方向错误。

```bash
cd examples/28_contact_wrench
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

程序刻意只有一个源码文件。请直接对照 `main.cc` 中 `frame` 的三个行向量和世界系求和代码，不需要先理解任何辅助框架。

<!-- EMBEDDED_EXAMPLE_BEGIN: 28_contact_wrench -->
### 可视化运行与效果

```bash
../../mujoco-3.11.0/bin/simulate model.xml
```

该命令使用发布包自带的官方 `simulate` 界面，可暂停、单步、施加扰动并开启接触等可视化标志。

![28_contact_wrench 实验运行效果](../assets/experiments/28_contact_wrench.png)

*28_contact_wrench 的真实 MuJoCo 原生渲染结果。*

### 实验完整源码

以下文件与 `examples/28_contact_wrench/` 中可直接编译的版本一致。

#### 模型文件：`model.xml`

```xml
<mujoco model="contact wrench">
  <option timestep="0.001" solver="Newton" cone="elliptic"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 .1" friction="0.8 0.01 0.001"/>
    <body name="box" pos="0 0 .4">
      <freejoint/>
      <geom name="box_geom" type="box" size=".12 .09 .06"
            mass="2" friction="0.8 0.01 0.001" condim="3"/>
    </body>
  </worldbody>
</mujoco>
```

#### 程序源码：`main.cc`

```cpp
#include <cmath>
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
    std::fprintf(stderr, "无法加载模型:\n%s\n", error);
    return EXIT_FAILURE;
  }
  mjData* d = mj_makeData(m);
  int box_body = mj_name2id(m, mjOBJ_BODY, "box");
  int box_geom = mj_name2id(m, mjOBJ_GEOM, "box_geom");

  while (d->time < 1.0) mj_step(m, d);       // 先让方块落稳
  for (int k = 0; k < 500; ++k) {
    d->xfrc_applied[6*box_body] = 8.0;       // 世界系 +x，低于 mu*m*g
    mj_step(m, d);
  }

  mjtNum world_force[3] = {0, 0, 0};
  mjtNum max_ratio = 0;
  int used = 0;
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact& c = d->contact[i];
    if (c.geom[0] != box_geom && c.geom[1] != box_geom) continue;
    mjtNum w[6];
    mj_contactForce(m, d, i, w);
    for (int axis = 0; axis < 3; ++axis) {
      world_force[axis] += c.frame[axis] * w[0]
                         + c.frame[3+axis] * w[1]
                         + c.frame[6+axis] * w[2];
    }
    mjtNum ratio = std::sqrt(w[1]*w[1] + w[2]*w[2]) / (0.8*w[0]);
    max_ratio = mju_max(max_ratio, ratio);
    ++used;
  }

  std::printf("contacts=%d  friction_cone=%s\n", used,
              mj_isPyramidal(m) ? "pyramidal" : "elliptic");
  std::printf("world contact force = [% .6f, % .6f, % .6f] N\n",
              world_force[0], world_force[1], world_force[2]);
  std::printf("expected approximately [-8, 0, %.5f] N\n", 2*9.81);
  std::printf("max contact friction utilization = %.4f\n", max_ratio);
  std::printf("box vx = %.6g m/s\n", d->qvel[0]);

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
```

#### 构建文件：`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(28_contact_wrench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
add_executable(demo main.cc)

set(MUJOCO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../mujoco-3.11.0)
target_include_directories(demo PRIVATE ${MUJOCO_ROOT}/include)
target_link_directories(demo PRIVATE ${MUJOCO_ROOT}/lib)
target_link_libraries(demo PRIVATE mujoco)
set_target_properties(demo PROPERTIES BUILD_RPATH ${MUJOCO_ROOT}/lib)
```
<!-- EMBEDDED_EXAMPLE_END: 28_contact_wrench -->

## 18.8 人形机器人接触诊断

“机器人摔倒”不是一个足够具体的故障描述。建议按下面的观测链定位：

1. **候选是否正确**：检查脚底 geom ID、`ncon`、接触点位置；
2. **法向是否正确**：可视化或打印 `frame[0..2]`；
3. **载荷是否守恒**：静止时双脚世界系竖直合力应接近总重力；
4. **摩擦裕量是否耗尽**：统计每只脚的切向合力及接触级 `rho`；
5. **支撑多边形是否合理**：接触点是否只集中在脚尖或边缘；
6. **数值参数是否收敛**：减半 timestep、增大 solver 精度后结论是否稳定。

对于平脚，零力矩点可由足底合 wrench 估计。但软接触、非共面接触以及非零扭转摩擦下，必须清楚参考平面和符号约定；不能把某个接触点的 `pos` 直接当作整只脚的压力中心。

## 18.9 常见误区

- **`ncon==0` 才说明无接触力**：有 gap 或内部筛选时，检测到的 contact 也可能不进入约束，应看 `exclude/efc_address`。
- **直接读 `efc_force` 当 XYZ**：金字塔锥内部轴不是直角 XYZ，应用 `mj_contactForce`。
- **把 `frame` 当列向量矩阵**：contact frame 的轴按行存储。
- **只调 friction 解决滑脚**：先检查控制、质量、接触软硬度、timestep 与 solver。
- **把所有碰撞 geom 都做成高精网格**：会增加候选与接触噪声；脚底常用少量凸几何更稳健。
- **每个接触点单独闭环**：接触点会生灭和重排，控制器更适合消费按脚聚合的 wrench。

## 18.10 习题与答案

1. 一个 `condim=6` 接触为何不等于六个接触点？  
   **答案：**它是一个几何接触点上的六维法向、滑动、扭转和滚动约束；接触点数量仍由 `ncon` 统计。

2. 静止 60 kg 双足机器人，两脚竖直合力长期只有约 450 N，应先检查什么？  
   **答案：**统一 wrench 符号与坐标变换，确认所有脚底 geom 都被统计，再检查模型总质量、重力和是否还有其他支撑接触；不要先调摩擦。

3. 为什么不能用 `group="3"` 禁止自碰撞？  
   **答案：**`group` 是分组/显示属性；碰撞资格由掩码、显式 pair/exclude 及拓扑规则决定。

4. 已知世界系接触力，怎样得到关于脚踝原点的力矩？  
   **答案：**对每点计算 `(contact_pos-ankle_pos) × force`，加上该点自身世界系接触力矩后求和。

5. 设计一个检验摩擦参数的实验。  
   **答案：**让已知质量方块在水平面稳定，缓慢增加水平力，记录开始持续滑动的阈值，并同时记录法向力与摩擦利用率；用多个 timestep 重复，避免把瞬态软约束误差当作静摩擦系数。

下一章将进入求解器、约束岛与 sleeping：同一个接触模型，为何 Newton、CG、PGS 的精度和耗时不同，以及怎样判断“迭代次数不够”。
