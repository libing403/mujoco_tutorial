# 第 28 章　mjSpec：程序化模型的生命周期

MJCF 是声明式模型源，`mjModel` 是编译后的高性能运行表示；`mjSpec` 位于二者之间，是可编辑的模型规范树。官方 `mjspec.ipynb` 展示了解析、修改、程序化树、height field、mesh、attach 和 recompile。本章把这些能力映射到 MuJoCo 3.11.0 C API。

## 28.1 学习目标

- 区分 `mjSpec`、`mjModel`、`mjData` 的职责与所有权；
- 从 XML 解析 spec，或从空 spec 构造 body/joint/geom；
- 编译、保存规范化 MJCF，并读取编译错误；
- 理解 attach 的深复制、命名冲突与 asset 解析；
- 安全使用 `mj_recompile` 保留兼容状态。

## 28.2 三种表示

```text
MJCF / URDF / decoder
        │ parse
        ▼
      mjSpec       ← 可编辑、接近作者意图、含名称与资产引用
        │ compile
        ▼
      mjModel      ← 定长扁平数组、已解析地址与常量、仿真只读
        │ makeData
        ▼
      mjData       ← 状态、控制、缓存、接触、求解器工作区
```

不能通过随意修改 `mjModel` 的结构计数来“添加一个关节”：其单块内存布局和所有地址在编译时确定。结构编辑应发生在 spec，然后重新 compile/recompile。

## 28.3 创建、解析和销毁

```cpp
mjSpec* empty = mj_makeSpec();
mjSpec* parsed = mj_parseXML("robot.xml", NULL, error, sizeof(error));
mjSpec* text = mj_parseXMLString(xml, NULL, error, sizeof(error));
...
mj_deleteSpec(empty);
```

spec 拥有其 element、字符串和动态数组。调用 `mjs_addBody` 返回的指针由 spec 管理，不能单独 `delete`。删除 spec 后所有 element pointer 失效。

解析错误通过 error buffer 返回；编译失败时 `mj_compile` 返回 NULL，详细信息用 `mjs_getError(spec)`。不要沿用 `mj_loadXML` 的错误处理假设。

## 28.4 构造模型树

空 spec 自动包含名为 `world` 的 world body：

```cpp
mjsBody* world = mjs_findBody(spec, "world");
mjsBody* link = mjs_addBody(world, NULL);
mjs_setName(link->element, "link");

mjsJoint* joint = mjs_addJoint(link, NULL);
mjs_setName(joint->element, "hinge");
joint->type = mjJNT_HINGE;
joint->axis[1] = 1;

mjsGeom* geom = mjs_addGeom(link, NULL);
geom->type = mjGEOM_CAPSULE;
geom->fromto[5] = -0.5;
geom->size[0] = 0.04;
geom->mass = 1;
```

add 函数已经按默认值初始化 element，只设置与默认不同的字段即可。名称不是裸 `char*` 字段，应使用 `mjs_setName` 或 `mjs_setString` 管理内部字符串。

`mjsDefault` 能复用 default class。批量程序化生成树时，先设计 default，再让元素引用它，比逐字段复制更容易统一 collision、rgba、density 和 joint damping。

## 28.5 编译边界

```cpp
mjModel* m = mj_compile(spec, NULL);
if (!m) fprintf(stderr, "%s\n", mjs_getError(spec));
```

compile 会完成惯量推断、名称解析、拓扑排序、地址分配、mesh 处理和常量计算。spec 中的作者顺序不保证等于 model 中的 DoF/constraint 内部顺序，应用仍应通过 name→id→address 查询。

编译得到的 model 与 spec 生命周期独立：model 使用 `mj_deleteModel`，spec 使用 `mj_deleteSpec`。一个 spec 可多次修改并编译出多个 model；每个 model 需要自己的 data。

## 28.6 保存为规范化 MJCF

`mj_saveXMLString` 和 `mj_saveXML` 将 spec 编码为 MJCF。应提供足够的字符串缓冲区；空间不足时错误信息会报告所需容量，应用可扩容后重试。保存结果适合：

- 审计程序化生成模型；
- 写入构建产物供 simulate 打开；
- code review 中比较结构变化；
- 将 URDF 导入结果转成后续维护的 MJCF 基线。

规范化 XML 不保证保留原注释、include 文件边界或原始属性拼写；它是语义表示，不是源代码格式化器。

## 28.7 查找、遍历和修改

`mjs_findBody(spec,name)` 等查找函数返回可编辑 element。修改字段只改变 spec；已有 `mjModel` 不会自动变化。典型 parse-edit-compile 工作流：

1. parse 基础机器人；
2. 找到工具法兰 body；
3. 添加相机、site 或工具子树；
4. compile 新 model；
5. 运行第 13 章 model audit；
6. 保存规范化 XML 作为生成产物。

长期保存 element pointer 时要谨慎：删除、attach 或 recompile 相关操作可能改变对象归属。优先在局部编辑阶段使用指针，跨阶段用稳定名称重新查找。

## 28.8 attach 与模块化机器人

`mjs_attach(parent, child, prefix, suffix)` 把 child subtree/asset 复制并附加到 parent。prefix/suffix 用于避免 body、joint、geom、mesh、material 等全局命名冲突。

典型用途：把同一夹爪装到不同机械臂，或左右镜像地附加腿部模块。attach 后必须验证：

- 名称和 actuator/sensor 引用已正确重写；
- asset 是否去重或冲突；
- frame/scale 与接口单位；
- 自碰撞掩码；
- keyframe 的 qpos/ctrl 维数；
- 被 attach spec 的 element pointer 归属与生命周期。

attach 是构建模型，不是运行时刚体焊接。仿真中临时抓取物体应使用 equality、接触或应用逻辑。

## 28.9 原地重编译

```cpp
int status = mj_recompile(spec, NULL, model, data);
```

它尝试更新 model/data 并保留可映射的运行状态，适合交互式编辑和参数化场景。结构改变后 `nq/nv/nu` 可能变化，所以外部缓存、controller state、日志 schema 和指针都必须重新审计。

不要把 recompile 当硬实时操作：编译可能加载资产、分配内存并改变结构。生产控制循环通常在非实时线程构建新模型，经验证后在安全同步点切换。

## 28.10 独立实验：零 XML 构造单摆

`examples/38_mjspec_build/` 不读取 `model.xml`，因为实验目标正是从空 spec 构造模型。目录只有一个 `main.cc` 和最小 `CMakeLists.txt`；程序 compile、仿真，并把生成的规范化 MJCF 打印到终端。

```bash
cd examples/38_mjspec_build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo
```

这里省略模型文件不是破坏示例规范，而是知识点要求的最小设计：引入一个从不读取的 XML 只会分散注意力。

## 28.11 常见误区

- 修改 spec 后继续使用旧 model，误以为变化会自动同步；
- 删除 spec 后继续访问 element pointer；
- 用普通字符串赋值绕过 `mjs_setName/mjs_setString`；
- compile 失败仍调用 `mj_makeData(NULL)`；
- attach 不加前缀导致全局名称冲突；
- recompile 后保留旧的 model/data 数组指针；
- 程序化模型只在内存中运行，从不保存 XML 做人工审计；
- 在实时控制线程解析 mesh 和重新编译。

## 28.12 习题与答案

1. 为什么 `mjModel` 不适合作结构编辑？  
   **答案：**它是编译后的定长扁平内存，地址、计数和常量互相依赖；结构变化需要重新编译。

2. 一个 spec 能否编译多个 model？  
   **答案：**可以；各 model 独立拥有编译结果并分别销毁。

3. 保存 XML 会保留 include 和注释吗？  
   **答案：**不保证；保存的是规范化语义模型，不是原始文本结构。

4. attach 与 equality weld 有什么本质区别？  
   **答案：**attach 在编译前构造一棵模型树；weld 是运行模型中的约束，可启停并产生约束力。

5. recompile 后为什么要重建控制器地址表？  
   **答案：**结构、顺序和数组地址可能改变，旧 ID、adr 和裸指针不能假设仍有效。
