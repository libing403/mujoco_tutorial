# 独立实验索引

程序不依赖源码仓库，也不共享隐藏的运行框架。通常每个目录只有 `main.cc`、`model.xml`、`CMakeLists.txt`；纯程序化建模实验不放置无用 XML。

| 程序 | 推荐模型 | 验证点 | 预期现象 |
|---|---|---|---|
| `01_hello` | `pendulum.xml` | 加载、步进 | 摆角周期变化并因阻尼衰减 |
| `02_inspect` | 任意本书模型 | model 地址与世界位姿 | 输出规模、joint 地址、body 坐标 |
| `03_pd_control` | `two_link.xml` | motor + PD | 3 s 后接近目标角 |
| `04_sensors` | `sensor_contact.xml` | sensor 解包、contact wrench | 静止球约 9.81 N 支撑力 |
| `05_jacobian` | `two_link.xml` | `mj_jacSite` | 输出 `Jp` 与末端线速度 |
| `06_inverse_dynamics` | `two_link.xml` | 静态逆动力学 | 输出两关节重力补偿力矩 |
| `07_state_snapshot` | `two_link.xml` | state spec 快照/恢复 | 恢复误差为零 |
| `08_split_step` | `two_link.xml` | `mj_step1/2` 控制时序 | 拆分步进闭环收敛 |
| `09_mass_matrix` | `two_link.xml` | 展开 `M(q)` | 对称误差接近机器精度 |
| `10_model_io` | `two_link.xml` | XML→内存 MJB→model | 规模保持一致 |
| `11_energy` | `conservative_pendulum.xml` | 势能、动能、积分误差 | RK4 短时能量漂移很小 |
| `12_keyframe` | `keyframe_arm.xml` | keyframe reset | 输出三个位姿的末端坐标 |
| `13_joint_types` | 四类关节模型 | `nq/nv`、地址、四元数 | 验证 13/11 维布局与单位范数 |
| `14_data_consistency` | 二连杆 | 主状态、派生量、状态快照 | 观察 stale 坐标与零恢复误差 |
| `15_defaults_compiler` | default 类 | 继承、覆盖、规范化 XML | 读取编译后的阻尼/颜色真值 |
| `16_integrator_compare` | 保守单摆 | Euler/implicitfast/RK4 | 比较时间步与能量漂移 |
| `17_chaotic_double_pendulum` | 双摆 | 初值敏感性 | 观察 1e-9 rad 微扰的指数放大 |
| `18_visual_collision` | visual/collision 双层球 | 惯量与碰撞分层 | 验证 visual 不贡献质量和接触 |
| `19_joint_passive` | 五关节台架 | damping/friction/spring/armature | 比较自由衰减与力矩响应 |
| `20_actuator_shortcuts` | motor/position/velocity | ctrl 语义和力映射 | 同一 ctrl 值产生三种响应 |
| `21_activation_filter` | 无状态/filter/filterexact | 三阶 actuator 状态 | 比较阶跃 activation 与 force |
| `22_fixed_tendon` | 二关节差动 tendon | 长度、速度和力映射 | 验证 `l=q1-2q2` 与 `Jᵀp` |
| `23_model_audit` | 浮动基座简化双足 | 结构/惯量/接口审计 | 检查 nq/nv、惯量和关键 frame |
| `24_jacobian_finite_difference` | 二连杆 TCP | 解析/数值 Jacobian | 流形中心差分验证 `mj_jacSite` |
| `25_mass_matrix_properties` | 二连杆动力学 | M 对称/正定/动能/求解 | 验证 `fullM/mulM/solveM` |
| `26_forward_inverse` | 二连杆动力学 | forward/inverse 一致性 | 从 qacc 恢复已知 applied force |
| `27_contact_softness` | 软/硬落球 | solref 对照 | 比较 penetration、峰值力和静态高度 |
| `28_contact_wrench` | 水平受力方块 | contact frame、摩擦锥、wrench | 世界系合力平衡重力与外力 |
| `29_solver_compare` | 五方块堆叠 | PGS/CG/Newton | 比较迭代数、耗时、穿入和终态 |
| `30_apply_force` | 二连杆末端受力 | `mj_applyFT`、虚功 | 验证任意点外力等于 `Jᵀf` |
| `31_sampled_pd` | 单关节台架 | 多速率、延迟、饱和 | 对比三种离散 PD 控制循环 |
| `32_damped_ik` | 二连杆机械臂 | Jacobian、DLS、流形更新 | 迭代到达二维目标位置 |
| `33_damping_identification` | 自由摆轨迹 | finite difference、LM、box | 从 rollout 反演关节阻尼 |
| `34_computed_torque` | 两连杆机械臂 | 轨迹、`mj_mulM`、bias | 对比 torque PD 与计算力矩控制 |
| `35_transition_fd` | 受控单摆 | `mjd_transitionFD` | 用独立扰动验证 A、B 一步预测 |
| `36_lqr_balance` | 倒立摆 | transition FD、DARE、LQR | 比较开环倒下与闭环平衡 |
| `37_ekf_pendulum` | 带噪单摆 | nonlinear predict、EKF | 从位置测量估计位置与速度 |
| `38_mjspec_build` | 程序化单摆 | `mjSpec` 构造、编译、保存 | 不读取 XML 生成可运行模型 |
| `39_parallel_rollout` | 128 条单摆轨迹 | 多 `mjData`、worker | 验证并行一致性与相对加速 |
| `40_policy_gradient` | PD gain policy | transition Jacobian、FoPG | 手工通过时间传播策略梯度 |
| `41_abstract_scene` | 落地球场景 | `mjvScene`、tracking camera | 无窗口构建抽象可视化场景 |
| `42_offscreen_render` | 固定相机场景 | EGL、RGB/depth、PPM | 无窗口生成 320×240 图像 |
| `43_sensor_plugin` | 自定义二维传感器 | engine plugin 生命周期 | 输出仿真时间和关节角 |
| `44_vfs_model` | 内存 MJCF | `mjVFS`、buffer loading | 不访问模型文件完成仿真 |
| `45_arm_reach` | 7-DoF 冗余机械臂 | DLS、`mj_mulM`、bias、限幅 | 任务空间到达综合验收 |
| `46_biped_standing` | 浮动基 10-DoF 双足 | free state、CoM、双脚 wrench | 站立 plant/sensor 综合审计 |

仓库根目录 `models/` 另外保留 tendon、equality、actuator 和 collision filter 等进阶模型素材。

```bash
cd 05_jacobian                 # 换成当前要学习的示例
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

没有 `enable_testing()`：运行结果本身就是实验观察对象。
