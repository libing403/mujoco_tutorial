# 附录 D　实验、输出与习题导航

> 本书示例代码仓库：[libing403/mujoco_tutorial](https://github.com/libing403/mujoco_tutorial)

全部实验索引和运行命令见 [`examples/README.md`](../examples/README.md)。每章习题后紧跟参考答案，便于自学；培训场景可先遮住答案完成实验报告。

## D.1 基础 invariants

| 实验 | 通过证据 |
|---|---|
| 13 joint types | `nq=13,nv=11`，quaternion norm=1 |
| 14 data consistency | stale 派生量可观察，restore error=0 |
| 16 integrators | energy drift 随方法/timestep 有序变化 |
| 17 chaos | `1e-9` perturb 长时放大 |
| 24 Jacobian FD | analytic/numeric error 接近浮点精度 |
| 25 mass matrix | symmetric、positive、energy/multiply/solve 一致 |
| 26 forward/inverse | applied force recover residual 接近 0 |

## D.2 Contact/control/optimization

| 实验 | 通过证据 |
|---|---|
| 27 softness | soft penetration 大、peak force 小；hard 相反 |
| 28 contact wrench | world force 平衡外力和重量 |
| 29 solvers | PGS/CG/Newton 迭代/耗时可量化，稳态 penetration 接近 |
| 31 sampled PD | delay 增加 RMS/saturation |
| 32 DLS IK | reachable target error 接近 0 |
| 33 sysID | damping 从错误初值收敛到生成真值 |
| 34 computed torque | 同控制代价量级下 RMS 显著优于直接 torque PD |
| 35 transition FD | independent perturbation 一步 prediction error 很小 |
| 36 LQR | open loop falling，closed loop upright |
| 37 EKF | estimated position RMS 小于 raw noise RMS |

## D.3 程序化/批量/渲染/扩展

| 实验 | 通过证据 |
|---|---|
| 38 mjSpec | 无 XML 构造/compile/simulate/save normalized MJCF |
| 39 rollout | multi-worker checksum 与 serial 相同 |
| 40 FoPG | cost 随 gradient updates 下降 |
| 41 scene | headless scene 有 static/dynamic/decor geom |
| 42 EGL | readback RGB/depth 并输出 PPM |
| 43 plugin | custom sensor dimension/output 正确 |
| 44 VFS | 无模型文件从 memory buffer load |

## D.4 综合项目

| 实验 | 通过证据 |
|---|---|
| 45 7-DoF arm | TCP error <5 mm、torque 有界 |
| 46 floating biped | state dimensions 正确、双脚支撑力≈重量、CoM/base 稳定 |

## D.5 推荐实验报告模板

1. 目标与 hypothesis；
2. model/version/platform/build；
3. independent/dependent/control variables；
4. command 与完整 output；
5. plot/table 与单位；
6. invariant/tolerance；
7. failure/异常解释；
8. timestep/solver/seed sensitivity；
9. 对人形/机械臂工程的迁移；
10. 仍未验证的假设。
