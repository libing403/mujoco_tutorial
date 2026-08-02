# 第 37 章　综合项目二：浮动基双足的站立平衡

人形机器人与固定基机械臂的根本差异是：基座 6-DoF 没有直接执行器，身体只能通过关节内力和环境接触改变整体动量。本项目用简化 10-actuator 双足模型建立站立诊断链，并给出扩展到 contact-aware LQR/全身控制的完整路线。

## 37.1 项目验收目标

- 正确解释 `nq=7+n_joint`、`nv=6+n_joint`；
- 稳定站立 2 s，base height/tilt 不发散；
- 左右脚聚合世界系 wrench，总竖直力接近整机重量；
- whole-body CoM projection 位于 support region；
- actuator force 不越界，脚底不持续滑移；
- 找到 inverse-dynamics root residual 足够小的平衡 state-control pair；
- 在该 mode 内线性化，并验证局部 feedback 而不声称跨 contact mode 全局稳定。

## 37.2 浮动基状态

free joint configuration：

\[
q_{base}=[p_x,p_y,p_z,q_w,q_x,q_y,q_z]\in\mathbb R^7,
\]

velocity tangent：

\[
v_{base}=[v_x,v_y,v_z,\omega_x,\omega_y,\omega_z]\in\mathbb R^6

\]

（具体 linear/angular ordering 应以 `mjData.qvel` free-joint convention 和第 2/3 章实验核对。）加 10 个 hinge 后，本项目 `nq=17,nv=16,nu=10`。LQR state 是 `2*nv+na=32`，不是 `nq+nv=33`。

姿态误差必须由 `mj_differentiatePos` 得到三维 tangent。controller、estimator、trajectory optimizer 共享同一 state layout 表，禁止各自硬编码不同 quaternion convention。

## 37.3 支撑力守恒

站立近似稳态：

\[
F_{L,z}+F_{R,z}\approx mg,
\qquad
F_{L,x}+F_{R,x}\approx0,
\qquad
F_{L,y}+F_{R,y}\approx0.
\]

每只脚可能有多个 geom contacts。按第 18 章用 `mj_contactForce` 取 contact-frame wrench，转世界系后按 foot geom ID 聚合。若只读某一个 contact，载荷会随接触点生灭跳变。

符号应在静态实验校准：聚合“地面对脚”的竖直力必须为正。若 geom ordering 导致相反，应统一翻转该 contact 的 force/torque，而不是最后对总和取绝对值。

## 37.4 质心与支撑区域

`d->subtree_com[3*pelvis_id:...]` 给出 pelvis subtree（整机）的世界系 CoM。双脚平放时，可用所有 active foot contact points 的 convex hull 作为瞬时 support polygon。

简化矩形检查只适合两脚平行且已知 foot bounds。一般情况应用 2D convex hull + point-in-polygon，并计算 CoM projection 到边界的 signed margin。CoM 在 polygon 内是准静态必要直觉，不是动态稳定充分条件；运动中还要看 capture point、centroidal momentum 和可实现接触 wrench cone。

## 37.5 Joint position baseline

项目 MJCF 使用 position actuator 保持 10 个 leg joints 的零 reference。shortcut 产生

\[
p_i=k_p(u_i-q_i)-k_v\dot q_i
\]

（具体 bias/gain 由第 9 章展开），再经 unit gear 映射。这个 baseline 能验证模型与接触，但不是高性能人形控制：

- 两腿独立 servo 不显式协调 CoM/contact force；
- base perturbation 只能通过 joint error 间接反馈；
- load distribution 不受控；
- saturation/contact feasibility 没进入 optimization。

它的价值是先得到一个可运行、可测的站立基线，供 LQR/WBC 对比。

## 37.6 平衡点可行性

设目标 `qpos=qbar,qvel=0,qacc=0`：

1. `mj_forward` 生成接触；
2. `mj_inverse` 求 `qfrc_inverse`；
3. 检查 free root 6 项 residual；
4. 将 actuated DoF 所需力通过 actuator moment matrix 映射为 `ubar`；
5. 写 controls 后 `mj_forward`，比较 `qacc` 与 force residual；
6. 优化 base height/tilt/joint pose，使 root residual 和 task residual 最小。

soft contact 对高度极敏感。官方 LQR notebook 展示了毫米内的 base z 扫描如何让 vertical root force 从向下“魔法力”跨到整机重力支撑。不能任取 keyframe 就称 equilibrium。

## 37.7 Contact-aware linearization

找到 \((\bar x,\bar u)\) 后，用 `mjd_transitionFD` 获取 32-state A 与 10-control B。Q 可组合：

\[
Q=Q_{posture}+w_cJ_{com-rel-foot}^TJ_{com-rel-foot}+Q_{velocity}.
\]

CoM 相对支撑脚 Jacobian 把 task quadratic 映射到 state tangent。支撑腿、躯干权重大，手臂/非支撑自由度可小，让系统利用它们平衡。

检查 stabilizability 与 closed-loop eigenvalues。应用 control：

\[
u=\bar u-K\begin{bmatrix}
q\ominus\bar q\\v-\bar v
\end{bmatrix}.
\]

该 K 只对双脚接触 mode 附近有效。脚离地、滑移或新碰撞时 A/B 变化，应切换 controller 或交给 contact-aware MPC/WBC。

## 37.8 全身控制动力学

浮动基动力学：

\[
M\dot v+c=S^T\tau+J_c^Tf_c.
\]

典型 QP decision variables \(\dot v,\tau,f_c\)，constraints 包括：

- 动力学 equality；
- stance foot acceleration \(J_c\dot v+\dot J_cv=0\)；
- unilateral normal force、friction cone；
- torque/velocity/joint limits；
- swing foot collision clearance。

objectives 包括 CoM、pelvis orientation、swing foot、posture 和 force regularization。MuJoCo 提供 M、bias、J、contact，但 QP solver/任务优先级属于 controller 实现。

## 37.9 独立综合实验

`examples/46_biped_standing/` 使用自由 pelvis、左右各 5 joints 和多点脚底 box contact。程序运行 position-servo baseline，聚合左右脚 wrench、读取 whole-body CoM，并检查总支撑力与重量。

```bash
cd examples/46_biped_standing
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/demo model.xml
```

输出 `nq/nv/nu`、mass、CoM、base height、左右脚 Fz、support error 和 PASS/FAIL。它是平衡算法前的 plant/sensor audit；本章 LQR 路线要求读者在此模型或官方 humanoid 上继续完成 equilibrium search 与 DARE。

## 37.10 扰动测试

在 pelvis 质心施加有限时长水平 force pulse：报告 peak CoM displacement、恢复时间、foot slip、min normal force 和 torque saturation。不要持续 `xfrc_applied` 后忘记清零。

逐级增加 impulse 估计 region of attraction；failure 分为 slip、tip、joint limit、torque saturation、contact loss 和 solver/numerical。单一“摔倒”标签不利于改进。

## 37.11 Sensor 与 estimator

真实 controller 输入：base orientation/angular velocity 来自 IMU+estimator，joint encoder，foot contact/F-T。不能直接读取 truth base linear velocity/CoM。EKF/leg odometry 依赖 stance assumption；foot slip 时 measurement model 要降权或切换。

仿真验收分两层：truth-state feedback 上界；sensor/estimator closed loop 部署候选。二者性能差距量化 estimator/control interface 的损失。

## 37.12 常见误区

- 把 free quaternion 四个差值放入 LQR；
- 只统计一个 foot contact point；
- 对支撑力取绝对值掩盖符号错误；
- CoM 在 polygon 内就声称动态稳定；
- keyframe 没做 root inverse-force feasibility；
- 对 free root 直接输出六维 torque；
- position servo 站住就称全身控制完成；
- LQR 跨脚离地/contact switch 使用；
- controller 读取 truth base velocity；
- 外力 pulse 未清零。

## 37.13 习题与答案

1. 本项目 `nq=17,nv=16` 的差异来自哪里？  
   **答案：**free joint orientation 用 4D quaternion configuration，但只有 3D angular velocity tangent。

2. 两脚 Fz 总和小于重量 20%，先检查什么？  
   **答案：**是否漏统计 foot geom/contact、force frame 转换/符号、是否有其他支撑、模型 total mass 和是否仍在加速。

3. CoM 在 support polygon 外是否一定立即倒？  
   **答案：**不一定，动态动量和可迈步可恢复；但准静态站立通常不可持续。

4. root inverse force 不为零意味着什么？  
   **答案：**所声称 equilibrium 需要未驱动外力维持，应调整 pose/contact/control setpoint。

5. 为什么 WBC 要同时决策 contact force？  
   **答案：**base acceleration 只能通过关节和环境 contact 产生，contact force 还受单边/摩擦约束，不能事后任意指定。
