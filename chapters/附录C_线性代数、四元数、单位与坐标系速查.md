# 附录 C　线性代数、四元数、单位与坐标系速查

> 本书示例代码仓库：[libing403/mujoco_tutorial](https://github.com/libing403/mujoco_tutorial)

## C.1 Row-major

MuJoCo dense matrix 通常 row-major：`A[row*ncol+col]`。Jacobian `3×nv` 的第 `axis,dof` 项为 `J[axis*nv+dof]`。contact frame 的三个轴也按行连续。

## C.2 不求显式逆

| 问题 | 方法 |
|---|---|
| SPD `Ax=b` | Cholesky (`mju_cholFactor/Solve`) |
| least squares | QR/SVD 或 normal equation + damping（小问题） |
| DLS high-DoF/low-task | 解 `(JJᵀ+λ²I)y=e`, 再 `dq=Jᵀy` |
| box local QP | `mju_boxQP` |

显式 `A^{-1}b` 通常更慢、更不稳且浪费内存。

## C.3 Quaternion

MuJoCo quaternion 采用 scalar-first `[w,x,y,z]`。`q` 与 `-q` 表示同一旋转。configuration 中 ball/free quaternion 必须单位化；位置 difference 在 3D tangent，而非四维分量差。

- 更新：`mj_integratePos`
- 差：`mj_differentiatePos`
- 单 quaternion residual：`mju_subQuat`
- 乘/逆/旋转 vector：`mju_mulQuat`, `mju_negQuat`, `mju_rotVecQuat` 等，查 header 签名

## C.4 Virtual work

\[
\delta W=f^T\delta x=\tau^T\delta q,\quad \delta x=J\delta q
\Rightarrow \tau=J^Tf.
\]

用于 contact、actuator transmission、external wrench 与 operational-space control。

## C.5 Dynamics

\[
M(q)\dot v+c(q,v)=\tau_{act}+\tau_{passive}+\tau_{applied}+J^Tf_c.
\]

固定基 fully actuated computed torque：

\[
\tau=M(\ddot q_d+K_d\dot e+K_pe)+c.
\]

浮动基：

\[
M\dot v+c=S^T\tau+J_c^Tf_c.
\]

## C.6 Contact wrench aggregation

局部→世界：若 contact axes 是 frame 行，

\[
f_w=R_{cw}^Tf_c.
\]

关于参考点：

\[
F=\sum f_i,\qquad
\tau_{p_0}=\sum[(p_i-p_0)\times f_i+\tau_i].
\]

## C.7 常用单位

| 量 | SI |
|---|---|
| length | m |
| mass | kg |
| time | s |
| angle in C API | rad |
| linear/angular velocity | m/s, rad/s |
| force/torque | N, N·m |
| inertia | kg·m² |
| stiffness | N/m or N·m/rad（按 DoF） |
| damping | N·s/m or N·m·s/rad |
| density | kg/m³ |

MJCF angle 可用 compiler degree/radian；长度没有自动 mm→m，mesh scale 必须审计。

## C.8 Frame convention 清单

每个 vector 都标注 `{world}`, `{body}`, `{site}`, `{contact}`, `{camera}`。测试正方向：单位 axis/force/rotation，打印 world result。camera/OpenGL/CV frames 必须用 marker 投影验证，禁止凭记忆换轴。
