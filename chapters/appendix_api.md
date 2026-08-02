# 附录 A　C API、数组形状与生命周期速查

本附录用于回忆，不替代 API Reference。函数签名、nullable、版本变化以 `mujoco-3.11.0/include/mujoco/mujoco.h` 与本地官方 HTML 为准。

## A.1 对象所有权

| 创建/取得 | 销毁 | 所有权备注 |
|---|---|---|
| `mj_loadXML`, `mj_loadModel`, `mj_compile` | `mj_deleteModel` | 返回独立 model |
| `mj_makeData` | `mj_deleteData` | 每 worker/trajectory 独占 |
| `mj_makeSpec`, `mj_parseXML*` | `mj_deleteSpec` | spec 拥有 elements/strings |
| `mj_makeVFS`/`mj_defaultVFS` | `mj_deleteVFS` | default 初始化后必须 delete |
| `mjv_makeScene` | `mjv_freeScene` | 先 default，再 make |
| `mjr_makeContext` | `mjr_freeContext` | platform GL context 必须 current |
| `mju_openResource` | `mju_closeResource` | provider 决定 byte buffer 生命周期 |

推荐销毁顺序与创建相反。data 必须先于它引用的 model 删除；render context 在 EGL/GLFW/OSMesa context 销毁前 free。

## A.2 加载、编译与保存

| 任务 | API |
|---|---|
| MJCF/URDF → model | `mj_loadXML` |
| MJB → model | `mj_loadModel` |
| XML file/string → spec | `mj_parseXML`, `mj_parseXMLString` |
| empty spec | `mj_makeSpec` |
| spec → model | `mj_compile` |
| state-preserving update | `mj_recompile` |
| model → MJB | `mj_saveModel` |
| spec → normalized XML | `mj_saveXML`, `mj_saveXMLString` |
| last loaded XML save | `mj_saveLastXML` |
| VFS buffer/file | `mj_addBufferVFS`, `mj_addFileVFS` |
| dependency enumeration | `mju_getXMLDependencies` |

`mj_loadXML` 的 error buffer 与 `mj_compile` 后 `mjs_getError(spec)` 不同。任何返回 NULL 都不能继续 make data。

## A.3 主仿真 pipeline

| API | 作用 | 是否推进 time |
|---|---|---:|
| `mj_forward` | 给定 q/v/act/control 计算派生量和 qacc | 否 |
| `mj_inverse` | 给定 q/v/qacc 计算 inverse force | 否 |
| `mj_step` | forward dynamics + integration | 是 |
| `mj_step1` | position/velocity stage | 否 |
| `mj_step2` | control/actuation/acceleration/integration | 是 |
| `mj_forwardSkip` | 跳过已知不变的早期 stage | 否 |
| `mj_inverseSkip` | inverse pipeline skip | 否 |
| `mj_resetData` | qpos0 + 其他 state default | 重置为 0 |
| `mj_resetDataKeyframe` | keyframe state | 设置 key time |

修改主状态后派生量 stale，调用 forward。RK4 与 step1/step2 control 语义有限制，查当前 API 文档。

## A.4 `mjModel` 常用计数

3.11 中多项计数类型是 `mjtSize`，打印时安全转换或使用匹配格式，不假设 `int`。

| 字段 | 含义 |
|---|---|
| `nq`, `nv` | configuration / velocity tangent dimension |
| `nu`, `na` | controls / activation states |
| `nbody`, `njnt`, `ngeom`, `nsite` | tree/geometry elements |
| `ncam`, `nlight` | model cameras/lights |
| `ntendon`, `neq` | tendons/equalities |
| `nsensor`, `nsensordata` | sensor definitions / packed output scalars |
| `nplugin`, `npluginstate` | plugin instances/state |
| `nM` | sparse qM storage |
| `nuser_*` | user-data widths |

## A.5 主状态与 input

| `mjData` 字段 | 形状 | 说明 |
|---|---:|---|
| `time` | 1 | simulation time |
| `qpos` | `nq` | configuration，含 quaternion |
| `qvel` | `nv` | tangent velocity |
| `act` | `na` | actuator activation |
| `qacc_warmstart` | `nv` | constraint solver warmstart |
| `ctrl` | `nu` | control inputs |
| `qfrc_applied` | `nv` | user generalized force |
| `xfrc_applied` | `6*nbody` | world force+torque at body COM |
| `mocap_pos` | `3*nmocap` | mocap world position |
| `mocap_quat` | `4*nmocap` | mocap world quaternion |
| `eq_active` | `neq` | equality enable state |
| `plugin_state` | `npluginstate` | engine-managed plugin state |

applied force buffers 持续存在，不是一步脉冲；不再施加时显式清零。

## A.6 地址表

| 元素 | 关键 model fields |
|---|---|
| joint | `jnt_qposadr`, `jnt_dofadr`, `jnt_type`, `jnt_range` |
| body | `body_parentid`, `body_jntadr/num`, `body_dofadr/num`, `body_mass` |
| geom | `geom_bodyid`, `geom_type`, `geom_size`, `geom_contype/conaffinity` |
| site | `site_bodyid`, `site_pos`, `site_quat` |
| actuator | `actuator_trntype`, `actuator_trnid`, `actuator_actadr/num`, ranges |
| sensor | `sensor_adr`, `sensor_dim`, `sensor_type`, `sensor_objtype/id` |
| keyframe | `key_qpos`, `key_qvel`, `key_act`, `key_ctrl`, `key_time` |
| plugin | `plugin_stateadr`, `plugin_attradr`, `sensor_plugin` |

始终 `id=mj_name2id(m,objtype,name)`，检查 `id>=0`，再查 address。不要把 ID 当 qpos index。

## A.7 派生 kinematics

| 字段 | 形状 | frame |
|---|---:|---|
| `xpos`, `xquat`, `xmat` | `3/4/9*nbody` | body frame in world |
| `xipos`, `ximat` | `3/9*nbody` | body inertial frame in world |
| `geom_xpos`, `geom_xmat` | `3/9*ngeom` | world |
| `site_xpos`, `site_xmat` | `3/9*nsite` | world |
| `cam_xpos`, `cam_xmat` | `3/9*ncam` | world |
| `subtree_com` | `3*nbody` | subtree CoM in world |
| `cdof` | `6*nv` | motion axes in COM-centered frame convention |

旋转矩阵一般 row-major。`mjContact.frame` 是特殊布局：三个 contact axes 按行连续。

## A.8 Jacobian 与流形

| 任务 | API | 形状 |
|---|---|---:|
| body COM Jacobian | `mj_jacBodyCom` | `jacp/jacr: 3*nv` |
| body frame Jacobian | `mj_jacBody` | `3*nv` |
| geom/site/point-axis | `mj_jacGeom`, `mj_jacSite`, `mj_jacPointAxis` | `3*nv` |
| subtree CoM Jacobian | `mj_jacSubtreeCom` | `3*nv` |
| integrate tangent | `mj_integratePos` | qpos + nv vector |
| configuration difference | `mj_differentiatePos` | nv result |
| normalize quaternion parts | `mj_normalizeQuat` | qpos in place |
| quaternion residual | `mju_subQuat` | 3-vector |

`jacp[row*nv+dof]`。finite difference configuration 必须用 integrate/differentiate，而非裸数组加减。

## A.9 Dynamics 与 force

| 任务 | API/字段 |
|---|---|
| sparse mass matrix | `d->qM` |
| dense mass matrix | `mj_fullM` |
| matrix-vector product | `mj_mulM`, `mj_mulM2` |
| solve Mx=b | `mj_solveM`, `mj_solveM2` |
| bias force | `d->qfrc_bias` |
| passive/actuator/constraint | `qfrc_passive/actuator/constraint` |
| inverse result | `qfrc_inverse` |
| Cartesian wrench→generalized | `mj_applyFT` |
| object velocity/acceleration | `mj_objectVelocity`, `mj_objectAcceleration` |
| forward-inverse diagnostic | `mj_compareFwdInv`, `solver_fwdinv` |

`mj_applyFT` 累加到 target array。unit motor 之外不能假设 ctrl 等于 generalized force。

## A.10 Contact 与 constraint

| 字段/API | 含义 |
|---|---|
| `ncon`, `contact[i]` | detected contact records |
| `contact.geom[2]` | geom IDs |
| `contact.pos`, `dist`, `frame`, `dim` | point/distance/frame/dimension |
| `contact.exclude`, `efc_address` | 是否进入 constraint 与 scalar address |
| `mj_contactForce` | intuitive contact-frame 6D wrench |
| `nefc`, `efc_*` | scalar constraint system |
| `qfrc_constraint` | generalized constraint force |
| `solver_niter[mjNISLAND]` | per-island iterations |
| `solver[]` | per-iteration statistics |
| `mj_isPyramidal` | friction cone representation |
| `mj_island` | discover constraint islands |

金字塔锥的 `efc_force` 分量不是 XYZ，优先 `mj_contactForce`。

## A.11 Sensor

```cpp
int id = mj_name2id(m, mjOBJ_SENSOR, "imu_gyro");
int adr = m->sensor_adr[id];
int dim = m->sensor_dim[id];
const mjtNum* value = d->sensordata + adr;
```

检查 sensor type/object/frame/stage。不要认为同一 family 所有 sensor 都固定 3 维。

## A.12 State specification

| API | 作用 |
|---|---|
| `mj_stateSize(m,spec)` | bitmask 对应 scalar 数 |
| `mj_getState` | data → flat buffer |
| `mj_setState` | flat buffer → data primary fields |

常用 bitmask：`mjSTATE_PHYSICS`、`mjSTATE_FULLPHYSICS`、`mjSTATE_INTEGRATION`，以及 time/qpos/qvel/act/ctrl/applied/mocap/equality/plugin/warmstart 等组合。具体枚举以 header 为准。

set state 后通常 forward。控制器/估计器/RNN hidden state 不自动包含在 engine state。

## A.13 Finite difference 与 optimization

| API | 输出 |
|---|---|
| `mjd_transitionFD` | A `(2nv+na)^2`、B `(2nv+na)*nu`、sensor C/D |
| `mjd_inverseFD` | inverse force/sensor derivatives |
| `mju_cholFactor/Solve` | SPD factor/solve |
| `mju_boxQP` | box-constrained local QP |
| `mju_eig3` | symmetric 3×3 eigendecomposition |

transition FD 不支持 RK4；contact model 先固定 solver path/epsilon sweep。

## A.14 Visualization 与 rendering

| 生命周期/任务 | API |
|---|---|
| defaults | `mjv_defaultCamera/Option/Perturb/Scene`, `mjr_defaultContext` |
| scene allocation | `mjv_makeScene/freeScene` |
| state→scene | `mjv_updateScene` |
| camera/select/perturb | `mjv_moveCamera`, `mjv_select`, `mjv_applyPerturb*` |
| GPU resources | `mjr_makeContext/freeContext` |
| framebuffer | `mjr_setBuffer`, `mjr_resizeOffscreen` |
| render/readback | `mjr_render`, `mjr_readPixels` |
| overlay/figure/UI | `mjr_overlay`, `mjr_figure`, `mjui_*` |

`mjv_*` 抽象场景不需要 GL；`mjr_*` 需要 current platform context。

## A.15 mjSpec

| 任务 | API |
|---|---|
| find | `mjs_findBody`, 其他 element find/traversal |
| add tree | `mjs_addBody/Joint/FreeJoint/Geom/Site/Frame/Camera/Light` |
| add top-level | `mjs_addActuator/Sensor/Tendon/Equality/Pair/...` |
| name/string/vector | `mjs_setName`, `mjs_setString`, `mjs_setDouble` |
| attach | `mjs_attach` |
| error/warning | `mjs_getError`, `mjs_getWarning` |
| ownership query | `mjs_getSpec`, `mjs_getOriginSpec`, `mjs_getParent` |

add 返回的 element 由 spec 拥有，不能单独 delete。compile/recompile 后重查所有 ID/address/pointer。

## A.16 Plugin/resources

| 任务 | API |
|---|---|
| plugin default/register/get | `mjp_defaultPlugin`, `mjp_registerPlugin`, `mjp_getPlugin` |
| instance config | `mj_getPluginConfig` |
| provider register | `mjp_registerResourceProvider` |
| resource I/O | `mju_open/read/closeResource`, `mju_writeResource` |
| decoder/encoder | `mjp_registerDecoder/Encoder`, `mju_decodeResource`, `mj_encode` |

注册表是进程级全局；plugin per-data state 必须隔离并实现完整生命周期。

## A.17 官方资料定位

| 问题 | 本地入口 |
|---|---|
| 物理/概念 | `docs/html/overview.html`, `modeling.html`, `computation/` |
| MJCF 属性 | `docs/html/XMLreference.html` |
| C types/functions/globals | `docs/html/APIreference/` |
| simulation/thread/contact | `docs/html/programming/simulation.html` |
| visualization/render/UI | `docs/html/programming/visualization.html`, `ui.html` |
| mjSpec | `docs/html/programming/modeledit.html` |
| plugins/resources | `docs/html/programming/extension.html` |
| official programs | `docs/html/programming/samples.html` |
| version changes | `docs/html/changelog.html` |

检索顺序：header signature → API preconditions/shapes → computation/modeling semantics → changelog → 最小独立实验。
