# 附录 B　MJCF 全元素族检索表

本表用于从任务定位 XML Reference。属性默认值、单位、继承、互斥和版本细节请打开本地 `docs/html/XMLreference.html`。

| 任务 | 元素/子族 | 主章节 |
|---|---|---:|
| 文档组合 | `include`, `frame`, `replicate`, `attach` | 4、13、28 |
| compiler 全局 | `compiler` | 4 |
| physics option | `option`, `flag` | 5、17–20 |
| capacity/memory | `size` | 13、18–19 |
| visualization scaling | `statistic`, `visual` | 31–32 |
| defaults | `default` 及 joint/geom/site/tendon/actuator 子项 | 4、8–12 |
| geometry assets | `mesh`, `hfield`, `skin` | 7、12、28 |
| appearance assets | `texture`, `material` | 7、32 |
| plugin/model assets | `asset/plugin`, `asset/model` | 28、34 |
| rigid tree | `worldbody`, `body`, `inertial` | 2、4、13 |
| joints | `joint`, `freejoint` | 2、8 |
| collision/visual | `geom`, geom plugin | 7、17–18、34 |
| marker/frame | `site` | 2、11、14 |
| vision | `camera`, `light` | 31–32 |
| automatic objects | `composite` 子族、`flexcomp` 子族 | 12 |
| explicit contact | `contact/pair`, `exclude` | 7、18 |
| deformable | `deformable/flex` 与 edge/contact/elasticity/skin | 12 |
| equality | `connect`, `weld`, `joint`, `tendon`, `flex*` | 12、17 |
| spatial tendon | `spatial/site/geom/pulley` | 12 |
| fixed tendon | `fixed/joint` | 12 |
| generic actuator | `general`, actuator plugin | 9–10、34 |
| force shortcut | `motor`, `damper`, `cylinder`, `muscle`, `adhesion`, `dcmotor` | 9–10 |
| servo shortcut | `position`, `velocity`, `pid`, `orientation`, `intvelocity` | 9–10、21 |
| kinematic sensors | joint/tendon/actuator position/velocity/force families | 11 |
| inertial sensors | `accelerometer`, `velocimeter`, `gyro`, `magnetometer`, `camprojection` | 11、27 |
| force/contact sensors | `touch`, `force`, `torque`, limit/contact/tactile families | 11、18、34 |
| frame sensors | `framepos/quat/xaxis/yaxis/zaxis/linvel/angvel/linacc/angacc` | 11 |
| subtree/global | `subtreecom`, `subtreelinvel`, `subtreeangmom`, energy/clock/user/plugin | 11、27、34 |
| snapshots | `keyframe/key` | 3、13 |
| application metadata | `custom/numeric`, `text`, `tuple` | 13、34 |
| extension declaration | `extension/plugin/instance/config` | 34 |

## 属性审计六问

检索任意属性时回答：

1. 单位和 angle convention？
2. local/world/parent/child/contact 哪个 frame？
3. compile-time constant 还是 runtime state？
4. default 值及 default class 继承？
5. 与哪些属性互斥、shortcut 最终展开为什么？
6. 编译后落在 `mjModel` 哪个字段，如何用实验读回？

只有 XML 文本和 compiled truth 闭环，才能避免“作者以为”和“engine 实际”的差异。
