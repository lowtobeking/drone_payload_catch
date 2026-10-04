# 主动保持（B 侧锁扣）：离线模型成立，Gazebo 落地受阻（记录）

> 承接 `report/hollow_funnel_cup.md`：敞口杯在倾斜>25°时会丢载荷，**保持/携带要靠主动锁扣**。
> 本文做两件事：**(A) 主动保持的离线模型**（去掉 `v_retain` 限制并量化）；**(B) 用 Gazebo
> `DetachableJoint` 把载荷锁到 B 漏斗的 SITL 尝试**（含一个需要后续解决的故障）。
>
> 复现：
> ```bash
> python3 tools/stack_run.py --scenario M6_stack_lock --mc 100    # 离线：锁扣
> PAYLOAD_LOCK=1 bash run_m6_sitl.sh 70                           # SITL：锁扣（当前有故障，见 §3）
> ```

---

## 0. 结论速览

1. **离线模型成立且收益巨大**：主动锁扣把捕获判据从"位置 + 速度"退化为**只看位置**
   （`v_retain` 不再受限），强下击暴流（8 m/s 向下风）下 **100/100** 捕获。
2. **Gazebo 落地先挫后成**：最初用“载荷侧 `DetachableJoint` → B + 远处停车位 spawn”**失败**
   （B 在 TRANSLATE 掉高）；**定位根因**（`DetachableJoint` **在模型 configure 即建关节**）后改成
   **“捕获时在 B 漏斗处就地重生载荷”** → **成功**：`STACK CAPTURED` → 载荷锁定到 B 漏斗 → **随 B 携带到着陆**。
3. 所以**主动保持已端到端跑通**（离线 + SITL 捕获→锁定→携带→落地）。

---

## 1. 离线模型：锁扣 = 去掉 `v_retain`

锁扣一旦合上，载荷被刚性固定，**不再受恢复系数/接触速度限制**（只受位置是否进入口内约束）。
在离线判据里等价于 `v_retain → 很大`。实现：`capture.arm_absorb` 取大值（本处 10 m/s），
并配合 `arm_reach=0.10`（口内有效半径 +0.10）。

新增工况 `M6_stack_lock`：`gap=1.0`，`vert_mode=minimal`，下击暴流 `wind=[0,0,8]`，

```
python3 tools/stack_run.py --scenario M6_stack_lock --mc 100
  成功率 = 100/100 (100%)
  捕获余量(eff_r−hmiss): mean=0.191m  min=0.101m
  接触相对速度=4.044 m/s，实际捕获相对速度=4.801 m/s  ← 已超过漏斗 v_retain=4.04，靠"锁扣"仍捕获
```

对比：不加锁扣时，wz≥6 的下击暴流以 `v_retain=4.04` 会失败（见 `report/m6_robustness_opt.md`）。

**结论**：锁扣直接把"能否保持"从**恢复系数问题**变成**结构强度问题**，是下击暴流/高速接触的**根本解**。

---

## 2. SITL 实现（已搭好，可开关）

- `models/payload_lock/model.sdf`（新增）：载荷 + `DetachableJoint`（`parent=link`，
  `child_model=x500_funnel_1`，`child_link=funnel_link`，attach `/payload/lock`）。
- `payload_node`：参数 `lock_to_b`；收到 ROS `/payload/lock_request` 后向 gz `/payload/lock` 发 `Empty`。
- `b_node`：参数 `lock_to_b`；捕获判定成功时向 `/payload/lock_request` 发 `Empty`。
- `launch/catch_stack_launch.py`：新增 `lock_to_b` 参数并传给 a/b/payload 节点。
- `run_m6_sitl.sh`：`PAYLOAD_LOCK=1` 选用 `payload_lock` 模型并传 `lock_to_b:=true`。
- 全部**默认关闭**，非破坏。

---

## 3. SITL：先失败，定位根因后成功

### 3.1 失败（负结果）：载荷侧关节 + 远处停车位

`PAYLOAD_LOCK=1`（载荷模型直接换成 `payload_lock`，spawn 在停车位 100,100）复现 2 次：
**B 在 TRANSLATE 阶段掉高度、卡在地面**（`pay=None`，未捕获）；而同代码**基线正常捕获**。

**根因（代码级证据）**：`payload_node._spawn_attached` 注释：
> “DetachableJoint 在模型 configure 时即已与 A 建关节；不可再发 attach……”

即 **`DetachableJoint` 在模型 spawn（configure）时就立即建关节**。载荷被生成在 (100,100) 时，
就在**100m 外**被关节到 B 漏斗；B 一机动，这个巨幅偏移的固定关节就把两者往一起拽 → B 掉高。
（M6-moving 没这问题：载荷在 A 正下方**就近生成**，关节偏移≈0。）

### 3.2 修法：捕获时在 B 漏斗处“就地重生”

既然 configure 即建关节，就**让 spawn 位置 = 关节期望位置**：
- 初始仍用**普通载荷**（自由落体，无关节）；
- 捕获时 `b_node` 把 **B 漏斗口的世界 NED 位姿**发给 `payload_node`（`/payload/lock_request`，`Float64MultiArray`）；
- `payload_node` **删除**当前自由载荷，在 B 漏斗处 **spawn `payload_lock`**（偏移≈0）→ configure 即锁定。

### 3.3 成功 ✅

```
B phase=CLIMB→…→TRANSLATE→ALIGN→DIVE→DONE→LAND
B: DIVE plan a_dive=0.00(auto=True) t_c=0.362s
PAYLOAD RELEASED
B: 请求主动保持（在 B 漏斗处重生成并锁定）
*** STACK CAPTURED *** horiz=0.022m rel_v=2.172m/s
payload: 主动保持 —— 已在 B 漏斗处重生成并锁定 @NED=[-0.01 -0.04 -3.71]
B phase=LAND ... pay=[2.12 ...] ← 载荷刚性跟随 B 横向移动（携带）  无 failsafe
```

（注意：`remove`+`create` 服务有 ~15s 延迟，但捕获时 B 已在等待/悬停，载荷重生在 B 漏斗处，结果正确。）

---

## 4. 结论与下一步

**结论**
1. **主动保持值得做**：离线证明它把"保持"从恢复系数问题变成结构问题，下击暴流 8 m/s 仍 100% 捕获。
2. **但用"载荷模型里的 DetachableJoint 指向 B"的 SITL 落地方式不通**（B 掉高度）。
3. 敞口杯（上一份日志）只解决"导向/侧向兜接"，不解决"倾角/携带"；**主动锁扣才解决携带**。

**下一步（把锁扣做得更真机化）**
1. **去掉“重生”带来的视觉跳变/服务延迟**：目前用 `remove`+`create`（~15s 服务延迟）。可改为
   **就近生成 + 当场 attach**（若能做到 detach 期间自由），或写一个**捕获时插入关节**的小插件。
2. **软锁（更接近夹爪/磁吸）**：用 `apply-link-wrench`/自定义系统对载荷施加
   “拉向 B 漏斗”的弹簧-阻尼伺服力，实现**软保持**，避免刚性关节的数值/语义问题。
3. **携带段控制**：锁扣后 B 的等效质量/惯量变化，需验证带载悬停/落点精度（目前能跟随落地）。
4. **真机化**：把“锁扣”换成真实夹爪/磁吸，并把捕获判定与锁扣触发对齐。
