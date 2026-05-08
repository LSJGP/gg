# hywgrading — Waymo 数据集闭环仿真 + 法规级碰撞豁免评测

一个**只接 Waymo Motion 数据集**作为输入的轻量闭环评测项目。一行流水线：

```
Waymo TFRecord
   │  tools/waymo_to_scenario.py
   ▼
scenarios/<name>/{scenario_meta, dynamic_objects, lane_graph}.json
   │  sim/run_sim.py
   ▼
ego (kinematic bicycle + OBB)  ←  IDM + Pure Pursuit (可换其他开源 planner)
   │
   ├──► OnlineGrader (Python 在线，stdout 实时打 PASS/FAIL)
   ├──► CppOnlineGrader (Python → grading_main --stream，C++ 在线打 [cpp] tick)
   └──► SimLog JSON (落盘留档；也可被 grading_main 离线再打一次)
        │
        ▼
   grading_report.json (planning_limit_checker / speed_checker /
                        regulatory_collision_checker)
```

整套不依赖 ROS、Autoware、Docker。Python 只用标准库（>= 3.10）；C++ 用 Bazel + protobuf，单次 `bazel build` 之后就一个二进制。

## 目录速览

```
hywgrading/
├── README.md                         你正在看的这份
├── tools/                            Waymo TFRecord -> scenario JSON
│   ├── waymo_to_scenario.py
│   ├── list_scenarios.py
│   └── run_converter.sh
├── scenarios/
│   └── waymo_scenario_5/             示例场景，已经转好可以直接跑
│       ├── scenario_meta.json
│       ├── dynamic_objects.json
│       └── lane_graph.json
├── sim/                              Python 闭环仿真器
│   ├── run_sim.py
│   ├── README.md
│   └── waymo_sim/
│       ├── geometry.py
│       ├── vehicle.py
│       ├── scenario.py
│       ├── lane_graph.py
│       ├── world.py
│       ├── grading_stream.py
│       └── planners/
│           ├── base.py
│           ├── idm_pure_pursuit.py
│           └── trajectory_follower.py
└── grading_mini/                     C++ 评分框架（DAG + metric 注册）
    ├── proto/grading/                proto 定义
    └── src/
        ├── entry/grading_main.cc     批处理 + --stream 入口
        ├── grading/                  metric DAG 调度核心
        ├── grading/metrics/          具体指标（example/, safety/）
        └── planning/simple_planner.* 仅在批处理 fallback 用的占位
```

## 三步跑通

```bash
# 0. 数据：要么有 Waymo TFRecord，要么直接用项目自带的 scenarios/waymo_scenario_5/
export HYW_DATA_DIR=/path/to/waymo/uncompressed/scenario   # 仅当你要转新场景时

# 1. 编一次 grading_mini（Bazel 自己拉依赖）
cd grading_mini
PATH=$(echo "$PATH" | sed 's|/usr/lib/ccache:||g') CC=/usr/bin/gcc CXX=/usr/bin/g++ \
  bazel build //src/entry:grading_main \
  --action_env=PATH=/usr/local/bin:/usr/bin:/bin \
  --action_env=CC=/usr/bin/gcc --action_env=CXX=/usr/bin/g++

# 2.（可选）转一个新场景
cd ..
SCENARIO_INDEX=5 tools/run_converter.sh scenarios/waymo_scenario_5

# 3. 跑闭环 + 同时在线 Python grader + C++ grader
cd sim
python3 run_sim.py \
    --scenario-dir ../scenarios/waymo_scenario_5 \
    --output /tmp/sim_log.json \
    --grading-bin ../grading_mini/bazel-bin/src/entry/grading_main \
    --grading-report /tmp/grading_report.json
```

退出码：`0` 表示所有指标 PASS，`10` 表示有指标 FAIL，方便 CI 卡点。

---

# 模块逐个讲解

下面每一节列出该模块所有面向使用者的函数 / 类 / 字段，附带它的输入输出和职责。

## tools/waymo_to_scenario.py

把一个 Waymo Motion `Scenario` protobuf（来自 TFRecord shard 里的某一条记录）转成 sim 直接吃的 3 个 JSON。整个过程只做"读取 + 平移 + 序列化"，不做任何插值或几何重建。

### Waymo 枚举映射常量

| 名称                  | 作用                                                              |
| --------------------- | ----------------------------------------------------------------- |
| `WAYMO_OBJECT_TYPE`   | track 的 `object_type` 整数 → 字符串 (`VEHICLE/PEDESTRIAN/...`)   |
| `WAYMO_LANE_TYPE`     | lane 的 `type` 整数 → 字符串 (`FREEWAY/SURFACE_STREET/...`)       |
| `DEFAULT_SPEED_LIMIT_KMH` | Waymo 没记限速时给的兜底值 (50 km/h)                          |

### 几何工具

`_polyline_to_xyz(polyline) -> List[(x,y,z)]`
把 Waymo `MapPoint` 列表展平成普通元组列表。`z` 字段如果不存在就给 0。

`_lane_speed_kmh(ln) -> float`
读取 lane 的 `speed_limit_mph` 字段并换算成 km/h。0 或不存在则返回 `DEFAULT_SPEED_LIMIT_KMH`。

### 核心数据类

`@dataclass ConvertedScene`
所有 writer 都接受这个对象作为参数。字段：

| 字段                       | 含义                                                          |
| -------------------------- | ------------------------------------------------------------- |
| `scenario_id`              | Waymo 提供的全局唯一 ID                                       |
| `world_offset`             | (ox, oy, oz)，把 SDC 起点平移到原点用的偏移                   |
| `init_pose / goal_pose`    | SDC 第一/最后一个有效帧的位姿（已减去 offset）                |
| `timestamps_seconds`       | 每帧的相对时间（约 91 帧、9 秒）                              |
| `current_time_index`       | 历史/未来分界帧（Waymo 的 prediction 任务概念）               |
| `sdc_track_index`          | `tracks[i].is_sdc=True` 的那个 i                              |
| `tracks`                   | 全部 track 的 dict 列表（含 SDC，SDC 标了 `is_sdc=True`）     |
| `track_type_counts`        | 非 SDC 各类 NPC 数量 `{VEHICLE: 40, PEDESTRIAN: 3, ...}`      |
| `lane_graph`               | 车道图的 dict 列表                                            |
| `bbox`                     | 整张地图的 XY 包围盒（`xmin/ymin/xmax/ymax`，已经平移过）     |
| `num_lanes / num_road_lines / num_road_edges / num_crosswalks` | 地图要素计数 |

### 三个抽取函数

`_extract_tracks(scenario, ox, oy, oz, sdc_idx) -> (tracks, counts)`
遍历每个 track 的逐帧状态：valid 帧填 `{x,y,z,yaw,vx,vy,length,width,height}`（坐标已减偏移），invalid 帧只填 `{valid: False}` 占位。返回 dict 列表 + 非 SDC 各类型计数。

`_extract_lane_graph(scenario, ox, oy, oz) -> List[Dict]`
扫所有 `MapFeature`，挑出类型是 lane 的，记录：`id / type / speed_limit_kmh / centerline / entry_lanes / exit_lanes`。`centerline` 是已经平移过的折线点列表。

`_scene_stats(scenario) -> (nl, nrl, nre, ncw, raw_bbox)`
按 lane / road_line / road_edge / crosswalk 各类要素计数，并扫所有点合并出 XY bbox（注意此时还没减 offset，调用方自己减）。

### 顶层入口

`convert_scenario(scenario, center_on_sdc=True) -> ConvertedScene`
完整流水线：找 SDC → 算 (ox, oy, oz) → 调三个 _extract_* → 组装 ConvertedScene。如果 `center_on_sdc=False`，offset 全置 0，保留 Waymo 原始全局坐标。

### Writer

`write_meta(scene, path, source, scenario_index)` — 写 `scenario_meta.json`，含 init / goal / world_offset / bbox / 各种统计。

`write_dynamic_objects(scene, path, source)` — 写 `dynamic_objects.json`，体积主要在 `tracks` 数组（每个 track 的逐帧状态）。

`write_lane_graph(scene, path, source)` — 写 `lane_graph.json`，主要是 `lanes` 数组。

`_pose(p)` — 把 `(x, y, yaw)` 元组序列化成 `{"x":..,"y":..,"yaw":..}` 字典，None 透传 None。

### CLI

`_resolve_data_dir(explicit) -> str`
按优先级查找 Waymo 数据目录：命令行参数 > `HYW_DATA_DIR` > `GRADING_DATA_DIR` > 项目内 `data/`。

`main()` — 解析参数、加载 TFRecord、定位到第 N 个 scenario、调 `convert_scenario`、调三个 writer、打印统计。

## tools/list_scenarios.py

辅助脚本：列出 TFRecord shard 前 N 个场景的"SDC 行驶距离 / 帧数 / 车道数"。挑场景用——travel_m 太小（< 5m）的场景一上来就到目标点了，不适合做闭环。

`_resolve_data_dir(explicit)` — 同上。

`main()` — 遍历前 `--limit` 个 scenario，对每个算 `n_lanes / n_frames / travel`，按对齐格式打印。

## tools/run_converter.sh

包装脚本，在 conda 环境里跑 `waymo_to_scenario.py`。环境变量：

- `SCENARIO_INDEX` — TFRecord shard 内第几条
- `TFRECORD` — 显式指定单个文件
- `HYW_DATA_DIR` — 数据目录
- `CONDA_ENV` — conda 环境名（默认 `waymo_env`）
- `NO_CENTER=1` — 关掉 SDC-anchor，保留全局坐标

---

## sim/run_sim.py

CLI 入口。整个 main flow：解析参数 → 加载 scenario → 用 lane graph 做 BFS 路由 → 构造 ego (BicycleVehicle) → 实例化 planner → 构造 World → 接 hooks（`OnlineGrader` + `SimLogWriter` + 可选 `CppOnlineGrader`）→ `World.run()` → 必要时再跑一次离线 grading_main → 按 `OnlineGrader.overall_passed` 决定退出码。

`_parse_args(argv)`
所有命令行选项都在这里。重点参数：

- `--scenario-dir` (必填) — 三个 JSON 所在目录
- `--planner` — 名字，从 `PLANNERS` 字典里挑
- `--dt` — 仿真步长（默认 0.1s）
- `--output` — SimLog 落盘路径
- `--grading-bin` — `grading_main` 二进制路径；不传就纯 Python grader
- `--grading-report` — C++ 报告输出路径
- `--cpp-mode {online,offline,both,off}` — C++ 何时跑（详见下文）
- `--ego-{length,width,wheelbase,rear-overhang,max-speed}` — 自车几何/动力学
- `--desired-speed` — IDM 的 v0
- `--reference-step` — 参考路径重采样间距 (m)
- `--no-interpolate-npcs` — NPC 时间插值改用最近邻
- `--stop-on-collision` — 第一次撞就停

`_build_route(scenario, lane_graph, init, goal, step) -> (path, speed_limit_mps, route_ids, used_fallback)`
做路由：起终点最近车道 → BFS 接 exit_lanes → 拼 centerline → 头尾接 init/goal → 等距重采样。失败兜底直线。

`main(argv)`
按上面列的 flow 串起来。返回 0 / 10。

---

## sim/waymo_sim/geometry.py

OBB（有向矩形包围盒）几何工具，用 SAT（分离轴定理）做相交测试。

`@dataclass(frozen=True) OBB(cx, cy, heading, half_length, half_width)`
中心点 + 朝向 + 沿朝向的半长 + 垂直方向的半宽。

`OBB.corners() -> List[(x, y)]`
把矩形 4 个角变换到世界坐标。顺序：(+L,+W) (+L,-W) (-L,-W) (-L,+W)。

`OBB.axes() -> List[(ax, ay)]`
SAT 的两个候选轴（前向 + 侧向）。两个 OBB 的 4 个轴做投影测重叠。

`_project(corners, axis) -> (min, max)`
把一组顶点往轴上投影，返回投影区间。

`obb_overlap(a, b) -> bool`
SAT：4 个轴上都重叠 → True；任何一个轴上不重叠就立刻 False。

`obb_overlap_with_inflation(a, b, inflate) -> bool`
把 b 的半长半宽都加 `inflate` 米，再判重叠。做安全裕度检查时方便。

`aabb_of(boxes) -> (xmin, ymin, xmax, ymax)`
一组 OBB 的轴对齐外包矩形。Pure 工具，目前主路径没用，留给 viz / 加速结构。

---

## sim/waymo_sim/vehicle.py

自车的车辆模型：`Bicycle` 动力学 + 车体几何中心 OBB。

`@dataclass VehicleParams`
车体常量：`length / width / height / wheelbase / rear_overhang / max_speed / max_accel / max_decel / max_steer / max_steer_rate`。默认值是一辆典型轿车（4.5×1.85m，35° 最大转角）。

`@dataclass VehicleState`
位姿：`x, y, heading, speed, acceleration, steer`。`x/y` 是后轴位置（bicycle 模型的标准做法），`heading` 是车身朝向，`steer` 是当前转角（用于做转角速率限制）。

`@dataclass BicycleVehicle(state, params)`

- `step(accel_cmd, steer_cmd, dt)`
  完整一拍的离散积分：
  1. accel 限幅到 `[-max_decel, max_accel]`；
  2. steer 限幅到 `[-max_steer, max_steer]`，再叠一次速率限幅 `max_steer_rate * dt`；
  3. 速度 `v += a*dt` 并夹到 `[0, max_speed]`（不允许倒车）；
  4. heading `θ += (v/wb) * tan(δ) * dt`；
  5. xy `x += v*cos(θ)*dt; y += v*sin(θ)*dt`。
- `bbox() -> OBB`
  把后轴坐标平移到车体几何中心（前移 `length/2 - rear_overhang`），得到 OBB。这个 OBB 是给碰撞检测用的，**和动力学积分用的后轴坐标分离**——这是关键的"模拟车辆体积"那一步。

---

## sim/waymo_sim/scenario.py

加载 `scenarios/<dir>/` 三个 JSON 到内存。

`@dataclass TrackState`
逐帧状态：`valid / x / y / z / yaw / vx / vy / length / width / height`。

`@dataclass Track`
一整条 track：`track_index / id / object_type / is_sdc / states (List[TrackState])`。

`@dataclass Pose2D(x, y, yaw)`
2D 位姿。`init_pose / goal_pose` 用。

`@dataclass Scenario`
顶层场景：`scenario_id / world_offset / init_pose / goal_pose / timestamps_seconds / current_time_index / sdc_track_index / tracks / lane_graph_path / dynamic_objects_path / meta`。

- `Scenario.num_steps` — 等于 `len(timestamps_seconds)`，便利属性。
- `Scenario.npcs_at(t_idx) -> List[Track]` — 第 t_idx 帧所有 valid 且非 SDC 的 track 列表。
- `Scenario.sdc_track() -> Optional[Track]` — 把 SDC 那一条 track 单独取出来（参考用，sim 主路径不用）。

`_pose(d) -> Optional[Pose2D]` — JSON 字典 → Pose2D，None 透传。

`load_scenario(scenario_dir) -> Scenario`
读三个 JSON，构造 Scenario 对象，必须的文件不存在直接抛 `FileNotFoundError`。

`scenario_dt(scenario) -> float`
从 `timestamps_seconds` 估计原生帧间隔（`mean(diff)`），不到 2 帧就返回 0.1s 兜底。

---

## sim/waymo_sim/lane_graph.py

车道图 + BFS 路由 + 折线工具。

`@dataclass Lane(id, type, speed_limit_kmh, centerline, entry_lanes, exit_lanes)`
单条车道。

`class LaneGraph(lanes: List[Lane])`

- `LaneGraph.load(path) -> LaneGraph` — 读 `lane_graph.json` 构造图。
- `closest_lane(x, y, heading=None, only_drivable=True, max_dh=π/2) -> Optional[Lane]`
  在所有 lane 的折线段上找离 `(x,y)` 最近的那一段，可选用 heading 排除掉反向车道（`max_dh` 是最大允许夹角）。
- `shortest_path(start_id, goal_id) -> List[int]`
  以 lane 为节点、`exit_lanes` 为有向边，跑 BFS。返回中间经过的 lane id 列表；找不到返回空列表。
- `route_centerline(lane_ids) -> List[(x,y,z)]`
  把 lane id 序列对应的 centerline 顺序拼接成一根折线，相邻 lane 的接缝点（< 0.5m）会去重。
- `speed_limit_mps(lane_ids, default_kmh=50) -> float`
  路径上所有 lane 限速取最小，再换算成 m/s。

`resample_polyline(pts, step=1.0) -> List[(x,y,z)]`
按弧长等间距重采样折线。planner 需要均匀采样的参考路径。

`fallback_path_to_goal(start, goal, step=2.0) -> List[(x,y,z)]`
路由失败时用：起终点之间直线均分，作为 reference path 兜底。

`_point_to_segment_dist(px, py, x1, y1, x2, y2)` — 点到线段的最短距离。

`_wrap_pi(a)` — 把角度归一到 `[-π, π]`。

---

## sim/waymo_sim/world.py

仿真主世界 + 法规豁免分类逻辑。

### 法规豁免阈值常量（顶部，可调）

| 常量                                | 含义                                              |
| ----------------------------------- | ------------------------------------------------- |
| `EXEMPT_REAR_END_EGO_SPEED_MAX`     | ego 视为"慢"的速度上限 (m/s, 默认 5.0)            |
| `EXEMPT_REAR_END_REL_SPEED_MIN`     | NPC 比 ego 至少快多少 (m/s, 默认 2.0)             |
| `EXEMPT_CUT_IN_LAT_VEL_MIN`         | NPC 横向冲入 ego 的最小横向速度 (m/s, 默认 0.5)   |
| `EXEMPT_HEAD_ON_ANGLE_DEG`          | 运动方向夹角超此值算逆向迎面 (deg, 默认 135)       |

### 数据结构

`@dataclass CollisionInfo`
一帧的碰撞结论。字段：`collided / other_id / kind / ego_at_fault / exempt / exempt_reason / relative_speed_mps / ego_speed_mps / approach_angle_deg`。`kind` 取 `ego_front_into_npc / npc_rear_into_ego / side_collision`。

`@dataclass FrameRecord`
一帧的全部产物：`frame_id / timestamp_us / ego (VehicleState) / command (PlanCommand) / collision (CollisionInfo) / num_npcs`。`World.run` 返回的就是 `List[FrameRecord]`。

`@dataclass WorldConfig`
仿真全局开关：`dt / max_seconds (0=用 scenario 自带时长) / stop_on_first_collision / interpolate_npcs / vehicle_params`。

`class FrameHook`
观察者基类：`on_frame(rec)` 每帧回调，`on_finish(records)` 跑完回调。grading_stream.py 里的 `OnlineGrader / SimLogWriter / CppOnlineGrader` 都是它的子类。

### `class World(scenario, ego, config)`

- `run(planner, hooks=None) -> List[FrameRecord]`
  主循环。每个 step：
  1. 算 scenario 时间 `t0 + step*dt`；
  2. 取这一刻的 NPCs（线性插值或最近邻）；
  3. 调 `planner.plan(ego.state, npcs, t, dt)`；
  4. `ego.step(accel, steer, dt)`；
  5. 用 ego 的 OBB 跟所有 NPC OBB 做 SAT 相交；
  6. 撞了就调 `_classify_collision` 决定是否豁免；
  7. 组装 `FrameRecord`，调每个 `hook.on_frame`；
  8. 如配置了 `stop_on_first_collision` 且撞了，break。
  最后调每个 `hook.on_finish`。

- `_npcs_at_time(scenario_time) -> List[NPCSnapshot]`
  按 scenario 时间在 `timestamps_seconds` 里二分定位区间，然后或最近邻、或调 `_interp_npcs` 插值。

- `_npcs_at_index(idx) -> List[NPCSnapshot]`
  最近邻：第 idx 帧 valid 的非 SDC track 列表。

- `_interp_npcs(lo, hi, a) -> List[NPCSnapshot]`
  线性插值：xy / vx / vy 直接 lerp，yaw 走最短路径角度插值。两端只有一个 valid 时退化为最近邻。

- `_detect_collision(npcs) -> CollisionInfo`
  ego OBB 跟每个 NPC OBB 跑 SAT。如果有多个同时相交，取"最严重"的那个：非豁免优先于豁免，同等条件下取相对速度更大的。

### 模块私有函数

`_track_to_snapshot(tr, st) -> NPCSnapshot`
单条 track 在某一帧的 state → planner 接口要的 `NPCSnapshot`。

`_scenario_dt(scenario) -> float`
跟 `scenario.py::scenario_dt` 一样的逻辑，重复定义为了避免循环 import。

`_classify_collision(ego, npc) -> CollisionInfo`
**法规豁免的核心逻辑**：

1. 在 ego 车身坐标系下算 NPC 的方位角 `bearing`（0=正前，±π=正后，±π/2=两侧）；
2. 算运动方向夹角 `approach_angle`（NPC 与 ego 速度方向之间）；
3. 按 |bearing| 把碰撞分到 `front / rear / side` 三个区；
4. 根据规则决定 `ego_at_fault / exempt / exempt_reason`：
   - **rear**: ego 慢（< 5m/s）且 NPC 明显更快 → 豁免 `rear_end_on_slow_ego`
   - **side**: NPC 横向速度往 ego 这一侧冲过来 → 豁免 `forced_cut_in`
   - **front**: 运动方向夹角 > 135° → 豁免 `wrong_way_head_on`
   - 其他: ego 担责。

---

## sim/waymo_sim/grading_stream.py

把仿真过程接到三个 grader 上去。

### Python 在线 grader

`class OnlineGrader(FrameHook)`
进程里实现的"复刻 C++ metric 的小型 grader"，零依赖，只为了实时打印。

- `__init__(max_speed_mps, max_desired_speed_mps, sink, print_every)` —
  装载阈值（要和 C++ 那边一致才会同结果）。
- `on_frame(rec)` —
  分别调 `_tick_speed / _tick_limit / _tick_collision`，每 `print_every` 帧或撞了就在 `sink` 上打一行 `[grader] f=K t=Ts ... -> PASS/FAIL`。
- `on_finish(records)` —
  打三个 metric 的最终摘要 + OVERALL。
- `summaries() -> List[MetricSummary]` —
  和 C++ 那边 `GradingReport.summaries` 同结构的列表（`name / passed / detail`）。
- `overall_passed: bool` —
  全部 metric 都 PASS。

私有 ticker：

- `_tick_speed(rec)` — `ego.speed > max_speed_mps` 算违规。
- `_tick_limit(rec)` — `command.desired_speed_mps > max_desired_speed_mps` 算违规。
- `_tick_collision(rec)` — 按 `rec.collision.exempt` 分桶；首次非豁免碰撞记下 `_first_collision_frame`。

### SimLog 落盘

`class SimLogWriter(FrameHook)`
把每帧序列化成 `MetricFrameInput` 加进数组，按 `flush_every` 频率以原子写（写到 `*.tmp` 再 rename）刷到磁盘。这样 C++ 离线 batch 模式能拿到完整 SimLog。

- `on_frame(rec)` — append + 必要时 flush。
- `on_finish(records)` — 最后再 flush 一次确保收尾。
- `_flush()` — 原子写。
- `_frame_to_dict(rec) -> dict` — 把 `FrameRecord` 转成 SimLog frame 的 dict。包含了 `vehicle_state / planning_command / collision_event`。

### C++ 在线 grader

`class CppOnlineGrader(FrameHook)`
启动一个常驻 `grading_main --stream` 子进程，每帧通过 stdin 喂一行 JSON，结束时关 stdin、等子进程收尾、`grading_main` 写出 `grading_report.json`。

- `__init__(binary_path, report_path, sink)` —
  `_start()` 出 subprocess.Popen：`stdin=PIPE`、`stdout=None`（直接打到当前 TTY，让 `[cpp]` 行和 `[grader]` 行交错出现）、`stderr=PIPE`（spdlog 的输出，由后台 daemon 线程 `_pump_stderr` 转发）。
- `on_frame(rec)` —
  `json.dumps(_frame_to_dict(rec))` 成一行，写 stdin、flush。stdin 异常就置 `_closed=True`，sim 继续跑（不会因为 grader 挂了而崩）。
- `on_finish(records)` / `close()` —
  关 stdin → `proc.wait(timeout=30)` → 超时就 kill。
- `_pump_stderr(stream)` —
  daemon 线程把 grading_main 的 stderr（spdlog 的 INFO/WARN/ERROR）转发到 Python 进程的 stderr。

---

## sim/waymo_sim/planners/

`PLANNERS: Dict[str, Type[Planner]]`
名字到类的注册表。CLI `--planner <name>` 走这里。

### planners/base.py

`@dataclass NPCSnapshot`
planner 看到的 NPC 视图：`id / object_type / x / y / z / heading / vx / vy / length / width / height`。注意 length/width 给到 planner 是为了它自己做"包络感知"决策，不是 sim 用的（sim 直接用 Track.states 的尺寸）。

`@dataclass PlanCommand`
planner 输出：`target_acceleration / steering_angle / desired_speed_mps / leader_id (Optional) / leader_gap / debug`。

- `target_acceleration / steering_angle` 直接喂给 `BicycleVehicle.step`。
- `desired_speed_mps` 是给 `planning_limit_checker` 这个指标看的"我打算开多快"。
- `leader_id / leader_gap` 用于 debug 日志和 viz。
- `debug` dict 任意字段，sim 不解释。

`class Planner(ABC)`
抽象基类。`name: str` 用于 CLI 注册；`plan(ego_state, npcs, t, dt) -> PlanCommand` 唯一必须实现的方法。

### planners/idm_pure_pursuit.py

默认 planner：纵向 IDM + 横向 Pure Pursuit 跟参考线。

`@dataclass IDMParams`
IDM 模型参数：`desired_speed (v0) / time_headway (T) / min_gap (s0) / max_accel (a) / comfort_decel (b) / delta / detection_radius / leader_lateral_tol`。

`@dataclass PurePursuitParams`
PP 参数：`k_lookahead / min_lookahead / max_lookahead / wheelbase`。lookahead 距离 `ld = clamp(min_ld + k * v, min_ld, max_ld)`。

`class IDMPurePursuitPlanner(Planner)`

- `__init__(reference_path, speed_limit_mps, idm=None, pp=None)` —
  把参考路径里 XY 距离做累计弧长 `_arc`，方便后面按弧长插点。`idm.desired_speed` 会被 clip 到 `speed_limit_mps`。
- `plan(ego, npcs, t, dt) -> PlanCommand` —
  1. 把 ego XY 投到参考路径，得到当前弧长 `ego_arc`；
  2. 横向：取 `ego_arc + ld` 处那个点，算 alpha，套 PP 公式 `δ = atan2(2L sin α, ld)`；
  3. 纵向：调 `_find_leader` 找路径前方"前车"，调 `_idm` 算加速度；
  4. 终点制动：若距离终点 `< max(2, v*1.5)` 强制最小减速度；
  5. 组 PlanCommand 返回。
- `_project(x, y) -> (seg, t, px, py)` — 点到折线投影。
- `_arc_at(seg, t)` — `(seg, t) -> 弧长`。
- `_lookahead(ego_arc, ld)` — 沿弧长前进 `ld` 距离的点。
- `_find_leader(ego, npcs, ego_arc) -> Optional[(id, gap, leader_v)]`
  在路径走廊内（横向 < `leader_lateral_tol + 0.5*width`）选最近的"在前方"NPC 当前车。`gap` 已减去车身一半。
- `_idm(v, gap, leader_v) -> a` — 标准 IDM 公式：`a*(1 - (v/v0)^δ - (s*/s)^2)`。
- `_stop_decel(v, dist)` — 终点制动用的固定减速 `v² / (2·d)`，最大 6 m/s²。

### planners/trajectory_follower.py

通用"轨迹 → PlanCommand"适配器。绝大多数开源 planner（Frenet / CommonRoad-RP / nuPlan IDM / ML 策略）输出的是一段 waypoint 轨迹，用这个 helper 把它转成 sim 能吃的 (accel, steer)。

`@dataclass Waypoint(x, y, yaw, speed, t)`
轨迹上一个采样点。`t` 是相对当前的时间偏移，用于挑"控制 horizon 处"的速度作为参考。

`trajectory_to_command(ego, trajectory, desired_speed_mps, wheelbase, control_horizon, lookahead_min, lookahead_k, lookahead_max, fallback_decel) -> PlanCommand`

- 横向：从 ego 沿轨迹累弧长找到 `lookahead` 处的 waypoint，套 Pure Pursuit。
- 纵向：取 `t >= control_horizon` 的第一个 waypoint 的 speed 当 `v_ref`，用 `(v_ref - v_ego) / control_horizon` 当目标加速度。
- 空轨迹 fallback：以 `fallback_decel` 减速、保持转向 0。

---

## grading_mini/ —— C++ 评分框架

Bazel 工程。所有指标走"注册表 + DAG 调度 + 单一二进制"模式。

### proto/grading/

`metric_input.proto`

| message              | 字段                                                                 |
| -------------------- | -------------------------------------------------------------------- |
| `VehicleState`       | `x / y / heading / speed / acceleration`                             |
| `PlanningCommand`    | `desired_speed_mps`                                                  |
| `CollisionEvent`     | `collided / other_id / kind / ego_at_fault / exempt / exempt_reason / relative_speed_mps / ego_speed_mps / approach_angle_deg` |
| `MetricFrameInput`   | `frame_id / timestamp_us / vehicle_state / planning_command (opt) / collision_event (opt)` |

`metric_output.proto`

| message              | 字段                                                                 |
| -------------------- | -------------------------------------------------------------------- |
| `MetricFrameOutput`  | `frame_id / bool_value / custom_info (Any)`                          |
| `MetricSummary`      | `metric_name / passed / detail`                                      |
| `GradingReport`      | `overall_passed / summaries (repeated MetricSummary)`                |

`sim_log.proto`

| message              | 字段                                                                 |
| -------------------- | -------------------------------------------------------------------- |
| `SimLog`             | `source (string) / frames (repeated MetricFrameInput)`               |

`metrics/example_metric.proto` — 单个 metric 自己的可选 config（`SpeedChecker { max_speed_threshold }` 等）。

### src/grading/ — DAG 框架

`metric_base.{h,cc}`
`class MetricBase`：所有 metric 的基类。要实现 3 个虚函数：

- `Init(config) -> Status` — 注入配置；本项目里所有 metric 都是 nullptr config。
- `CalculateOneFrame(input, history, output) -> Status` — 处理一帧；写 `output->set_bool_value(...)` 表达"这一帧本指标 PASS/FAIL"。
- `SummarizeResult(history) -> StatusOr<MetricSummary>` — 全局收尾，按累积状态决定 `passed / detail`。

附带 `set_name(name)` / `dependencies()` / `set_dependency_payload()`，配合 DAG 用。

`metric_register.h`
`REGISTER_METRIC(ClassName, "string_name")` 宏：把 (`name`, factory) 一对在静态初始化阶段塞进 `MetricFactory` 的全局表。

`metric_factory.{h,cc}`
`MetricFactory::Instance()` 单例。`Create(name) -> StatusOr<unique_ptr<MetricBase>>`。

`metric_result_payload.h`
`Payload<T>`：滑动窗口缓冲（默认 2048 帧），存 `MetricFrameOutput`。每个 metric 一个，方便"看历史"的指标实现（比如 jerk 累计）。

`metric_scheduling_policy.{h,cc}`
`DAGScheduler`：拿一组 metric 名 + 它们之间的依赖（A 依赖 B 表示 A 可以读 B 的 payload），跑拓扑分层，输出 `UpdatePlan = Vector<Vector<string>>`（每一层内部并行无依赖）。当前 3 个 metric 都没声明依赖，所以是 1 层 3 个并行。

`metric_manager.{h,cc}`
把 metrics、payloads、DAG plan 包在一起：

- `AddMetric(name, metric)` — 登记。
- `BuildGraph()` — 跑拓扑、把每个 metric 依赖的 payload 指针注入回去。
- `RunOneFrame(input)` — 按 DAG 层次调每个 metric 的 `CalculateOneFrame`，把输出存回 payload。
- `GenerateReport()` — 调每个 metric 的 `SummarizeResult`，组装 `GradingReport`。
- `LastFrameVerdicts() -> vector<(name, bool)>` — **新增**：返回最近一帧每个 metric 的 `bool_value`，给 `--stream` 模式打 tick 行用。

`grader.{h,cc}`
`Grader` 是 `MetricManager` 的薄包装（`Init / ProcessFrame / Finish / LastFrameVerdicts`），是入口程序唯一接触的对象。

### src/grading/metrics/

#### example/speed_checker.{h,cc}
`REGISTER_METRIC(SpeedChecker, "speed_checker")`

- 阈值 `max_speed_ = 33.3` (m/s, ~120 km/h)，可被 `SpeedChecker` proto config 覆写。
- `CalculateOneFrame` — `vehicle_state.speed > max_speed_` 算违规；`set_bool_value(!exceeded)`。
- `SummarizeResult` — `passed = (violation_count == 0)`，`detail = violations=K/N`。

#### example/planning_limit_checker.{h,cc}
`REGISTER_METRIC(PlanningLimitChecker, "planning_limit_checker")`

- 阈值 `max_speed_mps_ = 33.3`，硬编码（没接 config）。
- `CalculateOneFrame` — 没 `planning_command` 也算违规；有就比较 `desired_speed_mps`。
- `SummarizeResult` — `passed = (violation_frames == 0)`，`detail = bad_frames=K/N`。

#### safety/regulatory_collision_checker.{h,cc}
`REGISTER_METRIC(RegulatoryCollisionChecker, "regulatory_collision_checker")`

**法规级豁免指标，本项目的核心。**

- `CalculateOneFrame` — 没 `collision_event` 或 `collided=False` → 这一帧 PASS；否则按 `exempt` 字段分桶（豁免 → bool_value=true，非豁免 → bool_value=false），同时用 spdlog 打 WARN/ERROR 行说明现场。
- `SummarizeResult` — `passed = (non_exempt_frames_ == 0)`，`detail = "non_exempt=K exempt=M total_collision_frames=K+M/N"`。

### src/planning/simple_planner.{h,cc}

只在 batch 模式作占位用。它没有任何"规划智能"，只做一件事：如果 `MetricFrameInput` 没带 `planning_command`，就把当前速度（clip 到 `max_speed_mps`）填进去当 desired_speed。这样旧的不带 planner 输出的 SimLog（早期 ROS bridge 时代的格式）也能过 `planning_limit_checker`。

`SimplePlanner(max_speed_mps=33.3)` — 构造。
`Plan(MetricFrameInput*)` — 上面说的那一件事。

### src/entry/grading_main.cc

唯一的可执行二进制。两个入口：

```
grading_main <input.json|.pb> [output.json]    # batch
grading_main --stream         [output.json]    # online
```

#### 批处理路径

`PrintUsage(argv0)` — 打印帮助。

`ParseDouble(s, key)` — 极简 JSON 数字提取，给"老格式 legacy 数组"用。

`ParseLegacyFrames(content) -> List<FrameData>` — 处理早期 `[{frame_id,...}]` 数组。

`ToProto(FrameData) -> MetricFrameInput` — legacy → proto。

`ReadWholeFile / EndsWith / TrimCopy` — IO 辅助。

`LoadMetricInputs(path, *out, *source) -> bool` — 三态加载：`.pb` 读二进制 SimLog；JSON 起首是 `{"source": ...}` 当新格式 SimLog；起首是 `[` 当 legacy 数组。

`WriteReport(report, path) -> bool` — protobuf JSON 序列化。

`RunBatch(input_path, output_path)` — 上面这些拼起来的批处理 main：load → init Grader 注册三个 metric → 循环 ProcessFrame → Finish → WriteReport → 打总结。

#### 在线路径

`RunStream(output_path)`

1. `setvbuf(stdout, _IOLBF)` — 让 stdout 行缓冲，子进程的父进程能立刻读到 tick 行；
2. 注册 3 个 metric，初始化 Grader；
3. 打 `[cpp] stream ready`；
4. 循环 `getline(cin, line)`：每行 `JsonStringToMessage` 解 `MetricFrameInput`，跑 `planner.Plan`，跑 `grader.ProcessFrame`，调 `grader.LastFrameVerdicts()` 拿这一帧每个 metric 的 bool，打 `[cpp] frame=K t=Ts v=v coll=Y/n PASS/FAIL [m1=P/F m2=P/F m3=P/F]`；
5. EOF 后 `grader.Finish()` → 写 `grading_report.json` → 打跟批处理同样的总结。

`main(argc, argv)` — 派发：`--stream` → `RunStream`；`--help/-h` → 打帮助；其他 → `RunBatch`。

---

# 怎么扩展

| 想做的事                      | 改这里                                                                                       |
| ----------------------------- | -------------------------------------------------------------------------------------------- |
| **加一个 planner**            | `sim/waymo_sim/planners/<your>.py` 实现 `Planner` 子类，注册到 `planners/__init__.py::PLANNERS` |
| **改豁免规则 / 加新豁免类型** | `sim/waymo_sim/world.py::_classify_collision`；同步改 C++ 的 `regulatory_collision_checker.cc` |
| **加一个新 metric**           | `grading_mini/src/grading/metrics/<group>/<name>.{h,cc,BUILD}` + `REGISTER_METRIC` + 在 `grading_main.cc` 把名字加到 `Init` 列表；同步在 `grading_stream.py::OnlineGrader.summaries()` 加一份 Python 镜像（仅为在线显示一致） |
| **改 ego 体积**               | CLI `--ego-length / --ego-width / --ego-wheelbase / --ego-rear-overhang`；自动反映到 OBB     |
| **接其他数据集**              | 仿照 `tools/waymo_to_scenario.py`，写另一个 converter 输出同样格式的 3 个 JSON               |
| **接 ROS / Apollo planner**   | `planners/<your>.py` 里 `subprocess.Popen` 起外部进程，用 stdio JSON 来回喂                   |

---

# 已知限制 & 设计权衡

- **NPC 是录制重放，不是闭环**：Waymo NPC 不会因 ego 行为改变；这是评测"在固定历史画面下 planner 表现如何"，不是评测"系统的稳定性"。要做闭环 NPC，需要换 BARK / SUMO 之类的 reactive driver model，但接入点同样是 `World._npcs_at_time`。
- **Lane graph 是 Waymo 自己标的，质量参差**：路口处可能缺 `exit_lanes` 或方向错位，导致 BFS 路由失败。失败时 sim 会自动回退到"起终点直线"作为参考路径，并打一行 WARNING。
- **路径跟踪能力受 ego 动力学限制**：默认 `max_steer_rate=180°/s`、`max_accel=2.5/m/s²`、`max_decel=6.0/m/s²`，不会比真车更激进。如果你的 planner 的 PlanCommand 超出这些值，sim 会做硬截断。
- **法规豁免只看单帧瞬时几何**：没有 TTC 历史、没有事故链路追溯。规则简单但可解释性强；要做更严格的责任判定，扩 `_classify_collision` 即可。

---

# 退出码约定

| 退出码 | 含义                               |
| ------ | ---------------------------------- |
| 0      | 仿真完成且所有指标 PASS             |
| 2      | 启动失败（找不到 scenario 等）      |
| 3      | `grading_main` 二进制路径不存在    |
| 10     | 仿真完成，但有指标 FAIL             |
| 其他   | 透传 `grading_main` 的非零退出码   |

CI 里就 `exit_code == 0` 当通过；非 0 都是回归。
