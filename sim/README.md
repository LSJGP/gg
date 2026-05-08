# sim — Waymo NPC 上跑开源 planner 的闭环仿真

纯 Python (零第三方依赖, Python ≥ 3.10):

1. 重放 Waymo Motion 的 NPC 轨迹（车 / 人 / 自行车）；
2. 在 SDC 上跑一个开源 planner（默认 **IDM + Pure Pursuit**）；
3. 用真实矩形包围盒 (OBB) 模拟车辆体积，做 SAT 碰撞检测；
4. 按法规级豁免规则把碰撞分类（被追尾豁免 / 被加塞豁免 / 逆向迎面豁免 …）；
5. 同步喂给 `grading_mini` 的 C++ scorer (`grading_main --stream`)，
   并在进程里也跑一份 Python OnlineGrader，两边产出同样的 `grading_report.json`。

## 目录

```
sim/
├── run_sim.py                      CLI 入口
└── waymo_sim/
    ├── geometry.py                 OBB + SAT 碰撞测试
    ├── vehicle.py                  Bicycle 模型 + 车体几何中心 OBB
    ├── scenario.py                 解析 scenario_meta + dynamic_objects
    ├── lane_graph.py               BFS 路由 + 折线重采样
    ├── world.py                    主 tick 循环 + 法规豁免分类
    ├── grading_stream.py           Python 在线 grader + SimLog 写盘 + C++ stream pipe
    └── planners/
        ├── base.py                 Planner ABC + PlanCommand + NPCSnapshot
        ├── idm_pure_pursuit.py     IDM 跟车 + Pure Pursuit 转向
        └── trajectory_follower.py  通用 trajectory→PlanCommand 适配器
```

## 跑通三步

```bash
# 0. 已经有 scenarios/waymo_scenario_5/ 这个示例可以直接用；
#    要自己转新 scenario 见 ../tools/run_converter.sh

# 1. 编一次 grading_mini (只需第一次)
cd ../grading_mini
PATH=$(echo "$PATH" | sed 's|/usr/lib/ccache:||g') CC=/usr/bin/gcc CXX=/usr/bin/g++ \
  bazel build //src/entry:grading_main \
  --action_env=PATH=/usr/local/bin:/usr/bin:/bin --action_env=CC=/usr/bin/gcc --action_env=CXX=/usr/bin/g++

# 2. 跑闭环仿真，Python 与 C++ 同时在线打 PASS/FAIL
cd ../sim
python3 run_sim.py \
  --scenario-dir ../scenarios/waymo_scenario_5 \
  --output /tmp/sim_log.json \
  --grading-bin ../grading_mini/bazel-bin/src/entry/grading_main \
  --grading-report /tmp/grading_report.json
  # 默认 --cpp-mode online；其他: offline / both / off
```

## 加新 planner

```python
# sim/waymo_sim/planners/my_planner.py
from .base import Planner, PlanCommand, NPCSnapshot
from .trajectory_follower import trajectory_to_command, Waypoint

class MyPlanner(Planner):
    name = "my_planner"
    def __init__(self, reference_path, speed_limit_mps, **kw):
        self._impl = ThirdPartyPlanner(...)
    def plan(self, ego_state, npcs, t, dt) -> PlanCommand:
        traj = self._impl.run(ego_state, npcs, t)
        wps = [Waypoint(x=p.x, y=p.y, yaw=p.yaw, speed=p.v, t=p.t) for p in traj]
        return trajectory_to_command(ego_state, wps, desired_speed_mps=...)
```

注册到 `planners/__init__.py::PLANNERS`，命令行 `--planner my_planner` 即可切换。

## 法规豁免阈值（在 `world.py` 顶部）

| 常量                                  | 含义                                         |
|--------------------------------------|---------------------------------------------|
| `EXEMPT_REAR_END_EGO_SPEED_MAX`      | ego 视为 "慢速 / 停车" 的速度上限 (m/s)      |
| `EXEMPT_REAR_END_REL_SPEED_MIN`      | 后车比 ego 至少快多少才算 "明显追尾" (m/s)   |
| `EXEMPT_CUT_IN_LAT_VEL_MIN`          | NPC 横向冲入 ego 车道的最小横向速度 (m/s)    |
| `EXEMPT_HEAD_ON_ANGLE_DEG`           | 运动方向夹角超过此值视为逆向迎面 (deg)       |
