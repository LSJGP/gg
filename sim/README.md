# sim — C++ 闭环仿真（Bazel）

当前主链路已迁移到 C++（`sim/cpp/sim_runner`）并使用 Bazel 构建：

1. 读取 Waymo 导出的 JSON 场景（`scenario_meta` / `dynamic_objects` / `lane_graph`）；
2. 在 C++ world loop 中执行 planner、车辆模型、OBB/SAT 碰撞与豁免分类；
3. 每帧将 `MetricFrameInput` JSON **入队**，由后台线程写入 `grading_main --stream` 的 stdin（仿真线程不等待 pipe / grading 速度；`Run` 结束后 `Finish` 会排空队列并关闭管道）。
4. 同时落盘 `sim_log.json`，可用于离线 batch 评分。

`run_sim.py` 仍保留为兼容入口，但内部会调用 C++ `sim_runner`。  
Python 仿真代码已独立到 `../pysim`。

## 目录

```
sim/
├── run_sim.py                      C++ 仿真兼容入口（转调 Bazel binary）
├── cpp/                            C++ 仿真实现
├── WORKSPACE                       Bazel workspace
└── .bazelrc                        Bazel build 配置
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

# 2. 跑闭环仿真（调用 C++ sim_runner）
cd ../sim
python3 run_sim.py \
  --scenario-dir ../scenarios/waymo_scenario_5 \
  --output /tmp/sim_log.json \
  --grading-bin ../grading_mini/bazel-bin/src/entry/grading_main \
  --grading-report /tmp/grading_report.json
  # 默认 --cpp-mode online；其他: offline / both / off
```
