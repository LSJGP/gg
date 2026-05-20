#cd /path/to/hywgrading   # 改成他本机路径

echo "========== 基础 =========="
uname -a
which python3 && python3 --version
which bazel && bazel version 2>/dev/null || echo "MISSING: bazel"
which git && git --version

echo "========== Python 依赖（GIF 用）=========="
python3 -c "import matplotlib; print('matplotlib OK', matplotlib.__version__)" 2>&1 || echo "MISSING: matplotlib"

echo "========== 二进制是否已编译 =========="
SIM=sim/bazel-bin/cpp/sim_runner
GR=grading_mini/bazel-bin/src/entry/grading_main
for f in "$SIM" "$GR"; do
  if [[ -x "$f" ]]; then echo "OK  $f"; else echo "MISSING  $f"; fi
done

echo "========== 场景目录 =========="
ls scenarios/waymo_scenario_5/scenario_meta.json 2>/dev/null && echo "scenario_5 OK" || echo "MISSING scenario files"
ls scenarios/waymo_scenario_9 2>/dev/null | head -3

echo "========== 单场景试跑（看真实报错）=========="
cd sim
python3 run_sim.py \
  --scenario-dir ../scenarios/waymo_scenario_5 \
  --reference-source map \
  --cpp-mode off \
  --output /tmp/mac_test_sim_log.json 2>&1 | tail -40
echo "exit code: $?"
cd ..