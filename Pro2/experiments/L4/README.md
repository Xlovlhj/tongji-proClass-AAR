# L4 实验配置索引

本目录只保存可复现配置，不保存运行结果。正式数据必须使用 `--level L4` 写到
`Pro2/output/L4/` 下的独立实验目录。

- `path_tracking/`：直线、90° 转弯、圆形开环/闭环和多路径点跟踪。当前平地路径实验的新运行会显示青色参考路径，并用绿色、黄色和红色圆盘分别标出起点、途经点和终点；闭合圆的起终点重合标记为洋红色。这些几何没有碰撞属性。
- `terrain/`：5°/10°/组合坡道、三档粗糙地面、三个固定随机种子障碍和三类盲踩台阶。
- `Pro2/experiments/L4/jump/vertical_jump.json`：竖直跳跃正式配置。当前实现仿照上游 `Pro2/OpenLoong-Dyn-Control/demo/jump_mpc.cpp` 的阶段逻辑，依次执行准备、下蹲、蹬伸、上升腾空、下降腾空、落地、恢复和完成；蹬伸/落地使用原 C++ Convex MPC，腾空关闭接触力 MPC 并使用腿部 IK/PV，离地与落地由 Isaac PhysX 左右脚对地接触力判定。`Pro2/output/L4/03_terrain_and_jump/vertical_jump_run35/` 是首次开发通过运行；正式可复现证据来自 `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run01/` 与 `Pro2/output/L4/03_terrain_and_jump/vertical_jump_acceptance_run02/` 两个独立 13 s 运行。

跳跃运行前仍会生成 `jump_preflight.json`。该文件现在应显示三项能力均为 `true`、`ready: true`；旧目录 `Pro2/output/L4/03_terrain_and_jump/vertical_jump_preflight/` 是适配完成前的历史失败证据，保留用于说明功能演进，不代表当前能力。

示例：

```bash
python3 Pro2/run_isaac.py --level L4 \
  --config Pro2/experiments/L4/path_tracking/straight.json \
  --output Pro2/output/L4/02_path_tracking/straight
```

正式跳跃复现与独立验收示例：

```text
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --level L4 --config Pro2/experiments/L4/jump/vertical_jump.json --headless --log-control-decomposition --output Pro2/output/L4/03_terrain_and_jump/vertical_jump_repeat03
python3 Pro2/tools/analyze_l4_jump.py --run Pro2/output/L4/03_terrain_and_jump/vertical_jump_repeat03
```

每次复现必须换一个新的输出子目录；分析器也拒绝覆盖已有的 `analysis/`。

