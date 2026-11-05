# L4｜不平整地面、斜坡与跳跃补录

本目录保存 PPT 首页汇总视频使用的三项原片。每次运行独立保存，原 L4 正式报告和此前验收数据不覆盖。

| 子目录 | 原实验配置 | 仿真时长 |
|---|---|---|
| `rough_medium/` | `Pro2/experiments/L4/terrain/rough_medium.json` | 28 s |
| `slope_05deg/` | `Pro2/experiments/L4/terrain/slope_05deg.json` | 24 s |
| `vertical_jump/` | `Pro2/experiments/L4/jump/vertical_jump.json` | 13 s |

从项目根目录运行：

```text
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L4 --config Pro2/experiments/L4/terrain/rough_medium.json --record-video --video-fps 30 --output Pro2/output/L4/03_terrain_and_jump/ppt_video_20261104/rough_medium
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L4 --config Pro2/experiments/L4/terrain/slope_05deg.json --record-video --video-fps 30 --output Pro2/output/L4/03_terrain_and_jump/ppt_video_20261104/slope_05deg
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L4 --config Pro2/experiments/L4/jump/vertical_jump.json --record-video --video-fps 30 --output Pro2/output/L4/03_terrain_and_jump/ppt_video_20261104/vertical_jump --log-control-decomposition
```

重录应更换输出子目录。每组保存 `recording_command.json`、`config.json`、`run.json`、`states.csv`、`demo.mp4`。跳跃额外记录控制分解，便于核查起跳、腾空、触地与恢复过程；该日志开关不改变控制条件。

补录使用原有 play 仿真循环和正式 JSON 配置。地形段重点观察交替迈步、身体姿态和速度维持；“进入地形”与“穿过地形”只描述录像覆盖范围，不能把达到预设时长后停止录制解释为行走失败。跳跃段应依据本次 `jump_metrics.json`（分析后生成）描述腾空和落地，不能把目标高度当成实际高度。对应新分析保存在各次运行目录内。

最终校验保存在 `Pro2/output/final/checks/recordings_verification.json`，包含原片全程解码、帧数、仿真完成度、配置一致性、源码/控制库哈希和实际 QP 状态计数。历史报告中的定量结论仍对应历史正式运行。
