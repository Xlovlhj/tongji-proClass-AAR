# L2｜实验三：抬脚高度对照补录

此目录保存 PPT 首页视频使用的 Isaac Sim 实录，三组各 12 秒、30 fps。同屏播放按仿真时间对齐；每组只改变 `step_height`，共同使用同一 N=10 C++ 控制库。

| 子目录 | 原实验配置 | 抬脚高度 |
|---|---|---|
| `low_008/` | `Pro2/experiments/L2/step_height/low_008.json` | 0.08 m |
| `baseline_012/` | `Pro2/experiments/L2/step_height/baseline_012.json` | 0.12 m |
| `high_016/` | `Pro2/experiments/L2/step_height/high_016.json` | 0.16 m |

从项目根目录运行以下命令可复现低抬脚条件；另两组对应替换配置和输出子目录。重录应使用新的输出子目录，保留本次记录。

```text
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L2 --config Pro2/experiments/L2/step_height/low_008.json --record-video --video-fps 30 --output Pro2/output/L2/step_height/ppt_video_20261104/low_008
```

每组的完整命令另存 `recording_command.json`；`config.json`、`run.json`、`states.csv`、`demo.mp4` 分别保存实际配置、运行清单、状态数据和原速录像。补录使用原有 play 仿真循环，不修改正式实验条件。

原 L2 报告及历史比较图保持原数据来源。本目录用于展示动作，不能将旧报告数值冒充本次新运行结果。最终校验保存在 `Pro2/output/final/checks/recordings_verification.json`，包括唯一变量检查、完整解码、帧数、完成度及实际 QP 状态计数。

