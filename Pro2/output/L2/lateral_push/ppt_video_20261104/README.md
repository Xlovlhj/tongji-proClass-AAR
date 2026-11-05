# L2｜实验四：侧向扰动对照补录

此目录保存 PPT 首页视频使用的 Isaac Sim 实录，三组各 12 秒、30 fps。在同一个画面中同时播放，按仿真时间对齐，便于观察 7 秒时的侧推及后续恢复。

| 子目录 | 原实验配置 | 唯一变量 `push_force[1]` |
|---|---|---|
| `no_push/` | `Pro2/experiments/L2/lateral_push/no_push.json` | 0 N |
| `push_50N/` | `Pro2/experiments/L2/lateral_push/push_50N.json` | 50 N |
| `push_100N/` | `Pro2/experiments/L2/lateral_push/push_100N.json` | 100 N |

共同条件为 `push_at = 7.0 s`、`push_duration = 0.1 s`，三组使用同一 N=10 C++ 控制库。零扰动组仍保留相同扰动时间设置，力的大小为零。

从项目根目录运行以下命令可复现 50 N 条件；另两组对应替换配置和输出子目录。重录应使用新的输出子目录，保留本次记录。

```text
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L2 --config Pro2/experiments/L2/lateral_push/push_50N.json --record-video --video-fps 30 --output Pro2/output/L2/lateral_push/ppt_video_20261104/push_50N
```

每组的完整命令另存 `recording_command.json`；`config.json`、`run.json`、`states.csv`、`demo.mp4` 分别保存实际配置、运行清单、状态数据和原速录像。补录使用原有 play 仿真循环，不修改正式实验条件。

原 L2 报告及历史比较图保持原数据来源。本目录用于展示动作，不能将旧报告数值冒充本次新运行结果。最终校验保存在 `Pro2/output/final/checks/recordings_verification.json`，包括力的实际取值范围、完整解码、帧数、完成度及实际 QP 状态计数。

