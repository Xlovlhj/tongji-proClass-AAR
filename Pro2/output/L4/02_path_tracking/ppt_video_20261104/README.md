# L4｜给定路径跟踪补录

`waypoints/` 保存 PPT 首页汇总视频使用的路径跟踪原片。配置来源为 `Pro2/experiments/L4/path_tracking/waypoints.json`，运行 30 秒、录制 30 fps。参考折线路径保留在仿真场景地面上，便于比较机器人行进方向和目标路径。

从项目根目录运行：

```text
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L4 --config Pro2/experiments/L4/path_tracking/waypoints.json --record-video --video-fps 30 --output Pro2/output/L4/02_path_tracking/ppt_video_20261104/waypoints
```

重录应更换输出子目录。原命令保存于 `waypoints/recording_command.json`；本次实际配置、状态、运行清单、录像分别为 `config.json`、`states.csv`、`run.json`、`demo.mp4`。本次 `path_metrics.json` 和 `analysis/`（生成后）只解释本次运行。

原 L4 正式报告、正式路径指标和曲线不覆盖。片段字幕应依据本次状态与指标描述结果，不能仅凭 30 秒运行完成就写“完成整条路径”。最终视频、运行和配置校验保存在 `Pro2/output/final/checks/recordings_verification.json`。

