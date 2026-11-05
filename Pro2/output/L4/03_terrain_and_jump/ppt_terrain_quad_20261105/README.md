# L4 四种地形同屏视频与补录存档

本目录保存 2026-11-05 为 PPT 第一页四种地形同屏演示补录的随机障碍、盲踩上台阶视频，以及这两次运行独立的状态数据和分析图片。粗糙地面、5° 坡道直接复用已有完整录像。

- [24 秒四地形同屏视频](comparison/terrain_quad.mp4)
- [四格画面预览](comparison/preview.png)
- [完整 132 秒汇报演示](../../../final/assets/项目验收演示_L0_L2_L4.mp4)

四格画面共同截取仿真 **0–24 s**，按原速同时播放。四组都在 5 s 开始行走，画面展示从站立、走向地形到进入地形的过程。观察重点是两腿是否持续交替迈步、身体是否明显晃动、遇到起伏后是否继续行走。随机障碍还有后续 21 s 原始录像；主报告使用完整正式状态日志分析，避免只看共同短片而遗漏后段波动。

## 四格输入

以下路径均从项目根目录开始。

| 地形 | 原始录像 | 原始时长 | 四格显示区间 |
| --- | --- | --- | --- |
| 粗糙地面 | `Pro2/output/L4/03_terrain_and_jump/ppt_video_20261104/rough_medium/demo.mp4` | 28 s | 0–24 s |
| 5° 坡道 | `Pro2/output/L4/03_terrain_and_jump/ppt_video_20261104/slope_05deg/demo.mp4` | 24 s | 0–24 s |
| 随机障碍 | `Pro2/output/L4/03_terrain_and_jump/ppt_terrain_quad_20261105/random_obstacles_seed01_full/demo.mp4` | 45 s | 0–24 s |
| 盲踩上台阶 | `Pro2/output/L4/03_terrain_and_jump/ppt_terrain_quad_20261105/blind_step_up/demo.mp4` | 24 s | 0–24 s |

四组沿用各自地形实验配置，展示不同场景中的行走效果。它们的速度和抬脚高度不同，因此不作为只改变地形的单变量对照实验：

| 地形 | 配置路径 | 前进速度 | 抬脚高度 |
| --- | --- | --- | --- |
| 粗糙地面 | `Pro2/experiments/L4/terrain/rough_medium.json` | 0.08 m/s | 0.16 m |
| 5° 坡道 | `Pro2/experiments/L4/terrain/slope_05deg.json` | 0.10 m/s | 0.14 m |
| 随机障碍 | `Pro2/experiments/L4/terrain/random_obstacles_seed01_full.json` | 0.08 m/s | 0.18 m |
| 盲踩上台阶 | `Pro2/experiments/L4/terrain/blind_step_up.json` | 0.08 m/s | 0.18 m |

## 本次补录结果

两次补录均在 Isaac Sim 的仿真运行循环中直接录制，保留完整原始时长。每个运行目录分别保存 `recording_command.json`、`config.json`、`run.json`、`states.csv`、`demo.mp4` 和日志，没有通过动画或插值代替仿真。

| 补录 | 运行结果 | 视频帧数 | 状态采样数 | 末端基座 x | 地形进展 |
| --- | --- | --- | --- | --- | --- |
| 随机障碍 | 完成 45 s | 1350 | 4500 | 3.378 m | 基座约 40 s 越过实际障碍区域末端；结束时也已越过配置区域末端 3 m |
| 盲踩上台阶 | 完成 24 s | 720 | 2400 | 1.623 m | 进入台阶区域约 18%，尚未穿越整个平台 |

以上进展依据基座位置判断，只说明本段录像覆盖到了哪里。两次都正常执行到预设时长；没有走完整个平台不表示行走失败，也不能据此判断哪种地形更不稳定。状态文件没有记录足端接触的地形对象，不能据此断言双脚已踩过每个障碍或完整确认上台阶接触过程。原始精确时间、位置和误差指标保存在各运行的 `analysis/metrics.json` 中，表中展示时间按汇报习惯取整。

两组分别通过 18 项录像完整性检查，包括完整视频解码、帧数与仿真时长对应、CSV 数值有限、采样时间递增、有效物理参数与原配置一致、控制库哈希一致。核验记录位于 `Pro2/output/final/checks/terrain_recordings_verification.json`。这些检查确认录像与数据一致；行走质量仍要结合实测关节、姿态和速度分析，不能由覆盖比例替代。

## 独立分析与正式报告的关系

新增分析分别保存在：

- `Pro2/output/L4/03_terrain_and_jump/ppt_terrain_quad_20261105/random_obstacles_seed01_full/analysis/`
- `Pro2/output/L4/03_terrain_and_jump/ppt_terrain_quad_20261105/blind_step_up/analysis/`

每组包含 `metrics.json`、平面轨迹、基座高度、基座姿态、前进速度四张图片。两次补录结果与原正式实验存在数值差异，因此只在本目录归档，不覆盖原正式运行，也不更新 `Pro2/output/L4/03_terrain_and_jump/terrain_summary.json` 或 `.csv`。`Pro2/output/L4/L4验收报告.md` 和 PPT 的地形数据分析继续引用原正式数据，第一页同屏视频引用上表中的录像。正式步态与稳定性指标统一存于 `Pro2/output/L4/03_terrain_and_jump/gait_stability_analysis/summary.json`，原始几何进度继续作为覆盖信息保留。

需要重新核验这两段录像时，在项目根目录运行：

```text
python3 Pro2/output/final/verify_demo_recordings.py --only L4/random_obstacles_seed01_full --only L4/blind_step_up
```

该命令只核验这两组并更新独立的地形录像核验记录，不启动仿真，也不覆盖已有其他 17 段录像的总核验记录。
