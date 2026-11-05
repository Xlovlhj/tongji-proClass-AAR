# L2 速度权重三条件：PPT 补录演示

本目录保存 2026-11-04 为汇报补录的三条真实 Isaac Sim 视频，以及对应完整运行记录。使用已有 L2 实验配置，以当时最终控制库重新运行。三次串行执行，每条仿真 12 s，以 30 FPS 录制 360 帧，分辨率 960×720，编码 H.264。

本次没有改写 `Pro2/output/L2/mpc_vx_weight/` 下原正式实验及其分析。补录视频用于展示同类参数设置下机器人的运动；原验收报告中的数值和曲线仍来自原正式实验，不能把原曲线标成补录视频逐帧同步的分析。

| 条件 | 唯一变化的字段 | 配置（项目根相对路径） | 视频 |
|---|---:|---|---|
| low | `state_weights[9] = 0.2` | `Pro2/experiments/L2/mpc_vx_weight/low.json` | [low/demo.mp4](low/demo.mp4) |
| baseline | `state_weights[9] = 1` | `Pro2/experiments/L2/mpc_vx_weight/baseline.json` | [baseline/demo.mp4](baseline/demo.mp4) |
| high | `state_weights[9] = 5` | `Pro2/experiments/L2/mpc_vx_weight/high.json` | [high/demo.mp4](high/demo.mp4) |

共同条件为平地、12 s、3 s 开始行走、前向目标速度 0.2 m/s、物理步长 0.001 s、MPC 步长 0.005 s、预测步数 10、摆腿高度 0.12 m、无推力扰动。`u_weight` 均为 0.000001。

控制库版本：

- 本次补录：`8e8d5f8c6328ef5521428e4da7920dc650d11ca5d36652365d2b7f13b6e1bc48`。
- 原 L2 正式三条件：`e5d4396147be3567caf28bb9f6515d64de1536edab5c565aa31b181f8026301c`。

## 执行命令

以下为本次实际执行命令；重新录制时必须更换输出子目录名，不覆盖本次数据。

```text
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L2 --config Pro2/experiments/L2/mpc_vx_weight/low.json --duration 12 --record-video --video-fps 30 --output Pro2/output/L2/ppt_demo_20261104/low
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L2 --config Pro2/experiments/L2/mpc_vx_weight/baseline.json --duration 12 --record-video --video-fps 30 --output Pro2/output/L2/ppt_demo_20261104/baseline
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L2 --config Pro2/experiments/L2/mpc_vx_weight/high.json --duration 12 --record-video --video-fps 30 --output Pro2/output/L2/ppt_demo_20261104/high
```

## 验证方法

```text
python3 Pro2/output/L2/ppt_demo_20261104/verify_demo.py
```

该程序读取每组原始 `run.json`、`states.csv`、配置与视频，验证：

1. `completed`，12000 物理步，1200 行日志，全部日志数值有限；
2. 100 Hz 日志中 MPC/WBC QP 状态全部为 0；
3. FFmpeg 完整解码成功，360 帧，12.00 s，30 FPS，960×720；
4. 三组源配置和实际生效配置只在 `state_weights[9]` 上有物理参数差异，使用相同控制库；
5. 保存各源配置、运行文件、日志和视频的 SHA-256。

详细结果位于 [verification.json](verification.json)，每个运行目录的 `video_decode_verification.txt` 保留解码输出。QP 检查以 100 Hz 采样日志为依据，不冒充每个 1 kHz 物理步的完整求解统计。

每组目录还保留 `config.json`、`run.json`、`states.csv`、`isaac.log`、`scene.usda`、`final_frame.png` 和 `video_frames/`，便于后续追溯。保留录制帧，不把新数据混入原正式对照图。
