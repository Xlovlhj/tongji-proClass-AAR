# L2｜实验二：预测时域对照补录

此目录保存 PPT 首页视频使用的 Isaac Sim 实录，三组各 12 秒、30 fps，同一仿真时间起点。合成画面同时播放三组，而不是依次播放。

| 子目录 | 原实验配置 | 唯一控制变量 |
|---|---|---|
| `short_n5/` | `Pro2/experiments/L2/prediction_horizon/short_n5.json` | `mpc_horizon_steps = 5` |
| `baseline_n10/` | `Pro2/experiments/L2/prediction_horizon/baseline_n10.json` | `mpc_horizon_steps = 10` |
| `long_n15/` | `Pro2/experiments/L2/prediction_horizon/long_n15.json` | `mpc_horizon_steps = 15` |

三组使用相同 C++ 控制源码；预测长度通过编译参数 `OPENLOONG_MPC_N` 分别生成 N=5、10、15 的独立控制库。因此库的 SHA256 不同是预期现象。验证脚本比较编译命令、生成源码及实际加载库的哈希，不能用“全部使用同一个二进制库”描述这组实验。`mpc_dt = 0.005 s`，对应预测时域为 0.025、0.050、0.075 秒。

从项目根目录运行下列 Python 命令可复现短时域条件；另两组对应替换配置和输出子目录。重录应使用新的输出子目录，保留本次记录。

```text
python3 Pro2/run_isaac.py --build-only --mpc-horizon-steps 5 --jobs 2
python3 Pro2/run_isaac.py --isaac-root /home/xqw/isaacsim6.0.1 --no-build --headless --level L2 --config Pro2/experiments/L2/prediction_horizon/short_n5.json --record-video --video-fps 30 --output Pro2/output/L2/prediction_horizon/ppt_video_20261104/short_n5
```

每组的完整命令另存 `recording_command.json`；`config.json` 是实际配置，`run.json` 是运行清单，`states.csv` 是本次状态数据，`demo.mp4` 是原速录像。补录调用原有 play 仿真循环真实运行，不是根据历史曲线制作动画。

原 L2 报告及历史比较图保持原数据来源。本目录补录用于展示动作，不能将旧报告数值冒充本次新运行结果。最终校验保存在 `Pro2/output/final/checks/recordings_verification.json`，含视频完整解码、帧数、仿真完成度、配置差异、QP 状态计数及来源哈希。

