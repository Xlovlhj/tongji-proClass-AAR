# L0–L4 成果目录约定

本项目的所有报告、实验数据、图片和视频按课程层次严格分开：

| 目录 | 保存内容 |
|---|---|
| `L0/` | 环境、构建与启动记录，默认示例数据、跑通视频，以及 `u_weight` 三候选值补充调参 |
| `L1/` | 控制框图、MPC/WBC 源码定位、状态/输入/约束/频率分析 |
| `L2/` | 参数与扰动对照实验；每组实验再分 `baseline/` 和修改组 |
| `L3/` | MPC 与 LQR/iLQR、凸 MPC 与 NMPC 的理论联系分析 |
| `L4/` | 路径跟踪、地形、跳跃、控制量分解等挑战项成果 |

`run_isaac.py` 使用 `--level L0` 至 `--level L4` 指定成果层次，并检查 `--output` 是否位于对应目录。例如：

```bash
python3 Pro2/run_isaac.py --level L2 \
  --config Pro2/experiments/L2/mpc_vx_weight/high.json --headless \
  --output Pro2/output/L2/mpc_vx_weight/high
```

`project2` 分支提交各层报告、CSV 实验数据、JSON 配置与指标、分析图片和成片视频，并在 `final/` 保留既有的 15 页正式汇报、PDF 和出处文档。运行缓存、编译产物、日志、依赖本机缓存的 USD 场景和录像逐帧图片保留在本地，可按原 Python 入口重新生成。

## 总报告汇总入口

- `Pro2/output/总报告素材索引.md`：按 L0–L4 列出主报告、正式图片、结构化指标、原始实验目录以及结论边界；
- `Pro2/output/report_asset_manifest.json`：机器可读的报告资产、文件大小和 SHA-256 清单；
- `Pro2/tools/audit_stage_reports.py`：检查图片链接、阶段目录边界、公式分隔符、代码围栏，以及五份主报告应保留的图片数量。

在项目根目录执行：

```text
python3 Pro2/tools/audit_stage_reports.py --write-manifest
```

只有生成清单中的 `error_count` 为 0 且命令退出码为 0 时，当前 Markdown 图片与素材清单才视为同步。该检查不会把 L4 的调试、失败或作废目录自动认定为正式证据；正式取证范围仍以各阶段主报告和总报告素材索引为准。
