# tongji-proClass-AAR · 机械臂画家

本仓库保存机器人课程项目 **robot arm artist（raa-v0）**，代码位于 `Pro1/`。在 Isaac Sim 中加载 Franka Panda 七轴机械臂，让虚拟笔尖画出 ChatGPT 标志，并比较 PD、重力补偿 PD、RBF 自适应和模糊自适应控制。

## 成果展示

下面的动图来自八个真实 Isaac Sim **▶ Play 三维视口录制**，是完整视频的 **8 倍速预览**。点击动图或控制器名称，可打开或下载约 116 秒的完整 MP4。动图和视频均使用仓库内的相对路径。

### 正常绘画

| **[PD](Pro1/video/pd_normal.mp4)** | **[重力补偿 PD](Pro1/video/gc-pd_normal.mp4)** |
| :---: | :---: |
| [![PD正常绘画视频预览](Pro1/video/preview/pd_normal.gif)](Pro1/video/pd_normal.mp4)<br>按位置偏差纠正；自重使轮廓偏离明显。 | [![重力补偿 PD正常绘画视频预览](Pro1/video/preview/gc-pd_normal.gif)](Pro1/video/gc-pd_normal.mp4)<br>先托住机械臂自重，轮廓更贴近目标。 |
| **[RBF 自适应](Pro1/video/rbf_normal.mp4)** | **[模糊自适应](Pro1/video/fuzzy_normal.mp4)** |
| [![RBF 自适应正常绘画视频预览](Pro1/video/preview/rbf_normal.gif)](Pro1/video/rbf_normal.mp4)<br>边画边学习补偿，完成标志轮廓。 | [![模糊自适应正常绘画视频预览](Pro1/video/preview/fuzzy_normal.gif)](Pro1/video/fuzzy_normal.mp4)<br>根据误差规则补偿，本次正常整体误差最小。 |

### 外力干扰

四种控制器使用同样的三次笔尖推力：20–22 秒 `(3,0,0)` N、55–57 秒 `(0,−3,0)` N、90–92 秒 `(2,2,0)` N。目标路线保持一致，观察实际画线的偏移与撤力后的恢复。

| **[PD](Pro1/video/pd_disturbed.mp4)** | **[重力补偿 PD](Pro1/video/gc-pd_disturbed.mp4)** |
| :---: | :---: |
| [![PD外力干扰视频预览](Pro1/video/preview/pd_disturbed.gif)](Pro1/video/pd_disturbed.mp4)<br>观察笔尖受推后的额外偏移与恢复。 | [![重力补偿 PD外力干扰视频预览](Pro1/video/preview/gc-pd_disturbed.gif)](Pro1/video/gc-pd_disturbed.mp4)<br>受推时短暂偏离，本次撤力恢复最快。 |
| **[RBF 自适应](Pro1/video/rbf_disturbed.mp4)** | **[模糊自适应](Pro1/video/fuzzy_disturbed.mp4)** |
| [![RBF 自适应外力干扰视频预览](Pro1/video/preview/rbf_disturbed.gif)](Pro1/video/rbf_disturbed.mp4)<br>受推时在线学习补偿，观察轨迹修正过程。 | [![模糊自适应外力干扰视频预览](Pro1/video/preview/fuzzy_disturbed.gif)](Pro1/video/fuzzy_disturbed.mp4)<br>持续更新补偿，本次受推整体误差最小。 |

纸面青绿色线条显示实际笔尖的 XY 投影，三维误差按真实笔尖位置计算。灰线为目标轮廓；普通 PD 的高度偏差会遮挡推力箭头，可结合轨迹记录判断其受扰响应。

## 实验结果

落笔阶段三维均方根误差 RMSE，单位 mm，越小越好：

| 控制器 | 正常绘画 | 外力干扰 |
| --- | ---: | ---: |
| PD | 32.955 | 32.832 |
| 重力补偿 PD | 0.562 | 2.558 |
| RBF 自适应 | 0.567 | 2.096 |
| 模糊自适应 | 0.518 | 1.769 |

在本次图案、控制参数和推力设置下，**模糊控制整体误差最低，重力补偿 PD 撤力后恢复最快**。完整指标与恢复时间见[实验结果](Pro1/results/README.md)，目标／正常／受扰轨迹图见[图表目录](Pro1/pictures/)。

## 汇报文件

- [可编辑 PPT：10 页，内嵌八个完整视频](https://github.com/Xlovlhj/tongji-proClass-AAR/releases/download/raa-v0/raa-v0.pptx)。进入第 8、9 页时，四个视频分别自动同时播放；每个视频下方有可编辑解释。
- [静态 PDF 预览](Pro1/ppt/raa-v0.pdf)。PDF 中的视频位置显示封面，播放视频请使用 PPT。
- [逐页讲稿](Pro1/ppt/speaker_notes.md)。

## 运行

在已有 Isaac Sim 的电脑上，从仓库根目录执行：

```bash
python3 Pro1/main.py --config fuzzy --gui
```

`--config` 可选择 `pd`、`gc-pd`、`rbf`、`fuzzy`；加入 `--disturbance` 运行外力干扰实验。完整环境、轨迹规划与复现方法见[项目说明](Pro1/README.md)。

## 目录

| 路径 | 内容 |
| --- | --- |
| `Pro1/raa/` | 轨迹规划、逆运动学、力矩控制、外力和录制 |
| `Pro1/configs/` | 四种控制器的配置 |
| `Pro1/video/` | 八个完整 MP4、录制元数据和动图预览 |
| `Pro1/pictures/` | 目标、实际轨迹与误差图，SVG／PNG |
| `Pro1/results/` | 实验指标、恢复时间与独立核验摘要 |
| `Pro1/ppt/` | PDF、讲稿、汇报预览与说明 |

完整 MP4 每个约 43–46 MiB，通过 Git 提交；约 360 MiB 的内嵌视频 PPT 放在 [Release](https://github.com/Xlovlhj/tongji-proClass-AAR/releases/tag/raa-v0) 附件中。缓存、原始逐步 CSV 和本机日志保留在本地，完成批量实验后可重新生成。
