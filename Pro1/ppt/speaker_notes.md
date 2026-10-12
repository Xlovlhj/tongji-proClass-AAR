# raa-v0 汇报讲稿

## 1｜机械臂画家

这个项目让一台仿真机械臂像拿笔一样，沿桌面画出 ChatGPT 标志的轮廓。我们关心的不只是“能画出来”，还要比较画得准不准、被推开后能不能恢复。平台使用 Isaac Sim，机器人是 Franka Panda 七轴机械臂，末端固定偏移点作为虚拟笔尖。本次图案最大边长 16 厘米，共 8 笔，全部在电脑的物理仿真中完成，不使用实物机器人。

来源：`Pro1/raa/paths.py`；`Pro1/raa/scene.py`；`Pro1/output/normal/fuzzy/metrics.json`。

## 2｜机械臂怎样画出图案

先把标志轮廓变成笔尖要走的路线，再安排什么时候起步、减速、抬笔和落笔。随后把“笔尖去哪里”换算成“七个关节分别转多少”，最后由控制器计算关节力矩，推动机器人实际运动。专业上依次是 SVG 轮廓采样、带时间的笛卡尔轨迹、连续逆运动学、关节力矩控制和物理记录。逆运动学沿上一点的解连续更新，减少七轴机械臂出现姿态突跳的风险。

来源：`Pro1/raa/paths.py`；`Pro1/raa/kinematics.py`；`Pro1/raa/simulation.py`。

## 3｜四种控制器的区别

PD 根据位置偏差纠正，速度项帮助减小振荡；重力补偿 PD 还负责托住机械臂的重量。RBF 和模糊控制都在这个基础上边画边学习额外补偿，每次从零开始。令 e=q_d−q，PD 为 τ=K_p e+K_d ė；重力补偿再加 g(q)，自适应再加 Wᵀφ，并限制力矩。RBF 每轴用关节角度、角速度激活 9 个高斯节点；模糊每轴用误差和误差变化激活 25 条规则。

来源：`Pro1/raa/control.py`；`Pro1/raa/adaptive_control.py`；`Pro1/output/normal/fuzzy/controller_state.json`。

## 4｜同一张图，同样的三次推力

公平比较就像让四个人沿同一路线跑步，并在同样的位置受到同样的推力。四种控制器使用相同图案、初始稳定方式、PD 增益、力矩上限和目标轨迹，各做一次无干扰和一次受扰实验，共八组。物理步长为 1 毫秒，记录约 114 秒。推力实际施加在笔尖，方向按世界坐标定义：20–22 秒沿 +X 推 3 N，55–57 秒沿 −Y 推 3 N，90–92 秒施加 (2,2,0) N。

精确推力：20≤t<22，F=(3,0,0) N；55≤t<57，F=(0,−3,0) N；90≤t<92，F=(2,2,0) N，第三次合力约 2.83 N。

来源：`Pro1/output/disturbed/fuzzy/resolved_config.json`；`Pro1/output/experiment_manifest.json`；`Pro1/output/summary.csv`。

## 5｜正常绘画：补偿重力后更准确

看四幅图，普通 PD 的路线偏离比较明显，加入重力补偿后就能较准确地沿轮廓画。原因是普通 PD 还要靠产生偏差来抵抗自身重量，重力补偿则提前提供托举力矩。表格使用落笔阶段的三维均方根误差 RMSE，单位毫米，越小越好，也计入高度偏差。数值依次为 PD 32.955、重力补偿 PD 0.562、RBF 0.567、模糊 0.518；这里的平面图只显示 XY 形状。

来源：`Pro1/output/summary.csv`，字段 `drawing_tip_rmse_m`；`Pro1/output/normal/{pd,gc-pd,rbf,fuzzy}/drawing_result.svg`。

## 6｜受到推力：模糊整体误差更小

加入推力后，三种补偿方案的误差都上升，模糊控制在这次实验中的整体误差最小。它的受扰三维 RMSE 为 1.769 毫米，重力补偿 PD 为 2.558 毫米，相比减少约 30.85%。图中把目标、正常和受扰的平面轨迹叠在一起，可以直观看到推力造成的偏移。需要分别比较平均跟踪表现和恢复速度；普通 PD 的受扰 RMSE 略低，可能是推力局部抵消原有偏差，不能据此说抗扰更强。

落笔三维 RMSE（mm）：

| 控制器 | 无干扰 | 有干扰 |
| --- | ---: | ---: |
| PD | 32.955 | 32.832 |
| 重力补偿 PD | 0.562 | 2.558 |
| RBF | 0.567 | 2.096 |
| 模糊 | 0.518 | 1.769 |

来源：`Pro1/output/summary.csv`；`Pro1/output/comparisons/fuzzy/trajectory_comparison.svg`；`Pro1/output/comparisons/fuzzy/comparison_metrics.json`。

## 7｜撤去推力：重力补偿 PD 恢复最快

撤去推力后，我们问的是“多久回到没有被推时的路线”。第一段曲线展示 19–27 秒的三维位置差，20–22 秒正在施力。恢复的严格定义是：受扰位置与同一时刻正常位置的距离回到 2 毫米内，并连续保持至少 0.5 秒；恢复时间从撤力算到这段连续区间的开始。三次实验中重力补偿 PD 都恢复最快，约 0.2 秒。自适应方案虽然整体误差更小，学习留下的补偿可能让撤力后偏移持续更久。

恢复时间（s）：

| 控制器 | 第一次推力后 | 第二次推力后 | 第三次推力后 |
| --- | ---: | ---: | ---: |
| PD | 2.294 | 4.410 | 1.356 |
| 重力补偿 PD | 0.184 | 0.244 | 0.244 |
| RBF | 1.687 | 2.710 | 2.206 |
| 模糊 | 0.616 | 0.827 | 1.203 |

来源：`Pro1/output/comparisons/{pd,gc-pd,rbf,fuzzy}/comparison.csv`；对应 `comparison_metrics.json` 的 `recovery.windows`。

## 8｜正常绘画：四种控制器

四个视频都完整记录了 Isaac Sim 中机械臂从第一笔到最后一笔的绘画过程。进入本页时，四个视频以零延迟同时自动开始播放，视频已经存进 PPT。无须逐个点击启动，播放过程中仍可点击各个画面单独暂停或继续。普通 PD 画出的路线偏离比较明显；加入重力补偿后，三种方案都更贴近目标轮廓。这次正常实验中，模糊控制的三维整体误差最小。

灰线是目标，青绿色线是实际笔尖 XY 投影到纸面的墨迹，方便观察平面形状。高度偏差仍保留在真实笔尖 XYZ 数据和三维误差中。本版是虚拟画线，不包含笔与纸的接触压力控制。每个视频下方的控制器名称和解释都可以直接修改。

来源：`Pro1/video/{pd,gc-pd,rbf,fuzzy}_normal.mp4` 及同名 `.recording.json`；`Pro1/output/summary.csv`；`Pro1/output/normal/fuzzy/metrics.json` 中 `ink_visualization` 和 `timeline_play_enabled`。

## 9｜外力干扰：被推后怎样恢复

四个视频使用相同目标路线，在同样的时段向笔尖施加同样的推力。20–22 秒向 +X 推 3 N，55–57 秒向 −Y 推 3 N，90–92 秒施加 (2,2,0) N 的推力。可以先观察被推时路线怎样偏移，再看撤去推力后怎样回到原来的路线。模糊控制的整体跟踪误差更小，重力补偿 PD 在三次撤力后都恢复最快。

橙色箭头标出施力方向。普通 PD 的真实笔尖高度偏低，推力箭头被桌面遮挡；这组外力仍按同一安排真实施加，外力记录和位置偏移已经核验。纸面墨迹是实际 XY 的投影，不代表真实高度没有误差。八个视频均已内嵌，进入本页时，四个受扰视频以零延迟同时自动开始播放，无须逐个点击启动；播放过程中仍可点击各个画面单独暂停或继续。画面下面的名称和两行说明是可编辑文本。

来源：`Pro1/video/{pd,gc-pd,rbf,fuzzy}_disturbed.mp4` 及同名 `.recording.json`；`Pro1/output/disturbed/fuzzy/resolved_config.json`；`Pro1/output/comparisons/{pd,gc-pd,rbf,fuzzy}/comparison_metrics.json`；`Pro1/output/video_checks/summary.json`。

## 10｜本次实验的三个结论

第一，画得准之前要先托住机械臂的重量，重力补偿是这次精度提升的主要来源。第二，在当前图案、参数和推力下，模糊控制的落笔三维 RMSE 最低。第三，重力补偿 PD 在三次撤力后都恢复最快，因此“整体画得准”和“撤力后回得快”是两个指标。项目已完成四种控制、八组实验、视频和数据记录。下一步可从用户图片提取路线，并调整速度、控制增益和学习参数，检查结论是否仍成立。

来源：`Pro1/output/summary.csv`；`Pro1/output/comparisons/{pd,gc-pd,rbf,fuzzy}/comparison_metrics.json`；`Pro1/output/independent_batch_audit.json`。
