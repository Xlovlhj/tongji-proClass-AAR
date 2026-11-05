# L4 机器人模型规格

本目录只保存机器人模型的**接口规格**，不复制第三方模型资产。`unitree_h1_2_handless.json` 描述本机 H1-2 USD 的关节、浮动基、足端和质量等可验证信息。

默认资产查找顺序为：

1. 环境变量 `UNITREE_H1_USD` 指向的文件；
2. `~/xqw-file/unitree_model/H1-2/h1_2_handless/h1_2_handless.usd`；
3. 项目内可选位置 `Pro2/assets/unitree_h1_2_handless/h1_2_handless.usd`。

模型预检使用 Python 入口：

```bash
python3 Pro2/tools/preflight_l4_robot.py --robot unitree_h1_2_handless
```

预检把“USD 资产可加载”和“OpenLoong MPC/WBC 可直接控制”分开判断。H1 资产通过前者，并不自动满足后者。
