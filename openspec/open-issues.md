# Open issues

## O001 - geom:BAD

状态：未解决。

现象：

```text
GEOM bad 150
GEOM uvz ... xw/yw 250 0
```

判断：

- Final 矩阵已生成；
- 标定参数仍是占位；
- 不是 Modbus 问题。

下一步：

- 没机械臂阶段：只做文档/仿真模式讨论；
- 有机械臂阶段：正式采点标定。

## O002 - 机械臂和电机未接入验证

状态：未解决。

当前用户反馈：

- 现场没有机械臂；
- 电机没接，是否能动未知。

影响：

- 无法验证真实 IK；
- 无法验证零点脉冲；
- 无法验证 pick/place。

## O003 - 是否需要 test-only 几何仿真模式

状态：待讨论。

目的：

- 在无机械臂阶段让软件链路越过 `geom:BAD`，观察 Final 几何字段和 UI 行为。

风险：

- 如果和真实电机路径混用，会产生危险。

要求：

- 默认关闭；
- 编译宏命名清楚；
- LCD 或文档明确提示 test-only。

## O004 - 状态机仍使用整盘同类判断

状态：待讨论。

当前：

- LCD 已改为 E/W/Y 统计；
- `AppMatrix_CheckTrayFullOrEmpty()` 仍保留旧语义。

风险：

- 后续真实分拣策略如果仍按“整盘全同类”判断，可能不适合当前乒乓球业务。

处理建议：

- 等几何/机械臂方向明确后，再重新设计业务状态机。

## O005 - SIM 模式现场验证

状态：待烧录测试。

当前实现：

- `CFG_MATRIX_GEOM_SIM_MODE=1`
- `CFG_ARM_MOTION_SIM_MODE=1`
- Main 页 RunFlag 行显示 `SIM:G1A1`
- SER 页显示 `SIM MODE: geom 1 arm 1, no real arm/motor movement`

期望：

- KEY2 后 `geom:OK`；
- Final 页每行 `geom` 为 1；
- `Xw/Yw` 不再全部固定为 `250/0`；
- 状态机可以继续推进，用于观察 RunFlag/FL 逻辑。

真实机械臂接入前必须关闭两个 SIM 宏。
