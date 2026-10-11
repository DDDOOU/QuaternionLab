# 四元数旋转变换演示（QuaternionLab / Unity）

在 Unity 场景中交互演示**四元数旋转变换**：三种输入方式（轴-角 / 四元数分量 / 欧拉角），Slerp 最短弧动画与轨迹可视化，四元数乘法的左右乘（局部系/世界系）复合演示，实时显示当前姿态的四元数、欧拉角、轴-角与 3×3 旋转矩阵。

---

## 1. 快速开始

1. 用 Unity 打开本工程目录（推荐 **Unity 2022.3 LTS**，2021.3+ 均可；脚本已兼容 2023.x）。
2. 执行菜单 **Tools → QuaternionLab → 一键搭建四元数旋转演示场景**。
   - 自动创建：`Main Camera`（轨道相机）+ `Directional Light` + `QuaternionLab`（演示组件）。
   - 场景自动保存到 `Assets/Scenes/QuaternionLab.unity`。
3. 点击 **Play**。默认从恒等姿态沿 Slerp 匀速转到"轴-角(仰角35°, 方位25°, 60°)"目标，即可看到最短弧轨迹。

## 2. 演示内容与操作

| 操作 | 效果 |
|---|---|
| 轴-角 输入 | 轴仰角 / 轴方位角 / 旋转角度 三个滑条定义目标四元数 |
| 四元数 输入 | w,x,y,z 四个滑条，实时归一化后作为目标四元数 |
| 欧拉 YXZ 输入 | yaw/pitch/roll 滑条（应用顺序 Z→X→Y，与 Unity `Quaternion.Euler` 一致） |
| 直接 / Slerp 动画 | 目标立即生效 / 从当前姿态沿最短弧匀速转动 |
| 右乘 Ry90（局部系） | 目标 = 目标 × Ry90，演示"绕自身轴转" |
| 左乘 Ry90（世界系） | 目标 = Ry90 × 目标，演示"绕世界轴转"（乘法不可交换） |
| 绕 X/Y/Z 90° 预设 | 一步把目标设为绕某局部轴 90° |
| 重置 (R) | 目标回到恒等，滑条复位 |
| 播放/暂停 (Space) | 切换 Slerp 动画 |
| T / H / F1 / F | 最短弧轨迹开关 / 幽灵轴与网格开关 / 帮助面板 / 相机聚焦 |

**场景视觉元素**
- 六面异色立方体 + 局部坐标轴（X 红 / Y 绿 / Z 蓝）：当前姿态，由 `qCurrent` 驱动。
- 灰色"幽灵轴"：目标姿态 `qTarget`。
- 三条彩色弧线：Slerp 路径上 X/Y/Z 三个基向量尖端扫过的轨迹——直观展示最短弧与匀速性质。

**实时读数**：当前姿态的四元数分量、欧拉角、轴-角、3×3 矩阵、与目标夹角、动画进度。

## 3. 数学核心（纯 C#，可离线测试）

```
Assets/Scripts/Core/
  Vec3.cs        轻量三维向量
  Quat.cs        四元数结构体：Hamilton 积 / 共轭 / 逆 / 旋转向量 q·v·q⁻¹
  QuatMath.cs    轴-角 ↔ 四元数、欧拉(YXZ) ↔ 四元数、矩阵 ↔ 四元数、
                 Slerp / Lerp / SlerpPath / AngleBetween
```

- 核心不依赖 UnityEngine，可被 Unity 编译，也可被 .NET csc 独立编译验证。
- 约定：对向量 v 的旋转为 q v q⁻¹；两个旋转复合为 Hamilton 积；欧拉角按 R = Ry·Rx·Rz（Z→X→Y）组合。
- 离线测试 `Tests/QuatMathTest.cs`（**53 项断言，ALL PASS**，退出码 0）覆盖：
  基础 90°/180° 旋转、恒等与逆、保长度/保内积、复合与不可交换、轴-角互转（含零轴/0°/360° 边界）、欧拉互转（含万向锁 pitch=±90 合并 yaw−roll、大范围往返）、矩阵互转（行列式=1、行正交、往返还原）、Slerp 端点/单位长/中点公式/匀速性/最短弧取反、Lerp 非球面性、夹角、SlerpPath 采样与角距单调、随机 200 组往返一致。
- 重跑测试（Windows）：
  `C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe /nologo /out:Tests\QuatMathTest.exe Tests\QuatMathTest.cs Assets\Scripts\Core\Vec3.cs Assets\Scripts\Core\Quat.cs Assets\Scripts\Core\QuatMath.cs && Tests\QuatMathTest.exe`

## 4. 已知限制

- 本项目在无 Unity 编辑器的机器上开发，运行时体验（滑条、轨迹、相机）需在 Unity 中验证；数学核心已离线全量验证。
- 欧拉角约定为 YXZ（应用顺序 Z→X→Y），HUD 读数与该约定一致；不直接使用 `Transform.localEulerAngles`。
- 万向锁（pitch = ±90°）时欧拉读数按 roll=0 归一并合并到 yaw，符合惯例。
- 轨迹/网格/材质均为运行时创建，退出运行模式即还原，再次 Play 自动重建。
