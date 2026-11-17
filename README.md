# 战争雷霆第三人称 CAS 航弹与火箭弹 CCIP 物理与视觉反解原型

> [!CAUTION]
> ### 严正合规、使用范围与禁止传播声明（必读）
>
> 1. **严禁用于任何多人在线战局**：本仓库仅作为基于公开引擎源码（`DagorEngine`）、社区拆包数据与官方本地只读网页遥测接口（`localhost:8111`）的**外弹道空气动力学、透视相机几何与抗干扰图形检测算法研究原型**。**严禁在“试飞场 / 试车场（Test Flight / Test Drive）”及纯单人离线自定义任务以外的任何环境（包括但不限于街机 / 历史 / 全真多人匹配战局、联队战、守卫模式、锦标赛等）中使用本程序。**
> 2. **严禁向大众玩家社区传播或分发**：**严禁在非纯粹工程技术研讨环境——包括但不限于 QQ 群、哔哩哔哩（B站）、百度贴吧、小黑盒（Heybox）、NGA、抖音、快手、微信群等游戏玩家社区——宣传、搬运、录屏推广或打包分发本仓库源码、数据库及编译产物。**
> 3. **公平竞技与服务条款风险警示**：在多人在线游戏中使用外部弹道解算或视觉辅助叠层会导致非公平竞技优势，直接违反 **Gaijin Entertainment《最终用户许可协议》（EULA）与《服务条款》（Terms of Use）**，并可能导致账号永久封禁等严重后果。查阅、克隆或编译本仓库即表示您已充分知悉并同意严格遵守上述边界；任何越界使用或违规传播行为均与技术原理探讨无关，一切后果由行为人自行承担。

![预览效果](./dist/demo_hud_preview.png)

---

## 项目背景与核心技术难点

在《战争雷霆》（War Thunder）第三人称视角（TPS）下进行固定翼或武装直升机近距空中支援（CAS）俯冲投弹/发射火箭弹时，若完全遵循**零内存读取、零 DLL 注入、零驱动/DMA 介入**的非侵入式硬件与系统红线，计算并绘制连续计算落点（CCIP, Constantly Computed Impact Point）面临三个核心几何与物理耦合难题：

1. **相机自由旋转缺失（鼠标牵引盲区）**：官方本地网页遥测 `http://127.0.0.1:8111` 仅输出**机体本身**的航向、俯仰、横滚与速度参数，**完全不输出玩家鼠标自由牵引下的第三人称摄像机朝向 $(\psi_c, \theta_c)$**。在鼠标瞄准（Mouse Aim）机动中，玩家先甩动鼠标旋转相机，机头再滞后追随，相机光轴与机头纵轴之间常存在 $5^\circ \sim 35^\circ$ 的动态夹角。
2. **第三人称显著视差（Parallax）**：根据拆包相机配置 `config/camera.blkx`，固定翼第三人称相机并非位于飞机质心，而是悬挂在机身后方 $17\text{ m}$、上方 $4\text{ m}$ 处，且光轴斜向下指向机体前方 $500\text{ m}$ 的收敛点。在 $200 \sim 800\text{ m}$ 的常规 CAS 投弹斜距上，机腹挂架与相机眼睛之间的空间基线会产生显著的屏幕视差，不能用无穷远平行光近似。
3. **官方遥测符号反转与拆包阻力参数语义分歧**：`8111` 接口输出的俄式地平仪俯仰/横滚与真实姿态角符号相反，美系机型的 `bank` 字段实为限幅 $\pm 8^\circ$ 的侧滑仪小球；拆包火箭弹文件中的 `dragCx` 实为发射巢挂载寄生阻力，真实飞行阻力则由 `CxK` 控制。

本项目采用 **“机头准星亚像素光学锚定 + `2×2` 高斯-牛顿第三人称相机姿态反解 + 三维大气/变质量弹道 RK2 积分 + 零残影透明穿透图层”** 全链路闭环解决上述问题：

```mermaid
flowchart LR
    subgraph Input["只读非侵入输入源"]
        T8111["localhost:8111 (80Hz Keep-Alive 遥测)"]
        CAP["GDI 屏幕中心 768×768 无回授桌面截图"]
        DB["War-Thunder-Datamine (cas_db.json 词库)"]
        TERR["WTVPACK2 56张地图高程 / 无线电高度表"]
    end

    subgraph Solver["物理与几何核心解算"]
        ATT["遥测符号校准与 120FPS 角速度外推"]
        VIS["色度突出量积分图 + 正交四臂/对角空腔检测"]
        GN["2×2 高斯-牛顿反解 TPS 相机偏航与俯仰"]
        COAST["450ms 相对偏角惯性保持滤波"]
        RK2["Dagor 大气 + 航弹/火箭弹三维 RK2 积分"]
    end

    subgraph Output["零残影透明叠层"]
        PROJ["EFM_HOR_PLUS 16:9 透视重投影"]
        HUD["单一并集包络清除 + UpdateLayeredWindow"]
    end

    T8111 --> ATT
    CAP --> VIS
    DB --> RK2
    DB --> GN
    TERR --> RK2
    ATT --> GN
    VIS --> GN
    GN --> COAST
    ATT --> RK2
    COAST --> PROJ
    RK2 --> PROJ
    PROJ --> HUD
```

---

## 一、坐标系约定、机体正交基与 `8111` 遥测校准方程

### 1.1 Dagor 左手世界坐标系与小地图反归一化

对齐 `GaijinEntertainment/DagorEngine` 与游戏内陆战/空战地图网格，全局采用**左手三维笛卡尔世界坐标系**：

- **$+X$ 轴**：指向正东（East，单位：$\text{m}$）
- **$+Y$ 轴**：指向天顶（Up，海拔高度 ASL，单位：$\text{m}$）
- **$+Z$ 轴**：指向正北（North，单位：$\text{m}$）

`http://127.0.0.1:8111/map_info.json` 提供地图世界边界 $[x_{\min}, z_{\min}]$ 与 $[x_{\max}, z_{\max}]$，而 `/map_obj.json` 中玩家图标（`icon == "Player"`）输出的归一化坐标 $(x_{\text{norm}}, y_{\text{norm}}) \in [0, 1]^2$ 原点位于地图**左上角（西北角）**，$y_{\text{norm}}$ 自北向南递增。因此水平世界坐标 $(X, Z)$ 的反算关系为：

$$
\begin{aligned}
X(t_0) &= x_{\min} + x_{\text{norm}} \cdot (x_{\max} - x_{\min}) \\
Z(t_0) &= z_{\max} - y_{\text{norm}} \cdot (z_{\max} - z_{\min})
\end{aligned}
$$

由于 `/map_obj.json` 刷新率较低（本程序将其隔离在 `160 ms` 独立后台慢速线程以防阻塞姿态采集），主循环利用高频水平速度分量在两次地图采样之间做航位推算（上限外推 $\Delta t \le 0.35\text{ s}$）：

$$
V_h = \sqrt{\max\!\left(V_{\text{TAS}}^2 - V_y^2,\; 0\right)}, \qquad
\begin{bmatrix} X(t) \\ Z(t) \end{bmatrix} = \begin{bmatrix} X(t_0) \\ Z(t_0) \end{bmatrix} + V_h \Delta t \begin{bmatrix} \sin\psi \\ \cos\psi \end{bmatrix}
$$

### 1.2 机体左手欧拉正交基 $(\mathbf{f}, \mathbf{u}, \mathbf{r})$

设罗盘航向角为 $\psi \in [0^\circ, 360^\circ)$（$0^\circ$ 为正北 $+Z$，$90^\circ$ 为正东 $+X$），真实机头俯仰角为 $\theta$（抬头 $>0$，俯冲 $<0$），真实机体横滚角为 $\phi$（右翼下沉 $>0$，左翼下沉 $<0$）。定义未横滚时的水平右轴 $\mathbf{r}_0$、机头前轴 $\mathbf{f}$ 与零横滚上轴 $\mathbf{u}_0$：

$$
\mathbf{r}_0 = \begin{bmatrix} \cos\psi \\ 0 \\ -\sin\psi \end{bmatrix}, \qquad
\mathbf{f} = \begin{bmatrix} \cos\theta\sin\psi \\ \sin\theta \\ \cos\theta\cos\psi \end{bmatrix}, \qquad
\mathbf{u}_0 = \begin{bmatrix} -\sin\theta\sin\psi \\ \cos\theta \\ -\sin\theta\cos\psi \end{bmatrix}
$$

绕机头前轴 $\mathbf{f}$ 施加右手/左手定义一致的右倾正横滚 $\phi$ 后，机体正交右翼轴 $\mathbf{r}$ 与座舱顶轴 $\mathbf{u}$ 为：

$$
\begin{aligned}
\mathbf{r} &= \mathbf{r}_0 \cos\phi - \mathbf{u}_0 \sin\phi \\
\mathbf{u} &= \mathbf{r}_0 \sin\phi + \mathbf{u}_0 \cos\phi
\end{aligned}
$$

任意拆包机体局部偏移向量 $\mathbf{p}_{\text{local}} = [x_{\text{fwd}}, y_{\text{up}}, z_{\text{right}}]^\top$ 变换至世界坐标系的线性映射即为：

$$
\Delta\mathbf{p}_{\text{world}} = x_{\text{fwd}}\mathbf{f} + y_{\text{up}}\mathbf{u} + z_{\text{right}}\mathbf{r}
$$

### 1.3 官方 `localhost:8111` 遥测三大实机陷阱与修正

直接照搬 `/indicators` 字面字段会导致落点严重偏低或滚转错乱，源码（`src/telemetry.rs`）实施了以下严格物理修正：

#### （1）俄式地平仪刻度反向修正

`/indicators` 中的 `aviahorizon_pitch`（记作 $\theta_{\text{ah}}$）与 `aviahorizon_roll`（记作 $\phi_{\text{ah}}$）表示座舱地平仪球体相对机壳的位移——当飞机以 $-20^\circ$ 俯冲时，`aviahorizon_pitch` 输出 `+20.0`；当飞机向右横滚 $+28^\circ$ 时，`aviahorizon_roll` 输出 `-28.0`。两者必须同时取反：

$$
\theta = -\theta_{\text{ah}}, \qquad \phi = -\phi_{\text{ah}}
$$

对于无虚拟座舱（缺失 `aviahorizon_pitch`）的载具，由航迹倾角 $\gamma$、攻角 $\alpha$ 与横滚角 $\phi$ 精确重构俯仰角：

$$
\gamma = \arcsin\!\left(\operatorname{clamp}\!\left(\frac{V_y}{V_{\text{TAS}}}, -1, 1\right)\right), \qquad \theta = \gamma + \alpha \cos\phi
$$

#### （2）排除美系飞机 `bank` 侧滑球伪装

在 `a_10a_early` 等诸多美系飞机的拆包仪表中，`bank` 对应的是转弯侧滑仪底部限幅 $[-8, +8]$ 的侧滑玻璃球而非横滚坡度。横滚角解析优先级固定为 `-aviahorizon_roll` $\to$ `-aviahorizon_roll1` $\to$ `bank`。

#### （3）英尺/米制无线电高度表自动识别

现代攻击机与直升机在 `/indicators` 中提供 `radio_altitude`（离地真实净高 AGL），但英美机型常按英尺（`altitude_koef = 3.28`）输出。程序实时比对座舱气压高度表读数 $H_{10\text{k}}$（`altitude_10k`）与 `/state` 中的公制海拔 $H_{\text{ASL}}$（`H, m`）：

$$
\text{isFeet} \iff \left|\frac{H_{10\text{k}}}{H_{\text{ASL}}} - 3.28084\right| < 0.45
$$

自动折算为公制离地净高 $h_{\text{rad}}$ 后（剔除 $\ge 1420\text{ m}$ 的满量程钉死值），直接反推出飞机正下方实时地面海拔 $h_{\text{ground}} = H_{\text{ASL}} - h_{\text{rad}}$。

### 1.4 飞机三维初速矢量合成与 `120 FPS` 姿态外推

飞机机头指向 $\mathbf{f}$ 并不等于气流运动方向。设 `/state` 给出的迎角（攻角）为 $\alpha$（`AoA, deg`，$\alpha > 0$ 时机头高于速度矢量，气流相对运动沿 $-\mathbf{u}$ 偏转）、侧滑角为 $\beta$（`AoS, deg`），真空速为 $V_{\text{TAS}}$：

$$
\hat{\mathbf{v}}_{\text{aero}} = \operatorname{normalize}\!\big(\cos\alpha\cos\beta\,\mathbf{f} - \sin\alpha\cos\beta\,\mathbf{u} + \sin\beta\,\mathbf{r}\big), \qquad \mathbf{v}_0 = V_{\text{TAS}}\,\hat{\mathbf{v}}_{\text{aero}}
$$

当 `/state` 提供精确垂直升降率 $V_y$（`Vy, m/s`）且 $|V_y| \le 0.995\,V_{\text{TAS}}$ 时，保持总真空速模长 $\|\mathbf{v}_0\| = V_{\text{TAS}}$ 守恒，将水平分量按比例重标定：

$$
\eta_h = \frac{\sqrt{V_{\text{TAS}}^2 - V_y^2}}{\sqrt{v_{0,x}^2 + v_{0,z}^2}}, \qquad \mathbf{v}_{\text{ac}} = \begin{bmatrix} \eta_h v_{0,x} \\ V_y \\ \eta_h v_{0,z} \end{bmatrix}
$$

此外，为消除网页遥测离散采样在高速滚转/拉起时造成的阶梯跳帧感，后台线程基于 HTTP/1.1 `Connection: keep-alive` 以约 `80 Hz` 轮询并计算指数平滑角速度 $(\dot{\psi}, \dot{\theta}, \dot{\phi})$，主渲染线程在每帧按采样滞后 $\delta t = \operatorname{clamp}(t_{\text{render}} - t_{\text{sample}}, 0, 45\text{ ms})$ 线性外推瞬时姿态。

---

## 二、DagorEngine 标准大气与外弹道微分方程

### 2.1 Dagor 四阶多项式标准大气模型

严格对齐开源引擎源码 `GaijinEntertainment/DagorEngine/prog/gameLibs/gamePhys/props/atmosphere.cpp`，设海平面基准密度 $\rho_0 = 1.225\text{ kg/m}^3$（对应 $P_0 = 760\text{ mmHg} = 101300\text{ Pa}$，$T_0 = 15^\circ\text{C} = 288.16\text{ K}$），有效多项式截断高度 $H_{\max} = 18300\text{ m}$。令 $h_c = \operatorname{clamp}(h, -500, 18300)$，平流层上界衰减因子 $\sigma(h) = \min(1,\; H_{\max}/h)$，定义四阶霍纳多项式 $P_4(\mathbf{c}, h_c) = \sum_{k=0}^{4} c_k h_c^k$：

$$
\begin{aligned}
\rho(h) &= \rho_0 \cdot P_4(\mathbf{c}_\rho, h_c) \cdot \sigma(h) \\
T(h) &= T_0 \cdot P_4(\mathbf{c}_T, h_c) \\
P(h) &= P_0 \cdot P_4(\mathbf{c}_P, h_c) \cdot \sigma(h) \\
a(h) &= 20.1 \sqrt{\max(T(h), 50.0)}
\end{aligned}
$$

其中引擎硬编码四阶多项式系数表为：

| 物理量 | $c_0$ | $c_1$ | $c_2$ | $c_3$ | $c_4$ |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **空气密度 $\mathbf{c}_\rho$** | $1.0$ | $-9.59387 \times 10^{-5}$ | $3.53118 \times 10^{-9}$ | $-5.83556 \times 10^{-14}$ | $2.28719 \times 10^{-19}$ |
| **大气压强 $\mathbf{c}_P$** | $1.0$ | $-1.18441 \times 10^{-4}$ | $5.67630 \times 10^{-9}$ | $-1.37380 \times 10^{-13}$ | $1.60373 \times 10^{-18}$ |
| **绝对温度 $\mathbf{c}_T$** | $1.0$ | $-2.27712 \times 10^{-5}$ | $2.18069 \times 10^{-10}$ | $-5.71104 \times 10^{-14}$ | $3.97306 \times 10^{-18}$ |

指示空速（IAS）与真空速（TAS）的精确换算关系为 $V_{\text{TAS}} = V_{\text{IAS}} \sqrt{\rho_0 / \rho(h)}$。

### 2.2 自由落体航弹与高阻减速弹（Retarded Bomb）动力学

设航弹质量为 $m$，弹舱开门延迟为 $\Delta t_{\text{bay}}$（外挂架为 $0\text{ s}$，内埋弹舱典型为 $0.40\text{ s}$），挂架向下弹射分离速度为 $v_{\text{ej}} = 1.5\text{ m/s}$。投放瞬间初始状态为：

$$
\mathbf{p}(0) = \mathbf{p}_{\text{ac}} - 0.8\mathbf{u} + \mathbf{v}_{\text{ac}}\Delta t_{\text{bay}}, \qquad \mathbf{v}(0) = \mathbf{v}_{\text{ac}} - v_{\text{ej}}\mathbf{u}
$$

在 `War-Thunder-Datamine` 的 `bombguns/*.blkx` 中，航弹基础有效阻力面积参数由 `dragCx` 与修正乘子 `CxK` 共同决定；对于配备尾翼减速板或阻力伞的高阻弹（如美制 `Mk 82 Snakeye` 的 `brakeTime = [0.3, 0.5]`、`brakeCxK = 110.0`），投出后在 $[t_0, t_1]$ 区间内线性展开减速机构。时刻 $t$ 的瞬时有效阻力面积 $C_d S(t)$ 为：

$$
\beta_{\text{brake}}(t) = \begin{cases}
1, & t \le t_0 \text{ 或无减速机构} \\[4pt]
1 + (\text{brakeCxK} - 1)\,\operatorname{clamp}\!\left(\dfrac{t - t_0}{t_1 - t_0},\; 0,\; 1\right), & t > t_0
\end{cases}
$$

$$
C_d S(t) = \text{dragCx} \cdot \text{CxK} \cdot \kappa_{\text{scale}} \cdot \beta_{\text{brake}}(t)
$$

航弹在高度 $y$ 处的瞬时三维加速度微分方程（重力加速度 $g = 9.81\text{ m/s}^2$）：

$$
\mathbf{a}_{\text{bomb}}(\mathbf{p}, \mathbf{v}, t) = \begin{bmatrix} 0 \\ -g \\ 0 \end{bmatrix} - \frac{\rho(p_y)\, C_d S(t)\, \|\mathbf{v}\|}{2m}\,\mathbf{v}
$$

采用步长 $\Delta t = 0.005\text{ s}$ 的**二阶龙格-库塔法（Heun 预测-校正格式）**推进状态：

$$
\begin{aligned}
\mathbf{a}_1 &= \mathbf{a}(\mathbf{p}_n, \mathbf{v}_n, t_n) \\
\tilde{\mathbf{p}} &= \mathbf{p}_n + \mathbf{v}_n \Delta t, \qquad \tilde{\mathbf{v}} = \mathbf{v}_n + \mathbf{a}_1 \Delta t \\
\mathbf{a}_2 &= \mathbf{a}(\tilde{\mathbf{p}}, \tilde{\mathbf{v}}, t_n + \Delta t) \\
\mathbf{p}_{n+1} &= \mathbf{p}_n + \frac{\Delta t}{2}(\mathbf{v}_n + \tilde{\mathbf{v}}), \qquad \mathbf{v}_{n+1} = \mathbf{v}_n + \frac{\Delta t}{2}(\mathbf{a}_1 + \mathbf{a}_2)
\end{aligned}
$$

### 2.3 空射无制导火箭弹两级推进、烧蚀减重与尾翼风标动力学

> [!IMPORTANT]
> **拆包参数防坑说明**：在 `rocketGuns/*.blkx` 中，顶层字段 `dragCx`（例如蜂巢发射巢内的 `0.00001`）仅代表挂架/吊舱对载机的寄生气动阻力，**绝不能用作火箭弹离巢后的飞行阻力系数**；真实弹体气动阻力由弹径横截面积 $S = \frac{\pi}{4}d^2$ 与无量纲阻力系数 `CxK`（如 `Hydra-70` 为 $1.65$，`HVAR` 为 $1.10$）决定。

设发射管相对机体纵轴的固定安装仰角为 $\delta_{\text{rail}}$，则发射导轨单位方向为 $\hat{\mathbf{d}}_{\text{rail}} = \cos\delta_{\text{rail}}\,\mathbf{f} + \sin\delta_{\text{rail}}\,\mathbf{u}$，离管初速为 $\mathbf{v}(0) = \mathbb{I}_{\text{useStartSpeed}}\mathbf{v}_{\text{ac}} + v_{\text{start}}\hat{\mathbf{d}}_{\text{rail}}$。

火箭发动机支持点火延迟 $t_{\text{delay}}$、一级主推力段 $(F_0, \Delta t_0, m_0 \to m_1)$ 与二级续航推力段 $(F_1, \Delta t_1, m_1 \to m_2)$，燃料质量随燃烧时间分段线性烧蚀：

$$
(m(t),\; F(t)) = \begin{cases}
(m_0,\; 0), & 0 \le t < t_{\text{delay}} \\[4pt]
\left(m_0 + (m_1 - m_0)\dfrac{t - t_{\text{delay}}}{\Delta t_0},\; F_0\right), & t_{\text{delay}} \le t < t_{\text{delay}} + \Delta t_0 \\[6pt]
\left(m_1 + (m_2 - m_1)\dfrac{t - t_{\text{delay}} - \Delta t_0}{\Delta t_1},\; F_1\right), & t_{\text{delay}} + \Delta t_0 \le t < t_{\text{delay}} + \Delta t_0 + \Delta t_1 \\[4pt]
(m_2,\; 0), & t \ge t_{\text{delay}} + \Delta t_0 + \Delta t_1
\end{cases}
$$

在离轨初期（$t \le 0.03\text{ s}$），火箭弹纵轴 $\hat{\mathbf{e}}(t)$ 锁定在导轨方向 $\hat{\mathbf{d}}_{\text{rail}}$；离轨后受尾翼稳定力矩（风标效应，由质心至压心距离 $L_{\text{stab}} = \text{distCmToStab}$ 决定收拢速率 $\omega_{\text{wc}} = \operatorname{clamp}(4 + 18 L_{\text{stab}}, 4, 16)\text{ s}^{-1}$）快速转向瞬时速度单位向量 $\hat{\mathbf{v}}$：

$$
\hat{\mathbf{e}}(t + \Delta t) = \operatorname{normalize}\!\Big((1 - \omega_{\text{wc}}\Delta t)\,\hat{\mathbf{e}}(t) + (\omega_{\text{wc}}\Delta t)\,\hat{\mathbf{v}}(t)\Big)
$$

火箭弹瞬时三维加速度方程为：

$$
\mathbf{a}_{\text{rkt}}(\mathbf{p}, \mathbf{v}, \hat{\mathbf{e}}, t) = \frac{1}{m(t)}\left(F(t)\,\hat{\mathbf{e}} - \frac{1}{2}\rho(p_y)\,\text{CxK}\,\left(\frac{\pi d^2}{4}\right)\|\mathbf{v}\|^2 \hat{\mathbf{v}}\right) + \begin{bmatrix} 0 \\ -g \\ 0 \end{bmatrix}
$$

### 2.4 Dagor 四三角形网格地表高程与线性求交

当同级目录存在《婴儿弹道计算机》数据包时，程序自动只读挂载全部 `56` 张陆战地图的 `16-bit` 压缩高程网格（`.hpred` 采用二维 Paeth 型线性预测器 $q_{x,z} = \Delta_{x,z} + q_{x-1,z} + q_{x,z-1} - q_{x-1,z-1} \pmod{2^{16}}$ 无损还原）。对齐 Dagor 地形碰撞器，每个网格单元 $[x_0, x_0+1] \times [z_0, z_0+1]$ 以中心均值点 $h_m = \frac{1}{4}(h_{00} + h_{10} + h_{01} + h_{11})$ 剖分为上、下、左、右四个共顶点三角形。记单元中心局部偏移 $f_x = p_x - x_0 - 0.5 \in [-0.5, 0.5]$、$f_z = p_z - z_0 - 0.5 \in [-0.5, 0.5]$：

$$
h_{\text{terr}}(X, Z) = \begin{cases}
h_m + (h_{10} + h_{11} - 2h_m)f_x + (h_{11} - h_{10})f_z, & |f_x| \ge |f_z| \text{ 且 } f_x \ge 0 \\
h_m + (2h_m - h_{00} - h_{01})f_x + (h_{01} - h_{00})f_z, & |f_x| \ge |f_z| \text{ 且 } f_x < 0 \\
h_m + (h_{11} - h_{01})f_x + (h_{01} + h_{11} - 2h_m)f_z, & |f_x| < |f_z| \text{ 且 } f_z \ge 0 \\
h_m + (h_{10} - h_{00})f_x + (2h_m - h_{00} - h_{10})f_z, & |f_x| < |f_z| \text{ 且 } f_z < 0
\end{cases}
$$

当第 $n+1$ 积分步首次满足离地高度 $h_{\text{after}} = p_{n+1,y} - h_{\text{terr}}(p_{n+1,x}, p_{n+1,z}) \le 0$ 时，利用前一步正离地高度 $h_{\text{before}} > 0$ 做亚步长割线插值 $\lambda = \frac{h_{\text{before}}}{h_{\text{before}} - h_{\text{after}}} \in [0, 1]$，得到精确三维落点 $\mathbf{p}_{\text{hit}} = (1-\lambda)\mathbf{p}_n + \lambda\mathbf{p}_{n+1}$。

---

## 三、第三人称相机正向几何与 `2×2` 高斯-牛顿视觉反解

### 3.1 `EFM_HOR_PLUS` 16:9 归一化透视投影模型

对齐开源引擎源码 `DagorEngine/prog/gameLibs/render/cameraView.cpp.inl` 中的 `calc_hor_ver_fov`（模式 `EFM_HOR_PLUS`）及 `bhvProjectionES.cpp.inl`：拆包文件 `config/camera.blkx` 定义的视场角（常规态 `fov = 90°`，缩放态 `zoomFov = 30°`）是以 **$16:9$ 基准宽高比下的水平视场角** 为锚定基准，保持垂直视场角恒定并随实际屏幕宽高比 $W / H$ 自适应扩展水平视野（原生兼容 `16:9`、`16:10` 及 `21:9` 超宽带鱼屏）：

$$
\begin{aligned}
h_k &= \frac{16 / 9}{\tan(\text{fov} / 2)} \\
w_k &= h_k \cdot \frac{H}{W}
\end{aligned}
$$

设世界点 $\mathbf{P}$ 在相机正交基 $(\mathbf{c}_r, \mathbf{c}_u, \mathbf{c}_f)$（眼睛位置为 $\mathbf{E}_{\text{cam}}$）下的局部坐标为 $(x_c, y_c, z_c) = \big((\mathbf{P}-\mathbf{E}_{\text{cam}})\cdot\mathbf{c}_r,\; (\mathbf{P}-\mathbf{E}_{\text{cam}})\cdot\mathbf{c}_u,\; (\mathbf{P}-\mathbf{E}_{\text{cam}})\cdot\mathbf{c}_f\big)$。当 $z_c > 0$ 时，归一化相机射线 $\mathbf{q} = [x_c/z_c,\; y_c/z_c]^\top$ 与屏幕像素坐标 $(u, v)$（左上角为原点）的双向映射为：

$$
\begin{cases}
u = \dfrac{W}{2}\left(1 + \dfrac{x_c}{z_c} w_k\right) \\[8pt]
v = \dfrac{H}{2}\left(1 - \dfrac{y_c}{z_c} h_k\right)
\end{cases}
\quad \Longleftrightarrow \quad
\begin{cases}
q_x = \dfrac{2u / W - 1}{w_k} \\[8pt]
q_y = \dfrac{1 - 2v / H}{h_k}
\end{cases}
$$

### 3.2 第三人称追尾相机轨道几何（`config/camera.blkx`）

由拆包 `camera.blkx` 提取的固定翼第三人称相机精确几何结构如下：

#### （1）横滚防晕机抑制

当相机俯仰角 $|\theta_c| < \text{overturnAngle}\;(80^\circ)$ 时，相机轨道横滚角并非跟随飞机 $1:1$ 翻滚，而是乘以极小阻尼系数 $\phi_{\text{orb}} = \text{rollAngleFactor} \cdot \phi_{\text{ac}} = 0.01\,\phi_{\text{ac}}$，使地平线在机动中基本保持水平。

#### （2）相机眼睛视差偏移与收敛靶点

记相机轨道欧拉基为 $\mathcal{B}_{\text{orb}}(\psi_c, \theta_c, \phi_{\text{orb}})$。合并通用中立偏移 $\mathbf{o}_{\text{neutral}} = [-17.0, 4.0, 0.0]^\top\text{ m}$、单机专属修正 `tpsOffset` 以及升降舵/方向舵微位移 $\Delta\mathbf{o}(e_{\text{elev}}, e_{\text{rud}})$，相机世界坐标与光轴注视收敛点（位于轨道前方 $500\text{ m}$）分别为：

$$
\begin{aligned}
\mathbf{E}_{\text{cam}}(\psi_c, \theta_c) &= \mathbf{p}_{\text{ac}} + \mathcal{B}_{\text{orb}}(\psi_c, \theta_c)\,\big(\mathbf{o}_{\text{neutral}} + \mathbf{o}_{\text{unit}} + \Delta\mathbf{o}\big) \\
\mathbf{T}_{\text{look}}(\psi_c, \theta_c) &= \mathbf{p}_{\text{ac}} + \mathcal{B}_{\text{orb}}(\psi_c, \theta_c)\,[500,\; 0,\; 0]^\top
\end{aligned}
$$

#### （3）施密特左手正交化

相机主光轴 $\mathbf{c}_f = \operatorname{normalize}(\mathbf{T}_{\text{look}} - \mathbf{E}_{\text{cam}})$ 带有约 $\arctan(4 / 517) \approx 0.443^\circ$ 的固有下俯视差角，随后对轨道右轴与上轴执行施密特正交化得到 $(\mathbf{c}_r, \mathbf{c}_u, \mathbf{c}_f)$。

### 3.3 `2×2` 高斯-牛顿相机姿态反解算法

游戏 HUD 上的“机头小十字准星”（Boresight Crosshair）严格对应机头炮线方向 $\hat{\mathbf{g}}_{\text{bore}}$ 在机枪交汇距离 $D_{\text{conv}}$（默认 $500\text{ m}$）处的三维世界点：

$$
\mathbf{P}_{\text{bore}} = \mathbf{p}_{\text{ac}} + D_{\text{conv}}\,\hat{\mathbf{g}}_{\text{bore}}
$$

当视觉模块在屏幕上定位出机头小十字的亚像素坐标 $(u_{\text{nose}}, v_{\text{nose}})$ 后，其观测归一化射线 $\mathbf{q}_{\text{obs}} = [q_{x,\text{obs}}, q_{y,\text{obs}}]^\top$ 已知，整个第三人称相机系统中**仅剩相机轨道偏航角与俯仰角 $\mathbf{x} = [\psi_c, \theta_c]^\top \in \mathbb{R}^2$ 两个未知自由度**。定义二维非线性残差函数：

$$
\mathbf{r}(\psi_c, \theta_c) = \boldsymbol{\pi}_{\text{norm}}\!\big(\mathbf{P}_{\text{bore}};\; \psi_c, \theta_c\big) - \mathbf{q}_{\text{obs}} = \begin{bmatrix} r_x(\psi_c, \theta_c) \\ r_y(\psi_c, \theta_c) \end{bmatrix} = \mathbf{0}
$$

取小角度初值猜测 $\psi_c^{(0)} = \psi_{\text{ac}} - \arctan(q_{x,\text{obs}})$、$\theta_c^{(0)} = \theta_{\text{ac}} - \arctan(q_{y,\text{obs}})$，以步长 $\varepsilon = 10^{-3\circ}$ 构造 $2\times 2$ 数值雅可比矩阵 $\mathbf{J}_k$ 并执行阻尼牛顿步（单步限幅 $\pm 25^\circ$）：

$$
\mathbf{J}_k = \begin{bmatrix}
\dfrac{\partial r_x}{\partial \psi_c} & \dfrac{\partial r_x}{\partial \theta_c} \\[8pt]
\dfrac{\partial r_y}{\partial \psi_c} & \dfrac{\partial r_y}{\partial \theta_c}
\end{bmatrix}, \qquad
\begin{bmatrix} \Delta\psi_c \\ \Delta\theta_c \end{bmatrix} = -\mathbf{J}_k^{-1} \mathbf{r}(\psi_c^{(k)}, \theta_c^{(k)})
$$

由于 $\mathbf{r}(\psi_c, \theta_c)$ 在前向半球内严格单调且良态，迭代仅需 $3 \sim 5$ 步（耗时 $< 3\;\mu\text{s}$）即可收敛至 $\|\mathbf{r}\|_2 < 10^{-7}$。单元测试 `test_inverse_tps_camera_solver_exact_roundtrip` 验证了在 $1080\text{p}$、$1440\text{p}$ 右键开镜及 $3440\times 1440$ 带鱼屏下，即使鼠标大幅甩偏 $18^\circ$，反解相机投影地面落点的理论重投影误差恒小于 **$0.02\text{ px}$**。

---

## 四、抗干扰准星光学检测与零闪烁、零残影 HUD 工程机制

### 4.1 半透明 HUD 色度突出量（Chromatic Excess）软评分

《战争雷霆》原生 HUD 准星通常以 $75\% \sim 85\%$ 不透明度混合在蓝天、白云、森林或雪地背景上。传统硬 RGB 阈值在半透明红色准星叠上高亮蓝天（像素 B 通道高达 $180+$）时会直接漏检。本程序提取目标颜色通道相对于背景干扰通道的**色度突出量** $E(R, G, B)$ 转化为连续权重 $w \in [0, 255]$（以玩家最常用的**红色准星**为例）：

$$
E_{\text{red}}(R, G, B) = R - \max(G, B), \qquad
w(x, y) = \begin{cases}
\operatorname{clamp}\!\big(3(E_{\text{red}} - \tau_c) + 64,\; 0,\; 255\big), & R \ge R_{\min} \text{ 且 } E_{\text{red}} \ge \tau_c \\
0, & \text{其他（蓝天 } B>R \text{、草地 } G>R \text{、白云 } R\approx G\approx B \text{ 全归零）}
\end{cases}
$$

### 4.2 二维积分图与“正交四臂激活 + 四对角空腔抑制”卷积算子

在鼠标瞄准模式下，屏幕上同时存在三类完全同色的红色图形：**机头小十字准星**（真正的解算锚点）、**鼠标牵引大圆圈**（像素总数远多于小十字！）、以及**敌方地面单位实心菱形/方块红标与距离文字**。若使用普通颜色质心，识别点会被鼠标大圆圈或敌方红标强行拉偏。

源码（`src/vision.rs`）先在 $768\times 768$ 局部截图上以单遍扫描构建二维积分图（Summed-Area Table）$I(x, y) = \sum_{u < x,\, v < y} w(u, v)$，从而以 **$O(1)$ 常数时间** 计算任意轴对齐矩形响应和。针对覆盖 $1080\text{p}/1440\text{p}/4\text{K}$ 的多尺度十字模板 $(r_{\text{in}}, r_{\text{out}}, h_t)$，在候选中心 $(x, y)$ 处同时评估 **上下左右四正交臂** $(A_L, A_R, A_T, A_B)$、**外半段臂尖** 与 **紧贴十字臂外侧的四个对角象限空腔** $(Q_{\text{NW}}, Q_{\text{NE}}, Q_{\text{SW}}, Q_{\text{SE}})$：

$$
\begin{aligned}
A_{\text{sum}} &= A_L + A_R + A_T + A_B, \qquad A_{\min} = \min(A_L, A_R, A_T, A_B) \\
Q_{\text{diag}} &= Q_{\text{NW}} + Q_{\text{NE}} + Q_{\text{SW}} + Q_{\text{SE}}
\end{aligned}
$$

候选点必须同时通过三重几何门控：
1. **四向正交臂与臂尖连通门控**：$A_{\min} \ge 180$ 且四侧外半段臂尖响应均 $\ge 60$ —— **直接淘汰中心空心、仅局部圆弧过境的鼠标牵引大圆圈**；
2. **正交对称平衡门控**：$\frac{\min(A_L, A_R)}{\max(A_L, A_R)} \ge 0.30$ 且 $\frac{\min(A_T, A_B)}{\max(A_T, A_B)} \ge 0.30$；
3. **四对角空腔纯净度门控**：$Q_{\text{diag}} \le 0.14\,A_{\text{sum}}$ —— 正交十字在 $|dx| > h_t \land |dy| > h_t$ 的四个对角象限内必为空白背景，而**实心敌方红标、菱形方块或文字在对角象限充满同色像素，在此门控下被 $100\%$ 剔除**。

通过门控后按综合响应 $S(x, y) = A_{\text{sum}} + 3.5 A_{\min} - 4.0 Q_{\text{diag}}$ 锁定全局最优像素中心 $(x^*, y^*)$，最后执行**分离轴一维亚像素质心精化**——仅用窄竖臂像素解算水平坐标 $\hat{u}$，仅用窄横臂像素解算垂直坐标 $\hat{v}$，将定位精度推进至 $< 0.5\text{ px}$：

$$
\hat{u} = \frac{\sum_{y \in \text{VertArm}} \sum_{|x - x^*| \le h_t + 1} (x + 0.5)\,w(x, y)}{\sum_{y \in \text{VertArm}} \sum_{|x - x^*| \le h_t + 1} w(x, y)}, \qquad
\hat{v} = \frac{\sum_{|y - y^*| \le h_t + 1} \sum_{x \in \text{HorizArm}} (y + 0.5)\,w(x, y)}{\sum_{|y - y^*| \le h_t + 1} \sum_{x \in \text{HorizArm}} w(x, y)}
$$

### 4.3 防闪烁惯性保持滤波与零残影并集包络清除

针对实机渲染中的频闪与线条粘连问题，系统实现了三项底层保障：

1. **`450 ms` 相机相对偏角惯性保持（`CoastingCameraLock`）**：俯冲开炮火光、航弹爆炸黑烟或高亮白云可能导致机头小十字出现 $1 \sim 20$ 帧的瞬时光学遮挡。程序持续维护相机相对机头的偏角差 $(\Delta\psi_{\text{rel}}, \Delta\theta_{\text{rel}}) = (\psi_c - \psi_{\text{ac}}, \theta_c - \theta_{\text{ac}})$；一旦单帧光学漏检但在 $450\text{ ms}$ 保持窗内，立即以 $(\psi_{\text{ac}}(t) + \Delta\psi_{\text{rel}},\; \theta_{\text{ac}}(t) + \Delta\theta_{\text{rel}})$ 无缝驱动相机投影，彻底消除准星高频闪烁。
2. **单一并集脏矩形包络（`DirtyEnvelope`）与恒定全屏步幅**：透明图层画布分辨率始终锁定为物理显示器全屏尺寸 $(W_{\text{mon}}, H_{\text{mon}})$，游戏窗口客户区偏移 $(x_{\text{vp}}, y_{\text{vp}})$ 仅在坐标层叠加，杜绝窗口化切换引起的内存步幅（Stride）错位；每帧绘制的所有图元（顶部横幅、机头锚点框、连接虚线、航弹圆环、火箭弹菱形及文字阴影）统一汇入单一外扩包络盒 $[x_{\min} - P, y_{\min} - P, x_{\max} + P, y_{\max} + P]$，下一帧整块一次性清零，根治左倾滚转（$u_{\text{bomb}} < u_{\text{nose}}$）时多小矩形边界漏擦导致的虚线残影。
3. **防截图回授护栏（`WDA_EXCLUDEFROMCAPTURE`）**：透明置顶穿透窗口（`WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOPMOST | WS_EX_NOACTIVATE`）在创建时强制调用 `SetWindowDisplayAffinity(hwnd, WDA_EXCLUDEFROMCAPTURE)`，使系统桌面 `BitBlt` 截图对本程序的 HUD 图层完全透明不可见，从根源上阻断程序绘制图形污染自身视觉识别的死循环。

---

## 五、项目结构、快速验证与编译说明

### 5.1 仓库目录结构

| 路径 | 职责说明 |
| :--- | :--- |
| [`src/atmosphere.rs`](./src/atmosphere.rs) | 对齐 `DagorEngine/atmosphere.cpp` 的四阶多项式大气密度、温度、压强、音速与 IAS/TAS 换算 |
| [`src/ballistics.rs`](./src/ballistics.rs) | 航弹（含高阻伞弹延时展开）与空射火箭弹（两级推力、烧蚀减重、尾翼风标）三维 RK2 解算器 |
| [`src/camera_tps.rs`](./src/camera_tps.rs) | `EFM_HOR_PLUS` 16:9 透视模型、第三人称相机轨道几何与 `2×2` 高斯-牛顿姿态反解器 |
| [`src/vision.rs`](./src/vision.rs) | 半透明色度突出量评分、`O(1)` 二维积分图正交十字卷积检测与分离轴亚像素质心定位 |
| [`src/telemetry.rs`](./src/telemetry.rs) | `localhost:8111` 双线程 HTTP Keep-Alive 采集、俄式地平仪符号修正、无线电高度表折算与角速度外推 |
| [`src/terrain_pack.rs`](./src/terrain_pack.rs) | 只读解析 `WTVPACK2` 56 张陆战地图 `.hpred` 高程网格与 Dagor 四三角形地表插值 |
| [`src/hud_render.rs`](./src/hud_render.rs) | 基于 `tiny-skia` 的抗锯齿双武器准星绘制与 `DirtyEnvelope` 零残影增量清除器 |
| [`src/capture_win.rs`](./src/capture_win.rs) | Per-Monitor DPI v2 感知、战雷客户区视口追踪与中心 `768×768` 预分配 DIB 截图 |
| [`src/overlay_win.rs`](./src/overlay_win.rs) | 原生 Win32 中文参数控制台与 `UpdateLayeredWindow` 鼠标穿透透明图层 |
| [`src/main.rs`](./src/main.rs) | `120 FPS` 主事件循环、`CoastingCameraLock` 惯性保持状态机与 `--self-test` 离线自检入口 |
| [`scripts/extract_cas_db.py`](./scripts/extract_cas_db.py) | 从 `War-Thunder-Datamine` 原始 `.blkx` 与汉化 CSV 抽取生成 `data/cas_db.json` 的构建脚本 |
| [`data/cas_db.json`](./data/cas_db.json) | 内置拆包物理数据库（`1,233` 架空地飞机/直升机、`367` 种航弹、`169` 种无制导火箭弹） |

### 5.2 单机试飞场验证步骤

> [!WARNING]
> 请务必仅在**单机试飞场（Test Flight）**或**纯单人自定义房间**中运行测试，切勿带入任何在线匹配战局。

1. **启动程序**：运行 Release 中的单文件可执行程序（或执行 `cargo run --release`）。
   - 若当前未打开游戏，可直接勾选控制台上的 **“内置无游戏三维俯冲演示模式 (F10)”** 在桌面预览动态三维解算与重投影效果。
2. **游戏显示与准星核对**：
   - 将《战争雷霆》显示模式设为 **无边框窗口（全屏窗口化）** 或 **窗口化**（独占全屏会绕过桌面窗口管理器合成）；
   - 控制台默认适配游戏常用的 **红色准星**（亦可在下拉框切换品红 `#FF00FF` / 青色 / 荧光绿 / 黄色），并确认面板上的 **机枪交汇距离** 与游戏出击界面的设置一致（默认 `500 米`）。
3. **进入单机试飞场**：
   - 程序默认勾选“自动识别当前所驾机型”，进入试飞场后自动匹配机体并筛选该机实际可挂载的航弹与火箭弹；
   - 推荐优先使用带原厂机载 CCIP 的机型（列表中标有 `★原生CCIP对照`，如 `A-10A`、`F-4E`、`Su-25`）在试飞场开启游戏自带准星做 `1:1` 重合度对照校验；
   - 屏幕中 **琥珀金圆环** 为航弹实时连续落点（`BOMB`），**天青蓝菱形** 为火箭弹实时连续落点（`RKT`）。

### 5.3 全局快捷键一览

| 快捷键 | 功能说明 |
| :---: | :--- |
| `Z` | 同步游戏内默认 `Z` 键视角缩放（在常规 `90°` 与放大 `30°` 视场角间同步切换） |
| `F7` | 低空平飞掠地（离地约 `15m`）或跑道滑跑时一键标定当前试飞场地面基准海拔 |
| `F8` | 一键翻转开镜/关镜状态（若切出桌面聊天导致缩放状态与游戏反相，按一次即可重新对齐） |
| `F9` | 一键显示 / 隐藏全屏透明穿透 HUD 图层 |
| `F10` | 开启 / 关闭内置无游戏动态三维俯冲演示模式 |

### 5.4 从源码编译与运行自动化验证

环境要求：Windows 10/11 x64，Rust `1.85+`（Edition 2024）。

```powershell
# 1. 运行全部 8 项物理、透视反解、抗干扰红色准星识别与零残影单元测试
cargo test --release

# 2. 编译优化后的单文件 Release 程序
cargo build --release

# 3. 运行无头闭环自检并导出 1920x1080 合成预览图
.\target\release\wt-cas-ccip.exe --self-test --export-preview dist\demo_hud_preview.png
```

若本地拥有新版拆包仓库 `War-Thunder-Datamine`，可通过以下脚本重新生成 `data/cas_db.json`：

```powershell
python scripts\extract_cas_db.py --datamine <War-Thunder-Datamine路径> --out data\cas_db.json
```

---

## 六、开源参考致谢

本项目的数学建模与工程参数校验均建立在以下公开源码与拆包仓库之上：

- **[GaijinEntertainment/DagorEngine](https://github.com/GaijinEntertainment/DagorEngine)**：标准大气四阶多项式模型（`atmosphere.cpp`）与 `EFM_HOR_PLUS` 16:9 归一化透视投影矩阵（`cameraView.cpp.inl`、`bhvProjectionES.cpp.inl`）。
- **[gszabi99/War-Thunder-Datamine](https://github.com/gszabi99/War-Thunder-Datamine)**：全系空地航弹（`bombguns/*.blkx`）、火箭弹（`rocketGuns/*.blkx`）、载具挂架（`flightmodels/*.blkx`）与第三人称相机（`config/camera.blkx`）公开拆包数据。
- **[Warthunder-Open-Source-Foundation/wt_ballistics_calc](https://github.com/Warthunder-Open-Source-Foundation/wt_ballistics_calc)** 与 **[PowerBroker2/WarThunder](https://github.com/PowerBroker2/WarThunder)**：气动积分对照与 `localhost:8111` 俄式地平仪符号特性交叉验证。
