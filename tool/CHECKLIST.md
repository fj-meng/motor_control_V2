# 验证清单

从零到"拿到带时间戳的 CANLOG.BIN"，按顺序做。**每一步都有明确的通过判据，不要跳步。**

跳步的代价：如果直接改 PS 代码再上板，出问题时你分不清是 PL 的问题、AXI 的问题、  
还是 PS 补丁的问题，只能瞎猜。

---

## 阶段 0 · 动手前确认三件事

| # | 确认什么                 | 怎么确认                     | 不对会怎样                        |
| - | -------------------- | ------------------------ | ---------------------------- |
| 1 | **GNSS 1PPS 的极性和电平** | 示波器量一下，空闲是高还是低？脉冲宽度多少？   | 极性反了收不到 PPS，`pps_seen` 永远为 0 |
| 2 | **PL 有没有空闲管脚接 1PPS** | 看原理图，找一个没用到的 IO          | 没管脚就没法接                      |
| 3 | **Vivado 版本**        | 你用的是 2017.4 + SDK 2017.4 | 版本不匹配导出 xsa 会出问题             |

> 本工程按「**低电平有效、空闲高、脉宽 1 ms**」实现。  
> 如果是「高电平有效、空闲低」，改 `rtl/pps_sync.v` 两行，见 README 第 10 节第 1 条。

---

## 阶段 1 · 先跑仿真（不用硬件）

**这是最便宜的一步，别跳过。**

1. Vivado 新建工程，器件选你板子上的型号
2. 加入 `rtl/` 下 6 个 `.v` 和 `sim/tb_time_base.v`
3. 把 `tb_time_base` 设为 Simulation Top
4. Run Behavioral Simulation

### 通过判据

Tcl Console 里应该出现（**全部是英文，如果看到中文乱码说明你用的是旧版 testbench**）：

```
==================================================
 tb_time_base : start
==================================================
[958000] PASS tick_cnt = 9 at cyc_cnt = 180
[1043000] pps_stb #1 : prev_pps_cyc=0 sec_cnt=0
[2043000] pps_stb #2 : prev_pps_cyc=200 sec_cnt=1
[3043000] pps_stb #3 : prev_pps_cyc=200 sec_cnt=2
[4043000] pps_stb #4 : prev_pps_cyc=200 sec_cnt=3
[5043000] pps_stb #5 : prev_pps_cyc=200 sec_cnt=4
[6043000] pps_stb #6 : prev_pps_cyc=200 sec_cnt=5
... further pps_stb messages suppressed ...
--------------------------------------------------
 Phase 1 : normal PPS
--------------------------------------------------
PASS pps_cyc = 200
PASS pps_seen = 1
PASS holdover = 0
     sec_cnt  = 6
     tick_cnt = 0
     ts_now   = 0000000600000000
     ts_now[63:32] = 6  (expect 6)
     ts_now[23:0]  = 0  (expect 0)
PASS ts_now second field matches sec_cnt
--------------------------------------------------
 Phase 2 : stop PPS, expect holdover
--------------------------------------------------
[7548000] holdover asserted after 300 clocks, sec_cnt=6
PASS seconds advanced in holdover (6 -> 9)
--------------------------------------------------
 Phase 3 : PPS returns, expect holdover clear
--------------------------------------------------
PASS holdover cleared after 452 clocks
==================================================
 RESULT : ALL CHECKS PASSED
==================================================
```

**只要最后一行不是 `RESULT : ALL CHECKS PASSED`，先别往下走。**

注意两个关键数字：

- `prev_pps_cyc=200` —— 如果显示 199，说明 `time_base.v` 里  
  `pps_cyc <= cyc_cnt + 1'b1;` 漏了那个 `+1`，会让 ppm 永远算成 0
- `holdover asserted after 300 clocks` —— 300 是 `CLK_HZ + CLK_HZ/2`，  
  即 1.5 秒的守时门限

> 仿真里的 `CLK_HZ` 缩小到了 200（真实是 2 亿），逻辑行为完全一致，只是跑得快。  
> `pps_stb` 的打印只保留前 10 条，否则一秒一条会刷几千行把控制台淹掉。

### 关于仿真输出的中文乱码

**Vivado XSim 的 Tcl Console 不支持 UTF-8**（2017.4 尤其明显）。  
它按系统本地编码（中文 Windows 是 GBK/CP936）解释 `$display` 里的字节，  
而源文件是 UTF-8，一个汉字 3 字节被按 2 字节一组解读，就成了乱码；  
有些字节组合还会被当成非法序列直接丢掉。

所以本工程的 testbench **所有 `$display` 都用 ASCII**。  
RTL 里的中文注释不受影响（注释不输出到控制台）。

如果你自己加调试打印，也请用英文。

---

## 阶段 2 · 搭 Vivado 工程

### 2.1 Block Design

| IP                          | 设置                                                                        |
| --------------------------- | ------------------------------------------------------------------------- |
| **ZYNQ7 Processing System** | `M_AXI_GP0` **使能**；`FCLK_CLK0` = **100 MHz**；CAN0 保持 MIO10/MIO11；UART1 保持 |
| **Clocking Wizard**         | 输入 10.000 ns（FCLK_CLK0 100 MHz）；输出 **200 MHz**；`locked` 接出去 |
| **AXI Interconnect**        | Run Connection Automation 自动生成                                            |

**不需要** AXI DMA、不需要 HP 端口、不需要 PL 中断。

### 2.2 接线

Run Connection Automation 会自动插入一个 `proc_sys_reset`（`rst_ps7_0_100M`），
**留着它，别删**。检查它的五个输入：

| 端口 | 接什么 | 注意 |
|---|---|---|
| `slowest_sync_clk` | `FCLK_CLK0` | **必须接** |
| `ext_reset_in` | `FCLK_RESET0_N` | 直连，标准接法 |
| `dcm_locked` | 常量 `1'b1` | **悬空会让复位永不释放** |
| `aux_reset_in` | 常量 `1'b0` | |
| `mb_debug_sys_rst` | 常量 `1'b0` | |

```
clk_wiz/clk_in1         <- ps/FCLK_CLK0 (100 MHz)
clk_wiz/reset           <- rst_ps7_0_100M/peripheral_reset[0]  (都是高有效)
clk_wiz/clk_out1        -> timer_outer_top/clk_200m
clk_wiz/locked          -> timer_outer_top/rst_n_ext
ps/FCLK_CLK0            -> timer_outer_top/s_axi_aclk
                        -> rst_ps7_0_100M/slowest_sync_clk
ps/FCLK_RESET0_N        -> rst_ps7_0_100M/ext_reset_in
rst_ps7_0_100M/peripheral_aresetn[0]    -> timer_outer_top/s_axi_aresetn
rst_ps7_0_100M/interconnect_aresetn[0]  -> axi_ic/ARESETN
ps/M_AXI_GP0            -> axi_ic -> timer_outer_top/S_AXI
pps_n                   -> 外部端口，接 GNSS 的 1PPS
```

> `peripheral_reset` / `bus_struct_reset` 是高有效，**不要**接到 `s_axi_aresetn`。
> 但 `peripheral_reset[0]` 正好可以接 `clk_wiz/reset`（Clocking Wizard 的 reset 默认高有效）。
> **别把 `FCLK_RESET0_N` 直连到 `clk_wiz/reset`** —— 极性反了，MMCM 永远不锁定。

### 2.3 Address Editor

`timer_outer_top` 的 `S_AXI` 分配 **`0x43C00000`，范围 4K**。

> 改了地址就要同步改 `sw/timer_pl.h` 里的 `TIMER_PL_BASE`。

### 2.4 约束

加 `constr/timer_outer.xdc`，**把里面的管脚号换成你板子原理图上的实际管脚**  
（`pps_n` 和 `rst_n_ext`，后者如果接了 `locked` 就不用管脚了）。

### 2.5 综合 → 实现 → 生成 bitstream → 导出 xsa

通过判据：Implementation 时序收敛，无红色。这个设计很小，200 MHz 不该有问题。

---

## 阶段 3 · 单独验证 PL（**关键，别跳过**）

这一步的目的是：在碰任何 CAN 代码之前，先确认 PL 时钟是活的。

1. SDK 新建一个 **Empty Application**
2. 只加三个文件：`sw/timer_pl.c`、`sw/timer_pl.h`、`sw/timer_pl_selftest.c`
3. 编译下载，打开串口（115200 8N1）

### 通过判据（下面是 2026-11-01 20:10 的真实通过记录）

```
=== timer_pl selftest ===
base = 0x43C00000
TIME: id=0x54494D45, scratch ok
waiting for GNSS 1PPS (timeout 60 s)...
  start: level=1 fall_cnt=0 rise_cnt=0
  [ 1 s] PPS detected
polarity probe: sampling 1PPS level for 2 s...
  1998000 of 2000000 samples high = 99.9 %
  raw debug = 0x00030303 (fall=3 rise=3)
  -> IDLE HIGH: polarity CORRECT (active-low 1PPS)

1 s delta = 1000001705 ns (expect 1000000000 +/- 50000)
ppm=+0.00  holdover=0  pps_cyc=200000000

continuous output (reset to exit):
abs=7.603808700 s  tick=6038087  hold=0  ppm=0
abs=8.607975288 s  tick=6079752  hold=0  ppm=2
abs=9.607993276 s  tick=6079932  hold=0  ppm=4
```

六个检查点，逐个确认：

| 检查点 | 期望 | 说明 |
|---|---|---|
| `id=0x54494D45` | 必须对上 | 对不上就是 AXI 通路没通 |
| `scratch ok` | 必须对上 | 对不上说明写通路坏了 |
| `level=1` 且占空比 ≥ 90% | 必须 | **极性正确**；≤10% 说明模块是高有效，见下 |
| `fall` 与 `rise` 都每秒 +1 | 必须 | 低有效 1 ms 脉冲：起点一个下降沿、终点一个上升沿 |
| `1 s delta` | 1000000000 ± 50000 | 注意这测的是 `sleep(1)`，不是时钟 |
| `ppm` | 绝对值 < 5000（±50 ppm） | 实测这块板子只有 ±0.3 ppm |

> `99.9%` 不是估计值：2 秒里 2 个 1 ms 低脉冲 = 2000 个采样低，正好 2000/2000000 = 0.1%。
> 占空比对得上，说明脉宽确实是 1 ms，极性确实是低有效。


### 收不到 PPS 时，看 `polarity probe` 的占空比

**先记住一点：`fall` 和 `rise` 在两种极性下都是每秒各 +1。**
因为 1 ms 脉冲无论是低还是高，都必然有一个起点边沿和一个终点边沿。
所以**不能用边沿计数判断极性，只能看占空比。**

| 现象 | 结论 | 怎么修 |
|---|---|---|
| 占空比 ≥ 90% 高 | ✅ 正常（低有效，空闲高） | — |
| **占空比 ≤ 10% 高** | ❌ 模块是**高有效**，锁存的是后沿，时间戳固定晚 1 ms | 改 `rtl/pps_sync.v`，见 README 第 10 节第 1 条 |
| 占空比在 40%~60% | ❌ 是方波不是 1PPS | 查信号源 |
| `fall` / `rise` 都不涨 | ❌ 管脚上没信号 | 查接线 / GNSS 是否定位 / 管脚号 |
| `fall` / `rise` 每秒涨几十次 | ❌ 噪声 | 查地线、加滤波 |
| `raw debug` 恒为 `0x00000000` | ❌ **bitstream 里没有 0x18 寄存器** | 重新综合 + 重新导出硬件 + 重导 platform |

> 最后一条是真实踩过的坑：Vivado 用 module reference 引用外部 RTL 时**不会自动跟随**
> 源文件变化。改完 RTL 必须把文件同步进工程 `sources_1/imports/rtl/`，
> 再 Reset Output Products → Reset Synthesis → 重跑，否则综合出来还是旧的。

> **最常见的其实是 GNSS 还没定位。** 冷启动室内可能要几分钟才出 1PPS，
> 不是接错线。先看模块上的 PPS/定位指示灯亮不亮。

> **PPS 还没接线时**：看到 `waiting for GNSS 1PPS ...` 然后超时，是**正常**的。
> 这时只要 `id` 和 `scratch` 都对了，就说明 AXI、复位、时钟、自测逻辑全通，
> 可以先跳到阶段 4 改 PS 代码（此时时间戳走的是 holdover 本地时钟，
> 绝对精度不行，但整条数据通路能验证）。

**跑到这里全部正常，再往下走。**

---

## 阶段 4 · 改 PS 代码

按 `sw/PATCH.md` 改 4 个文件（约 30 行）：

| 文件              | 改什么                                                           |
| --------------- | ------------------------------------------------------------- |
| `logger.h`      | `Pad[12]` → `AbsNsLo / AbsNsHi / Pad[4]`，加 `Logger_PushEx` 声明 |
| `logger.c`      | `Logger_Push` 改造成 `Logger_PushEx` + 兼容包装                      |
| `can0_driver.c` | 中断里读一次 PL 时间，`Logger_Push` → `Logger_PushEx`                  |
| `main.c`        | `TimerPl_Init()`；`PrintLogStatus()` 里加 `TimerPl_UpdatePpm()`  |

另外把 `sw/timer_pl.c/h` 加进这个工程，**把 `timer_pl_selftest.c` 移除**  
（它自带 `main()`，和原来的 `main.c` 冲突）。

### 通过判据

编译必须通过。特别注意 **`logger.c` 里的 `LogRecordSizeCheck` 编译期断言**：

```c
typedef char LogRecordSizeCheck[(sizeof(LogRecord) == LOG_RECORD_SIZE) ? 1 : -1];
```

如果报了 `negative size array`，说明结构体不是 32 字节了 —— 检查是不是  
不小心用了 `u64 AbsNs`（`u64` 要 8 字节对齐，会插入 padding，必须用两个 `u32`）。

---

## 阶段 5 · 跑一次记录

1. 下载 bitstream + elf，串口应该打印：

```
TIME: PL clock ready, id=0x54494d45
CAN0 ready: 1 Mbps, motor ID=1, host ID=100
CAN: RX interrupt on, IRQ=...
```

1. 启动电机自动上传（1 kHz）
2. **串口发 `setutc <Unix秒>` 对表**（在秒的中间发，比如 .500）
   —— 不发也能录，但文件里只有「上电后过了多久」，没有 UTC
3. 串口发 `writemotordata` 开始记录
4. 跑 5~10 秒
5. 发停止命令（把 RAM 里剩下的数据全部落盘）

Unix 秒从电脑取：

```bash
python -c "import time; print(int(time.time()))"
```

### 通过判据（心跳行）

```
rx=1000/s tot=5000 gap=0 ovf=0 drop=0 boff=0 tec=0 rec=0 emask=0 | ... | REC: 4096
```

- `rx` 稳定在 1000 左右
- `ovf`、`drop`、`gap` 全是 0
- `tppm` 稳定在一个小数值（如果加了 4.4 的打印）

---

## 阶段 6 · 验证数据

把 `CANLOG.BIN` 拷到电脑，跑：

```bash
python tools/parse_log.py CANLOG.BIN -v
```

### 通过判据

```
记录数    : 5000
AbsNs=0  : 0 条（PL 未就绪，已排除出拟合）
时间跨度  : Ts 4.999 s, AbsNs 4.999 s
Ts 帧间隔 : 中位数 1000 us, 范围 1000 ~ 1000 us

---- 拟合 AbsNs = a + b·Ts（全局）----
斜率 b    : 1000.00xx ns/us
          （理论 1000.0，偏差 +xx.xx ppm）
截距 a    : xxxxxxxxxxx ns
残差 std  : xxxx ns
```

| 指标           | 良好            | 可接受          | 有问题                      |
| ------------ | ------------- | ------------ | ------------------------ |
| `AbsNs=0` 条数 | 0             | 0            | > 0 → PL 没读到             |
| `斜率 b` 偏差    | < 100 ppm     | < 1000 ppm   | 更大 → 检查 PPS              |
| `残差 std`     | < 1000 ns     | 1000~5000 ns | **> 10000 ns → ISR 被阻塞** |
| 时间跨度         | Ts 和 AbsNs 一致 | 差 < 10 ms    | 差很多 → offset 或校正有问题      |

**残差 std 是最关键的指标** —— 它就是你的时间戳抖动。小于 1 µs 说明中断响应很稳定，  
超过 10 µs 说明中断被什么东西挡住了（比如 ISR 里做了打印或写卡）。

---

## 故障对照表

| 现象                              | 最可能的原因                                                  | 怎么查                               |
| ------------------------------- | ------------------------------------------------------- | --------------------------------- |
| 串口输出中文乱码                        | 终端按 GBK 解释 UTF-8 字节；xil_printf 原样输出源文件字节          | 把 xil_printf 全改英文（本工程已改）          |
| 收不到 PPS，`fall` 不涨               | 没接 / GNSS 没定位 / 管脚号错                                  | 看 `rise` 涨不涨：涨=极性反，都不涨=没信号      |
| 收不到 PPS，`rise` 涨 `fall` 不涨      | 模块输出是高有效，代码按低有效写的                                 | 改 `pps_sync.v` 的边沿检测和复位值            |
| 仿真报 `FAIL pps_cyc = 199`        | `time_base.v` 的 `pps_cyc <= cyc_cnt + 1'b1` 少了 `+1`     | 会让 ppm 恒为 0，必须修                   |
| 仿真报 `FAIL holdover not cleared` | 旧版 TB 用 `wait(pps_stb); @(posedge clk);`，同一时间步内可能没等到下一拍 | 新版改成轮询，已修；若仍报则查 RTL               |
| 串口完全没输出                         | bitstream 没下载 / UART 没配对                                | 先跑 Hello World 确认串口               |
| `TIME: PL clock NOT found`      | 地址不对 / `M_AXI_GP0` 没使能 / bitstream 没下                   | 查 Address Editor 是不是 `0x43C00000` |
| `id` 不是 `0x54494d45`            | AXI 接到了别的从机，或地址错位                                       | 用 ILA 抓 AXI 读事务                   |
| AXI 读全 0 / 全 F，总线像挂了          | `proc_sys_reset` 的 `dcm_locked` 悬空，复位永不释放                | 把 `dcm_locked` 接常量 `1'b1`          |
| 等 30 秒没有 PPS                    | GNSS 冷启动没定位（最常见）                                     | 看模块 PPS 指示灯，室外等几分钟              |
| `ppm` 数值是几万甚至几十万                | 上电首秒的残秒读数                                               | 正常，等 3 秒后就正常了                     |
| `ppm` 一直在 0                     | `PPS_CYC` 没锁存 / 一直 holdover                             | 看 `STATUS` 的 bit2                 |
| `holdover=1` 常亮                 | PPS 断了，或周期不是 1 s                                        | 量 PPS 周期                          |
| `1 秒实测差值` 偏离很大                  | PPS 不是标准 1PPS（可能是别的脉冲）                                  | 量周期                               |
| 编译报 `negative size array`       | 结构体不是 32 字节                                             | 别用 `u64`，用两个 `u32`                |
| 记录里 `AbsNs` 全 0                 | `TimerPl_Init()` 失败或没在开中断前调用                            | 看启动打印有没有 `TIME: PL clock ready`   |
| 残差 std 十几微秒                     | ISR 里有耗时操作                                              | 检查 `Can0_RecvHandler` 有没有打印       |
| `AbsNs` 非单调                     | offset 在跑的过程中变了                                         | 只在启动时同步一次，别周期性改                   |
| SD 卡写不进去                        | 卡不是 FAT32 / 没插好                                         | 看 `LOG:` 开头的报错                    |

---

## 一句话总结

```
仿真通过 → 综合时序收敛 → id/scratch 对上 → PPS OK → ppm 合理
        → 改 PS 代码编译通过 → rx=1000 gap=0 ovf=0 → 残差 std < 1us
```

**任何一步不过，就停在那一格查清楚，不要带着问题往下走。**
