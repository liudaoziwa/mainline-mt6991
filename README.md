# RMX6688 (realme GT7 / Dimensity 9400+ / MT6991) mainline bring-up

## 功能现状

### 已实现 / 正常工作 ✅

| 子系统 | 说明 |
|---|---|
| CPU 频率 | `cpufreq-hw`(performance-domain):A720 339–2400 / X4 622–3300 / X925 798–3730 MHz |
| UFS 存储 | 挂载 Ubuntu rootfs 正常启动 |
| DRM + 面板 | EXDMA3+BLENDER+OUTPROC 新架构;1280×2800;**60/90/120/144Hz**;DSC 1.2;DPMS off/on;`mediatekdrmfb` fbdev 控制台 |
| 显示颜色 | 强制 OVL 只 advertise 8-bit(历史原因:24.04 mutter 会选 10-bit XR30;当前 **26.10** 继续沿用 8-bit,颜色正常) |
| 屏幕亮度 | AE031 面板 DCS `0x51`(非 PWM),自带 `backlight` 设备;保守上限 `0x800`/默认 `0x600`(厂商正常上限 `0xe0e`/硬件 `0xffe`),GNOME 滑块可调 |
| GPU | panthor / Immortalis-G925:probe、渲染、devfreq/DVFS(338–1612MHz);修了空闲 runtime-PM 循环崩溃 |
| 音频 / 扬声器 | ✅ **扬声器已出声**:TFA9873×2(i2c6 0x34/0x35)+ AFE `I2SOUT4`。根因是 MT6991 i2c 的 `SLAVE_ADDR1=0x94`(mainline 误写 0x04,导致所有 i2c6 器件 NAK);已补 9873 支持、DPCM `DL1` FE→`I2SOUT4` BE、BE startup 自动打开 `AFE_CONN026/027` 的 `DL1` 连接;`pw-play` 440Hz 实机正常。**耳机/听筒/麦克风仍依赖未移植的 MT6681** |
| 触摸屏 | Synaptics S3910 (TCM v1, SPI6):单点/多点、量程 12799×27999→1280×2800、边缘校准;固件不自动刷写。AVDD(MT6373 VTP 3.1V)已交给内核:`mt6991-pmic-early` 在 subsys_initcall 用 DT 序列(bare SWINF)拉起,早于 SPI6/syna-tcm 首次 probe,init 不再上电、不再 unbind/rebind |
| USB-C (host/OTG) | 端口 **host-only**(MTU3 host + mtk-xhci):OTG 转接器下 U 盘/键鼠枚举正常。**MT6379 TCPC 自动 CC 角色切换**(500ms 轮询,PD 窗口须单字节读):待机 DRP toggling;外设(CC=Rd)插入自动开 OTG 5V boost,拔掉自动关并 re-arm;充电器(CC=Rp)插入自动关 boost,5V 充电不受影响。U 盘/充电器交替插拔任意顺序实测可靠(b60,5 个循环零报错)。调试用串口/RNDIS gadget 已退休,eUSB2 repeater phy 链保留 |
| WiFi | MT6653 (connv3, PCIe HIF):扫描/认证/关联/WPA2 四次握手/联网全部正常;conninfra+connadp+connfem+gen4m+PCIe 全量内建。**日志降噪**:`.is_en_sw_emi_read=FALSE` 关掉那条永不回 MSI 的 CB-DMA debug 读(`halMbuRead` 1698→0、EMI MPU 告警 18→0),`kalIsNoMmioReadReason` 不再成立 ⇒ debug dump 自动回退 MMIO,信息不丢 |
| Connectivity 电源栈 | connv3/conninfra、connfem(FEM)、NVRAM 加载、自动上电 |
| 电池电量计 | TI bq27541 @ i2c0 0x55;`/sys/class/power_supply/` 暴露容量/电压/电流/温度,GNOME 显示电量。只读。**i2c0 必须 100kHz**(400kHz 会超时读不到) |
| 充电 (5V 标准) | MT6379(SPMI USID 0x0e):内核驱动(SPMI 控制器 + `mt6379_charger`)把 `AICR`/`ICHG` 提到 3A 并每 3s 轮询重设(抗拔插)。实测 `Charging` ~2.6A;**厂商快充(VOOC)未做** |
| PMIC 栈 | mt6363/mt6373/mt6368/mt6369 四颗全部 `pmic_mng_register_device ok`;connv3 PMIC(mt6373)注册成功;MT6379 eUSB2 repeater probe+power on。主线无 MTK PMIC MFD ⇒ 触摸 VTP 已由内核 `mt6991-pmic-early`(DT 序列 + bare SWINF,subsys_initcall)接管;MT6363 三轨 `md-rails` 已由同一个 pmic-early 的 DT 子节点接管(b42 验证 `already in the target state`,init 副本已删);MT6373 PIO1/2/VAUD18 + GPIO 复位(codec-en GPIO9)由 pmic-early 早期序列接管,init 副本已删(2026-10-17) |
| 按键 (电源/音量) | 电源=MT6363 PWRKEY、音量上=MT6363 HOMEKEY(`mt6363-spmi-keys` 轮询 `TOPSTATUS`,因 MT6991 SPMI 无 IRQ domain);音量下=`gpio-keys` GPIO11。实机可用 |
| SCP (RV33 tinysys) | 以模块形式跑:`scp.ko` / `ccci_fsm_scp.ko`(AP 侧 CCISM)/ `connscp.ko` / `clk-mt6991-drv.ko` / `sensorhub_probe.ko`。`SCP READY` + `freq=190`(DVFS 工作)、ULPOSC cali 通过、AP↔SCP IPI/CCISM 双向通;`scp_ap_ready` sysfs + init `wait_scp_ready()` 保证 modem 在 SCP 完全就绪后再启动 |
| 基带 / Modem (M80) | CCCI/eccci 全量内建 + port_tty.c 给出**真 AT tty `/dev/ttyCCCI0`**;ch14 FS 由内核 `port_fs_reply()` 全量应答(§CCCI_FS_RE_HANDOFF),RPC `0x400F`/`0x4010`/`0x4014` 已内核实现;mdota(901 文件 8.6MB)以 `port_char_mdata` 编进内核。实测 `md_state 0→2→3→4 READY`(25–28 s),长时间零 EE / 零 `-1001`;ModemManager(`mtk-soc` 插件)控制面已通 |
| 热管理 (LVTS) | 31 传感器(25 有效);LVTS 硬件热复位保护,不做软件降频 |
| 传感器 (SCP sensorhub) | 复用 stock SCP 固件,由它驱动 **bmi2xy 加速度+陀螺 / akm09919 磁力计 / tcs3720 距离+ALS**;AP 侧 `sensorhub_probe.ko` 做 IPI/ready 握手、6 个共享内存环(数据/super/列表/调试/cmd)与按 IIO buffer 开关 FW 传感器类型(accel 1 / gyro 4 / mag 2 / prox 8 / cct 95)。用户态 5 个 IIO 设备:`bmi2xy`(s16 mg,9806 µm/s²,mount matrix 修正 ⇒ GNOME 自动旋转)、`bmi2xy_gyro`(s32 µrad/s,10 ms)、`akm09919_mag`(s32 nT ⇒ µT)、`tcs3720_ps`(FW 已阈值化,nearlevel=1)、`tcs3720_light`(lux 取自 cct 多通道 **super 环** value[10],~1 Hz ⇒ 自动亮度);cct 值布局 v[0]=色温K、v[3..6]=RGBC、v[10]=lux |
| 桌面系统 | Ubuntu 26.10 + GNOME/mutter 稳定运行 |
| 日志通道 | pstore/ramoops 可保存 panic 前日志;CCCI 日志分三档,`echo 1 > /sys/kernel/ccci/mdsys/debug` 可临时放开 `[ccci_rx]` 等降噪项 |

### 未完成 / 阻塞 ❌

| 子系统 | 状态 / 卡点 |
|---|---|
| 蓝牙 (MT6653) | 正在从厂商 `bt/linux_v2` 改走**主线 BlueZ(`btmtk`/`btmtkuart`, serdev+H4+STP)**:驱动绑定、`hci0` 注册、connv3 电源、MT6653 握手代码均已实现;但 BT UART 经 **UARTHUB**,AP 第一帧始终无回应(`query uart failed -110`)。引脚复用/复位/uarthub 放行序列厂商 pre_on 顺序均已试过,疑点转向 uarthub 数据面与 connv3 上电竞争 |
| WiFi 热点 (SoftAP) | 开放 AP + NAT 可用(`GT7-Open`);**WPA2 4-way 仍失败**;为避免影响正常启动已 `systemctl disable hotspot`。详见 `doc/CONNECTIVITY_PORT_NOTES.md` §13.7 |
| 数据面(上网) | ModemManager 控制面(握手/注册/APN profile/`mobile_data`)已通,modem 也已注册在 5G NR(`+C5GREG: 0,1`;`AT+CGDCONT?` 裸 `OK` = AP 从未配过 APN);但 `DATA_ACT` 被 modem 本地拒(`D2CPM_INVAL_ARG/NOT_PERM/NW_UNKNOWN`),`AT+CGACT=1,0` 报同一个 cause ⇒ 门槛在 modem 侧状态、与 MIPC payload 无关。缺 RIL 等价物给 ccmni 配地址 |
| 系统 suspend/resume | 电源域下电会丢 bootloader 状态(MIPI TX PHY、blender 链、SMMU、唤醒源均未自包含) |
| 快充 VOOC/SuperVOOC | 需电荷泵 `mt6379_2p`(i2c3 0x53)+专有协议+OPPO 框架;当前仅 5V 标准充电 |
| GPS | **内核侧 100% 打通**(`gps_drv_dl_v062` + DTS `gps@40c00000` + connv2 上电 WDT 修复;chipid/MCU/GDL 链路全通);卡在厂商 userspace(`mnld` 依赖 Android binder)。详见 `doc/GPS_PORT_NOTES.md` |
| 冷启动无 LK 显示 | 依赖 bootloader 显示的中间级(PHY/mutex/blender)尚未由主线完整接管 |
| 10-bit / HDR scanout | **2026-10-18 实测可行、暂不采用**:XR30 编码 `0x20102` 能正确显示;残余朦胧来自下游 8bpp 量化,桌面收益≈0,默认保持 8-bit。详见 `doc/DISPLAY_COLOR_NOTES.md` |
| 卫星通信 | **本机不支持**(FDT 有 `oplus,701_satellite` 节点但硬件未实配) |
| MT6373/MT6363 PMIC 驱动 | 卡在主线 PMIF 的 EXT_REG_LONG(16-bit 长地址)超时 ⇒ 完整 regulator 驱动未启用;触摸 AVDD 由内核 `mt6991-pmic-early`(DT 序列,bare SWINF)接管,其它上电仍走 raw SPMI/寄存器 |
| 相机 / 闪光灯 / ToF | FDT 有 cam0/1/2/4/6、EEPROM、AF(`ak7377m`)/OIS、`tof8801`、`flashlight_core`;mainline **整块未移植**(无 `/dev/video*`) |
| 屏下指纹 | **⏸ 暂停(TEE 依赖)**;`oplus,fp_spi`(SPI3:CLK106/CS107/MOSI108/MISO109,IRQ114,RESET115,DVDD=MT6373 VFP 3.0V);**2026-11-13 内核 transport 已上机通**:SPI3+pinctrl+`fingerprint@0`+VFP pmic-early+`reg_fp_dvdd`+UFF **REE SPI** 驱动(`drivers/misc/mediatek/fingerprint/`,`/dev/fingerprint_dev`,不走 TEE);`/dev/fingerprint_dev` 创建,power/reset/raw SPI/IRQ 全通,**userspace 未做**;stock HAL 实测走 TEE(`/dev/mobicore-user`),REE SPI 驱动无法直接配;最新 Image md5 `6f89706c…` |
| 振动马达 | `awinic,aw8697_haptic` @ i2c5 0x5a(chip id 0x9270 = AW86927);**2026-11-13 已上机**:`aw86927-haptics` 注册,FF 播放时 `GLBRD5 0x00→0x07→0x00`、`DONEI` 置位;仅 EINT 28 未计数(不影响播放) |
| NFC | `tms,nfc`(THN31)@ i2c13 0x28;未移植 |
| 红外遥控 | `oplus,kookong_ir_core` / `oplus,kookong_ir_pwm`;未移植 |
| 耳机 / 听筒 / 麦克风 | MT6681(`scp-i2c1` 0x6b,codec/accdet/auxadc)未移植 ⇒ 耳机检测/录音/通话音频不可用 |
| 语音通话 / VoLTE | 依赖 modem 语音 + MT6681 通话音频 + IMS/RIL;未做 |
| 双卡 / DSDS(SIM2) | FDT 有 `oplus,sim_detect`;目前只在单卡上验证过,SIM2/双卡未验证 |
| USB-C DP Alt Mode / 外接显示 | FDT 有 `usb-dp-selector`/`dptx`/`edp-tx`;当前 USB-C 只做 host 数据,未移植 DP/eDP,外接屏未验证 |
| USB device / gadget 模式 | 当前 `USB_MTU3_HOST=y` + mtk-xhci,只能做 host;要 adb/MTP/RNDIS 需重新启用 gadget(早期 CDC-ACM 见 `doc/usb-gadget-progress-summary.md`) |
| 硬件 RTC / 定时唤醒 | FDT 有 `mediatek,mt6685-rtc`(SPMI MT6685 MFD),无驱动 ⇒ 开机时间不持久、`wakealarm`/定时开机不可用 |

## 布局

| 路径 | 内容 |
|---|---|
| `arch/arm64/boot/dts/mediatek/mt6991*.dts*` | 板级/SoC DTS + 保留内存(提交进内核树) |
| `arch/arm64/kernel/embedded-dtb.S` 等 | CONFIG_ARM64_EMBEDDED_DTB:把 DTB 用 .incbin 链进 Image |
| `initramfs/` | **独立嵌套 git 仓库**:initramfs 源码(init.c 调试 init / init-rootfs.c rootfs 切换 init)以及 `firmware/`(触摸 + mali CSF + MT6653 WiFi/BT 固件/NVRAM + conninfra/connfem cfg + tfa98xx.cnt + md1drdi.bin)——固件与 init 源码必须在嵌套仓库里,否则本仓库构建的镜像会缺固件/缺 init。`init` 与 `cpio_list` 是生成物,已被 .gitignore |
| `out/` | kbuild O=out 全部产物(.config/Image/System.map/dtb,已 gitignore) |
| `build.sh` | 一次构建(6 步,见下;`O=out`,构建 loadable modules + 静态 init 后链进 Image) |
| `../bt/`(工作区根) | `magiskboot` + 原厂 `boot.img`;成品 `boot-new.img` 生成在工作区根(`/root/mainline/boot-new.img`),用两条 magiskboot 命令打包 |
| `rmx6688.fragment` | defconfig 之上合并的 Kconfig 增量 |
| `gen_resmem.py` / `verify_dtb.py` / `fdt` | resmem dtsi 的生成/核对工具 + 原厂 FDT |
| 仓库根 `*.ko` | 模块构建的缓存位置(build.sh 从树内 find 后 cp 过来再嵌进 initramfs);`.ko` 不入库 |

## 一次构建

```sh
# 默认 INIT_SRC=init-rootfs.c（切 Ubuntu rootfs）；INIT_SRC=init.c 为 bring-up 测试 init
./build.sh
# 内核工具链默认 clang/LLVM/lld(kbuild LLVM=1);TOOLCHAIN=gcc ./build.sh
# 整体切回 gcc/binutils,LLVM=-<N> 可钉某个 clang 版本(如 LLVM=-18)。
#    glibc:用仓内 minic(initramfs/minic/,手写最小 libc)。与 build.sh [1/6] 等价:
clang -static -O2 -Wall -Wextra -ffreestanding -fno-builtin -fno-stack-protector \
  -fno-pie -nostdinc -nostdlib -Wl,-no-pie -Wl,--build-id=none \
  -I initramfs/minic/include -o initramfs/init \
  initramfs/minic/crt0.S initramfs/minic/syscalls.c initramfs/minic/string.c \
  initramfs/minic/stdio.c initramfs/minic/alloc.c initramfs/minic/dirent.c \
  initramfs/minic/misc.c initramfs/init-rootfs.c
strip initramfs/init
#    (TOOLCHAIN=gcc 时命令行照改用 gcc;调试 init 换成 init.c,它仍链接 glibc)
```

整套构建(`./build.sh`)同样可在 x86_64 主机上直接运行:默认 clang/LLVM
工具链(clang≥17 + lld)即可交叉编译,无需交叉 binutils。注意复制源码时
必须带上 `initramfs/`(嵌套仓库,含固件与 init 源码),`out/` 是产物不要带。

## 注意事项

| 项目 | 说明 |
|---|---|
| 获取KVM | 擦除gz分区以获得KVM，刷回安卓也要一并刷回gz分区，获取KVM有一定风险，在mtk新v6设备上对于gz hyp，bl2_ext有gz hyp为空时的跳过路径，所以只需要擦除gz分区就可以让内核跑在el2上。但在安卓上并非擦除gz分区就可以正常启动到系统，依赖gz hyp服务的厂商内核模块会导致kernel panic反复重启，由于mtk设备历来的ctrl_boot没有修复，这可能会导致无限重启变砖。至于在安卓上如何开启kvm看动态coolapk@六道纸花，可以在主线内核与安卓上都能用kvm。|
| initramfs | 内嵌于内核镜像，这样可以只刷boot镜像不需要单独刷initrd到别的分区，内嵌使用了zstd压缩尽可能减少内核体积 |
| 嵌入dtb | 具体实现看倒数1-2条commit，可以在不移植uefi，u-boot等引导程序的情况下让lk直接引导主线内核且内嵌dtb自动忽略lk传递的安卓dtb，不需要动vendor_boot，dtbo存在的dtb，只需要刷入boot分区，内核使用内嵌的dtb启动 |
