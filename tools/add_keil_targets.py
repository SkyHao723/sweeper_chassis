#!/usr/bin/env python3
r"""给 Keil 工程加第二个 target: sweeper / scrubber 各一个。

为什么要这么做: 靠改源码里的 VEHICLE_SEL 再编译, 两个车型产出的 hex **文件名
完全一样**(都是 stm32-chassis.hex), 编完就分不清哪个是哪个 —— 而几何选错是
安全级错误(转向增益差 2.19 倍), 不能靠"我记得刚才改过"。

做法是**纯文本复制** <Target> 块(第 9..720 行), 只改**五处**:
  TargetName       -> sweeper / scrubber
  OutputName       -> sweeper / scrubber   (于是 hex 叫 sweeper.hex / scrubber.hex)
  OutputDirectory  -> .\Objects\<车型>\    ★ 见下面那条, 必须分开
  ListingPath      -> .\Listings\<车型>\   (只是好看; 实测 Keil 会在下次编译时
                                            把它规范化回 .\Listings\, 无所谓 ——
                                            .htm/.lst 本来就按 target 命名, 不会串)
  Define           -> 追加 VEHICLE_SEL=1 / =2
不用 XML 库重写整个文件, 是为了**除了这几行, 其它字节一个都不动**。

★★ 为什么 OutputDirectory 必须按车型分开(2026-10-12 踩过的坑) ★★
  纯文本复制 <Target> 块会把原来的 `.\Objects\` 一起复制过去, 于是两个 target
  **共用同一个输出目录**, 而 .o 是按源文件名命名的 —— 于是 **main.o 只有一个**。
  然后: 先 Build 洗地机(编出 scrubber 版的 main.o), 再切到扫地机按 F7(Build,
  增量)**它不会重编 main.c**(main.c 比 main.o 旧), 直接把洗地机的 main.o 链成
  sweeper.axf/hex。
  **后果: 选了 sweeper 这个 target, 烧进去的却是洗地机固件** —— 灯闪得快(5Hz 而不是
  2Hz)、几何全错、导航跑飞。而 Keil 的编译日志是干净的 0 Error 0 Warning, 完全看不出来。
  (只有 Rebuild / `UV4 -r` 才会重编, 但谁也不会每次切 target 都 Rebuild。)
  修法就是每个 target 一个独立输出目录 —— 本脚本现在会自动设好。

用法: python tools\\add_keil_targets.py <uvprojx 路径>   (会先备份)
"""
import io
import os
import re
import shutil
import sys

TARGET_OPEN = "    <Target>\n"
TARGET_CLOSE = "    </Target>\n"

SWEEPER_DEF = "USE_STDPERIPH_DRIVER,VEHICLE_SEL=1"
SCRUBBER_DEF = "USE_STDPERIPH_DRIVER,VEHICLE_SEL=2"

OLD_OUT = "<OutputDirectory>.\\Objects\\</OutputDirectory>"
OLD_LST = "<ListingPath>.\\Listings\\</ListingPath>"


def patch_block(block, name, outname, defines):
    """只改 TargetName / OutputName / OutputDirectory / ListingPath / <Define>,
    其余原样。"""
    n1 = block.count("<TargetName>")
    n2 = block.count("<OutputName>")
    n3 = block.count("<Define>USE_STDPERIPH_DRIVER</Define>")
    n4 = block.count(OLD_OUT)
    n5 = block.count(OLD_LST)
    if (n1, n2, n3, n4, n5) != (1, 1, 1, 1, 1):
        raise SystemExit("预期各出现 1 次, 实际 TargetName=%d OutputName=%d Define=%d "
                         "OutputDirectory=%d ListingPath=%d "
                         "—— 工程结构和脚本假设的不一样, 停下来人工看"
                         % (n1, n2, n3, n4, n5))
    block = re.sub(r"<TargetName>[^<]*</TargetName>",
                   "<TargetName>%s</TargetName>" % name, block, count=1)
    block = re.sub(r"<OutputName>[^<]*</OutputName>",
                   "<OutputName>%s</OutputName>" % outname, block, count=1)
    # ★ 每个车型一个独立输出目录 —— 否则两个 target 共用 main.o, 增量编译会串
    block = block.replace(OLD_OUT,
                          "<OutputDirectory>.\\Objects\\%s\\</OutputDirectory>" % name, 1)
    block = block.replace(OLD_LST,
                          "<ListingPath>.\\Listings\\%s\\</ListingPath>" % name, 1)
    # ★ 只改 C 编译器那一处 (Cads)。汇编器(Aads)那处是空的 <Define></Define>,
    #   不匹配这个模式, 所以不会被误改。
    block = block.replace("<Define>USE_STDPERIPH_DRIVER</Define>",
                          "<Define>%s</Define>" % defines, 1)
    return block


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    path = sys.argv[1]
    raw = io.open(path, "r", encoding="utf-8", newline="").read()

    if "<TargetName>sweeper</TargetName>" in raw:
        raise SystemExit("已经加过了(找到 sweeper target), 不重复加。"
                         "要重做就先 git checkout 这个文件。")

    i = raw.index(TARGET_OPEN)
    j = raw.index(TARGET_CLOSE) + len(TARGET_CLOSE)
    block = raw[i:j]

    # 原来的 Target 1 改名成 sweeper 并带上 -D
    sweeper = patch_block(block, "sweeper", "sweeper", SWEEPER_DEF)
    scrubber = patch_block(block, "scrubber", "scrubber", SCRUBBER_DEF)

    out = raw[:i] + sweeper + scrubber + raw[j:]

    bak = path + ".bak"          # .gitignore 里有 *.bak
    shutil.copyfile(path, bak)
    io.open(path, "w", encoding="utf-8", newline="").write(out)
    print("已备份到 %s" % os.path.basename(bak))
    print("已写入两个 target: sweeper / scrubber")
    print("  sweeper : OutputName=sweeper,  Objects\\sweeper\\,   Define=%s" % SWEEPER_DEF)
    print("  scrubber: OutputName=scrubber, Objects\\scrubber\\,  Define=%s" % SCRUBBER_DEF)
    print("")
    print("★ 每个 target 有独立输出目录 —— 换 target 后按 F7 增量编译也不会串。")
    print("★ 但改完这个脚本/工程后, 请务必先 **Rebuild** 一次两个 target, 并确认:") 
    print("    Objects\\sweeper\\sweeper.hex  和  Objects\\scrubber\\scrubber.hex")
    print("    **内容不同**; 上车再用 PB9 灯频(扫地机 2Hz / 洗地机 5Hz)复核一次。")


if __name__ == "__main__":
    main()
