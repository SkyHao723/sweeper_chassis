#!/usr/bin/env python3
"""给 Keil 工程加第二个 target: sweeper / scrubber 各一个。

为什么要这么做: 靠改源码里的 VEHICLE_SEL 再编译, 两个车型产出的 hex **文件名
完全一样**(都是 stm32-chassis.hex), 编完就分不清哪个是哪个 —— 而几何选错是
安全级错误(转向增益差 2.19 倍), 不能靠"我记得刚才改过"。

做法是**纯文本复制** <Target> 块(第 9..720 行), 只改三处:
  TargetName   -> sweeper / scrubber
  OutputName   -> sweeper / scrubber   (于是 hex 叫 sweeper.hex / scrubber.hex)
  Define       -> 追加 VEHICLE_SEL=1 / =2
不用 XML 库重写整个文件, 是为了**除了这三行, 其它字节一个都不动**。

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


def patch_block(block, name, outname, defines):
    """只改 TargetName / OutputName / <Define> 三处, 其余原样。"""
    n1 = block.count("<TargetName>")
    n2 = block.count("<OutputName>")
    n3 = block.count("<Define>USE_STDPERIPH_DRIVER</Define>")
    if (n1, n2, n3) != (1, 1, 1):
        raise SystemExit("预期各出现 1 次, 实际 TargetName=%d OutputName=%d Define=%d "
                         "—— 工程结构和脚本假设的不一样, 停下来人工看" % (n1, n2, n3))
    block = re.sub(r"<TargetName>[^<]*</TargetName>",
                   "<TargetName>%s</TargetName>" % name, block, count=1)
    block = re.sub(r"<OutputName>[^<]*</OutputName>",
                   "<OutputName>%s</OutputName>" % outname, block, count=1)
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
    print("  sweeper : OutputName=sweeper,  Define=%s" % SWEEPER_DEF)
    print("  scrubber: OutputName=scrubber, Define=%s" % SCRUBBER_DEF)


if __name__ == "__main__":
    main()
