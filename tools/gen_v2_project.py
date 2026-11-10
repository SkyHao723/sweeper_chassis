"""从旧底盘工程 uvprojx 生成 chassis-v2 的 Keil 工程文件。

做法: 只保留第一个 <Target>, 改名/改输出目录/改宏, 再替换 user 和 hardware 两个组的文件清单。
其余 XML(编译选项/调试器/优化等级等)原样继承, 避免手写 5 万字 XML 出错。
"""
import re
import sys

SRC = r"firmware/stm32-chassis/stm32-chassis.uvprojx"
DST = r"firmware/stm32-chassis-v2/stm32-chassis-v2.uvprojx"

with open(SRC, "r", encoding="utf-8") as fh:
    text = fh.read()

# 1) 只保留第一个 Target
spans = [m.span() for m in re.finditer(r"<Target>.*?</Target>", text, re.S)]
if len(spans) < 2:
    sys.exit("expected 2 targets, got %d" % len(spans))
for start, end in reversed(spans[1:]):
    text = text[:start] + text[end:]

# 2) 基本信息
text = text.replace("<TargetName>sweeper</TargetName>", "<TargetName>chassis-v2</TargetName>", 1)
text = text.replace(r"<OutputDirectory>.\Objects\sweeper\</OutputDirectory>",
                    r"<OutputDirectory>.\Objects\chassis-v2\</OutputDirectory>", 1)
text = text.replace("<OutputName>sweeper</OutputName>", "<OutputName>chassis-v2</OutputName>", 1)
text = text.replace("<Define>USE_STDPERIPH_DRIVER,VEHICLE_SEL=1,TRACK_WIDTH_MM=716</Define>",
                    "<Define>USE_STDPERIPH_DRIVER</Define>", 1)
text = text.replace(r"<ListingPath>.\Listings\</ListingPath>",
                    r"<ListingPath>.\Listings\chassis-v2\</ListingPath>", 1)


def file_xml(name, ftype):
    return (
        "            <File>\n"
        "              <FileName>%s</FileName>\n"
        "              <FileType>%d</FileType>\n"
        "              <FilePath>.\\%s\\%s</FilePath>\n"
        "            </File>\n" % (name, ftype, "user" if ftype in (1, 5) and name.startswith(("main", "board", "stm32f10x_")) else "", name)
    )


def group(group_name, entries):
    body = "".join(
        "            <File>\n"
        "              <FileName>%s</FileName>\n"
        "              <FileType>%d</FileType>\n"
        "              <FilePath>%s</FilePath>\n"
        "            </File>\n" % (name, ftype, path)
        for name, ftype, path in entries
    )
    return (
        "        <Group>\n"
        "          <GroupName>%s</GroupName>\n"
        "          <Files>\n%s          </Files>\n"
        "        </Group>\n" % (group_name, body)
    )


USER_GROUP = group("user", [
    ("main.c", 1, ".\\user\\main.c"),
    ("board.c", 1, ".\\user\\board.c"),
    ("board.h", 5, ".\\user\\board.h"),
    ("mode.c", 1, ".\\user\\mode.c"),
    ("mode.h", 5, ".\\user\\mode.h"),
    ("proto.c", 1, ".\\user\\proto.c"),
    ("proto.h", 5, ".\\user\\proto.h"),
    ("stm32f10x_conf.h", 5, ".\\user\\stm32f10x_conf.h"),
    ("stm32f10x_it.c", 1, ".\\user\\stm32f10x_it.c"),
    ("stm32f10x_it.h", 5, ".\\user\\stm32f10x_it.h"),
])

HW_GROUP = group("hardware", [
    ("ws2812.c", 1, ".\\hardware\\ws2812.c"),
    ("ws2812.h", 5, ".\\hardware\\ws2812.h"),
    ("ssd1306.c", 1, ".\\hardware\\ssd1306.c"),
    ("ssd1306.h", 5, ".\\hardware\\ssd1306.h"),
])

pat_user = re.compile(r"<Group>\s*<GroupName>user</GroupName>.*?</Group>", re.S)
pat_hw = re.compile(r"<Group>\s*<GroupName>hardware</GroupName>.*?</Group>", re.S)

text, n1 = pat_user.subn(lambda _: USER_GROUP.rstrip("\n"), text, count=1)
text, n2 = pat_hw.subn(lambda _: HW_GROUP.rstrip("\n"), text, count=1)
if n1 != 1 or n2 != 1:
    sys.exit("group replace failed: user=%d hardware=%d" % (n1, n2))

# 3) 确认没有遗留旧文件引用
for bad in ("OLED", "YbImu", "LED.c", "VEHICLE_SEL", "TRACK_WIDTH_MM"):
    if bad in text:
        sys.exit("leftover reference: %s" % bad)

with open(DST, "w", encoding="utf-8", newline="\r\n") as fh:
    fh.write(text)

# 4) 兜底: XML 必须能解析
import xml.etree.ElementTree as ET
ET.parse(DST)

print("written:", DST)
print("targets:", len(re.findall(r"<Target>", text)))
print("user group files:", text.count("user\\"))
print("hw group files:", text.count("hardware\\"))
