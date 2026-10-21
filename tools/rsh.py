#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用密码走 SSH 跑一条命令(Windows 上用, 因为新主控没配免密登录)。

用法:
    python tools/rsh.py 192.168.5.11 123456 "ls -l /dev/ttyUSB*"
    python tools/rsh.py 192.168.5.11 123456 "cat /tmp/x" --put 本地文件 远端路径
    python tools/rsh.py 192.168.5.11 123456 --get 远端路径 本地路径

⚠️ 密码放在命令行上不安全 —— 只在局域网/临时调试用。
"""
import sys

import paramiko

HOST = sys.argv[1]
PASS = sys.argv[2]
USER = "smart"
PORT = 22


def conn():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, port=PORT, username=USER, password=PASS, timeout=8,
              allow_agent=False, look_for_keys=False)
    return c


def main():
    rest = sys.argv[3:]

    # --put 本地 远端   /  --get 远端 本地
    if rest and rest[0] in ("--put", "--get"):
        mode, a, b = rest[0], rest[1], rest[2]
        c = conn()
        sftp = c.open_sftp()
        if mode == "--put":
            sftp.put(a, b)
            print("put: %s -> %s" % (a, b))
        else:
            sftp.get(a, b)
            print("get: %s -> %s" % (a, b))
        sftp.close()
        c.close()
        return 0

    cmd = " ".join(rest)
    c = conn()
    stdin, stdout, stderr = c.exec_command(cmd, timeout=180)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    code = stdout.channel.recv_exit_status()
    c.close()
    if out:
        sys.stdout.write(out)
    if err:
        sys.stderr.write(err)
    return code


if __name__ == "__main__":
    sys.exit(main())
