#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
P() { echo; echo "##################### $* #####################"; }

P SYS
hostname; date '+%F %T %Z'; uptime

P ROS_DISTROS
ls -1 /opt/ros 2>/dev/null
echo "ROS_DISTRO=$ROS_DISTRO"

P PROCESSES_ALL
ps -eo pid,ppid,user,etimes,args --sort=start_time | sed 1d | cut -c1-200

P PORT_LISTEN
ss -lntup 2>/dev/null | cut -c1-180

P SYSTEMD_RUNNING
systemctl list-units --type=service --state=running --no-pager --no-legend | cut -c1-160

P SYSTEMD_UNITFILES_MATCH
systemctl list-unit-files --no-pager --no-legend | grep -iE 'ros|robot|chassis|wheel|car|tb01|web|nav|inspect|drive|motor|ly|hmi|cam' | cut -c1-160

P SYSTEMD_CUSTOM_UNITS
ls -l /etc/systemd/system/ 2>/dev/null | cut -c1-160
echo "--- multi-user.target.wants ---"
ls -l /etc/systemd/system/multi-user.target.wants/ 2>/dev/null | cut -c1-160

P CRONTAB
crontab -l 2>&1
echo "--- /etc/cron.* ---"
ls /etc/cron.d/ 2>/dev/null

P RCLOCAL
ls -l /etc/rc.local 2>&1
cat /etc/rc.local 2>/dev/null

P SERIAL
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH* /dev/ttyS0 /dev/ttyS1 /dev/ttyS2 /dev/ttyS3 2>&1 | head -40
echo "--- by-id ---"
ls -l /dev/serial/by-id/ 2>&1
echo "--- can ---"
ip -brief link 2>/dev/null | grep -i can

P NET
ip -brief addr

P DISK_HOME
du -sh /home/smart/*/ 2>/dev/null | sort -h | tail -40

P BASHRC_ROS
grep -nE 'source|export|alias' /home/smart/.bashrc | cut -c1-160
