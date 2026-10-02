# -*- coding: utf-8 -*-
"""Jetson 部署辅助脚本：SSH 执行命令 / SFTP 上传下载。用法见 main()。"""
import sys, io, time, warnings
warnings.filterwarnings("ignore", message="Blowfish has been deprecated")
import paramiko

HOST = "192.168.144.100"
USER = "nvidia"
PASS = "nvidia"

def conn():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=15)
    return c

def run(cmd):
    c = conn()
    try:
        stdin, stdout, stderr = c.exec_command(cmd, timeout=120)
        out = stdout.read().decode("utf-8", "replace")
        err = stderr.read().decode("utf-8", "replace")
        return out + err
    finally:
        c.close()

def console(cmd):
    """实时流式执行远程命令（如 tail -F），Ctrl+C 退出。"""
    c = conn()
    try:
        chan = c.get_transport().open_session()
        chan.get_pty()
        chan.exec_command(cmd)
        while True:
            if chan.recv_ready():
                data = chan.recv(65536)
                if not data:
                    break
                sys.stdout.write(data.decode("utf-8", "replace"))
                sys.stdout.flush()
            elif chan.exit_status_ready():
                time.sleep(0.1)
                if not chan.recv_ready():
                    break
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        c.close()

def put(local, remote):
    c = conn()
    try:
        sftp = c.open_sftp()
        sftp.put(local, remote)
        sftp.close()
        return "uploaded %s -> %s" % (local, remote)
    finally:
        c.close()

def get(remote, local):
    c = conn()
    try:
        sftp = c.open_sftp()
        with sftp.open(remote, "rb") as f:
            data = f.read()
        sftp.close()
        with open(local, "wb") as f:
            f.write(data)
        return "downloaded %s -> %s (%d bytes)" % (remote, local, len(data))
    finally:
        c.close()

if __name__ == "__main__":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    if len(sys.argv) < 3:
        print("usage: _jetson.py run <cmd> | console <cmd> | put <local> <remote> | get <remote> <local>")
        sys.exit(1)
    op = sys.argv[1]
    if op == "run":
        print(run(sys.argv[2]))
    elif op == "console":
        console(sys.argv[2])
    elif op == "put":
        print(put(sys.argv[2], sys.argv[3]))
    elif op == "get":
        print(get(sys.argv[2], sys.argv[3]))
    else:
        print("unknown op", op)
