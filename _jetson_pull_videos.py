# -*- coding: utf-8 -*-
"""从机载批量拉取录像(recordings/*.mp4)。
- 跳过“正在被写”的文件（/proc 里打开着的），保证拉下来的都能播
- 保留 mtime；校验大小一致
用法: python _jetson_pull_videos.py <本地目录>
"""
import sys, io, os, stat, time, warnings
warnings.filterwarnings("ignore", message="Blowfish has been deprecated")
import paramiko

HOST, USER, PASS = "192.168.144.100", "nvidia", "nvidia"
REMOTE_DIR = "/home/nvidia/cuadc/recordings"

def conn():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=15)
    return c

def remote_open_files(c):
    """找出 recordings 里正被进程打开(在写)的文件路径"""
    cmd = "ls -l /proc/*/fd 2>/dev/null | grep -o '/home/nvidia/cuadc/recordings/[^ ]*' | sort -u"
    _, out, _ = c.exec_command(cmd, timeout=30)
    return set(l.strip() for l in out.read().decode("utf-8", "replace").splitlines() if l.strip())

def main():
    if len(sys.argv) < 2:
        print("usage: _jetson_pull_videos.py <local_dir>")
        sys.exit(1)
    local_dir = sys.argv[1]
    os.makedirs(local_dir, exist_ok=True)

    c = conn()
    try:
        sftp = c.open_sftp()
        inflight = remote_open_files(c)
        names = sorted(n for n in sftp.listdir(REMOTE_DIR) if n.endswith((".mp4", ".avi")))
        todo = [n for n in names if f"{REMOTE_DIR}/{n}" not in inflight]
        skipped = [n for n in names if n not in todo]
        total = sum(sftp.stat(f"{REMOTE_DIR}/{n}").st_size for n in todo)
        print(f"共 {len(names)} 个文件, 拉取 {len(todo)} 个 ({total/1e6:.0f} MB), 跳过正在写入 {len(skipped)} 个: {skipped}")
        done_b = 0
        for i, n in enumerate(todo, 1):
            rp, lp = f"{REMOTE_DIR}/{n}", os.path.join(local_dir, n)
            rsize = sftp.stat(rp).st_size
            t0 = time.time()
            sftp.get(rp, lp)  # preserve mtime
            lsize = os.path.getsize(lp)
            ok = "OK" if lsize == rsize else f"!!大小不符 {lsize}!={rsize}"
            done_b += lsize
            dt = time.time() - t0
            print(f"[{i}/{len(todo)}] {n}  {lsize/1e6:.1f}MB  {dt:.1f}s ({lsize/1e6/max(dt,0.01):.1f}MB/s)  {ok}")
        print(f"完成: {done_b/1e6:.0f} MB -> {local_dir}")
        sftp.close()
    finally:
        c.close()

if __name__ == "__main__":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    main()
