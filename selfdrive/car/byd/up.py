import os
import paramiko
import time

def upload_directory(sftp, local_dir, remote_dir):
    for root, dirs, files in os.walk(local_dir):
        for file in files:
            local_path = os.path.join(root, file)
            relative_path = os.path.relpath(local_path, local_dir)
            remote_path = os.path.join(remote_dir, relative_path)
            sftp.put(local_path, remote_path)
            print(f"Transferring {local_path} to {remote_path}")


# 硬编码密码
password = "Bao209900.."

# 创建SSH客户端对象
client = paramiko.SSHClient()

# 设置客户端选项，用于接受远程主机的密钥
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

# 连接到远程主机
# client.connect(hostname="192.168.1.131", username="comma", password=password)
# client.connect(hostname="192.168.100.146", username="comma", password=password)
client.connect(hostname="192.168.21.146", username="comma", password=password)
# client.connect(hostname="192.168.182.146", username="comma", password=password)

# 创建SCP客户端对象
scp = client.open_sftp()

upload_directory(scp, '/Users/cmlnt/DSM/0-OP/openpilot_song/selfdrive/car/byd/', '/data/openpilot/selfdrive/car/byd/')

# 定义上传文件的绝对路径列表
absolute_paths = [
    "/Users/cmlnt/DSM/0-OP/openpilot_song/cereal/car.capnp",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/panda/python/__init__.py",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/panda/board/safety.h",
    # "/Users/cmlnt/DSM/0-OP/openpilot_song/panda/board/safety/safety_byd_canfd.h",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/launch_env.sh",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/selfdrive/car/torque_data/override.toml",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/selfdrive/car/values.py",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/openpilot/selfdrive/car/fingerprints.py",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/opendbc/byd_general_pt.dbc",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/opendbc/tang.dbc",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/panda/board/safety/safety_byd.h",
    "/Users/cmlnt/DSM/0-OP/openpilot_song/openpilot/selfdrive/controls/controlsd.py",
]

# 定义目标目录
target_dir = "/data/openpilot/"

# 上传文件
for absolute_path in absolute_paths:

    relative_path = os.path.relpath(absolute_path, "/Users/cmlnt/DSM/0-OP/openpilot_song/")
    # 构建远程路径
    remote_path = os.path.join(target_dir, relative_path)
    # 上传文件
    scp.put(absolute_path, remote_path)
    print(f"Transferring {absolute_path} to {remote_path}")

# 执行远程重启命令
# stdin, stdout, stderr = client.exec_command("sudo reboot")

# 关闭SCP客户端对象和SSH连接
scp.close()
client.close()
