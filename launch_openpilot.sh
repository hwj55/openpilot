#!/usr/bin/env bash
export API_HOST=https://api.konik.ai
export ATHENA_HOST=wss://athena.konik.ai
#export MAPS_HOST=https://api.konik.ai/maps
export MAPBOX_TOKEN='pk.eyJ1IjoibXJvbmVjYyIsImEiOiJjbHhqbzlkbTYxNXUwMmtzZjdoMGtrZnVvIn0.SC7GNLtMFUGDgC2bAZcKzg'

# 获取当前脚本所在目录
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

# --- 子模块自动检查与补全脚本 ---
# 检查标准：如果 opendbc_repo 里的关键文件 car.capnp 不存在，则认为子模块不全
if [ ! -f "$DIR/opendbc_repo/opendbc/car/car.capnp" ]; then
  echo "--- [Check] Submodules missing, starting repair ---"

  # 1. opendbc_repo (指定使用 tn 分支)
  echo "Cloning opendbc_repo..."
  rm -rf opendbc_repo
  git clone https://gh-proxy.com/https://github.com/sunnypilot/opendbc.git opendbc_repo -b tn --depth=1
  # 修复 SP 特有的软链接：让 opendbc_repo/opendbc 指向自己
  ln -sfn . opendbc_repo/opendbc

  # 2. panda
  echo "Cloning panda..."
  rm -rf panda
  git clone https://gh-proxy.com/https://github.com/sunnyhaibin/panda.git panda --depth=1

  # 3. msgq_repo
  echo "Cloning msgq_repo..."
  rm -rf msgq_repo
  git clone https://gh-proxy.com/https://github.com/commaai/msgq.git msgq_repo --depth=1

  # 4. rednose_repo
  echo "Cloning rednose_repo..."
  rm -rf rednose_repo
  git clone https://gh-proxy.com/https://github.com/commaai/rednose.git rednose_repo --depth=1

  # 5. teleoprtc_repo
  echo "Cloning teleoprtc_repo..."
  rm -rf teleoprtc_repo
  git clone https://gh-proxy.com/https://github.com/commaai/teleoprtc teleoprtc_repo --depth=1

  # 6. tinygrad_repo
  echo "Cloning tinygrad_repo..."
  rm -rf tinygrad_repo
  git clone https://gh-proxy.com/https://github.com/commaai/tinygrad.git tinygrad_repo --depth=1

  # 7. neural_network_data
  echo "Cloning neural_network_data (模型数据)..."
  mkdir -p sunnypilot
  rm -rf sunnypilot/neural_network_data
  git clone https://gh-proxy.com/https://github.com/sunnypilot/neural-network-data.git sunnypilot/neural_network_data --depth=1

  echo "--- Success ---"
else
  echo "--- Submodules OK  ---"
fi
exec ./launch_chffrplus.sh
