import os
import sys
import json
import requests
import shutil
import re
import tempfile
import time
import hashlib
import logging
from urllib.parse import unquote

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# === 配置区域 ===
# 目标安装目录
TARGET_DIR = "selfdrive/modeld/models"

# 动态源：Sunnypilot 源码中定义 MODEL_URL 的文件地址 (使用 Raw 格式)
FETCHER_PY_URL = "https://raw.githubusercontent.com/sunnypilot/sunnypilot/master/sunnypilot/models/fetcher.py"

# 兜底地址 (万一源码获取失败时使用)
DEFAULT_JSON_URL = "https://raw.githubusercontent.com/sunnypilot/sunnypilot-docs/refs/heads/gh-pages/docs/driving_models_v10.json"
# =================

def check_env():
    """检查脚本是否在 DP 根目录下运行"""
    if not os.path.exists(TARGET_DIR):
        logger.error(f"❌ 错误: 找不到目录 {TARGET_DIR}")
        logger.error("请确保你是在 dragonpilot 的根目录下运行此脚本！")
        sys.exit(1)
    logger.info(f"✅ 环境检查通过，目标目录: {TARGET_DIR}")

def get_with_retry(url, max_retries=3, timeout=10):
    """带重试机制的网络请求"""
    for i in range(max_retries):
        try:
            logger.debug(f"📡 发送请求: {url} (尝试 {i+1}/{max_retries})")
            r = requests.get(url, timeout=timeout)
            r.raise_for_status()
            return r
        except requests.exceptions.RequestException as e:
            logger.warning(f"⚠️ 请求失败 ({i+1}/{max_retries}): {e}")
            if i < max_retries - 1:
                time.sleep(2)
            else:
                logger.error(f"❌ 请求多次失败，放弃: {url}")
                raise


def get_latest_json_url():
    """
    核心功能：从 fetcher.py 源码中动态解析 MODEL_URL
    """
    logger.info("🕵️  正在检查 fetcher.py 获取最新配置源...")
    try:
        r = get_with_retry(FETCHER_PY_URL)
        content = r.text

        # 使用正则表达式查找 MODEL_URL = "..." 或 '...'
        # 解释：查找 MODEL_URL 后面跟着等号，然后是引号，捕获引号中间的内容
        match = re.search(r'MODEL_URL\s*=\s*["\']([^"\']+)["\']', content)

        if match:
            found_url = match.group(1)
            logger.info(f"✅ 成功解析最新 JSON 地址: {found_url}")
            return found_url
        else:
            logger.warning("⚠️  警告: 在源码中未找到 MODEL_URL 定义，将使用默认地址。")
            return DEFAULT_JSON_URL

    except Exception as e:
        logger.warning(f"⚠️  获取源码失败 ({e})，将使用默认地址。")
        return DEFAULT_JSON_URL

def fetch_models(json_url):
    """获取模型列表"""
    logger.info(f"⏳ 正在下载模型列表: {json_url}")
    try:
        r = get_with_retry(json_url)
        data = r.json()

        # 验证数据结构
        if not isinstance(data, list):
            logger.error(f"❌ 模型列表数据格式错误，预期是列表，实际是: {type(data).__name__}")
            logger.debug(f"数据内容: {data}")
            # 尝试从其他可能的结构中提取模型列表
            if isinstance(data, dict):
                # 检查是否有'bundles'、'models'或'items'字段
                models_field = data.get('bundles', data.get('models', data.get('items', [])))
                if isinstance(models_field, list):
                    logger.warning(f"⚠️  数据结构不符合预期，但找到了可能的模型列表字段")
                    return models_field
            sys.exit(1)

        logger.debug(f"✅ 模型列表数据结构验证通过")
        return data
    except json.JSONDecodeError as e:
        logger.error(f"❌ JSON 解析失败: {e}")
        logger.debug(f"响应内容: {r.text[:100]}...")
        sys.exit(1)
    except Exception as e:
        logger.error(f"❌ 获取模型列表失败: {e}")
        import traceback
        logger.debug(traceback.format_exc())
        sys.exit(1)

def parse_onnx_url(pkl_url, filename):
    """将 pkl 的 url 修改为 onnx 的 url"""
    base_dir = os.path.dirname(pkl_url)
    return f"{base_dir}/{filename}"

def check_disk_space(path, required_bytes):
    """检查磁盘空间是否足够"""
    try:
        _, _, free = shutil.disk_usage(path)
        logger.debug(f"💾 可用磁盘空间: {free/1024/1024:.2f} MB, 需要: {required_bytes/1024/1024:.2f} MB")
        return free >= required_bytes
    except Exception as e:
        logger.error(f"❌ 检查磁盘空间失败: {e}")
        return False


def calculate_file_hash(file_path, hash_algo='sha256'):
    """计算文件的哈希值"""
    hasher = hashlib.new(hash_algo)
    try:
        with open(file_path, 'rb') as f:
            for chunk in iter(lambda: f.read(4096), b""):
                hasher.update(chunk)
        return hasher.hexdigest()
    except Exception as e:
        logger.error(f"❌ 计算文件哈希失败: {e}")
        raise


def download_file(url, local_path):
    """下载文件并显示进度，包含磁盘空间检查"""
    logger.info(f"⬇️  正在下载: {url}")
    try:
        # 先发送HEAD请求获取文件大小
        head_r = get_with_retry(url, timeout=5)
        total_length = head_r.headers.get('content-length')

        # 检查磁盘空间
        if total_length:
            total_bytes = int(total_length)
            if not check_disk_space(os.path.dirname(local_path), total_bytes):
                logger.error(f"❌ 磁盘空间不足，需要 {total_bytes/1024/1024:.2f} MB")
                return False

        # 开始下载
        with get_with_retry(url, timeout=30, max_retries=5) as r:
            r.raise_for_status()
            total_length = r.headers.get('content-length')

            with open(local_path, 'wb') as f:
                if total_length is None:
                    logger.info("📥 开始下载 (未知大小)...")
                    f.write(r.content)
                else:
                    dl = 0
                    total_length = int(total_length)
                    logger.info(f"📥 开始下载 (大小: {total_length/1024/1024:.2f} MB)...")
                    for data in r.iter_content(chunk_size=4096):
                        dl += len(data)
                        f.write(data)
                        done = int(50 * dl / total_length)
                        sys.stdout.write(f"\r[{'=' * done}{' ' * (50-done)}] {dl/1024/1024:.2f} MB")
                        sys.stdout.flush()
        print("\n")

        # 验证文件大小
        if total_length:
            actual_size = os.path.getsize(local_path)
            if actual_size != int(total_length):
                logger.error(f"❌ 文件大小不匹配: 预期 {int(total_length)} bytes, 实际 {actual_size} bytes")
                return False

        logger.info("✅ 下载完成")
        return True
    except Exception as e:
        logger.error(f"❌ 下载失败: {e}")
        return False

def install_model(temp_file, target_name, need_backup=True):
    """安装模型（单文件模式，不处理 big_）"""
    dest = os.path.join(TARGET_DIR, target_name)

    try:
        # 备份
        if need_backup and os.path.exists(dest):
            backup_file = dest + ".bak"
            if not os.path.exists(backup_file):
                shutil.copy(dest, backup_file)
                logger.info(f"📦 已备份原文件至: {target_name}.bak")
            else:
                logger.warning(f"⚠️  备份文件已存在，跳过备份: {target_name}.bak")

        # 覆盖
        shutil.copy(temp_file, dest)
        logger.info(f"🚀 已安装: {target_name}")

        # 验证安装后的文件
        if os.path.exists(dest):
            installed_size = os.path.getsize(dest)
            logger.debug(f"📊 安装的文件大小: {installed_size/1024/1024:.2f} MB")
            return True
        else:
            logger.error(f"❌ 安装失败，目标文件不存在: {target_name}")
            return False

    except (IOError, PermissionError, shutil.Error) as e:
        logger.error(f"❌ 安装失败: {e}")
        return False

def main():
    check_env()

    # 1. 动态获取 JSON URL
    latest_url = get_latest_json_url()

    # 2. 下载列表
    data = fetch_models(latest_url)
    logger.info(f"📋 共加载 {len(data)} 个模型")

    # 3. 搜索交互
    while True:
        try:
            query = input("\n🔍 请输入搜索关键词 (例如 'Dark', 'Plan', 'v2') 或直接回车显示全部: ").strip().lower()
        except KeyboardInterrupt:
            logger.info("\n👋 用户中断，程序退出")
            sys.exit(0)

        matches = []
        try:
            # 验证数据结构
            if not isinstance(data, list):
                logger.error(f"❌ 模型列表数据结构错误，预期是列表，实际是: {type(data).__name__}")
                sys.exit(1)

            for model in data:
                if not isinstance(model, dict):
                    logger.warning(f"⚠️  跳过无效的模型条目 (非字典类型): {model}")
                    continue

                name = model.get('display_name', 'Unknown')
                short = model.get('short_name', 'Unknown')
                if query in name.lower() or query in short.lower():
                    matches.append(model)

            if not matches:
                logger.error("❌ 没有找到匹配的模型，请重试。")
                continue
        except Exception as e:
            logger.error(f"❌ 搜索过程出错: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            continue

        logger.info(f"\n找到 {len(matches)} 个模型:")
        for idx, m in enumerate(matches):
            logger.info(f"[{idx}] {m['display_name']} (ID: {m['short_name']})")

        try:
            selection = input("\n👉 请输入序号选择模型 (q退出): ")
            if selection.lower() == 'q':
                logger.info("👋 用户退出")
                sys.exit(0)

            selected_idx = int(selection)
            if 0 <= selected_idx < len(matches):
                selected_model = matches[selected_idx]
                break
            else:
                logger.error(f"❌ 输入无效，序号应在 0-{len(matches)-1} 之间")
        except ValueError:
            logger.error("❌ 输入无效，请输入数字序号")
        except KeyboardInterrupt:
            logger.info("\n👋 用户中断，程序退出")
            sys.exit(0)

    logger.info(f"\n🌟 你选择了: {selected_model['display_name']}")

    # 4. 解析地址
    policy_pkl_url = ""
    vision_pkl_url = ""

    for m in selected_model.get('models', []):
        if m['type'] == 'policy':
            policy_pkl_url = m['artifact']['download_uri']['url']
        elif m['type'] == 'vision':
            vision_pkl_url = m['artifact']['download_uri']['url']

    if not policy_pkl_url:
        logger.error("❌ 错误：该模型配置中找不到 Policy URL")
        sys.exit(1)

    # 构造 ONNX 地址
    policy_onnx_url = parse_onnx_url(policy_pkl_url, "driving_policy.onnx")
    vision_onnx_url = parse_onnx_url(vision_pkl_url if vision_pkl_url else policy_pkl_url, "driving_vision.onnx")

    logger.debug(f"🔗 Policy URL: {policy_onnx_url}")
    logger.debug(f"🔗 Vision URL: {vision_onnx_url}")

    # 5. 下载并安装
    with tempfile.TemporaryDirectory(prefix="model_download_") as temp_dir:
        logger.debug(f"📁 创建临时目录: {temp_dir}")
        try:
            # Policy
            logger.info("\n--- 处理驾驶模型 (Policy) ---")
            temp_policy = os.path.join(temp_dir, "policy.onnx")
            if download_file(policy_onnx_url, temp_policy):
                if not install_model(temp_policy, "driving_policy.onnx", need_backup=False):
                    logger.error("❌ Policy 模型安装失败")

            # Vision
            logger.info("\n--- 处理视觉模型 (Vision) ---")
            temp_vision = os.path.join(temp_dir, "vision.onnx")
            if download_file(vision_onnx_url, temp_vision):
                if not install_model(temp_vision, "driving_vision.onnx", need_backup=False):
                    logger.error("❌ Vision 模型安装失败")

            logger.info("\n🎉🎉🎉 全部完成！")
            logger.info("💡 提示: 如果这是你第一次使用该模型，重启后可能需要几分钟编译，请耐心等待。")

        except Exception as e:
            logger.error(f"❌ 程序运行出错: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            sys.exit(1)
        finally:
            logger.debug(f"🧹 清理临时目录: {temp_dir}")

if __name__ == "__main__":
    main()