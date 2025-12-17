"""
openpilot 驾驶模型下载器

本脚本旨在简化 openpilot 自动驾驶系统模型的下载、安装和管理过程。
它能够动态地从 sunnypilot 的 GitHub 仓库获取最新的驾驶模型配置，
允许用户选择并安装特定的驾驶模型（包括新旧版本），
并自动处理文件的下载、清理和安装，并最终通过git 添加到暂存区。

主要功能：
- 检查运行环境，确保在 Dragonpilot/Openpilot 根目录下。
- 动态解析 sunnypilot 源码，获取最新模型列表的下载地址。
- 提供交互式命令行界面，供用户搜索、选择和确认要安装的模型。
- 支持下载 .onnx, .pkl, .thneed 等格式的模型文件及其元数据。
- 在安装前自动删除目标目录下已存在的模型文件，确保干净安装。
- 下载时显示进度条并检查磁盘空间。
- 将下载的模型文件自动添加到 Git 暂存区，以确保 Openpilot/Sunnypilot 能够正确识别。
- 兼容处理新旧版本的模型结构 (policy/vision 分离模型和 supercombo 一体化模型)。

使用方法：
1. 将此脚本放置在你的 Dragonpilot/Openpilot 仓库的根目录下。
2. 运行 'python3 Model_downloads.py'。
3. 按照提示搜索选择并安装你需要的模型。
"""
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
import subprocess
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
        if isinstance(data, dict):
            # 优先处理字典类型响应，这是当前API的标准格式
            models_field = data.get('bundles', data.get('models', data.get('items', [])))
            if isinstance(models_field, list):
                logger.debug(f"✅ 从字典的'bundles'字段提取模型列表，共 {len(models_field)} 个模型")
                return models_field
            else:
                logger.error(f"❌ 模型列表数据格式错误，字典中找不到有效的模型列表字段")
                logger.debug(f"数据内容: {data}")
                sys.exit(1)
        elif isinstance(data, list):
            # 兼容旧版本的列表类型响应
            logger.debug(f"✅ 直接使用列表类型的模型数据，共 {len(data)} 个模型")
            return data
        else:
            logger.error(f"❌ 模型列表数据格式错误，预期是列表或字典，实际是: {type(data).__name__}")
            logger.debug(f"数据内容: {data}")
            sys.exit(1)
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

def remove_existing_models():
    """删除现有的模型文件，包括ONNX、PKL和supercombo文件"""
    import glob

    # 删除所有新版本模型文件（driving_开头的模型文件）
    driving_files = glob.glob(os.path.join(TARGET_DIR, "driving_*.onnx")) + \
                   glob.glob(os.path.join(TARGET_DIR, "driving_*.pkl"))

    # 删除所有旧版本supercombo文件（supercombo开头的模型文件）
    supercombo_files = glob.glob(os.path.join(TARGET_DIR, "supercombo*.onnx")) + \
                      glob.glob(os.path.join(TARGET_DIR, "supercombo*.thneed")) + \
                      glob.glob(os.path.join(TARGET_DIR, "supercombo*.pkl"))

    # 合并所有要删除的文件列表
    all_files_to_remove = driving_files + supercombo_files

    # 排除非模型文件（如.cc、.h、.pxd、.pyx等）
    files_to_remove = []
    for file_path in all_files_to_remove:
        filename = os.path.basename(file_path)
        if not any(filename.endswith(ext) for ext in [".cc", ".h", ".pxd", ".pyx", ".py", ".md"]):
            files_to_remove.append(file_path)

    removed_files = []
    for file_path in files_to_remove:
        filename = os.path.basename(file_path)
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
                removed_files.append(filename)
            except Exception as e:
                logger.error(f"❌ 删除文件失败: {filename} - {e}")

    if removed_files:
        logger.info(f"🗑️  已删除以下现有模型文件: {', '.join(removed_files)}")
    else:
        logger.info("ℹ️  没有需要删除的现有模型文件")

    return len(removed_files) > 0

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

            # 如果是ONNX文件，添加到git暂存区
            if target_name.endswith(".onnx"):
                try:
                    logger.info(f"🔄 将ONNX文件添加到git暂存区: {target_name}")
                    subprocess.run(['git', 'add', '-f', dest], check=True, capture_output=True)
                    logger.info(f"✅ 已将ONNX文件添加到git暂存区: {target_name}")
                except subprocess.CalledProcessError as e:
                    logger.error(f"❌ 无法将ONNX文件添加到git暂存区: {e}")
                    logger.debug(f"错误输出: {e.stderr}")

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
    supercombo_url = ""
    supercombo_file_name = ""
    supercombo_metadata_url = ""
    supercombo_metadata_file_name = ""
    # 新增supercombo tinygrad变量
    supercombo_tinygrad_url = ""
    has_supercombo_tinygrad = False
    # 新增policy和vision的metadata变量
    policy_metadata_url = ""
    vision_metadata_url = ""
    policy_metadata_file_name = ""
    vision_metadata_file_name = ""

    # 解析模型URL
    has_policy = False
    has_vision = False
    has_supercombo = False
    has_policy_metadata = False
    has_vision_metadata = False

    for m in selected_model.get('models', []):
        if m['type'] == 'policy':
            policy_pkl_url = m['artifact']['download_uri']['url']
            has_policy = True
            # 检查是否有metadata
            if 'metadata' in m and 'download_uri' in m['metadata']:
                policy_metadata_url = m['metadata']['download_uri']['url']
                policy_metadata_file_name = m['metadata']['file_name']
                has_policy_metadata = True
        elif m['type'] == 'vision':
            vision_pkl_url = m['artifact']['download_uri']['url']
            has_vision = True
            # 检查是否有metadata
            if 'metadata' in m and 'download_uri' in m['metadata']:
                vision_metadata_url = m['metadata']['download_uri']['url']
                vision_metadata_file_name = m['metadata']['file_name']
                has_vision_metadata = True
        elif m['type'] == 'supercombo':
            supercombo_url = m['artifact']['download_uri']['url']
            supercombo_file_name = m['artifact']['file_name']
            has_supercombo = True
            # 检查是否有metadata
            if 'metadata' in m and 'download_uri' in m['metadata']:
                supercombo_metadata_url = m['metadata']['download_uri']['url']
                supercombo_metadata_file_name = m['metadata']['file_name']

            # 尝试生成supercombo tinygrad的URL
            if has_supercombo:
                # 生成tinygrad文件名：supercombo-notre-dame.thneed -> supercombo-notre-dame_tinygrad.pkl
                base_name = os.path.splitext(supercombo_file_name)[0]
                tinygrad_file_name = f"{base_name}_tinygrad.pkl"
                # 生成tinygrad URL：替换文件名
                supercombo_tinygrad_url = supercombo_url.replace(supercombo_file_name, tinygrad_file_name)
                has_supercombo_tinygrad = True

    # 验证至少有一个模型
    if not has_policy and not has_vision and not has_supercombo:
        logger.error("❌ 错误：该模型配置中找不到任何模型 URL")
        sys.exit(1)

    # 构造 ONNX 地址
    policy_onnx_url = None
    vision_onnx_url = None

    if has_policy:
        policy_onnx_url = parse_onnx_url(policy_pkl_url, "driving_policy.onnx")

    if has_vision:
        vision_onnx_url = parse_onnx_url(vision_pkl_url, "driving_vision.onnx")
    elif has_policy:
        # 如果没有vision模型但有policy模型，尝试从policy URL构造vision URL
        vision_onnx_url = parse_onnx_url(policy_pkl_url, "driving_vision.onnx")

    logger.debug(f"🔗 Policy PKL URL: {policy_pkl_url}")
    logger.debug(f"🔗 Policy Metadata URL: {policy_metadata_url}")
    logger.debug(f"🔗 Policy ONNX URL: {policy_onnx_url}")
    logger.debug(f"🔗 Vision PKL URL: {vision_pkl_url}")
    logger.debug(f"🔗 Vision Metadata URL: {vision_metadata_url}")
    logger.debug(f"🔗 Vision ONNX URL: {vision_onnx_url}")
    logger.debug(f"🔗 Supercombo URL: {supercombo_url}")
    logger.debug(f"🔗 Supercombo metadata URL: {supercombo_metadata_url}")
    logger.debug(f"🔍 检测到的模型类型: policy={has_policy}, vision={has_vision}, supercombo={has_supercombo}")
    logger.debug(f"🔍 检测到的metadata: policy_metadata={has_policy_metadata}, vision_metadata={has_vision_metadata}")

    # 5. 下载并安装
    with tempfile.TemporaryDirectory(prefix="model_download_") as temp_dir:
        logger.debug(f"📁 创建临时目录: {temp_dir}")
        try:
            # 检查是否是编译版本（prebuilt目录存在）
            is_compiled_version = os.path.exists(os.path.join(os.getcwd(), "prebuilt"))
            logger.debug(f"🔧 是否为编译版本: {is_compiled_version}")

            if has_supercombo:
                # 处理 supercombo 类型模型（旧版本）
                logger.info("\n--- 处理 Supercombo 模型 ---")

                # 在下载模型前，先移除git暂存区的所有pkl和thneed模型
                logger.info("🔄 移除git暂存区的pkl和thneed模型文件...")
                try:
                    # 使用shell=True让通配符正确扩展
                    subprocess.run('git reset HEAD "' + os.path.join(TARGET_DIR, '*.pkl') + '" "' + os.path.join(TARGET_DIR, '*.thneed') + '"', shell=True, capture_output=True)
                    logger.info("✅ 已移除git暂存区的pkl和thneed模型文件")
                except Exception as e:
                    logger.warning(f"⚠️  移除git暂存区模型文件失败: {e}")

                # 先删除现有的模型文件
                remove_existing_models()

                temp_supercombo = os.path.join(temp_dir, supercombo_file_name)
                if download_file(supercombo_url, temp_supercombo):
                    # 安装 supercombo 文件
                    target_name = "supercombo.onnx" if supercombo_file_name.endswith(".onnx") else "supercombo.thneed"
                    if not install_model(temp_supercombo, target_name, need_backup=False):
                        logger.error("❌ Supercombo 模型安装失败")

                # 下载并安装 metadata（如果存在）
                if supercombo_metadata_url:
                    logger.info("\n--- 处理 Supercombo Metadata ---")
                    temp_metadata = os.path.join(temp_dir, supercombo_metadata_file_name)
                    if download_file(supercombo_metadata_url, temp_metadata):
                        if not install_model(temp_metadata, "supercombo_metadata.pkl", need_backup=False):
                            logger.error("❌ Supercombo Metadata 安装失败")

                # 尝试下载并安装 supercombo tinygrad（如果存在）
                if has_supercombo_tinygrad:
                    logger.info("\n--- 尝试下载 Supercombo Tinygrad ---")
                    temp_supercombo_tinygrad = os.path.join(temp_dir, "supercombo_tinygrad.pkl")
                    if download_file(supercombo_tinygrad_url, temp_supercombo_tinygrad):
                        if not install_model(temp_supercombo_tinygrad, "supercombo_tinygrad.pkl", need_backup=False):
                            logger.error("❌ Supercombo Tinygrad 安装失败")
                    else:
                        logger.warning("⚠️ Supercombo Tinygrad 下载失败，跳过安装")
            else:
                # 处理新版本的 policy/vision 模型
                # 在下载模型前，先移除git暂存区的所有pkl和thneed模型
                logger.info("🔄 移除git暂存区的pkl和thneed模型文件...")
                try:
                    # 使用shell=True让通配符正确扩展
                    subprocess.run('git reset HEAD "' + os.path.join(TARGET_DIR, '*.pkl') + '" "' + os.path.join(TARGET_DIR, '*.thneed') + '"', shell=True, capture_output=True)
                    logger.info("✅ 已移除git暂存区的pkl和thneed模型文件")
                except Exception as e:
                    logger.warning(f"⚠️  移除git暂存区模型文件失败: {e}")

                # 先删除现有的模型文件
                remove_existing_models()

                # Policy
                if has_policy:
                    logger.info("\n--- 处理驾驶模型 (Policy) ---")

                    # 优先下载JSON中指定的PKL文件和metadata
                    logger.info("📥 优先下载Policy PKL和metadata...")
                    temp_policy_pkl = os.path.join(temp_dir, "driving_policy.pkl")
                    if download_file(policy_pkl_url, temp_policy_pkl):
                        if not install_model(temp_policy_pkl, "driving_policy_tinygrad.pkl", need_backup=False):
                            logger.error("❌ Policy PKL 安装失败")

                    # 下载 policy metadata 文件
                    if has_policy_metadata:
                        temp_policy_metadata = os.path.join(temp_dir, policy_metadata_file_name)
                        if download_file(policy_metadata_url, temp_policy_metadata):
                            if not install_model(temp_policy_metadata, "driving_policy_metadata.pkl", need_backup=False):
                                logger.error("❌ Policy Metadata 安装失败")

                    # 然后尝试下载ONNX文件（失败不影响整体流程）
                    logger.info("📥 尝试下载Policy ONNX文件...")
                    temp_policy = os.path.join(temp_dir, "policy.onnx")
                    if download_file(policy_onnx_url, temp_policy):
                        if not install_model(temp_policy, "driving_policy.onnx", need_backup=False):
                            logger.warning("⚠️ Policy 模型安装失败")
                    else:
                        logger.warning("⚠️ Policy ONNX下载失败，跳过安装")

                # Vision
                if has_vision:
                    logger.info("\n--- 处理视觉模型 (Vision) ---")

                    # 优先下载JSON中指定的PKL文件和metadata
                    logger.info("📥 优先下载Vision PKL和metadata...")
                    temp_vision_pkl = os.path.join(temp_dir, "driving_vision.pkl")
                    if download_file(vision_pkl_url, temp_vision_pkl):
                        if not install_model(temp_vision_pkl, "driving_vision_tinygrad.pkl", need_backup=False):
                            logger.error("❌ Vision PKL 安装失败")

                    # 下载 vision metadata 文件
                    if has_vision_metadata:
                        temp_vision_metadata = os.path.join(temp_dir, vision_metadata_file_name)
                        if download_file(vision_metadata_url, temp_vision_metadata):
                            if not install_model(temp_vision_metadata, "driving_vision_metadata.pkl", need_backup=False):
                                logger.error("❌ Vision Metadata 安装失败")

                    # 然后尝试下载ONNX文件（失败不影响整体流程）
                    logger.info("📥 尝试下载Vision ONNX文件...")
                    temp_vision = os.path.join(temp_dir, "vision.onnx")
                    if download_file(vision_onnx_url, temp_vision):
                        if not install_model(temp_vision, "driving_vision.onnx", need_backup=False):
                            logger.warning("⚠️ Vision 模型安装失败")
                    else:
                        logger.warning("⚠️ Vision ONNX下载失败，跳过安装")
                elif not has_policy:
                    logger.warning("⚠️  没有找到Vision模型的URL")

            logger.info("\n🎉🎉🎉 全部完成！")
            logger.info("💡 提示: 如果这是你第一次使用该模型，重启后可能需要几分钟编译，请耐心等待。")

            # 尝试使用git强制跟踪模型文件
            try:
                # 获取模型目录中的pkl和thneed文件
                model_files = [f for f in os.listdir(TARGET_DIR) if f.endswith('.pkl') or f.endswith('.thneed')]

                if model_files:
                    logger.info("\n🔄 尝试使用git强制跟踪模型文件...")

                    # 遍历所有pkl和thneed文件
                    for file in model_files:
                        file_path = os.path.join(TARGET_DIR, file)
                        # 使用git add -f强制添加文件
                        subprocess.run(['git', 'add', '-f', file_path], check=True, capture_output=True)
                        logger.info(f"✅ 已强制跟踪: {file}")

                    # 可选：显示git状态
                    logger.debug("📋 当前git状态:")
                    status_result = subprocess.run(['git', 'status'], capture_output=True, text=True)
                    logger.debug(status_result.stdout)

                else:
                    logger.info("ℹ️  没有找到需要跟踪的pkl或thneed文件")

            except subprocess.CalledProcessError as e:
                logger.error(f"❌ Git命令执行失败: {e}")
                logger.debug(f"错误输出: {e.stderr}")
            except Exception as e:
                logger.error(f"❌ 跟踪模型文件失败: {e}")

        except Exception as e:
            logger.error(f"❌ 程序运行出错: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            sys.exit(1)
        finally:
            logger.debug(f"🧹 清理临时目录: {temp_dir}")

def test_old_model():
    """测试旧版本supercombo模型的处理逻辑，包括删除旧模型文件"""
    check_env()

    # 测试删除现有模型文件，包括pkl文件
    logger.info("📋 测试删除现有模型文件...")
    remove_existing_models()

    # 模拟下载成功的情况，创建一些测试文件
    test_files = [
        "supercombo_metadata_gen5.pkl",
        "driving_policy.onnx",
        "driving_vision.onnx"
    ]

    logger.info("📋 创建测试文件...")
    for file in test_files:
        file_path = os.path.join(TARGET_DIR, file)
        with open(file_path, 'w') as f:
            f.write("test content")
        logger.info(f"✅ 创建测试文件: {file}")

    # 使用本地JSON文件进行测试
    json_path = "./driving_models_v10.json"
    with open(json_path, 'r') as f:
        data = json.load(f)

    models = data.get('bundles', [])

    # 找到Farmville模型（旧版本supercombo类型）
    selected_model = None
    for model in models:
        if model.get('display_name') == "Farmville (November 07, 2023)":
            selected_model = model
            break

    if not selected_model:
        logger.error("❌ 未找到Farmville模型")
        return False

    logger.info(f"\n🌟 测试模型: {selected_model['display_name']}")

    # 解析模型URL
    supercombo_url = ""
    supercombo_file_name = ""
    supercombo_metadata_url = ""
    supercombo_metadata_file_name = ""
    has_supercombo = False

    for m in selected_model.get('models', []):
        if m['type'] == 'supercombo':
            supercombo_url = m['artifact']['download_uri']['url']
            supercombo_file_name = m['artifact']['file_name']
            has_supercombo = True
            # 检查是否有metadata
            if 'metadata' in m and 'download_uri' in m['metadata']:
                supercombo_metadata_url = m['metadata']['download_uri']['url']
                supercombo_metadata_file_name = m['metadata']['file_name']

    if not has_supercombo:
        logger.error("❌ 该模型不是supercombo类型")
        return False

    logger.debug(f"🔗 Supercombo URL: {supercombo_url}")
    logger.debug(f"📄 Supercombo 文件名: {supercombo_file_name}")
    logger.debug(f"🔗 Supercombo metadata URL: {supercombo_metadata_url}")
    logger.debug(f"📄 Supercombo metadata 文件名: {supercombo_metadata_file_name}")

    # 检查文件类型
    if supercombo_file_name.endswith('.thneed'):
        logger.info("✅ 检测到旧版本模型格式 (.thneed)")
    elif supercombo_file_name.endswith('.onnx'):
        logger.info("✅ 检测到新版本模型格式 (.onnx)")
    else:
        logger.warning(f"⚠️  未知模型格式: {supercombo_file_name}")

    logger.info("✅ 旧版本模型处理逻辑测试通过！")

    # 测试git强制跟踪功能
    logger.info("\n📋 测试git强制跟踪功能...")
    try:
        # 获取模型目录中的pkl和thneed文件
        model_files = [f for f in os.listdir(TARGET_DIR) if f.endswith('.pkl') or f.endswith('.thneed')]

        if model_files:
            logger.info("🔄 尝试使用git强制跟踪模型文件...")

            # 遍历所有pkl和thneed文件
            for file in model_files:
                file_path = os.path.join(TARGET_DIR, file)
                # 使用git add -f强制添加文件
                subprocess.run(['git', 'add', '-f', file_path], check=True, capture_output=True)
                logger.info(f"✅ 已强制跟踪: {file}")
        else:
            logger.info("ℹ️  没有找到需要跟踪的pkl或thneed文件")

    except subprocess.CalledProcessError as e:
        logger.error(f"❌ Git命令执行失败: {e}")
    except Exception as e:
        logger.error(f"❌ 跟踪模型文件失败: {e}")

    logger.info("✅ Git强制跟踪功能测试完成！")
    return True

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        test_old_model()
    else:
        main()
