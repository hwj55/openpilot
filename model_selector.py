# -*- coding: utf-8 -*-
"""
Dragonpilot 模型选择器
注意：替换模型后有可能因版本问题无法正常使用（如无法显示车道线等），请恢复默认模型或下载与默认模型日期相近模型
功能:
1. 从 sunnypilot 仓库获取最新模型列表
2. 下载模型文件到 /data/media/0/models/ 目录
3. 安装时将文件重命名为 DP 标准名称 (driving_policy_tinygrad.pkl 等)
4. 第一次更换模型时自动备份原模型到 /data/media/0/models/ 目录（保持与sunnypilot兼容）
5. 支持查看、切换已下载的模型
6. 支持删除已下载的模型（保留默认模型）
7. 支持恢复默认模型

使用方法:
1. 直接运行脚本: python3 model_selector.py
2. 选择功能:
   - 1: 下载并安装最新模型
   - 2: 查看并切换已下载的模型（输入'r'可恢复默认模型）
   - 3: 删除已下载的模型
   - q: 退出
3. 下载模型时:
   - 可以搜索模型名称
   - 可以直接输入序号下载
   - 下载完成后可选择立即安装
4. 切换模型时:
   - 输入模型序号切换
   - 输入'r'恢复默认模型
5. 任何情况下按q返回上一级

特性说明:
- 第一次更换模型时，会自动备份原模型到 /data/media/0/models/ 目录
- 下载模型时，会自动下载对应的 ONNX 模型，并命名为 driving_policy_{modelname}.onnx 格式
- 更换模型时，会同时更换 ONNX 文件
- 删除模型时，不会删除备份的默认模型文件
- 恢复默认模型时，会恢复所有备份的模型文件（包括 metadata 和 ONNX 文件）
- 支持在没有网络的情况下使用本地缓存的模型列表

注意事项:
- 确保 /data/media/0/models/ 目录有读写权限
- 下载模型可能需要较长时间，取决于网络速度
- 安装模型后需要重启 Dragonpilot 才能生效
- 模型文件较大，请确保有足够的存储空间
"""
import os
import sys
import json
import urllib.request
import shutil
import re
import time
import logging
import subprocess
from datetime import datetime

# 日志配置
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s', datefmt='%H:%M:%S', handlers=[logging.StreamHandler(sys.stdout)])
logger = logging.getLogger(__name__)

# === 核心配置 ===
# 使用绝对路径，确保在任何环境下都能正常运行
INSTALL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "selfdrive/modeld/models")     # DP 运行目录
LIBRARY_DIR = "/data/media/0/models"        # 本地仓库
# 全局模型配置文件
GLOBAL_INFO_FILE = os.path.join(LIBRARY_DIR, "info.json")
# 本地模型列表文件
LOCAL_MODEL_LIST_FILE = os.path.join(LIBRARY_DIR, "driving_models_v10.json")
# 你的 JSON 来源，如果网络不通，可以改为本地文件路径
JSON_SOURCE_URL = "https://raw.githubusercontent.com/sunnypilot/sunnypilot-docs/refs/heads/gh-pages/docs/driving_models_v10.json"
FETCHER_PY_URL = "https://raw.githubusercontent.com/sunnypilot/sunnypilot/master/sunnypilot/models/fetcher.py"

class ModelSelector:
    def __init__(self):
        self.check_env()
        self.model_list = []

    def check_env(self):
        if not os.path.exists(INSTALL_DIR):
            logger.error(f"❌ 错误: 未找到 {INSTALL_DIR}，请在 Dragonpilot 根目录运行！")
            sys.exit(1)

        # 检查并创建模型仓库目录，处理权限错误
        global LIBRARY_DIR, GLOBAL_INFO_FILE, LOCAL_MODEL_LIST_FILE

        # 尝试使用 /data/media/0/models 目录
        data_models_dir = "/data/media/0/models"
        try:
            if not os.path.exists(data_models_dir):
                os.makedirs(data_models_dir, exist_ok=True)
            # 测试写入权限
            test_file = os.path.join(data_models_dir, ".test_write")
            with open(test_file, 'w') as f:
                f.write("test")
            os.remove(test_file)
            # 如果成功，使用 /data/media/0/models 目录
            LIBRARY_DIR = data_models_dir
            logger.info(f"📁 使用模型目录: {LIBRARY_DIR}")
        except PermissionError:
            # 检查是否使用 sudo 运行
            if os.geteuid() != 0:
                logger.warning(f"⚠️  没有权限访问 {data_models_dir}，请使用 sudo 运行脚本")
            # 使用本地目录作为备选
            LIBRARY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
            logger.warning(f"⚠️  将使用本地目录: {LIBRARY_DIR}")
            os.makedirs(LIBRARY_DIR, exist_ok=True)
        except Exception as e:
            # 其他错误，使用本地目录
            logger.warning(f"⚠️  无法访问 {data_models_dir}: {e}，将使用本地目录")
            LIBRARY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
            os.makedirs(LIBRARY_DIR, exist_ok=True)

        # 更新全局路径
        GLOBAL_INFO_FILE = os.path.join(LIBRARY_DIR, "info.json")
        LOCAL_MODEL_LIST_FILE = os.path.join(LIBRARY_DIR, "driving_models_v10.json")

    def get_with_retry(self, url):
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=15) as response:
                # 模拟requests.Response对象
                class MockResponse:
                    def __init__(self, response):
                        self.text = response.read().decode('utf-8')
                        self.status_code = response.status
                    def raise_for_status(self):
                        pass
                    def json(self):
                        import json
                        return json.loads(self.text)
                return MockResponse(response)
        except: return None

    def fetch_model_list(self):
        """获取并解析 JSON，支持本地缓存"""
        models = []

        # 1. 尝试从 fetcher.py 抓取最新 URL，失败则用默认
        json_url = JSON_SOURCE_URL
        r = self.get_with_retry(FETCHER_PY_URL)
        if r:
            m = re.search(r'MODEL_URL\s*=\s*["\']([^"\']+)["\']', r.text)
            if m: json_url = m.group(1)

        # 2. 尝试从网络获取
        logger.info(f"📡 获取模型列表: {json_url}")
        r = self.get_with_retry(json_url)
        if r:
            try:
                data = r.json()
                # 兼容处理：有些 JSON 根就是 list，有些在 bundles 里
                models = data if isinstance(data, list) else data.get('bundles', data.get('models', []))

                # 尝试保存到本地，如果失败也没关系，继续执行
                try:
                    with open(LOCAL_MODEL_LIST_FILE, 'w') as f:
                        json.dump(data, f, indent=2)
                    logger.info(f"✅ 模型列表已更新到本地: {LOCAL_MODEL_LIST_FILE}")
                except PermissionError:
                    # 如果没有权限写入 /data/media/0/models，忽略保存错误，继续执行
                    logger.warning(f"⚠️  没有权限保存模型列表到 {LOCAL_MODEL_LIST_FILE}，将跳过本地保存")
            except Exception as e:
                logger.error(f"❌ JSON 解析失败: {e}")
                return []
        else:
            logger.warning("⚠️  无法从网络获取模型列表")

            # 尝试从本地缓存读取
            if os.path.exists(LOCAL_MODEL_LIST_FILE):
                try:
                    with open(LOCAL_MODEL_LIST_FILE, 'r') as f:
                        data = json.load(f)
                        models = data if isinstance(data, list) else data.get('bundles', data.get('models', []))
                    logger.info(f"✅ 使用本地模型列表: {LOCAL_MODEL_LIST_FILE}")
                except Exception as e:
                    logger.error(f"❌ 本地模型列表解析失败: {e}")
                    return []
            else:
                logger.error("❌ 本地模型列表不存在，无法获取模型信息")
                return []

        return models

    def add_existing_models_to_info(self, models):
        """
        扫描目录中的现有模型，并将它们添加到 info.json 中
        """
        # 读取当前全局 info.json
        global_info = {
            "downloaded_models": [],
            "current_model": None,
            "current_files": {},
            "last_updated": datetime.now().isoformat()
        }

        if os.path.exists(GLOBAL_INFO_FILE):
            try:
                with open(GLOBAL_INFO_FILE, 'r') as f:
                    existing_info = json.load(f)
                    global_info.update(existing_info)
            except Exception as e:
                logger.warning(f"⚠️  无法读取现有 info.json: {e}")

        # 获取目录中所有文件
        existing_files = set(os.listdir(LIBRARY_DIR))
        logger.info(f"📁 扫描到 {len(existing_files)} 个文件")

        # 遍历所有模型，检查其文件是否存在
        added_count = 0
        for model_info in models:
            model_id = model_info.get('short_name')

            # 检查模型是否已在 info.json 中
            if any(model.get('short_name') == model_id for model in global_info['downloaded_models']):
                continue

            # 检查模型的文件是否存在
            has_all_files = True
            file_mapping = {}

            models_in_bundle = model_info.get('models', [])
            for m in models_in_bundle:
                artifact = m['artifact']
                original_fname = artifact.get('file_name', os.path.basename(artifact['download_uri']['url']))

                if original_fname not in existing_files:
                    has_all_files = False
                    break

                m_type = m['type']
                file_mapping[m_type] = original_fname

                # 检查 Metadata 文件
                if 'metadata' in m:
                    meta = m['metadata']
                    meta_fname = meta.get('file_name', os.path.basename(meta['download_uri']['url']))
                    if meta_fname not in existing_files:
                        has_all_files = False
                        break
                    file_mapping[f"{m_type}_metadata"] = meta_fname

            # 如果所有文件都存在，添加到 info.json
            if has_all_files:
                global_info['downloaded_models'].append({
                    "name": model_info.get('display_name'),
                    "short_name": model_id,
                    "files": file_mapping,
                    "downloaded_at": datetime.now().isoformat()
                })
                added_count += 1
                logger.info(f"✅ 发现并添加现有模型: {model_info.get('display_name')}")

        # 更新 last_updated
        global_info['last_updated'] = datetime.now().isoformat()

        # 保存更新后的全局 info.json
        try:
            with open(GLOBAL_INFO_FILE, 'w') as f:
                json.dump(global_info, f, indent=2)
            if added_count > 0:
                logger.info(f"✅ 共添加 {added_count} 个现有模型到 info.json")
            else:
                logger.info("✅ 没有发现新的现有模型")
        except Exception as e:
            logger.error(f"❌ 无法保存 info.json: {e}")

    def download_file(self, url, local_path):
        try:
            with urllib.request.urlopen(url, timeout=30) as response:
                total = int(response.headers.get('Content-Length', 0))
                with open(local_path, 'wb') as f:
                    dl = 0
                    while True:
                        chunk = response.read(8192)
                        if not chunk:
                            break
                        dl += len(chunk)
                        f.write(chunk)
                        if total:
                            progress = int(dl/total*100)
                            sys.stdout.write(f"\r   ⬇️  {progress}% - {os.path.basename(local_path)}")
                            sys.stdout.flush()
            print("")
            return True
        except Exception as e:
            logger.error(f"   ❌ 下载失败: {e}")
            if os.path.exists(local_path): os.remove(local_path)
            return False

    def download_model_to_library(self, model_info):
        """
        核心逻辑：根据 JSON 下载文件，直接保存到模型根目录，更新全局 info.json
        """
        # 建立映射关系： DP需要的类型 -> 实际下载的文件名
        file_mapping = {}
        model_name = model_info.get('short_name', 'unknown')

        models = model_info.get('models', [])
        # 存储下载的模型类型，用于后续下载对应的onnx文件
        downloaded_types = set()

        # 1. 首先下载所有模型文件（pkl等）
        for m in models:
            m_type = m['type'] # policy, vision, supercombo
            artifact = m['artifact']
            url = artifact['download_uri']['url']
            original_fname = artifact.get('file_name', os.path.basename(url))

            # 下载主文件，直接按原名称保存到模型根目录
            if self.download_file(url, os.path.join(LIBRARY_DIR, original_fname)):
                file_mapping[m_type] = original_fname
                downloaded_types.add(m_type)

            # 下载 Metadata (如果有)
            if 'metadata' in m:
                meta = m['metadata']
                meta_url = meta['download_uri']['url']
                meta_fname = meta.get('file_name', os.path.basename(meta_url))
                # 直接按原名称保存到模型根目录
                if self.download_file(meta_url, os.path.join(LIBRARY_DIR, meta_fname)):
                    file_mapping[f"{m_type}_metadata"] = meta_fname

            # 处理 onnx 文件下载
            # 1. 首先检查当前模型的 artifact 是否是 onnx 文件
            if original_fname.endswith('.onnx'):
                # 如果主文件就是 onnx，按规则重命名
                if m_type == 'policy':
                    renamed_fname = f"driving_policy_{model_name}.onnx"
                elif m_type == 'vision':
                    renamed_fname = f"driving_vision_{model_name}.onnx"
                elif m_type == 'supercombo':
                    renamed_fname = f"supercombo_{model_name}.onnx"
                else:
                    renamed_fname = f"{m_type}_{model_name}.onnx"

                # 如果已经下载了，重命名
                src = os.path.join(LIBRARY_DIR, original_fname)
                dst = os.path.join(LIBRARY_DIR, renamed_fname)
                if os.path.exists(src):
                    os.rename(src, dst)
                    file_mapping[m_type] = renamed_fname
                    logger.info(f"   🔄 重命名: {original_fname} -> {renamed_fname}")
            # 2. 检查是否有 additional_files 中的 onnx 文件
            elif 'additional_files' in m:
                for add_file in m['additional_files']:
                    if add_file.get('file_name', '').endswith('.onnx') or add_file.get('download_uri', {}).get('url', '').endswith('.onnx'):
                        onnx_url = add_file['download_uri']['url']
                        onnx_fname = add_file.get('file_name', os.path.basename(onnx_url))

                        # 根据 m_type 重命名 onnx 文件
                        if m_type == 'policy':
                            renamed_fname = f"driving_policy_{model_name}.onnx"
                        elif m_type == 'vision':
                            renamed_fname = f"driving_vision_{model_name}.onnx"
                        elif m_type == 'supercombo':
                            renamed_fname = f"supercombo_{model_name}.onnx"
                        else:
                            renamed_fname = f"{m_type}_{model_name}.onnx"

                        # 下载并重命名
                        if self.download_file(onnx_url, os.path.join(LIBRARY_DIR, renamed_fname)):
                            file_mapping[f"{m_type}_onnx"] = renamed_fname
                            logger.info(f"   📄 下载: {onnx_fname} -> {renamed_fname}")

        # 3. 自动下载对应的onnx文件（根据pkl文件的URL构建onnx文件的URL）
        logger.info("   📥 尝试自动下载对应的ONNX模型...")
        for m in models:
            m_type = m['type']
            if m_type in ['policy', 'vision']:
                artifact = m['artifact']
                pkl_url = artifact['download_uri']['url']

                # 构建onnx文件的URL
                # 例如：pkl_url = https://gitlab.com/.../driving_policy_wmiv9_tinygrad.pkl
                # onnx_url = https://gitlab.com/.../driving_policy.onnx
                onnx_url = pkl_url.rsplit('/', 1)[0] + f"/driving_{m_type}.onnx"

                # 构建重命名后的onnx文件名
                renamed_onnx_fname = f"driving_{m_type}_{model_name}.onnx"
                onnx_path = os.path.join(LIBRARY_DIR, renamed_onnx_fname)

                # 下载onnx文件
                if self.download_file(onnx_url, onnx_path):
                    # 添加到file_mapping
                    if f"{m_type}_onnx" not in file_mapping:
                        file_mapping[f"{m_type}_onnx"] = renamed_onnx_fname
                    logger.info(f"   📄 自动下载: driving_{m_type}.onnx -> {renamed_onnx_fname}")

        # 读取当前全局info.json
        global_info = {
            "downloaded_models": [],
            "current_model": None,
            "current_files": {},
            "last_updated": datetime.now().isoformat()
        }

        if os.path.exists(GLOBAL_INFO_FILE):
            try:
                with open(GLOBAL_INFO_FILE, 'r') as f:
                    existing_info = json.load(f)
                    global_info.update(existing_info)
            except Exception as e:
                logger.warning(f"⚠️  无法读取现有 info.json: {e}")

        # 更新已下载模型列表
        model_id = model_info.get('short_name', 'unknown')
        # 检查模型是否已在列表中
        model_exists = False
        for idx, model in enumerate(global_info['downloaded_models']):
            if model.get('short_name') == model_id:
                # 更新现有模型
                global_info['downloaded_models'][idx] = {
                    "name": model_info.get('display_name'),
                    "short_name": model_id,
                    "files": file_mapping,
                    "downloaded_at": datetime.now().isoformat()
                }
                model_exists = True
                break

        if not model_exists:
            # 添加新模型
            global_info['downloaded_models'].append({
                "name": model_info.get('display_name'),
                "short_name": model_id,
                "files": file_mapping,
                "downloaded_at": datetime.now().isoformat()
            })

        # 不自动设置当前模型，只更新 last_updated
        global_info['last_updated'] = datetime.now().isoformat()

        # 保存更新后的全局 info.json
        with open(GLOBAL_INFO_FILE, 'w') as f:
            json.dump(global_info, f, indent=2)

        logger.info(f"✅ 下载完成: {model_info.get('display_name')}")
        return model_info.get('display_name')

    def install_model(self, model_id=None):
        """
        核心安装逻辑：读取全局info.json -> 复制 -> 重命名为 DP 标准名
        """
        # 1. 读取全局info.json
        if not os.path.exists(GLOBAL_INFO_FILE):
            logger.error("❌ 全局 info.json 不存在")
            return

        try:
            with open(GLOBAL_INFO_FILE, 'r') as f:
                global_info = json.load(f)
        except Exception as e:
            logger.error(f"❌ 无法读取全局 info.json: {e}")
            return

        # 2. 获取要安装的模型信息
        target_model = None
        if model_id:
            # 从已下载模型列表中查找指定模型
            for model in global_info.get('downloaded_models', []):
                if model.get('short_name') == model_id:
                    target_model = model
                    break

            if not target_model:
                logger.error(f"❌ 未找到 ID 为 {model_id} 的已下载模型")
                return
        else:
            # 使用当前模型
            current_model_id = global_info.get('current_model')
            if current_model_id:
                for model in global_info.get('downloaded_models', []):
                    if model.get('short_name') == current_model_id:
                        target_model = model
                        break

                if not target_model:
                    logger.error(f"❌ 未找到当前模型 {current_model_id}")
                    return
            else:
                logger.error("❌ 没有设置当前模型")
                return

        model_name = target_model.get('name', '未知模型')
        logger.info(f"🚀 正在安装模型: {model_name}")

        # 3. 获取文件映射
        file_mapping = target_model.get('files', {})
        if not file_mapping:
            logger.error(f"❌ 模型 {model_name} 没有文件映射")
            return

        # 4. 定义 DP/OP 的标准文件名映射规则
        # 左边是 JSON 中的 type，右边是 DP 运行时的文件名
        # 注意：modeld.py 期望的文件名带有 _tinygrad 后缀
        target_names = {
            "policy": "driving_policy_tinygrad.pkl",
            "vision": "driving_vision_tinygrad.pkl",
            "policy_metadata": "driving_policy_metadata.pkl",
            "vision_metadata": "driving_vision_metadata.pkl",
            # 旧版兼容
            "supercombo": "supercombo.thneed",
            "supercombo_metadata": "supercombo_metadata.pkl"
        }

        # 5. 只有当默认模型文件不存在时，才备份原模型到 data 目录
        data_dir = "/data/media/0/models"
        # 检查默认模型文件是否已经存在
        default_files_exist = False
        for default_file in [
            "driving_policy_default.pkl",
            "driving_vision_default.pkl",
            "driving_policy_metadata_default.pkl",
            "driving_vision_metadata_default.pkl",
            "driving_policy_default.onnx",
            "driving_vision_default.onnx"
        ]:
            if os.path.exists(os.path.join(data_dir, default_file)):
                default_files_exist = True
                break

        # 只有当没有current_model且默认模型文件不存在时，才执行备份
        if not global_info.get('current_model') and not default_files_exist:
            logger.info("📦 第一次更换模型，备份原模型到 data 目录")
            if not os.path.exists(data_dir):
                os.makedirs(data_dir, exist_ok=True)

            # 备份模型文件 - 包括主模型、metadata 和 onnx 文件
            for f in os.listdir(INSTALL_DIR):
                if f == "driving_policy_tinygrad.pkl":
                    src = os.path.join(INSTALL_DIR, f)
                    dst = os.path.join(data_dir, "driving_policy_default.pkl")
                    if os.path.exists(src):
                        # 使用copy2，自动处理权限
                        shutil.copy2(src, dst)
                        logger.info(f"   📄 备份: {f} -> {os.path.basename(dst)}")
                elif f == "driving_vision_tinygrad.pkl":
                    src = os.path.join(INSTALL_DIR, f)
                    dst = os.path.join(data_dir, "driving_vision_default.pkl")
                    if os.path.exists(src):
                        shutil.copy2(src, dst)
                        logger.info(f"   📄 备份: {f} -> {os.path.basename(dst)}")
                # 备份 metadata 文件
                elif f == "driving_policy_metadata.pkl":
                    src = os.path.join(INSTALL_DIR, f)
                    dst = os.path.join(data_dir, "driving_policy_metadata_default.pkl")
                    if os.path.exists(src):
                        shutil.copy2(src, dst)
                        logger.info(f"   📄 备份: {f} -> {os.path.basename(dst)}")
                elif f == "driving_vision_metadata.pkl":
                    src = os.path.join(INSTALL_DIR, f)
                    dst = os.path.join(data_dir, "driving_vision_metadata_default.pkl")
                    if os.path.exists(src):
                        shutil.copy2(src, dst)
                        logger.info(f"   📄 备份: {f} -> {os.path.basename(dst)}")
                # 备份 onnx 文件 - 只备份标准 onnx 文件
                elif f == "driving_policy.onnx":
                    src = os.path.join(INSTALL_DIR, f)
                    dst = os.path.join(data_dir, "driving_policy_default.onnx")
                    if os.path.exists(src):
                        # 强制替换，先删除目标文件
                        if os.path.exists(dst):
                            os.remove(dst)
                        shutil.copy2(src, dst)
                        logger.info(f"   📄 备份: {f} -> {os.path.basename(dst)}")
                elif f == "driving_vision.onnx":
                    src = os.path.join(INSTALL_DIR, f)
                    dst = os.path.join(data_dir, "driving_vision_default.onnx")
                    if os.path.exists(src):
                        # 强制替换，先删除目标文件
                        if os.path.exists(dst):
                            os.remove(dst)
                        shutil.copy2(src, dst)
                        logger.info(f"   📄 备份: {f} -> {os.path.basename(dst)}")

        # 6. 清理环境 (Reset git 防止校验失败)
        subprocess.run(f'git reset HEAD "{INSTALL_DIR}/*"', shell=True, stderr=subprocess.DEVNULL)
        # 删除旧文件，包括 pkl、thneed 和 onnx 文件
        for f in os.listdir(INSTALL_DIR):
            if f.startswith(("driving_", "supercombo")) and f.endswith((".pkl", ".thneed", ".onnx")):
                try: os.remove(os.path.join(INSTALL_DIR, f))
                except: pass

        # 7. 复制并重命名
        count = 0
        for m_type, original_fname in file_mapping.items():
            # 处理不同类型的文件
            target = target_names.get(m_type)

            # 特殊处理：supercombo 模型
            if m_type == "supercombo":
                if original_fname.endswith(".onnx"):
                    target = "supercombo.onnx"
                elif original_fname.endswith(".thneed"):
                    target = "supercombo.thneed"
            # 处理 policy 模型
            elif m_type == "policy":
                if original_fname.endswith(".onnx"):
                    target = "driving_policy.onnx"
                elif original_fname.endswith(".pkl"):
                    target = "driving_policy_tinygrad.pkl"
            # 处理 vision 模型
            elif m_type == "vision":
                if original_fname.endswith(".onnx"):
                    target = "driving_vision.onnx"
                elif original_fname.endswith(".pkl"):
                    target = "driving_vision_tinygrad.pkl"
            # 处理 metadata 文件
            elif m_type.endswith("_metadata"):
                # metadata 文件保持原来的命名逻辑
                pass
            # 处理单独的 onnx 文件映射
            elif m_type.endswith("_onnx"):
                base_type = m_type[:-5]  # 去掉 _onnx 后缀
                if base_type == "policy":
                    target = "driving_policy.onnx"
                elif base_type == "vision":
                    target = "driving_vision.onnx"
                elif base_type == "supercombo":
                    target = "supercombo.onnx"
            # 处理主文件是 onnx 的情况（兜底逻辑）
            elif original_fname.endswith(".onnx"):
                # 从文件名中提取类型信息
                if "policy" in original_fname.lower():
                    target = "driving_policy.onnx"
                elif "vision" in original_fname.lower():
                    target = "driving_vision.onnx"
                elif "supercombo" in original_fname.lower():
                    target = "supercombo.onnx"

            if not target: continue # 未知的类型跳过

            src = os.path.join(LIBRARY_DIR, original_fname)
            dst = os.path.join(INSTALL_DIR, target)

            if os.path.exists(src):
                shutil.copy2(src, dst)
                # 欺骗 git
                subprocess.run(['git', 'add', '-f', dst], stderr=subprocess.DEVNULL)
                logger.info(f"   📄 {original_fname} -> {target}")
                count += 1
            else:
                logger.warning(f"   ⚠️ 缺失源文件: {original_fname}")

        # 8. 额外检查：如果模型包含 onnx 文件但未被复制，尝试手动复制
        if count > 0:
            # 检查模型目录中是否有对应的 onnx 文件
            model_name = target_model.get('short_name', 'unknown')
            for onnx_type in ['policy', 'vision']:
                # 检查是否存在命名为 driving_{onnx_type}_{model_name}.onnx 的文件
                expected_onnx = f"driving_{onnx_type}_{model_name}.onnx"
                expected_onnx_path = os.path.join(LIBRARY_DIR, expected_onnx)
                target_onnx = f"driving_{onnx_type}.onnx"
                target_onnx_path = os.path.join(INSTALL_DIR, target_onnx)

                if os.path.exists(expected_onnx_path) and not os.path.exists(target_onnx_path):
                    shutil.copy2(expected_onnx_path, target_onnx_path)
                    subprocess.run(['git', 'add', '-f', target_onnx_path], stderr=subprocess.DEVNULL)
                    logger.info(f"   📄 {expected_onnx} -> {target_onnx}")
                    count += 1

        # 7. 更新当前模型信息
        global_info['current_model'] = target_model.get('short_name')
        global_info['current_files'] = file_mapping
        global_info['last_updated'] = datetime.now().isoformat()
        with open(GLOBAL_INFO_FILE, 'w') as f:
            json.dump(global_info, f, indent=2)

        if count > 0:
            logger.info("✅ 安装成功！请重启 Dragonpilot (tmux kill-server)。")
        else:
            logger.error("❌ 安装失败，未复制任何文件。")

    def delete_model(self, index, global_info):
        """
        删除指定索引的模型
        """
        downloaded_models = global_info.get('downloaded_models', [])
        if index < 0 or index >= len(downloaded_models):
            logger.error(f"❌ 无效的模型索引: {index}")
            return

        model = downloaded_models[index]
        model_id = model.get('short_name')
        model_name = model.get('name')
        file_mapping = model.get('files', {})

        # 删除文件系统中的模型文件，但保留默认模型文件
        deleted_files = 0
        # 定义需要保留的默认模型文件名
        default_files = {
            "driving_policy_default.pkl",
            "driving_vision_default.pkl",
            "driving_policy_metadata_default.pkl",
            "driving_vision_metadata_default.pkl",
            "driving_policy_default.onnx",
            "driving_vision_default.onnx"
        }

        for filename in file_mapping.values():
            # 跳过默认模型文件
            if filename in default_files:
                continue

            file_path = os.path.join(LIBRARY_DIR, filename)
            if os.path.exists(file_path):
                try:
                    os.remove(file_path)
                    deleted_files += 1
                    logger.info(f"   🗑️ 删除文件: {filename}")
                except Exception as e:
                    logger.warning(f"   ⚠️ 无法删除文件 {filename}: {e}")

        # 从 info.json 中移除模型
        downloaded_models.pop(index)

        # 更新 current_model 和 current_files
        if global_info.get('current_model') == model_id:
            global_info['current_model'] = None
            global_info['current_files'] = {}
            logger.info("   🔄 当前模型已重置，因为删除的是当前模型")

        # 更新 last_updated
        global_info['last_updated'] = datetime.now().isoformat()

        # 保存更新后的 info.json
        try:
            with open(GLOBAL_INFO_FILE, 'w') as f:
                json.dump(global_info, f, indent=2)
            logger.info(f"✅ 成功删除模型: {model_name} ({model_id})")
            logger.info(f"   删除了 {deleted_files} 个文件")
        except Exception as e:
            logger.error(f"❌ 无法保存 info.json: {e}")

    def delete_all_models(self, global_info):
        """
        删除所有模型，但保留默认模型文件
        """
        downloaded_models = global_info.get('downloaded_models', [])
        total_files = 0
        deleted_files = 0

        # 定义需要保留的默认模型文件名
        default_files = {
            "driving_policy_default.pkl",
            "driving_vision_default.pkl",
            "driving_policy_metadata_default.pkl",
            "driving_vision_metadata_default.pkl",
            "driving_policy_default.onnx",
            "driving_vision_default.onnx"
        }

        # 删除所有模型文件，但保留默认模型文件
        for model in downloaded_models:
            file_mapping = model.get('files', {})
            total_files += len(file_mapping)
            for filename in file_mapping.values():
                # 跳过默认模型文件
                if filename in default_files:
                    continue

                file_path = os.path.join(LIBRARY_DIR, filename)
                if os.path.exists(file_path):
                    try:
                        os.remove(file_path)
                        deleted_files += 1
                        logger.info(f"   🗑️ 删除文件: {filename}")
                    except Exception as e:
                        logger.warning(f"   ⚠️ 无法删除文件 {filename}: {e}")

        # 清空 info.json 中的模型列表
        global_info['downloaded_models'] = []
        global_info['current_model'] = None
        global_info['current_files'] = {}
        global_info['last_updated'] = datetime.now().isoformat()

        # 保存更新后的 info.json
        try:
            with open(GLOBAL_INFO_FILE, 'w') as f:
                json.dump(global_info, f, indent=2)
            logger.info(f"✅ 成功删除所有 {len(downloaded_models)} 个模型")
            logger.info(f"   总共 {total_files} 个文件，成功删除 {deleted_files} 个")
            logger.info(f"   已保留默认模型文件")
        except Exception as e:
            logger.error(f"❌ 无法保存 info.json: {e}")

    def main(self):
        while True:
            print(f"\n=== Dragonpilot 模型选择器 ===")
            print("1. [下载] 获取并安装 SP 最新模型")
            print("2. [管理] 查看并切换已下载的模型")
            print("3. [删除] 删除已下载的模型")
            print("q. 退出")

            sel = input("👉 选择: ").strip().lower()
            if sel == 'q': break

            if sel == '1':
                models = self.fetch_model_list()
                if not models: continue

                # 扫描并添加现有模型到 info.json
                self.add_existing_models_to_info(models)

                # 首先列出倒序10个最新模型
                print("\n📋 最新10个模型:")
                latest_models = models[::-1][:10]  # 倒序取前10个
                for i, m in enumerate(latest_models):
                    print(f" {i}. {m.get('display_name')}")
                print(f"... 还有 {len(models) - 10} 个模型")

                kw = input("🔍 搜索模型 (例如 wmi, dark) 或输入序号直接下载: ").lower()

                # 检查是否直接输入序号下载最新模型
                if kw.isdigit():
                    idx = int(kw)
                    if 0 <= idx < len(latest_models):
                        selected_model = latest_models[idx]
                        model_id = selected_model.get('short_name')

                        # 检查模型是否已存在
                        model_exists = False
                        if os.path.exists(GLOBAL_INFO_FILE):
                            try:
                                with open(GLOBAL_INFO_FILE, 'r') as f:
                                    global_info = json.load(f)
                                    for model in global_info.get('downloaded_models', []):
                                        if model.get('short_name') == model_id:
                                            model_exists = True
                                            break
                            except Exception as e:
                                logger.warning(f"⚠️  无法读取模型信息: {e}")

                        if model_exists:
                            logger.info(f"⚠️  模型 {selected_model.get('display_name')} 已存在")
                            # 输出当前应用的模型
                            current_model = "default"
                            try:
                                with open(GLOBAL_INFO_FILE, 'r') as f:
                                    global_info = json.load(f)
                                    if global_info.get('current_model'):
                                        current_model = global_info['current_model']
                            except Exception as e:
                                logger.warning(f"⚠️  无法读取当前模型信息: {e}")
                            logger.info(f"📌 当前应用模型: {current_model}")
                            if input("⚡️ 立即安装激活已存在的模型? (y/n): ").lower() == 'y':
                                self.install_model(model_id)
                            continue

                        # 下载模型
                        model_name = self.download_model_to_library(selected_model)
                        # 输出当前应用的模型
                        current_model = "default"
                        if os.path.exists(GLOBAL_INFO_FILE):
                            try:
                                with open(GLOBAL_INFO_FILE, 'r') as f:
                                    global_info = json.load(f)
                                    if global_info.get('current_model'):
                                        current_model = global_info['current_model']
                            except Exception as e:
                                logger.warning(f"⚠️  无法读取当前模型信息: {e}")
                        logger.info(f"📌 当前应用模型: {current_model}")
                        if model_name and input("⚡️ 立即安装激活? (y/n): ").lower() == 'y':
                            self.install_model(model_id)
                        continue

                # 搜索模型
                matches = [m for m in models if kw in m.get('display_name', '').lower() or kw in m.get('short_name', '').lower()]

                print(f"找到 {len(matches)} 个模型:")
                for i, m in enumerate(matches):
                    print(f" {i}. {m.get('display_name')}")

                idx = input("👉 输入序号下载: ")
                if idx.isdigit() and int(idx) < len(matches):
                    selected_model = matches[int(idx)]
                    model_id = selected_model.get('short_name')

                    # 检查模型是否已存在
                    model_exists = False
                    if os.path.exists(GLOBAL_INFO_FILE):
                        try:
                            with open(GLOBAL_INFO_FILE, 'r') as f:
                                global_info = json.load(f)
                                for model in global_info.get('downloaded_models', []):
                                    if model.get('short_name') == model_id:
                                        model_exists = True
                                        break
                        except Exception as e:
                            logger.warning(f"⚠️  无法读取模型信息: {e}")

                    if model_exists:
                        logger.info(f"⚠️  模型 {selected_model.get('display_name')} 已存在")
                        # 输出当前应用的模型
                        current_model = "default"
                        try:
                            with open(GLOBAL_INFO_FILE, 'r') as f:
                                global_info = json.load(f)
                                if global_info.get('current_model'):
                                    current_model = global_info['current_model']
                        except Exception as e:
                            logger.warning(f"⚠️  无法读取当前模型信息: {e}")
                        logger.info(f"📌 当前应用模型: {current_model}")
                        if input("⚡️ 立即安装激活已存在的模型? (y/n): ").lower() == 'y':
                            self.install_model(model_id)
                        continue

                    # 下载模型
                    model_name = self.download_model_to_library(selected_model)
                    # 输出当前应用的模型
                    current_model = "default"
                    if os.path.exists(GLOBAL_INFO_FILE):
                        try:
                            with open(GLOBAL_INFO_FILE, 'r') as f:
                                global_info = json.load(f)
                                if global_info.get('current_model'):
                                    current_model = global_info['current_model']
                        except Exception as e:
                            logger.warning(f"⚠️  无法读取当前模型信息: {e}")
                    logger.info(f"📌 当前应用模型: {current_model}")
                    if model_name and input("⚡️ 立即安装激活? (y/n): ").lower() == 'y':
                        self.install_model(model_id)

            elif sel == '2':
                # 查看并切换已下载的模型
                if not os.path.exists(GLOBAL_INFO_FILE):
                    logger.error("❌ 全局 info.json 不存在")
                    continue

                try:
                    with open(GLOBAL_INFO_FILE, 'r') as f:
                        global_info = json.load(f)
                except Exception as e:
                    logger.error(f"❌ 无法读取全局 info.json: {e}")
                    continue

                downloaded_models = global_info.get('downloaded_models', [])

                print("\n📋 已下载的模型:")
                current_model_id = global_info.get('current_model')

                for i, model in enumerate(downloaded_models):
                    model_id = model.get('short_name')
                    model_name = model.get('name')
                    is_current = " * " if model_id == current_model_id else "   "
                    print(f"{is_current}{i}. {model_name} ({model_id})")

                # 添加恢复默认模型选项
                print("  r. 恢复默认模型")

                idx = input("👉 输入序号切换模型或输入'r'恢复默认模型: ").strip().lower()
                if idx == 'r':
                    # 恢复默认模型
                    logger.info("🚀 正在恢复默认模型")
                    data_dir = "/data/media/0/models"

                    # 清理环境
                    subprocess.run(f'git reset HEAD "{INSTALL_DIR}/*"', shell=True, stderr=subprocess.DEVNULL)
                    for f in os.listdir(INSTALL_DIR):
                        if f.startswith(("driving_", "supercombo")) and f.endswith((".pkl", ".thneed", ".onnx")):
                            try: os.remove(os.path.join(INSTALL_DIR, f))
                            except: pass

                    # 复制默认模型文件
                    count = 0
                    # 复制模型文件 - 包括主模型、metadata 和 onnx 文件
                    for src_fname, dst_fname in [
                        ("driving_policy_default.pkl", "driving_policy_tinygrad.pkl"),
                        ("driving_vision_default.pkl", "driving_vision_tinygrad.pkl"),
                        ("driving_policy_metadata_default.pkl", "driving_policy_metadata.pkl"),
                        ("driving_vision_metadata_default.pkl", "driving_vision_metadata.pkl"),
                        ("driving_policy_default.onnx", "driving_policy.onnx"),
                        ("driving_vision_default.onnx", "driving_vision.onnx")
                    ]:
                        src = os.path.join(data_dir, src_fname)
                        dst = os.path.join(INSTALL_DIR, dst_fname)
                        if os.path.exists(src):
                            shutil.copy2(src, dst)
                            subprocess.run(['git', 'add', '-f', dst], stderr=subprocess.DEVNULL)
                            logger.info(f"   📄 {src_fname} -> {dst_fname}")
                            count += 1
                        else:
                            logger.warning(f"   ⚠️ 缺失默认文件: {src_fname}")

                    # 更新全局 info.json
                    global_info['current_model'] = None
                    global_info['current_files'] = {}
                    global_info['last_updated'] = datetime.now().isoformat()

                    try:
                        with open(GLOBAL_INFO_FILE, 'w') as f:
                            json.dump(global_info, f, indent=2)
                    except Exception as e:
                        logger.error(f"❌ 无法保存 info.json: {e}")

                    if count > 0:
                        logger.info("✅ 默认模型恢复成功！请重启 Dragonpilot (tmux kill-server)。")
                    else:
                        logger.error("❌ 默认模型恢复失败，未找到默认模型文件。")
                elif idx.isdigit():
                    idx = int(idx)
                    if 0 <= idx < len(downloaded_models):
                        selected_model = downloaded_models[idx]
                        model_id = selected_model.get('short_name')
                        self.install_model(model_id)
                    else:
                        logger.error(f"❌ 序号超出范围 (0-{len(downloaded_models)-1})")
                elif idx == 'q':
                    continue
                else:
                    logger.error("❌ 无效输入")

            elif sel == '3':
                # 删除已下载的模型
                if not os.path.exists(GLOBAL_INFO_FILE):
                    logger.error("❌ 全局 info.json 不存在")
                    continue

                try:
                    with open(GLOBAL_INFO_FILE, 'r') as f:
                        global_info = json.load(f)
                except Exception as e:
                    logger.error(f"❌ 无法读取全局 info.json: {e}")
                    continue

                downloaded_models = global_info.get('downloaded_models', [])
                if not downloaded_models:
                    logger.error("❌ 还没有下载任何模型")
                    continue

                print("\n📋 已下载的模型:")
                current_model_id = global_info.get('current_model')

                for i, model in enumerate(downloaded_models):
                    model_id = model.get('short_name')
                    model_name = model.get('name')
                    is_current = " * " if model_id == current_model_id else "   "
                    print(f"{is_current}{i}. {model_name} ({model_id})")

                print("\n🔧 删除选项:")
                print("a. 删除所有模型")
                print("r. 返回")

                delete_sel = input("👉 输入序号删除指定模型或选择 'a' 删除所有: ").strip().lower()

                if delete_sel == 'r':
                    continue
                elif delete_sel == 'a':
                    # 删除所有模型
                    if input("⚠️  确定要删除所有模型吗？此操作不可恢复！(y/n): ").lower() == 'y':
                        self.delete_all_models(global_info)
                elif delete_sel.isdigit():
                    # 删除指定模型
                    idx = int(delete_sel)
                    if 0 <= idx < len(downloaded_models):
                        selected_model = downloaded_models[idx]
                        if input(f"⚠️  确定要删除模型 '{selected_model.get('name')}' 吗？此操作不可恢复！(y/n): ").lower() == 'y':
                            self.delete_model(idx, global_info)
                    else:
                        logger.error(f"❌ 序号超出范围 (0-{len(downloaded_models)-1})")
                else:
                    logger.error("❌ 无效输入")

if __name__ == "__main__":
    try: ModelSelector().main()
    except KeyboardInterrupt: pass
