# -*- coding: utf-8 -*-
"""
Dragonpilot 模型选择器 (适配 Sunnypilot JSON v10)
针对 Comma 3/3X 设备优化

功能:
1. 解析 SP 格式 JSON (支持 supercombo 和 policy/vision 分离架构)。
2. 下载并以【原文件名】存入 /data/media/0/models/。
3. 安装时将文件【重命名】为 DP 标准名称 (driving_policy.pkl 等)。
"""
import os
import sys
import json
import requests
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
        try:
            if not os.path.exists(LIBRARY_DIR):
                os.makedirs(LIBRARY_DIR, exist_ok=True)
        except PermissionError:
            # 如果没有权限创建 /data/media/0/models，使用本地目录
            LIBRARY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
            logger.warning(f"⚠️  没有权限创建 /data/media/0/models，将使用本地目录: {LIBRARY_DIR}")
            os.makedirs(LIBRARY_DIR, exist_ok=True)

        # 更新全局路径
        GLOBAL_INFO_FILE = os.path.join(LIBRARY_DIR, "info.json")
        LOCAL_MODEL_LIST_FILE = os.path.join(LIBRARY_DIR, "driving_models_v10.json")

    def get_with_retry(self, url):
        try:
            r = requests.get(url, timeout=15)
            r.raise_for_status()
            return r
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
            with requests.get(url, stream=True, timeout=30) as r:
                r.raise_for_status()
                total = int(r.headers.get('content-length', 0))
                with open(local_path, 'wb') as f:
                    dl = 0
                    for chunk in r.iter_content(chunk_size=8192):
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

        models = model_info.get('models', [])
        for m in models:
            m_type = m['type'] # policy, vision, supercombo
            artifact = m['artifact']
            url = artifact['download_uri']['url']
            original_fname = artifact.get('file_name', os.path.basename(url))

            # 下载主文件，直接按原名称保存到模型根目录
            if self.download_file(url, os.path.join(LIBRARY_DIR, original_fname)):
                file_mapping[m_type] = original_fname

            # 下载 Metadata (如果有)
            if 'metadata' in m:
                meta = m['metadata']
                meta_url = meta['download_uri']['url']
                meta_fname = meta.get('file_name', os.path.basename(meta_url))
                # 直接按原名称保存到模型根目录
                if self.download_file(meta_url, os.path.join(LIBRARY_DIR, meta_fname)):
                    file_mapping[f"{m_type}_metadata"] = meta_fname

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

        # 5. 清理环境 (Reset git 防止校验失败)
        subprocess.run(f'git reset HEAD "{INSTALL_DIR}/*"', shell=True, stderr=subprocess.DEVNULL)
        # 删除旧文件，但保留 ONNX 文件
        for f in os.listdir(INSTALL_DIR):
            if f.startswith(("driving_", "supercombo")) and f.endswith((".pkl", ".thneed")):
                try: os.remove(os.path.join(INSTALL_DIR, f))
                except: pass

        # 6. 复制并重命名
        count = 0
        for m_type, original_fname in file_mapping.items():
            # 特殊处理：如果 JSON 说它是 supercombo，但后缀是 .onnx，我们要改成 supercombo.onnx
            target = target_names.get(m_type)
            if m_type == "supercombo" and original_fname.endswith(".onnx"):
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

        # 删除文件系统中的模型文件
        deleted_files = 0
        for filename in file_mapping.values():
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
        删除所有模型
        """
        downloaded_models = global_info.get('downloaded_models', [])
        total_files = 0
        deleted_files = 0

        # 删除所有模型文件
        for model in downloaded_models:
            file_mapping = model.get('files', {})
            total_files += len(file_mapping)
            for filename in file_mapping.values():
                file_path = os.path.join(LIBRARY_DIR, filename)
                if os.path.exists(file_path):
                    try:
                        os.remove(file_path)
                        deleted_files += 1
                        logger.info(f"   🗑️ 删除文件: {filename}")
                    except Exception as e:
                        logger.warning(f"   ⚠️ 无法删除文件 {filename}: {e}")

        # 清空 info.json
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

                idx = input("👉 输入序号切换模型: ").strip()
                if idx.isdigit():
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
