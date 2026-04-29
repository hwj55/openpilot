import numpy as np
from cereal import log
# 🌟 直接匯入 MPC 原廠核心公式、變數與前車動能緩衝函數，確保距離計算 100% 同步
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE, get_safe_obstacle_distance, get_stopped_equivalence_factor, get_T_FOLLOW

# ==========================================
# ⚙️ 全域變數定義區 (Global Configurations)
# ==========================================

# --- 1. 距離與狀態機閾值 (百分比) ---
EXIT_PERCENT = 1.00         # ⚪ 退出點：當實體距離大於目標距離 100% 時，空間極度充裕，徹底休眠不干涉
COAST_START_PERCENT = 0.98  # 🟢 進入點：距離小於 98% 時，ACM 開始作動，進入純滑行狀態
COAST_END_PERCENT = 0.80    # 🟡 警戒線：距離小於 80% 時，結束純滑行，開始套用 TTA 線性微煞車
SAFE_DIST_PERCENT = 0.70    # 🔴 交接線：跌破 70% 代表微煞車失敗或前車急煞，立刻交還控制權給 MPC

# --- 2. 加速度動作極限變數 (m/s²) ---
COAST_MAX_BRAKE = -0.4      # 🌊 滑行極限：無車狀態下，強制抹平原廠 0.0 到 -0.4 之間的神經質微煞車
MIN_RECOVERY_ACCEL = -1.0   # 🛡️ 煞車極限：TTA 算出的微煞車力道，最重不允許超過 -1.0，確保無點頭頓挫
MAX_RECOVERY_ACCEL = 0.0    # 🐢 加速極限：微煞車區間內，最高加速度鎖死在 0.0，絕對不允許系統補油門
MPC_FALLBACK_ACCEL = -1.2   # 💣 緊急交接線：掃描未來軌跡時，若發現原廠將給出小於 -1.2 的重煞，立刻退出保命

# --- 3. 意圖預測與訊號濾波參數 ---
TRAJECTORY_HORIZON = 6      # 🔭 未來預判：掃描原廠 MPC 未來 6 個軌跡點 (約涵蓋未來 0.6 秒的預測)
INTENT_LOOKAHEAD = 3        # 🧠 意圖確認：在 6 個軌跡點中，只要有 3 個點具備加速特徵，即觸發意圖
INTENT_V_LOW = 0.0          # 📏 意圖濾波下限：時速 0 m/s 時 (用於動態決定確認幀數)
INTENT_V_HIGH = 20.0        # 📏 意圖濾波上限：時速 20 m/s (72 km/h) 時 (用於動態決定確認幀數)
INTENT_FRAMES_LOW = 1       # ⚡ 低速起步幀數：靜止或低速時，只需 1 幀即可確認前車起步，追求極致敏捷與零延遲
INTENT_FRAMES_HIGH = 20     # 🛡️ 高速巡航幀數：高速時需連續 20 幀才確認加速意圖，防止風吹草動造成誤判放行
FILTER_ALPHA = 0.2          # 📡 雷達平滑係數：20% 新雷達數據 + 80% 歷史數據，有效撫平毫米波雷達的雜訊跳動
LEAD_LOST_TICKS = 5         # 🔒 丟失容忍度：雷達若瞬間沒看到車，容忍 5 幀 (約0.25秒) 內不解除有車狀態，防閃爍

# --- 4. TTA 線性煞車魔法與物理基準 ---
TARGET_V_REL = 0.6          # 🎯 TTA 目標速差：保留微小的相對速度差，讓煞車收尾能平滑貼近前車
TTA_TIME_RATIO = 0.8        # ⏳ 時間壓縮魔法：將時間壓縮為 0.8 倍，放軟初期煞車力道
TTA_MULTIPLIER = 1.2        # 🚀 力道放大魔法：放大基礎數學公式算出的力道，克服變速箱與車重慣性，使煞車更扎實
MIN_BRAKE_ZONE_M = 3.0      # 🧱 低速市區防線：(80%~70% 的物理長度) 若小於 3m，代表目前在低速市區，直接退出

# --- 5. 動態滑行權重參數 (Lerp Blend) ---
BLEND_V_MIN = 6.0           # ⚖️ 動態權重下限：時速 6 m/s 時，ACM 滑行權重為 0% (完全依賴原廠 MPC)
BLEND_V_MAX = 20.0          # ⚖️ 動態權重上限：時速 20 m/s (72 km/h) 時，ACM 滑行權重為 100% (強制輸出 0.0)


class ACM:
  """
  自適應滑行管理模組 (ACM) - 邏輯嚴謹與註解滿血版
  特色：保留所有數學算式的區域變數，並解耦舊版 UI 依賴，確保安全無虞。
  """
  def __init__(self):
    self.enabled = False
    self.personality = log.LongitudinalPersonality.standard 
    self._dtsc_is_active = False          
    self._is_normal_mode = True           

    # ==========================================
    # 邏輯控制旗標與記憶體
    # ==========================================
    self.acm_active = False           # 記錄 ACM 當下是否具備修改軌跡的資格
    self.intent_accelerating = False  # 記錄是否偵測到前車的加速意圖
    self.accel_intent_counter = 0     # 加速意圖連續發生幀數的計數器
    
    # 雷達訊號與 EMA 濾波記憶體
    self.filtered_d_rel = 0.0         # 平滑處理後的前車距離
    self.filtered_v_rel = 0.0         # 平滑處理後的前車相對速度
    self.lead_status_prev = False     # 記錄上一幀的雷達有無抓到車
    self.lead_lost_counter = 0        # 丟失前車的容忍倒數計時器
    self.has_lead_locked = False      # 最終系統認定的「有效前車狀態」
    self.last_valid_d_rel = 0.0       # 備份最後有效的距離 (供雷達閃爍時盲算使用)
    self.last_valid_v_rel = 0.0       # 備份最後有效的速差 (供雷達閃爍時盲算使用)

    # 跨函數暫存變數 (為解耦 update_states 與 update_a_desired_trajectory 介面)
    self._saved_lead = None
    self._saved_v_ego = 0.0

  @property
  def active(self):
    return self.acm_active

  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise, mode='acc', personality=log.LongitudinalPersonality.standard, dtsc_is_active=False):
    # 紀錄系統狀態與環境變數，以供下一個函式取用
    self.personality = personality
    self._dtsc_is_active = dtsc_is_active 
    self._is_normal_mode = (mode == 'acc')
    self._saved_lead = rs.leadOne
    self._saved_v_ego = v_ego

    # 若系統禁用 ACM 或是人為接管，重置 active 狀態
    if not self.enabled or user_ctrl_lon:
      self.acm_active = False

  def update_a_desired_trajectory(self, a_desired_trajectory, v_ego=0.0, lead=None, t_follow=None):
    # 防護機制：系統無效或處於非一般巡航模式時，直接回傳原廠軌跡
    if self._dtsc_is_active or not self._is_normal_mode or not self.enabled:
        self.acm_active = False
        return a_desired_trajectory

    # 補足預設參數 (若未傳入，則取自暫存記憶體)
    if t_follow is None:
        t_follow = get_T_FOLLOW(self.personality)
    lead = lead if lead is not None else self._saved_lead
    v_ego = v_ego if v_ego != 0.0 else self._saved_v_ego

    # 複製一份獨立的 Numpy 軌跡陣列，避免改動原始記憶體
    result = np.copy(a_desired_trajectory)

    # ==========================================
    # 📡 步驟 1：雷達訊號預處理與 EMA 濾波
    # ==========================================
    if lead and lead.status:
      # 雷達抓到車，重置丟失計數，鎖定有車狀態
      self.lead_lost_counter = 0
      self.has_lead_locked = True
      
      if not self.lead_status_prev:
        # 若前一幀無車，瞬間同步真實數據，避免濾波爬升太慢導致延遲
        self.filtered_d_rel, self.filtered_v_rel = lead.dRel, lead.vRel
      else:
        # 正常狀態下執行 EMA 指數移動平均，消除雷達微小跳動雜訊
        self.filtered_d_rel = (FILTER_ALPHA * lead.dRel) + ((1.0 - FILTER_ALPHA) * self.filtered_d_rel)
        self.filtered_v_rel = (FILTER_ALPHA * lead.vRel) + ((1.0 - FILTER_ALPHA) * self.filtered_v_rel)
      
      self.lead_status_prev = True
      self.last_valid_d_rel, self.last_valid_v_rel = self.filtered_d_rel, self.filtered_v_rel
    else:
      # 雷達沒看到車，啟動丟失倒數
      if self.has_lead_locked:
        self.lead_lost_counter += 1
      # 滿 5 幀依然無車，才正式宣告前方淨空，清除所有狀態
      if self.lead_lost_counter >= LEAD_LOST_TICKS:
        self.has_lead_locked, self.lead_status_prev, self.intent_accelerating = False, False, False
        self.accel_intent_counter = 0

    # ==========================================
    # 🧮 步驟 2：物理運算與 Bypass 退出判定
    # ==========================================
    bypass_acm = False  
    raw_a_calc = 0.0  
    acm_weight = 0.0  
    dist_percent = 0.0

    if self.has_lead_locked:
      # 🌟 [真・動態跟車] 完美重現 MPC 真實物理動態跟車距離公式
      # 概念：真實跟車距離 = (自車對靜止物體的煞停距離) - (前車的動能煞停緩衝)
      v_lead = max(0.0, v_ego + self.last_valid_v_rel)
      mpc_target_dist = get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead)
      
      # 確保最終目標距離不低於 6m 絕對防線
      target_dist = max(mpc_target_dist, STOP_DISTANCE)
      
      # 回歸純粹的物理比例相除，確保距離跨越邊界時不會產生比例扭曲
      dist_percent = self.last_valid_d_rel / target_dist

      # 🧠 [加速意圖鎖定] (線性插值過濾)
      recent_traj = result[:TRAJECTORY_HORIZON]
      intent_v_ratio = np.clip((v_ego - INTENT_V_LOW) / (INTENT_V_HIGH - INTENT_V_LOW), 0.0, 1.0)
      dynamic_intent_frames = int(round(INTENT_FRAMES_LOW + intent_v_ratio * (INTENT_FRAMES_HIGH - INTENT_FRAMES_LOW)))
      
      # 啟動開關：原廠給出加速軌跡 且 前車物理上正在遠離
      moment_accel = sum(1 for a in recent_traj if a > 0.05) >= INTENT_LOOKAHEAD and self.last_valid_v_rel > 0.05
      # 關閉開關：原廠給出減速軌跡 或 前車物理上正在靠近
      moment_decel = sum(1 for a in recent_traj if a < -0.05) >= INTENT_LOOKAHEAD or self.last_valid_v_rel < -0.05

      if moment_accel:
        self.accel_intent_counter += 1
      else:
        self.accel_intent_counter = 0

      # 計數器達標，確認加速意圖
      if self.accel_intent_counter >= dynamic_intent_frames:
        self.intent_accelerating = True

      # 如果發現原廠想減速，或距離已經被拉開，立刻收回加速特權
      if moment_decel or dist_percent >= COAST_END_PERCENT:
        self.intent_accelerating = False

      # 💣 [緊急退出防線] TTC 防撞與軌跡重煞監控
      # 當 TTC 過低或原廠預測軌跡中有任何點低於 -1.2 時，準備立刻退出交還 MPC
      ttc = (self.last_valid_d_rel / abs(self.last_valid_v_rel)) if self.last_valid_v_rel < -0.5 else 999.0
      is_emergency = (ttc < (t_follow * 1.2)) or any(a < MPC_FALLBACK_ACCEL for a in recent_traj)
      
      # 🧮 [計算 TTA 線性微煞車與動態滑行權重]
      # 計算距離跌破 70% 死亡線，還剩下多少真實的物理緩衝空間
      safe_buffer_dist = max(self.last_valid_d_rel - (target_dist * SAFE_DIST_PERCENT), 0.0)
      safe_v_rel = max(abs(self.last_valid_v_rel), 1e-3) # 取絕對速差，避免除以零
      
      # 🪄 魔法 1：時間壓縮 (欺騙公式時間只剩一半，逼出早期線性煞車力道)
      tta = (safe_buffer_dist / safe_v_rel) * TTA_TIME_RATIO
      # 基礎 TTA 物理減速度計算
      base_a_calc = -(TARGET_V_REL - self.last_valid_v_rel) / max(tta, 1.0)
      # 🪄 魔法 2：力道放大器 (讓煞車更扎實)
      raw_a_calc = base_a_calc * TTA_MULTIPLIER

      # 時速 <= 6m/s = 0.0 (100% MPC), 時速 >= 20m/s = 1.0 (100% ACM 滑行)
      acm_weight = np.clip((v_ego - BLEND_V_MIN) / (BLEND_V_MAX - BLEND_V_MIN), 0.0, 1.0)

      # 🌟 [ACM 實體防線與退出總結]
      # 計算 80%~70% 微煞車區間的物理實體公尺數
      brake_zone_length = target_dist * (COAST_END_PERCENT - SAFE_DIST_PERCENT)

      # 只要符合以下任一條件，全部導向 Bypass (將控制權平滑交還原廠 MPC)
      if target_dist <= STOP_DISTANCE:
        bypass_acm = True  # 🛑 絕對距離防線：目標距離 <= 6m 時，禁止啟動 ACM
      elif dist_percent >= EXIT_PERCENT:
        bypass_acm = True  # ⚪ 空間充裕，徹底休眠
      elif dist_percent < SAFE_DIST_PERCENT:
        bypass_acm = True  # 🔴 跌破危險線，交還原廠處理重煞
      elif is_emergency:
        bypass_acm = True  # 💣 緊急狀況 (TTC過低或原廠重煞)，立刻讓權保命
      elif self.intent_accelerating:
        bypass_acm = True  # 🚀 偵測到起步或加速，放行原廠補油門
      # 🧱 延遲市區防線：只有真正跌破 80% 準備進入微煞區時，才檢查 3m 區間防線
      elif dist_percent < COAST_END_PERCENT and brake_zone_length < MIN_BRAKE_ZONE_M:
        bypass_acm = True
      # 🟢 距離恰好，正式啟用 ACM 介入
      elif dist_percent <= COAST_START_PERCENT:
        self.acm_active = True
      else:
        # 處於遲滯區，且原本沒啟動，則繼續保持退出
        if not self.acm_active:
          bypass_acm = True

    # ==========================================
    # 🎨 步驟 3：統一軌跡修飾
    # ==========================================
    # 若無前車或判定 Bypass，直接退出，回傳原廠軌跡
    if not self.has_lead_locked or bypass_acm:
      self.acm_active = False
      return result

    # 對未來預測軌跡逐點執行邏輯修飾
    for i in range(len(result)):
      if dist_percent < COAST_END_PERCENT: 
        # 🟡 煞車平滑區間 (80% ~ 70%)：以 TTA 力道為界，限制不允許加速或煞過頭
        a_limit = np.clip(raw_a_calc, MIN_RECOVERY_ACCEL, MAX_RECOVERY_ACCEL)
        result[i] = float(min(result[i], a_limit))
      else: 
        # 🟢 純滑行區間 (98% ~ 80%)：抹平微煞車，配合速度進行權重融合
        coast_val = np.clip(result[i], COAST_MAX_BRAKE, MAX_RECOVERY_ACCEL)
        result[i] = float((coast_val * acm_weight) + (result[i] * (1.0 - acm_weight)))

    return result
