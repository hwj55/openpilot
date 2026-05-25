"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import custom
import numpy as np
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

# 定義加速度個性化設定的列舉型態 (Eco, Normal, Sport)
AccelPersonality = custom.LongitudinalPlanDP.AccelerationPersonality
ACCEL_PERSONALITY_OPTIONS = [AccelPersonality.eco, AccelPersonality.normal, AccelPersonality.sport]

# ==============================================================================
# 加速度與減速度設定檔 (Profiles & Breakpoints)
# ==============================================================================

# 加速度上限的中斷點 (車速, 單位: m/s)
A_MAX_BP = [0.0,  0.5,  1.0,  4.0,   6.0,  9.0,  11.0, 16.0,  20.0, 25.0, 30.0, 55.0]
# 各種個性化設定下的最大加速度值 (對應 A_MAX_BP)
A_MAX_V = {
  AccelPersonality.eco:       [1.00, 0.60, 1.00, 1.40,  1.20, 1.00, 0.80, 0.60,  0.50, 0.40, 0.12, 0.08],
  AccelPersonality.normal:    [1.50, 0.80, 1.20, 1.80,  1.40, 1.20, 1.00, 0.80,  0.70, 0.60, 0.24, 0.10],
  AccelPersonality.sport:     [2.00, 1.00, 1.40, 2.00,  1.60, 1.40, 1.20, 1.00,  0.90, 0.80, 0.36, 0.12],
}

# 滑行阻力 (Coast Drag) 的中斷點 (車速, 單位: m/s)
COAST_DRAG_BP = [0.0, 10.0, 25.0, 40.0]
# 各種個性化設定下的滑行減速度值 (對應 COAST_DRAG_BP，模擬放開油門時的自然減速)
COAST_DRAG_V = {
  AccelPersonality.eco:    [-0.03, -0.05, -0.08, -0.12],
  AccelPersonality.normal: [-0.04, -0.07, -0.12, -0.18],
  AccelPersonality.sport:  [-0.06, -0.10, -0.18, -0.28],
}

# 煞車底線 (A_MIN Floor) 的中斷點 (車速, 單位: m/s)
A_MIN_FLOOR_BP =    [3,     4.5,   7.,    9.,     25]
# 各種個性化設定下的最大允許減速度值 (對應 A_MIN_FLOOR_BP)
A_MIN_FLOOR_V = {
  AccelPersonality.eco:    [-.003, -0.25, -0.35, -0.44, -2.0],
  AccelPersonality.normal: [-.004, -0.27, -0.37, -0.46, -2.0],
  AccelPersonality.sport:  [-.005, -0.29, -0.39, -0.48, -2.0],
}

# ==============================================================================
# 控制模型常數設定
# ==============================================================================
DEFICIT_TO_FLOOR = 8.5  # 當車速低於巡航速度在此範圍內時，逐漸過渡到煞車底線
COAST_DEADBAND = 0.5    # 巡航死區 (m/s)，在此速差範圍內優先進入滑行狀態以維持車速穩定
RAMP_OFF_RANGE = 3.0    # 接近巡航速度時，加速度上限開始線性遞減的緩衝範圍 (m/s)

# 非對稱變化率限制 (Rate Limiting)
A_MIN_TIGHTEN_RATE = 1.5  # 煞車加重時的變化率上限 (m/s³，對應原本的 MAX_DECEL_INCREASE_RATE)
A_MIN_RELAX_RATE = 0.6    # 煞車放鬆時的變化率上限 (m/s³，對應原本的 MAX_DECEL_DECREASE_RATE)
A_MAX_RATE = 0.8          # 加速度上限的變化率 (m/s³)

# 動態安全廊道間距 (Dynamic Safety Corridor Gap)
# 確保最小加速度永遠嚴格小於最大加速度，防止解算器崩潰 (Solver Crash)
MIN_MAX_GAP = 0.05

# 參數重新讀取的幀數間隔 (每秒更新一次 Params)
PARAM_REFRESH_FRAMES = max(1, int(1.0 / DT_MDL))


class AccelPersonalityController:
  """
  升級版縱向加減速控制器 (新版模型 tn)
  引入巡航速度 (v_cruise) 依賴性，將減速邏輯區分為滑行阻力與煞車底線，
  並透過接近緩和與巡航死區控制，大幅提升車輛在接近目標車速時的平穩度與舒適性。
  """
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self._first = True  # 標記是否為首次執行，用於初始化加減速狀態

    # 從系統參數讀取當前的個性化設定與啟用狀態
    val = self.params.get('AccelPersonality')
    self._personality = val if val is not None else AccelPersonality.normal
    self._enabled = self.params.get_bool('AccelPersonalityEnabled')

    # ==============================================================================
    # 老司機模式開關狀態初始化 (預設開啟以便直接測試，可綁定至 OP UI 參數)
    # ==============================================================================
    self._ed_enabled = True   # 老司機總開關
    self._ed_class1 = True    # Class 1: 前車緩行或靜止提早滑行
    self._ed_class2 = True    # Class 2: 前車加速度不足時匹配並優化加速度

    # 讀取參數 (若有在 UI 建立對應 toggle 可解除註解使用)
    # self._ed_enabled = self.params.get_bool('ExperiencedDriverEnabled')
    # self._ed_class1 = self.params.get_bool('ExperiencedDriverClass1')
    # self._ed_class2 = self.params.get_bool('ExperiencedDriverClass2')

    # 初始化巡航速度與當前加減速限制值
    self._v_cruise = 0.0
    self._a_min = -0.05
    self._a_max = 1.50

    # 效能優化快取機制 (Caching)
    self._cache_v: float | None = None
    self._cache_v_cruise: float | None = None
    self._cache_a_min = self._a_min
    self._cache_a_max = self._a_max

    # 前車狀態與老司機標記
    self._force_early_coast = False
    self._lead_status = False
    self._lead_a_lead = 0.0
    self._in_3s_dist = False          # 動態車距 3 秒內標記
    self._ed_class2_counter = 0       # Class 2 濾波計數器

  def update(self, sm=None):
    """
    更新控制器狀態，讀取最新車輛巡航速度並定期刷新系統參數
    """
    self.frame += 1
    # 每個 cycle 重設快取
    self._cache_v = None
    self._cache_v_cruise = None

    # 每個週期重設狀態，確保條件不滿足時能恢復正常控制
    self._force_early_coast = False
    self._lead_status = False
    self._lead_a_lead = 0.0
    self._in_3s_dist = False

    # 從開源車輛狀態 (carState) 獲取設定的巡航速度，並將時速 (km/h) 轉換為秒速 (m/s)
    if sm is not None:
      try:
        # 取得巡航速度
        self._v_cruise = float(sm['carState'].vCruise) * CV.KPH_TO_MS

        # 取得前車狀態
        if 'radarState' in sm:
          lead_one = sm['radarState'].leadOne
          self._lead_status = lead_one.status
          
          if self._lead_status:
            # 記錄前車加速度供 Class 2 使用
            self._lead_a_lead = lead_one.aLeadK
            
            # ==============================================================================
            # [老司機判定準備] 導入動態相對速度與 3 秒車距閾值
            # ==============================================================================
            v_ego = float(sm['carState'].vEgo)

            # 判斷前車是否在自車 3 秒距離內 (距離 <= 自車秒速 * 3)
            self._in_3s_dist = lead_one.dRel <= (v_ego * 3.0)

            # 根據車速動態計算逼近閾值：
            v_rel_thresh = float(np.interp(v_ego, [16.0, 22.0], [0.5, 1.0]))

            # Class 1 條件：有前車、相對速度小於動態閾值、距離 > 20m 避開低速緩行，且【在 3 秒車距內】
            self._force_early_coast = bool(
                lead_one.vRel < -v_rel_thresh and 
                lead_one.dRel > 20.0 and 
                self._in_3s_dist
            )

      except Exception:
        pass

    # 定期刷新外部參數，避免每幀讀取 Params 造成 I/O 負擔
    if self.frame % PARAM_REFRESH_FRAMES == 0:
      val = self.params.get('AccelPersonality')
      self._personality = val if val is not None else AccelPersonality.normal
      self._enabled = self.params.get_bool('AccelPersonalityEnabled')

  @property
  def accel_personality(self) -> int:
    return self._personality

  def get_accel_personality(self) -> int:
    return int(self._personality)

  def set_accel_personality(self, personality: int):
    if personality in ACCEL_PERSONALITY_OPTIONS:
      self._personality = personality
      self.params.put('AccelPersonality', personality)

  def cycle_accel_personality(self) -> int:
    idx = ACCEL_PERSONALITY_OPTIONS.index(self._personality) if self._personality in ACCEL_PERSONALITY_OPTIONS else 0
    nxt = ACCEL_PERSONALITY_OPTIONS[(idx + 1) % len(ACCEL_PERSONALITY_OPTIONS)]
    self.set_accel_personality(nxt)
    return int(nxt)

  def is_enabled(self) -> bool:
    return self._enabled

  def set_enabled(self, enabled: bool):
    self._enabled = bool(enabled)
    self.params.put_bool('AccelPersonalityEnabled', self._enabled)

  def toggle_enabled(self) -> bool:
    self.set_enabled(not self._enabled)
    return self._enabled

  def reset(self, personality: int | None = None):
    if personality is None or personality not in ACCEL_PERSONALITY_OPTIONS:
      personality = AccelPersonality.normal
    self._personality = personality
    self.params.put('AccelPersonality', self._personality)
    self.frame = 0
    self._first = True
    self._a_min = -0.05
    self._a_max = 1.50
    self._cache_v = None
    self._cache_v_cruise = None
    
    # 重置老司機狀態
    self._force_early_coast = False
    self._lead_status = False
    self._lead_a_lead = 0.0
    self._in_3s_dist = False
    self._ed_class2_counter = 0

  def get_accel_limits(self, v_ego: float) -> tuple[float, float]:
    v_ego = max(0.0, v_ego)
    if (self._cache_v is not None
        and abs(self._cache_v - v_ego) < 0.01
        and self._cache_v_cruise == self._v_cruise):
      return self._cache_a_min, self._cache_a_max

    self._cache_a_min, self._cache_a_max = self._step(v_ego)
    self._cache_v = v_ego
    self._cache_v_cruise = self._v_cruise
    return self._cache_a_min, self._cache_a_max

  def get_min_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[0]

  def get_max_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[1]

  def _ramp_off(self, v_ego: float) -> float:
    if self._v_cruise <= 0.0:
      return 1.0
    return float(np.clip((self._v_cruise - v_ego) / RAMP_OFF_RANGE, 0.0, 1.0))

  def _target_max(self, v_ego: float) -> float:
    base = float(np.interp(v_ego, A_MAX_BP, A_MAX_V[self._personality]))
    return base * self._ramp_off(v_ego)

  def _target_min(self, v_ego: float) -> float:
    coast = float(np.interp(v_ego, COAST_DRAG_BP, COAST_DRAG_V[self._personality]))
    if self._v_cruise <= 0.0 or v_ego >= self._v_cruise:
      return coast

    floor = float(np.interp(v_ego, A_MIN_FLOOR_BP, A_MIN_FLOOR_V[self._personality]))
    deficit = self._v_cruise - v_ego
    t = float(np.clip(deficit / DEFICIT_TO_FLOOR, 0.0, 1.0)) ** 1.5
    return coast + t * (floor - coast)

  def _apply_coast_deadband(self, v_ego: float, t_min: float, t_max: float) -> tuple[float, float]:
    if self._v_cruise <= 0.0 or abs(v_ego - self._v_cruise) >= COAST_DEADBAND:
      return t_min, t_max
    coast = float(np.interp(v_ego, COAST_DRAG_BP, COAST_DRAG_V[self._personality]))
    return coast, max(0.05, t_max * 0.25)

  def _rate_limit(self, last: float, target: float, rate_down: float, rate_up: float) -> float:
    rate = rate_up if target > last else rate_down
    step = rate * DT_MDL
    return float(np.clip(target, last - step, last + step))

  def _step(self, v_ego: float) -> tuple[float, float]:
    # 1. 計算目標極值
    t_max = self._target_max(v_ego)
    t_min = self._target_min(v_ego)

    # 2. 應用巡航死區修正
    t_min, t_max = self._apply_coast_deadband(v_ego, t_min, t_max)

    # ==============================================================================
    # [新增] 老司機模式管理 (Experienced Driver Mode)
    # ==============================================================================
    if self._ed_enabled:
        
        # Class 1: 前車緩行或靜止提早滑行 (已在 update 確認 3 秒車距)
        if self._ed_class1 and self._force_early_coast:
            # 強制將加速上限壓至 -1e-3 (釋放動能完美滑行)
            t_max = -1e-3
            self._ed_class2_counter = 0  # 阻斷並重置 Class 2 計數
            
        # Class 2: 在 3 秒動態車距內，且前車有數值的狀態下
        elif self._ed_class2 and self._lead_status and self._in_3s_dist:
            # 判斷前車加速度是否低於自車查表設定的最高加速度上限 (t_max)
            if self._lead_a_lead < t_max:
                self._ed_class2_counter += 1
            else:
                self._ed_class2_counter = 0
                
            # 連續 4 幀 (4fps) 滿足條件才正式介入
            if self._ed_class2_counter >= 4:
                # 套用前車加速度 + 0.1
                t_max = self._lead_a_lead + 0.1
        else:
            # 若脫離 3 秒車距或沒有前車，重置濾波計數器
            self._ed_class2_counter = 0

    # ==============================================================================

    # 3. 首次執行直接賦值
    if self._first:
      self._a_min, self._a_max = t_min, t_max
      self._first = False
      return self._a_min, self._a_max

    # 4. 應用嚴格的線性變化率限制
    new_min = self._rate_limit(self._a_min, t_min, rate_down=A_MIN_TIGHTEN_RATE, rate_up=A_MIN_RELAX_RATE)
    new_max = self._rate_limit(self._a_max, t_max, rate_down=A_MAX_RATE, rate_up=A_MAX_RATE)

    # 5. 安全廊道約束：確保 min 永遠小於 max 減去最小間距，防止解算器異常
    new_min = min(new_min, new_max - MIN_MAX_GAP)

    self._a_min, self._a_max = new_min, new_max
    return self._a_min, self._a_max