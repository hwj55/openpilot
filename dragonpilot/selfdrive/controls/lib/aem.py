"""
Copyright (c) 2025, Rick Lan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, and/or sublicense,
for non-commercial purposes only, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
- Commercial use (e.g. use in a product, service, or activity intended to
  generate revenue) is prohibited without explicit written permission from
  the copyright holder.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import numpy as np

# 車速閾值 (將 km/h 轉換為 m/s)
SPEED_ENABLE_MS = 70.0 / 3.6   # 約 19.44 m/s (低於此速度允許啟用)
SPEED_DISABLE_MS = 80.0 / 3.6  # 約 22.22 m/s (高於此速度強制關閉)

# 動態車距參數
TIME_GAP = 1.5         # 秒 (動態車距時間)
MIN_DISTANCE = 8.0     # 公尺 (最短觸發距離)

# 綠燈起步/暢通判定參數
GREEN_LIGHT_X_THRESHOLD = 20.0 # 公尺 (軌跡大於此值視為綠燈或路況暢通)


class AEM:
  def __init__(self):
    self._active = False
    self._speed_condition_met = True

  def update_states(self, v_ego, should_stop, a_target, trajectory_length):
    """
    更新 AEM 狀態 (專注於判定是否切換至實驗模式)
    :param v_ego: 當前車速 (m/s)
    :param should_stop: boolean, 模型判定是否該停 (紅綠燈/停止線)
    :param a_target: float, 模型目標加速度 (m/s^2)
    :param trajectory_length: float, 預測軌跡總長 (m)
    """
    
    # 1. 處理車速遲滯區間
    if v_ego < SPEED_ENABLE_MS:
      self._speed_condition_met = True
    elif v_ego > SPEED_DISABLE_MS:
      self._speed_condition_met = False

    # 若車速大於 80km/h，強制維持 ACC 不允許切換
    if not self._speed_condition_met:
      self._active = False
      return

    # 2. 綠燈起步 / 前方暢通優先判定
    # 必須在模型「沒有要停 (not should_stop)」的前提下，軌跡大於 20m 才切回 ACC
    # 避免遠處紅燈被誤判為暢通綠燈
    if not should_stop and trajectory_length > GREEN_LIGHT_X_THRESHOLD:
      self._active = False
      return

    # 3. 計算動態觸發距離閾值：用於對付前車急煞或動態障礙物
    trigger_threshold = max(MIN_DISTANCE, v_ego * TIME_GAP)

    # 4. 模式切換核心邏輯
    if should_stop:
      # 【情況 A：遇到紅綠燈或停止線】
      # 只要模型認為該停，立刻切換至 blended 模式，讓 E2E 模型接管縱向規劃
      self._active = True

    elif a_target < -1.5:
      # 【情況 B：遇到動態障礙物 / 前車急煞】
      # 雖然模型有強烈減速意圖，但我們等到距離逼近到 1.5 秒/8 公尺內，才切換為 blended
      if trajectory_length <= trigger_threshold:
        self._active = True
      else:
        self._active = False
        
    else:
      # 無需煞停或減速，保持一般 ACC 模式
      self._active = False

  def get_mode(self, current_mode):
    """
    獲取當前應使用的模式
    """
    if self._active:
      return 'blended'  
    else:
      return 'acc'
