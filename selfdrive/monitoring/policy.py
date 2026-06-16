class DRIVER_MONITOR_SETTINGS:
  def __init__(self):
    # https://eur-lex.europa.eu/legal-content/EN/TXT/PDF/?uri=CELEX:42018X1947&rid=2
    self._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT = 30.  # 原為 15.
    self._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT = 48.  # 原為 24.
    self._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT = 60.  # 原為 30.
    # https://cdn.euroncap.com/cars/assets/euro_ncap_protocol_safe_driving_driver_engagement_v11_a30e874152.pdf
    self._VISION_POLICY_ALERT_1_TIMEOUT = 6.       # 原為 3.
    self._VISION_POLICY_ALERT_2_TIMEOUT = 10.      # 原為 5.
    self._VISION_POLICY_ALERT_3_TIMEOUT = 22.      # 原為 11.

    self._TIMEOUT_RECOVERY_FACTOR_MAX = 5.
    self._TIMEOUT_RECOVERY_FACTOR_MIN = 1.25

    self._MAX_TERMINAL_ALERTS = 6  # 原為 3，允許更多次的終端警告
    self._MAX_TERMINAL_DURATION = int(60 / DT_DMON)  # 原為 30，終端警告時間限制加倍

    self._FACE_THRESHOLD = 0.7
    self._EYE_THRESHOLD = 0.65
    self._SG_THRESHOLD = 0.9
    self._BLINK_THRESHOLD = 0.9325  # 原為 0.865，提高數值使其更難觸發閉眼判定
    self._PHONE_THRESH = 0.75       # 原為 0.5，放寬手機偵測閥值
    
    # 將頭部俯仰 (Pitch) 與偏航 (Yaw) 的容忍度提高約 50%
    self._POSE_PITCH_THRESHOLD = 0.47          # 原為 0.3133
    self._POSE_PITCH_THRESHOLD_SLACK = 0.485   # 原為 0.3237
    self._POSE_PITCH_THRESHOLD_STRICT = self._POSE_PITCH_THRESHOLD
    self._POSE_YAW_THRESHOLD = 0.603           # 原為 0.4020
    self._POSE_YAW_THRESHOLD_SLACK = 0.756     # 原為 0.5042
    self._POSE_YAW_THRESHOLD_STRICT = self._POSE_YAW_THRESHOLD
    
    self._POSE_YAW_MIN_STEER_DEG = 30
    self._POSE_YAW_STEER_FACTOR = 0.15
    self._POSE_YAW_STEER_MAX_OFFSET = 0.3927
    self._PITCH_NATURAL_OFFSET = 0.011 
    self._PITCH_NATURAL_THRESHOLD = 0.673      # 原為 0.449，按比例放寬
    self._YAW_NATURAL_OFFSET = 0.075 
    self._PITCH_NATURAL_VAR = 3*0.01
    self._YAW_NATURAL_VAR = 3*0.05
    self._PITCH_MAX_OFFSET = 0.124
    self._PITCH_MIN_OFFSET = -0.0881
    self._YAW_MAX_OFFSET = 0.289
    self._YAW_MIN_OFFSET = -0.0246

    self._DCAM_UNCERTAIN_ALERT_THRESHOLD = 0.1
    self._DCAM_UNCERTAIN_ALERT_COUNT = int(60  / DT_DMON)
    self._DCAM_UNCERTAIN_RESET_COUNT = int(2  / DT_DMON)
    self._HI_STD_THRESHOLD = 0.3
    self._HI_STD_FALLBACK_TIME = int(20  / DT_DMON)  # 原為 10，模型不確定時退回方向盤策略的時間加倍
    self._DISTRACTED_FILTER_TS = 0.25  

    self._POSE_CALIB_MIN_SPEED = 13  
    self._POSE_OFFSET_MIN_COUNT = int(60 / DT_DMON)  
    self._POSE_OFFSET_MAX_COUNT = int(360 / DT_DMON)  
    self._WHEELPOS_CALIB_MIN_SPEED = 11
    self._WHEELPOS_THRESHOLD = 0.5
    self._WHEELPOS_FILTER_MIN_COUNT = int(15 / DT_DMON) 
    self._WHEELPOS_DATA_AVG = 0.03
    self._WHEELPOS_DATA_VAR = 3*5.5e-5
    self._WHEELPOS_MAX_COUNT = -1
