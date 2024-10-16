#!/usr/bin/env python3


# f(angle, velocity) = steer command
# Only use units of steer: [-1,1], m/s, and degrees

# remove points with poor distribution: std() threshold? P99?

import math
import argparse
import os
import pickle
from copy import deepcopy
from typing import NamedTuple
import shutil
import tempfile
import bz2
import numpy as np
# import seaborn as sns
from tqdm import tqdm  # type: ignore
from p_tqdm import p_map
import re
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from pathlib import Path
from collections import deque
from common.numpy_fast import interp

import matplotlib.pyplot as plt
from tools.tuning.lat_plot import fit, plot
import glob


from tools.tuning.lat_settings import *
if not PREPROCESS_ONLY:
  from scipy.stats import describe
  from scipy.signal import correlate, correlation_lags
  import matplotlib.pyplot as plt
  from tools.tuning.lat_plot import fit, plot
  import sys
  if not os.path.isdir('plots'):
    os.mkdir('plots')
  class Logger(object):
      def __init__(self):
          self.terminal = sys.stdout
          self.log = open("plots/logfile.txt", "a")
      def write(self, message):
          self.terminal.write(message)
          self.log.write(message)  
      def flush(self):
          # this flush method is needed for python 3 compatibility.
          # this handles the flush command by doing nothing.
          # you might want to specify some extra behavior here.
          pass    
  sys.stdout = Logger()

from selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.common.conversions import Conversions as CV
from tools.lib.logreader import LogReader
# from logreader import MultiLogIterator
from tools.lib.route import Route

MULTI_FILE = True

MAX_DRIVER_TORQUE = 0.0
MAX_EPS_TORQUE = 0.0
MAX_SPEED = 0.0
MIN_CURVATURE_RATE = 0.0
MAX_CURVATURE_RATE = 0.0
MIN_STEER_RATE = 0.0
MAX_STEER_RATE = 0.0

# Reduce samples using binning and outlier rejection
def regularize(speed, angle, steer, sort_var):
  print("Regularizing...")
  # Bin by rounding
  speed_bin = np.around(speed*2)/2
  angle_bin = np.around(angle*2, decimals=0 if IS_ANGLE_PLOT else 1)/2

  i = 0
  std = []
  count = []
  while i < len(speed):
      # Select bins by mask
      mask = (speed_bin == speed_bin[i]) & (angle_bin == angle_bin[i])

      # Exclude outliers
      sigma = np.std(steer[mask])
      mean = np.mean(steer[mask])
      inliers = mask & (np.fabs(steer - mean) <= BIN_SIGMA * sigma)
      
      c = inliers.sum()
    #   if len(steer[inliers]) == 0:
    #      print(f'No inliers found for {speed[i]}, {angle[i]}')
    #      continue
      s = np.std(steer[inliers])
      # Use this bin
      if c > BIN_COUNT and s < BIN_STD:
        sort_var_tmp = np.abs(sort_var[inliers])
        sort_ids = np.argsort(sort_var_tmp)
        
        npts = max(BIN_COUNT, int(len(inliers) * 0.1))
        speed_tmp = speed[inliers][sort_ids][:npts]
        angle_tmp = angle[inliers][sort_ids][:npts]
        steer_tmp = steer[inliers][sort_ids][:npts]
        
        speed[i] = np.mean(speed_tmp)
        angle[i] = np.mean(angle_tmp)
        steer[i] = np.mean(steer_tmp)

        count.append(c)
        std.append(s)
        mask[i] = False
        i += 1

      # Remove samples
      speed = speed[~mask]
      angle = angle[~mask]
      steer = steer[~mask]
      sort_var = sort_var[~mask]
      speed_bin = speed_bin[~mask]
      angle_bin = angle_bin[~mask]

  count = np.log(np.sort(count))
  std = np.sort(std)
  plt.figure(figsize=(12,8))
  plt.plot(std, label='std')
  plt.title('std')
  if not os.path.isdir('plots'):
    os.mkdir('plots')
  plt.savefig('plots/std.png')
  plt.close()

  plt.figure(figsize=(12,8))
  plt.plot(count, label='count')
  plt.title('count')
  if not os.path.isdir('plots'):
    os.mkdir('plots')
  plt.savefig('plots/count.png')
  plt.close()

  print(f'Regularized samples: {len(speed)}')
  return speed, angle, steer

def lag(x, y):
  assert (len(x) == len(y))
  # Normalize
  x = np.array(x)
  x = (x - np.mean(x)) / np.std(x)
  y = np.array(y)
  y = (y - np.mean(y)) / np.std(y)
  corr = correlate(x, y, mode='valid')
  lags = correlation_lags(x.size, y.size, "valid")
  return lags[np.argmax(corr)]

  # # Determine lag of this section
  # x = np.array([line['steer_command'] for line in data[-1]])
  # y = np.array([line['torque_eps'] for line in data[-1]])
  # l = lag(x,y)
  # if -30 < l < -10: # reasonable clipping
  #   lags.append(lag(x,y))

  #   if lags == []:
  # else:
  #   print(lags)
  #   print(describe(lags))
  #   print(f'lag median: {np.median(lags)}')
  #   print(f'Max seq. len: {max([len(line) for line in data])}')

class Sample():
  enabled: bool = False
  v_ego: float = np.nan
  a_ego: float = np.nan
  steer_angle: float = np.nan
  steer_rate: float = np.nan
  steer_offset: float = np.nan
  steer_offset_average: float = np.nan
  torque_eps: float = np.nan # -1,1
  torque_driver: float = np.nan # -1,1
  # curvature_plan: float = np.nan # lag
  # curvature_true: float = np.nan # lag
  lateral_accel: float = np.nan
  lateral_jerk: float = np.nan
  roll: float = np.nan
  pitch: float = np.nan
  curvature_device: float = np.nan
  lateral_accel_device: float = np.nan
  steer_cmd: float = np.nan
  steer_cmd_out: float = np.nan
  desired_accel: float = np.nan
  gas_cmd: float = np.nan
  gas_cmd_out: float = np.nan
  gas: float = np.nan
  gas_pressed: bool = False
  brake_cmd: float = np.nan
  brake_cmd_out: float = np.nan
  brake: float = np.nan
  brake_pressed: bool = False
  car_make: str = ''
  car_fp: str = ''
  car_eps_fp: str = ''
  long_actuator_delay: float = np.nan
  t_cs: float = np.nan
  t_cc: float = np.nan
  t_llk: float = np.nan
  t_lp: float = np.nan
  t_lat_p: float = np.nan
  t: float = np.nan
  yaw_rate: float = np.nan
  stiffnessFactor: float = np.nan
  steerRatio: float = np.nan
  

class CleanSample(NamedTuple):
  angle: float = np.nan
  speed: float = np.nan
  steer: float = np.nan
  sort_var: float = np.nan

def collect_log_data(lr):
  s = Sample()
  sample_deque = deque(maxlen=16)
  deque_mid_idx = 8
  t_interp_field = "t_llk"
  max_t_delta = 0.1
  samples: list[Sample] = []
  section: list[Sample] = []

  section_start: int = 0
  section_end: int = 0
  last_msg_time: int = 0
  
  CP = None
  VM = None
  lat_angular_velocity = np.nan
  lrd = dict()
  for msg in lr:
    try:
      msgid = f"{msg.logMonoTime}:{msg.which()}"
      if msgid in lrd:
        break
      lrd[msgid] = msg
    except:
      continue
  lr1 = list(lrd.values())
  # if not MULTI_FILE: print(f"{len(lr1)} messages")
  # type_set = set()
  for msg in sorted(lr1, key=lambda msg: msg.logMonoTime) if MULTI_FILE else tqdm(sorted(lr1, key=lambda msg: msg.logMonoTime)):
    # print(f'{msg.which() = }')
    try:
      # type_set.add(msg.which())
      if msg.which() == 'carState':
        s.t_cs = msg.logMonoTime
        #  m/s
        s.v_ego  = msg.carState.vEgoRaw
        if s.v_ego > 20*3.6:
          tt =1
        s.a_ego  = msg.carState.aEgo
        s.steer_angle = msg.carState.steeringAngleDeg
        s.steer_rate = msg.carState.steeringRateDeg
        s.torque_eps = msg.carState.steeringTorqueEps
        s.torque_driver = msg.carState.steeringTorque
        s.gas = msg.carState.gas
        s.gas_pressed = msg.carState.gasPressed
        s.brake = msg.carState.brake
        s.brake_pressed = msg.carState.brakePressed
      elif msg.which() == 'liveParameters':
        s.t_lp= msg.logMonoTime
        s.steer_offset = msg.liveParameters.angleOffsetDeg
        s.steer_offset_average = msg.liveParameters.angleOffsetAverageDeg  
        s.stiffnessFactor = msg.liveParameters.stiffnessFactor
        s.steerRatio = msg.liveParameters.steerRatio
        s.roll = msg.liveParameters.roll
        continue
      elif msg.which() == 'carControl':
        s.t_cc = msg.logMonoTime
        s.enabled = msg.carControl.latActive
        if hasattr(msg.carControl, "actuatorsOutput"):
          s.steer_cmd_out = msg.carControl.actuatorsOutput.steer
          s.gas_cmd_out = msg.carControl.actuatorsOutput.gas
          s.brake_cmd_out = msg.carControl.actuatorsOutput.brake
        s.steer_cmd = msg.carControl.actuators.steer
        s.gas_cmd = msg.carControl.actuators.gas
        s.brake_cmd = msg.carControl.actuators.brake
        s.desired_accel = msg.carControl.actuators.accel
        continue
      elif msg.which() == 'liveLocationKalman':
        s.t_llk = msg.logMonoTime
        s.yaw_rate = msg.liveLocationKalman.angularVelocityCalibrated.value[2]
        s.pitch = msg.liveLocationKalman.orientationNED.value[1]
      elif VM is None and msg.which() == 'carParams':
        CP = msg.carParams
        VM = VehicleModel(CP)
      else:
        continue

      # assert all messages have been received
      valid = not np.isnan(s.v_ego) and \
              not np.isnan(s.t_llk) and \
              not np.isnan(s.t_cc) and \
              VM is not None and \
              not np.isnan(s.t_lp)
      
      if valid:        
        # Check that the current sample time is not too far from the previous sample time.
        # If it is, reset the deque.
        if len(sample_deque) > 0:
          t_sample = getattr(s, t_interp_field)
          t_last = getattr(sample_deque[-1], t_interp_field)
          if (t_sample - t_last) * 1e-9 > max_t_delta:
            # print(f"Resetting deque at {t_sample} because t_sample - t_last = {(t_sample - t_last) * 1e-9} > {max_t_delta}")
            sample_deque.clear()
          else:
            # print(f"deque length = {len(sample_deque)}")
            sample_deque.append(s)
        else:
          # print(f"deque length = {len(sample_deque)}")
          sample_deque.append(s)
        
        # now interpolate a sample from the deque
        if len(sample_deque) == sample_deque.maxlen:
          # print(f"deque length = {len(sample_deque)}")
          cur_sample = sample_deque[deque_mid_idx]
          t_sample = getattr(cur_sample, t_interp_field)
          cur_sample.t = t_sample
          # need to do one interpolation for each field, using the field's
          # corresponsing time value.
          # This works by making a list of time values and then, for each field that shares the same
          # time value, interpolating the field value at that time.
          
          # we'll use a dict to organize this, where each key is the name of a time field, and the value
          # is a list of the fields that correspond to that time field.
          t_field_dict = {
            "t_cs": ["v_ego", "a_ego", "steer_angle", "steer_rate", "torque_eps", "torque_driver", "gas", "gas_pressed", "brake", "brake_pressed"],
            "t_lp": ["steer_offset", "steer_offset_average", "roll", "stiffnessFactor", "steerRatio"],
            "t_cc": ["enabled", "steer_cmd_out", "gas_cmd_out", "brake_cmd_out", "steer_cmd", "gas_cmd", "brake_cmd", "desired_accel"],
            }
          for t_field, field_list in t_field_dict.items():
            t_field_list = [getattr(s, t_field) for s in sample_deque]
            for field in field_list:
              field_list = [getattr(s, field) for s in sample_deque]
              setattr(cur_sample, field, interp(t_sample, t_field_list, field_list))
          
          # now compute derived values
          cur_sample.car_make = CP.carName
          cur_sample.car_fp = CP.carFingerprint
          cur_sample.car_eps_fp = str(next((fw.fwVersion for fw in CP.carFw if fw.ecu == "eps"), ""))
          cur_sample.long_actuator_delay = (CP.longitudinalActuatorDelayUpperBound + CP.longitudinalActuatorDelayLowerBound) / 2
          VM.update_params(max(cur_sample.stiffnessFactor, 0.1), max(cur_sample.steerRatio, 0.1))
          current_curvature = -VM.calc_curvature(math.radians(s.steer_angle - s.steer_offset), s.v_ego, s.roll)
          current_curvature_rate = -VM.calc_curvature(math.radians(s.steer_rate), s.v_ego, 0.)
          cur_sample.lateral_accel = current_curvature * s.v_ego**2
          cur_sample.lateral_jerk = current_curvature_rate * s.v_ego**2
          cur_sample.curvature_device = (cur_sample.yaw_rate / s.v_ego) if s.v_ego > 0.01 else 0.
          cur_sample.lateral_accel_device = cur_sample.yaw_rate * s.v_ego  - (np.sin(s.roll) * ACCELERATION_DUE_TO_GRAVITY)
            
          # then append the interpolated sample to the list of samples
          samples.append(deepcopy(cur_sample))
          
          # print out the interpolated sample as python dict
          # print(f"{cur_sample.__dict__}")
          
        s.v_ego = np.nan
          
          
    except:
      continue
  
  # print("message types found:\n" + ", ".join(list(type_set)))

  # # Terminated during valid section
  # if (section_end - section_start) * 1e-9 > MIN_SECTION_SECONDS:
  #   samples.extend(section)

  min_speed_reached = any([s.v_ego > 0.5 for s in samples])
  speed_data = np.array([s.v_ego for s in samples])
  max_speed = np.max(speed_data)
  print("max_speed(kph): ", max_speed*3.6)
  if len(samples) == 0 or not min_speed_reached:
    print(f'{len(samples)} samples found, {min_speed_reached = }')
    return np.array([])

  return np.array(samples)

def filter(samples):
  global MAX_DRIVER_TORQUE, MAX_EPS_TORQUE, MAX_SPEED, MIN_CURVATURE_RATE, MAX_CURVATURE_RATE, MAX_STEER_RATE, MIN_STEER_RATE
  # Order these to remove the most samples first
  
  steer_torque_key = "torque_eps" # vw
  MAX_DRIVER_TORQUE = max(MAX_DRIVER_TORQUE, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  MIN_STEER_RATE = min(MIN_STEER_RATE, np.min(np.array([s.steer_rate for s in samples])))
  MAX_STEER_RATE = max(MAX_STEER_RATE, np.max(np.array([s.steer_rate for s in samples])))

#   speed_list = []
#   for s in samples:
#     speed_list.append(s.v_ego)
  speed_data = np.array([s.v_ego for s in samples])
  max_speed_1 = np.max(speed_data)
  
  # Enabled and no steer pressed or not enabled and driver steer under threshold
  mask_steer = np.array([(s.enabled and abs(s.torque_driver) <= STEER_PRESSED_MIN) or (not s.enabled and abs(s.torque_driver) <= STEER_PRESSED_MAX) for s in samples])
#   samples = samples[mask]

  if IS_ANGLE_PLOT:
    # only take high angles if there was enough lateral acceleration
    max_lat_accel = 4.0
    curv_per_deg = 1/3000.0
    mask_curve = np.array([s.enabled or np.abs(((s.steer_angle - s.steer_offset) * curv_per_deg) * s.v_ego**2) < max_lat_accel  for s in samples])
    # samples = samples[mask]
  else:
    mask_curve = np.array([abs(s.torque_driver + getattr(s, steer_torque_key)) > 0.25 * abs(s.lateral_accel) for s in samples])
    # samples = samples[mask]

#   speed_data = np.array([s.v_ego for s in samples])
#   max_speed_2 = np.max(speed_data)
#   print(f"max_speed_1(kph): {max_speed_1*3.6}, max_speed_2(kph): {max_speed_2*3.6}")
  
  # filter steer rate  
  # No steer rate: holding steady curve or straight
  data = np.array([s.steer_rate for s in samples])
  mask_steer_rate = np.abs(data) < STEER_RATE_MIN
#   samples = samples[mask]
  
  #  filter speed  
  speed_all = np.array([s.v_ego for s in samples])
  if IS_ANGLE_PLOT:
    mask_speed = SPEED_MIN_ANGLE <= speed_all
  else:
    mask_speed = SPEED_MIN <= speed_all
  mask_speed &= speed_all <= SPEED_MAX

  mask_all = mask_steer & mask_curve & mask_steer_rate & mask_speed
  samples = samples[mask_all]

  return [CleanSample(
    speed = s.v_ego,
    angle = -s.lateral_accel if not IS_ANGLE_PLOT else s.steer_angle - s.steer_offset,
    steer = (s.torque_driver + (getattr(s, steer_torque_key) if not np.isnan(getattr(s, steer_torque_key)) else 0.0)),
    sort_var = getattr(s, BIN_SORT_VAR)
  ) for s in samples]

def load_cache(path):
  # print(f'Loading {path}')
  try:
    with open(path,'rb') as file:
      return pickle.load(file)
  except Exception as e:
    print(e)

def search_dir(root_folder, keyword='rlog', is_part_name=False):
    result = []
    # return glob.glob(f'{root_folder}/**/rlog', recursive=True)

    for dirpath, dirnames, files in os.walk(root_folder):
        if is_part_name:
            # 遍历文件列表，查找以 .lat 结尾的文件
            for file in files:
                if file.endswith('.lat'):
                    # 使用 os.path.join 拼接完整路径
                    result.append(os.path.join(dirpath, file))
        else:
            if keyword in files:
                result.append(os.path.join(dirpath, keyword))
    return result

def process_rlog_files(path, outpath):
    if not os.path.exists(outpath):
        os.makedirs(outpath)
    rlog_dir_list = search_dir(path, 'rlog')
    for rlog_path in rlog_dir_list:
      folder_name = rlog_path.split('/')[-2]
      latfile = os.path.join(outpath, f"{folder_name}.lat")
      filename = "rlog"
      try:
        print(f"\nLoading segment file: {rlog_path}")
        lr = LogReader(rlog_path, sort_by_time=True)
        data1 = collect_log_data(lr)
        if len(data1):
          with open(latfile, 'wb') as f:
            pickle.dump(data1, f)
        # if outpath == path:
        #   os.remove(os.path.join(path, filename))
      except Exception as e:
        print(f"Failed to load segment file: {rlog_path}:\n{e}")


def parser_data(latpath):
    global MULTI_FILE
    old_num_points = 0
    g_data = None
    g_data1 = None
    data_list = search_dir(latpath, keyword=".lat", is_part_name=True)
    for data_path in data_list:
        data_cache = load_cache(data_path)
        if data_cache is  None:
           continue
        data_cur = filter(data_cache)
        if g_data is not None:
            g_data.extend(data_cur)
        else:
            g_data = data_cur

    # if latpath is not None and os.path.exists(latpath):
    #     for filename in os.listdir(latpath):
    #         if filename.endswith(".lat"):
    #             # with open(os.path.join(latpath, filename), 'rb') as f:
    #             #     data = pickle.load(f)
    #             #     old_num_points += len(data)
    #             data_cache = load_cache(os.path.join(latpath, filename))
    #             data_cur = filter(data_cache)
    #             # g_data.append(data_cur)
    #             if g_data is not None:
    #                 g_data.extend(data_cur)
    #             else:
    #                 g_data = data_cur
  
    newlen = len(g_data)
    if not os.path.isdir('plots'):
        os.mkdir('plots')
    # with open('plots/out.txt','w') as f:
    #     if old_num_points > 0 and newlen > 0:
    #       f.write(f"{old_num_points} points filtered down to {newlen}\n")
    #     else:
    #       f.write(f"{newlen} filtered points\n")
    speed_l = []
    angle_l = []
    steer_l = []
    sort_var_l = []
    for sample in g_data:
        speed_l.extend([sample.speed])
        angle_l.extend([sample.angle])
        steer_l.extend([sample.steer])
        sort_var_l.extend([sample.sort_var])
    
    speed = np.array(speed_l)
    angle = np.array(angle_l)
    steer = np.array(steer_l)
    sort_var = np.array(sort_var_l)

    print(f'Samples: {len(speed) = }, {len(angle) = }, {len(steer) = }')

    return speed, angle, steer,sort_var


if __name__ == '__main__':
    global IS_ANGLE_PLOT
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str)
    parser.add_argument('--outpath', type=str)
    parser.add_argument('--lat_path', type=str)
    parser.add_argument('--route', type=str)
    parser.add_argument('--is_preprocess', type=int, default=0)
    parser.add_argument('--dongleid', type=str)

    #define debug args using '      python3 /home/haiiro/openpilot-batch/openpilot/tools/tuning/lat.py --preprocess --path "$curdir/$dongle" --outpath "$outdir"'
    # debug_args = ['--preprocess', '--path', '/mnt/video/scratch-video/rlogs/hyundai/HYUNDAI SANTA FE 2022/1cbf25ced339760c', '--outpath', '/mnt/video/scratch-video/latfiles/hyundai/HYUNDAI SANTA FE 2022']
    # args = parser.parse_args(debug_args)

    args = parser.parse_args()


    # IS_ANGLE_PLOT = True
    # regfile = 'regularized'
    # if REGULARIZED and os.path.isfile(regfile):
    #   print("Opening regularized data")
    #   with open(regfile,'rb') as file:
    #     speed, angle, steer = pickle.load(file)
    # else:
    #   print("Loading new data")
    #   speed, angle, steer, sort_var = load(args.path, args.route, args.preprocess, args.dongleid, args.outpath)
    #   speed, angle, steer = regularize(speed, angle, steer, sort_var)
    #   with open(regfile, 'wb') as f:
    #     pickle.dump([speed, angle, steer], f)

    # fit(speed, angle, steer, IS_ANGLE_PLOT)
    # plot(speed, angle, steer)

    IS_ANGLE_PLOT = False
    if args.is_preprocess:
        process_rlog_files(args.path, args.outpath)
    
    # m/s
    speed_raw, angle_raw, steer_raw, sort_var_raw = parser_data(args.outpath)

    speed, angle, steer = regularize(speed_raw, angle_raw, steer_raw, sort_var_raw)
    max_speed_raw = np.max(speed_raw)
    max_speed = np.max(speed)
    print(f"max_speed_raw(kph): {max_speed_raw*3.6}, max_speed(kph): {max_speed*3.6}")
    
    fit(speed, angle, steer, IS_ANGLE_PLOT)
    plot(speed, angle, steer)
