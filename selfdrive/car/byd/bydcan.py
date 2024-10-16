from cereal import car
from openpilot.selfdrive.car import make_can_msg



def byd_checksum(dat):
    # key_offset = 0x75
    # 将字节串转换为十六进制字符串
    hex_string = dat.hex()
    # 取前7组字符
    hex_string_seven_groups = hex_string[:14]
    # 将前7组字符转换为整数并求和
    total_sum = sum(
        int(hex_string_seven_groups[i : i + 2], 16)
        for i in range(0, len(hex_string_seven_groups), 2)
    )
    # total_sum = total_sum + key_offset

    # 取求和结果的后八位二进制
    binary_sum = bin(total_sum)[-8:].zfill(8)
    # 计算反码
    inverted_binary_sum = "".join("1" if bit == "0" else "0" for bit in binary_sum)
    # 将反码转换为十六进制并返回
    # crc = hex(int(inverted_binary_sum, 2))[2:].upper()
    crc = int(inverted_binary_sum, 2)
    # crc_decimal = int(crc, 16)
    return crc  # 直接返回十六进制字符串，并将其转换为大写形式

# reference: http://sunshine2k.de/articles/coding/crc/understanding_crc.html#ch3
crc8_lut_8h2f = []
def init_lut_crc8_8h2f():
  poly = 0x2F

  for i in range(256):
    crc = i
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
    crc8_lut_8h2f.append(crc)

def get_crc8_8h2f(dat):
  crc = 0xFF    # initial value for crc8_8h2f
  for i in range(len(dat)):
    crc ^= dat[i]
    crc = crc8_lut_8h2f[crc]
  return crc ^ 0xFF


def crc8_8h2f(data):
    polynomial = 0x183
    initial = 0xFF
    final_xor = 0x00
    crc = initial
 
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
 
    return crc ^ final_xor

def crc8(data):
  crc = 0xFF    # standard init value
  poly = 0xD5   # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size - 1, -1, -1):
    crc ^= data[i]
    for _ in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc

# acc
def create_acc_cmd(packer, CAN, raw_msg, raw_cnt, accel, enabled, bypass_raw_acc=True):
    values = raw_msg
    if not bypass_raw_acc:
        values.update({
            "ACC_Axv_Cv_Aim_S": accel,
            # "LKS_torque_request_staus_S": 1,
        })

    values.update({
            "Alive_Counter32E_S": raw_cnt,
            # "LKS_torque_request_staus_S": 1,
        })
    
    dat = packer.make_can_msg("MRR_0x32E", CAN.ECAN, values)[1]
    crc = byd_checksum(dat)
    values.update({
      "Checksum32E_S" : crc,
      })

    dat2 = packer.make_can_msg("MRR_0x32E", CAN.ECAN, values)
    return dat2

# lcc
def create_lcc_cmd(packer, CAN, raw_msg, counter, torque_req, apply_steer_req, eps_active=False, bypass_raw_lcc=False, bypass_raw_msg=False):
    values = raw_msg
    if not bypass_raw_msg:
        crc_raw = int(raw_msg.get("Messge_Checksum316_S", 0))
        raw_counter = int(raw_msg.get("Message_Alive_Counter316_S", 0))
        LKS_torque_request_staus_S_raw = int(raw_msg.get("LKS_torque_request_staus_S", 0))
        # MPC_Take_Over_Req1_S_raw = int(raw_msg.get("MPC_Take_Over_Req1_S", 0))
        Left_Line_Tracking_Status_S_raw = int(raw_msg.get("Left_Line_Tracking_Status_S", 0))
        Right_Line_Tracking_Stats_S_raw = int(raw_msg.get("Right_Line_Tracking_Stats_S", 0))
        Lane_Assistant_Mode_Status_S_raw = int(raw_msg.get("Lane_Assistant_Mode_Status_S", 0))
        TJA_ICA_function_status_S_raw =  int(raw_msg.get("TJA_ICA_function_status_S", 0))
        
        # 如果eps_active=True，或者 原车本来就激活了LKS，那么就激活LKS
        if (eps_active or LKS_torque_request_staus_S_raw == 2) and apply_steer_req:
          LKS_torque_request_staus_S = 2
          # MPC_Take_Over_Req1_S = 0
          Left_Line_Tracking_Status_S = 1
          Right_Line_Tracking_Stats_S = 1
          TJA_ICA_function_status_S = 2
        else:
          LKS_torque_request_staus_S = LKS_torque_request_staus_S_raw
          # MPC_Take_Over_Req1_S = MPC_Take_Over_Req1_S_raw
          Left_Line_Tracking_Status_S = Left_Line_Tracking_Status_S_raw
          Right_Line_Tracking_Stats_S = Right_Line_Tracking_Stats_S_raw
          TJA_ICA_function_status_S = TJA_ICA_function_status_S_raw

        if not bypass_raw_lcc:
            values.update({
                "PT31":31,
                "PT1_1":0x1,
                "PT3":3,
                "PT7":7,
                "PTFF":0xFF,
                "Output_LKS_torque_request_S": torque_req,
                "LKS_torque_request_staus_S": LKS_torque_request_staus_S,
                "Left_Line_Tracking_Status_S": Left_Line_Tracking_Status_S,
                "Right_Line_Tracking_Stats_S": Right_Line_Tracking_Stats_S,
                "TJA_ICA_function_status_S": TJA_ICA_function_status_S,
            })

        values.update({
                "Message_Alive_Counter316_S": counter,
            })

        dat_1 = packer.make_can_msg("MRR_0x316", CAN.ECAN, values)[1]
        crc = byd_checksum(dat_1[:-1]) 
        values.update({
            "Messge_Checksum316_S" : crc,
        })
        
    dat = packer.make_can_msg("MRR_0x316", CAN.ECAN, values)
    return dat

# MPC_0x34B
def create_lane_hmi(packer, CAN, raw_msg, left_lane_coeff, right_lane_coeff):
    values = raw_msg
    # print("raw: ", raw_msg)
    values.update({
            "Left_Lane_Line_Type_S": 1,
            "Right_Lane_Line_Type_S": 1,
            "Left_Lane_Line_Y_Distance_S": -left_lane_coeff.c0 ,
            "Right_Lane_Line_Y_Distance_S": -right_lane_coeff.c0 ,
            "Lane_Line_Curvature_S":-(left_lane_coeff.c2 + right_lane_coeff.c2)*0.5 ,
            # "LKS_torque_request_staus_S": 1,
        })
    # print("new: ", values)

    dat = packer.make_can_msg("MPC_0x34B", CAN.ECAN, values)[1]
    crc = byd_checksum(dat)
    values.update({
            "CheckSum_34B_S" : crc,
        })

    dat = packer.make_can_msg("MPC_0x34B", CAN.ECAN, values)
    return dat


def send_buttons(packer, CAN, count):
    """Spoof ACC Button Command."""
    values = {
        "COUNTER": count,
    }
    dat = packer.make_can_msg("SWS_0x3B0", CAN.ECAN, values)[1]
    crc = byd_checksum(dat)
    # values["CHECKSUM"] = crc
    return packer.make_can_msg("SWS_0x3B0", CAN.ECAN, values)


# init_lut_crc8_8h2f()