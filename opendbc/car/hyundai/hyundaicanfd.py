import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanFdLeadData


class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, lka_steering=None) -> None:
    super().__init__(CP, fingerprint)

    if lka_steering is None:
      lka_steering = CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. LKA steering cars
    # have a different harness than the LFA steering variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if lka_steering:
      self._a, self._e = 0, 1

    self._a += self.offset
    self._e += self.offset
    self._cam = 2 + self.offset

  @property
  def ECAN(self):
    return self._e

  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    return self._cam


def create_steering_messages(packer, CP, CAN, enabled, lat_active, apply_torque, lkas_icon, vEgoRaw):
  damp_factor_v = [0., 15.6, 17.8, 22.3]
  damp_factor_lkp = [100, 100, 116, 125]

  values = {
    "LKA_OptUsmSta": 2,
    "LKA_SysIndReq": lkas_icon,
    "StrTqReqVal": apply_torque,
    "LKA_SysWrn": 0,
    "ActToiSta": 1 if lat_active else 0,
    "LKA_UsmMod": 0,  # hide LKAS settings
    "LKA_RcgSta": 0,
    "Damping_Gain": int(np.interp(vEgoRaw, damp_factor_v, damp_factor_lkp)),
  }

  ret = []
  if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
    lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
    ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret


def create_suppress_lfa(packer, CAN, lfa_block_msg, lka_steering_alt):
  suppress_msg = "CAM_0x362" if lka_steering_alt else "CAM_0x2a4"
  msg_bytes = 32 if lka_steering_alt else 24

  values = {f"BYTE{i}": lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values["COUNTER"] = lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0
  values["RIGHT_LANE_LINE"] = 0
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)


def create_buttons(packer, CP, CAN, cnt, btn, is_metric:bool = False):
  cruise_btns_msg = "CRUISE_BUTTONS_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else "CRUISE_BUTTONS"
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }
  if (CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
    values.update({
      "SET_ME_2": 6,
      "DISTANCE_UNIT": 0 if is_metric else 1,
    })

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else CAN.CAM
  return packer.make_can_msg(cruise_btns_msg, bus, values)


def create_acc_cancel(packer, CP, CAN, cruise_info_copy):
  # CAN FD camera-based SCC requires additional signals to be preserved
  # verbatim from the previous SCC_CONTROL frame to avoid checksum or
  # state validation faults. Classic CAN SCC only validates a subset.
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "SCC_SysFlrSta",
      "MainMode_ACC",
      "ACCMode",
      "ZEROS_9",
      "CRUISE_STANDSTILL",
      "SCC_HeadwayDstSetVal",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "ACCMode",
      "VSetDis",
      "CRUISE_STANDSTILL",
    ]}
  values.update({
    "ACCMode": 4,
    "aReqRaw": 0.0,
    "aReqValue": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_lfahda_cluster(packer, CAN, enabled, lfa_icon):
  values = {
    "HDA_OptUsmSta": 2 if enabled else 0,
    "HDA_CntrlModSta": 2 if enabled else 1,
    "HDA_LFA_SymSta": lfa_icon,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)


def create_acc_control(packer, CAN, enabled, accel_last, accel, gas_override, set_speed, hud_control,
                       lead_data: CanFdLeadData, main_cruise_enabled, tuning):
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel  # noqa: F841
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)  # noqa: F841

  values = {
    "ACCMode": 0 if not main_cruise_enabled else 4 if not enabled else 2 if gas_override else 1,
    "MainMode_ACC": 1 if main_cruise_enabled else 0,
    "SCC_NSCCOnOffSta": 2 if main_cruise_enabled else 0,
    "SCC_NSCCOpSta": 0 if not enabled else (1 if gas_override else 2),
    "StopReq": 1 if tuning.stopping else 0,
    "aReqValue": tuning.actual_accel,
    "aReqRaw": tuning.actual_accel,
    "VSetDis": set_speed,
    "JerkLowerLimit": tuning.jerk_lower,
    "JerkUpperLimit": tuning.jerk_upper,
    "SCC_AccelLimBandUppVal": tuning.comfort_band_upper,
    "SCC_AccelLimBandLwrVal": tuning.comfort_band_lower,

    "ACC_ObjDist": lead_data.lead_distance,
    "ObjValid": int(not lead_data.lead_visible),
    "SET_ME_3": 0x3,
    "SET_ME_TMP_64": 0x64,
    "ACC_ObjRelSpd": lead_data.lead_rel_speed,
    "SCC_ObjDstLvlVal": lead_data.object_gap,
    "SCC_HeadwayDstSetVal": hud_control.leadDistanceBars,
    "SCC_ObjSta": 0 if not (enabled and lead_data.lead_visible) else 1 if gas_override else 2,
    "SCC_TrgtDstVal": lead_data.target_distance,
  }

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_hda2_cluster(packer, CAN, lfa_icon, left_blinker, right_blinker, hud_control, lead_data: CanFdLeadData, out):
  curvature = int(out.steeringAngleDeg / 2)

  values = {
    "HDA_LCFuncOptUsmSta": 2,
    "HDA_LCFuncSta": 3 if left_blinker or right_blinker else lfa_icon,
    "HDA_LCTurnSigReq": 1 if (left_blinker and not out.leftBlindspot) else 2 if (right_blinker and not out.rightBlindspot) else 0,
    "HDA_PathSta": 4 if (right_blinker and not out.rightBlindspot)
                    else 3 if (left_blinker and not out.leftBlindspot)
                    else 2 if right_blinker else 1 if left_blinker else 0,
    "HDA_LtLCAvailSta": 0 if not left_blinker else 2 if hud_control.leftLaneDepart else 1, # TODO: 2 when lane change in progress
    "HDA_RtLCAvailSta": 0 if not right_blinker else 2 if hud_control.rightLaneDepart else 1, # TODO: 2 when lane change in progress
    "HDA_LtLineLatPos": lead_data.left_laneline,
    "HDA_RtLineLatPos": lead_data.right_laneline,
    "HDA_LaneCvrtLvlVal": min(abs(curvature), 14) + (-1 if curvature < 0 else 0),
    "HDA_LaneCvrtDir": 1 if curvature < 0 else 0,
  }

  return packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values)


def create_spas_messages(packer, CAN, left_blink, right_blink):
  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, values))

  blink = 0
  if left_blink:
    blink = 3
  elif right_blink:
    blink = 4
  values = {
    "BLINKER_CONTROL": blink,
  }
  ret.append(packer.make_can_msg("SPAS2", CAN.ECAN, values))

  return ret


def create_fca_warning_light(packer, CAN, frame):
  ret = []

  if frame % 2 == 0:
    values = {
      'AEB_SETTING': 0x1,  # show AEB disabled icon
      'SET_ME_2': 0x0,
      'SET_ME_FF': 0xff, #match
      'SET_ME_FC': 0xfc, #match
      'SET_ME_9': 0x1,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))
  return ret


_ADAS_DRV_TEMPLATES = {
  0x160: bytes.fromhex("0000000100000000fffc0100a8001000"),
  0x161: bytes.fromhex("000000c000000000c0fff0c003000040000000000000000000ff000000000000"),
  0x162: bytes.fromhex("0000002700000000c0ff00000000000000000000000000000000000000000000"),
  0x1BA: bytes.fromhex("00000000000000880200000000000000000100000000000f"),
  0x1DA: bytes.fromhex("0000002200010000000000000000000000000000000000000000000000000000"),
  0x1E5: bytes.fromhex("00000000000000000000220300000080"),
  0x1EA: bytes.fromhex("000000040000000000000000000000ff000000000000000000000000000f0f00"),
  0x200: bytes.fromhex("00000014801a0000"),
  0x345: bytes.fromhex("0000001500560000"),
  0x38C: bytes.fromhex("000000f79f000000000000000000000000000000000000000000000000000000"),
}

_ADAS_DRV_PERIODS = {
  0x160: 2,
  0x1DA: 100,
  0x1EA: 5,
  0x200: 5,
  0x345: 20,
  0x38C: 20,
}


def _create_adas_drv_spoof(address, bus, counter):
  d = bytearray(_ADAS_DRV_TEMPLATES[address])
  d[2] = counter & 0xFF
  crc = hkg_can_fd_checksum(address, None, d)
  d[0] = crc & 0xFF
  d[1] = (crc >> 8) & 0xFF
  return (address, bytes(d), bus)


def create_adas_drv_messages(packer, CAN, frame):
  ret = [packer.make_can_msg("ADRV_0x51", CAN.ACAN, {})]
  for addr, period in _ADAS_DRV_PERIODS.items():
    if frame % period == 0:
      ret.append(_create_adas_drv_spoof(addr, CAN.ECAN, frame // period))
  if frame % 5 == 0:
    counter = frame // 5
    for addr in (0x161, 0x162, 0x1BA, 0x1E5):
      ret.append(_create_adas_drv_spoof(addr, CAN.ECAN, counter))
  return ret


def hkg_can_fd_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0
  for i in range(2, len(d)):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ d[i]]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 0) & 0xFF)]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 8) & 0xFF)]) & 0xFFFF
  if len(d) == 8:
    crc ^= 0x5F29
  elif len(d) == 16:
    crc ^= 0x041D
  elif len(d) == 24:
    crc ^= 0x819D
  elif len(d) == 32:
    crc ^= 0x9F5B
  return crc
