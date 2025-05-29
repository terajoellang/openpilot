import time

import crcmod

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car.tesla.values import CANBUS, CarControllerParams
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX


class TeslaCAN:
  def __init__(self, packer):
    self.packer = packer
    self.crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)
    self.time_gas_released = time.time()

  @staticmethod
  def checksum(msg_id, dat):
    # TODO: get message ID from name instead
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, angle, enabled, counter):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
      "DAS_steeringControlCounter": counter,
    }

    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[2]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr, v_ego, active):
    set_speed = 0 if accel < 0 else V_CRUISE_MAX
    if not active:
      set_speed = abs(v_ego * CV.MS_TO_KPH)

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[2]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def hybrid_longitudinal(self, acc_state, accel, das_control, cntr, speed, gas_pressed):
    speed = speed * CV.MS_TO_KPH

    time_since_gas_released = time.time() - self.time_gas_released

    # Improve behavior during stop-and-go traffic
    if speed <= 25:
      max_accel = das_control["DAS_accelMax"]
    elif not gas_pressed and 0 < time_since_gas_released < 2:
      factor = time_since_gas_released / 2.0
      max_accel = (1 - factor) * das_control["DAS_accelMax"] + factor * max(accel, 0.4)
    elif gas_pressed:
      max_accel = das_control["DAS_accelMax"]
      self.time_gas_released = time.time()
    elif 25 < speed < 35:
      # Blending from stock ACC to openpilot longitudinal between 25 and 35 km/h
      factor = (speed - 25) / (35 - 25)
      max_accel = (1 - factor) * das_control["DAS_accelMax"] + factor * accel
    else:
      max_accel = max(accel, 0.4)

    if accel < -0.5 and accel > das_control["DAS_accelMin"]:
      min_accel = das_control["DAS_accelMin"]
    elif gas_pressed:
      min_accel = das_control["DAS_accelMin"]
    else:
      min_accel = accel

    max_accel = clip(max_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    min_accel = clip(min_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

    values = {
      "DAS_setSpeed": das_control["DAS_setSpeed"],
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": das_control["DAS_jerkMin"],
      "DAS_jerkMax": das_control["DAS_jerkMax"],
      "DAS_accelMin": min(min_accel, -0.4),
      "DAS_accelMax": max(max_accel, 0),
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[2]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)
