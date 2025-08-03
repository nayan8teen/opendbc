"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass, field

from opendbc.car.hyundai.values import CAR


@dataclass
class CarTuningConfig:
  lookahead_jerk_bp: list[float] = field(default_factory=lambda: [2., 5., 20.])
  lookahead_jerk_upper_v: list[float] = field(default_factory=lambda: [0.3, 0.45, 0.6])
  lookahead_jerk_lower_v: list[float] = field(default_factory=lambda: [0.3, 0.45, 0.6])
  longitudinal_actuator_delay: float = 0.50
  jerk_limits: float = 4.0
  upper_jerk_v: list[float] = field(default_factory=lambda: [3.0, 3.0, 1.6])
  lower_jerk_v: list[float] = field(default_factory=lambda: [5.0, 5.0, 3.0])
  min_jerk_lower: float = 2.0
  min_jerk_upper: float = 0.5


# Default configurations for different car types
TUNING_CONFIGS = {
  "CANFD": CarTuningConfig(),
  "EV": CarTuningConfig(),
  "HYBRID": CarTuningConfig(),
  "DEFAULT": CarTuningConfig(),
}

# Car-specific configs
CAR_SPECIFIC_CONFIGS = {
  CAR.KIA_EV6: CarTuningConfig(
    lookahead_jerk_bp=[2., 5., 20.],
    lookahead_jerk_upper_v=[0.05, 0.05, 0.05],
    lookahead_jerk_lower_v=[0.05, 0.10, 0.3],
    jerk_limits=5.0,
    upper_jerk_v=[5.0, 4.0, 2.5],
    lower_jerk_v=[1.5, 2.0, 3.2],
    min_jerk_upper=2.0,
    min_jerk_lower=1.2,
  ),
  CAR.KIA_NIRO_EV: CarTuningConfig(
    jerk_limits=3.3,
  ),
  CAR.KIA_NIRO_PHEV_2022: CarTuningConfig(
    jerk_limits=5.0,
  ),
}
