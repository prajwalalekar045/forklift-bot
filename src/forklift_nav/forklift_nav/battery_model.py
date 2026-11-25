"""Battery model for the forklift: pure Python, no ROS, so it can be unit tested anywhere.

An 8-cell LiFePO4 pack (24 V class, 1.5 kWh). Power draw is a base load (computer, sensors,
lights) plus a motion term and a fork lift term. State of charge is integrated from that power;
voltage is a linear function of state of charge, which is good enough for a status display.

`time_scale` speeds the drain up for demos: 60 means one real minute drains an hour of energy.
"""
from dataclasses import dataclass


@dataclass
class BatteryParams:
    capacity_wh: float = 1500.0
    v_full: float = 29.2
    v_empty: float = 24.0
    idle_w: float = 60.0              # computer, lidar, camera, lights
    drive_w_per_mps: float = 350.0    # extra power per m/s of body speed (rolling resistance, motors)
    turn_w_per_radps: float = 60.0    # extra power per rad/s of yaw rate
    lift_w: float = 450.0             # hydraulic pump while the fork moves
    charge_w: float = 600.0           # charger output while charging
    low_soc: float = 0.20
    critical_soc: float = 0.10
    time_scale: float = 1.0


class BatteryModel:
    def __init__(self, params=None, soc=0.95):
        self.p = params or BatteryParams()
        self.soc = min(1.0, max(0.0, soc))
        self.power_w = 0.0            # positive = discharging, negative = charging
        self.charging = False

    @property
    def voltage(self):
        return self.p.v_empty + (self.p.v_full - self.p.v_empty) * self.soc

    @property
    def current(self):
        """Amps, ROS convention: negative while discharging, positive while charging."""
        return -self.power_w / self.voltage

    @property
    def level(self):
        """'ok', 'low' or 'critical'."""
        if self.soc <= self.p.critical_soc:
            return 'critical'
        if self.soc <= self.p.low_soc:
            return 'low'
        return 'ok'

    def step(self, dt, speed, yaw_rate, fork_moving, charging=False):
        """Advance by dt seconds of simulated time. speed in m/s, yaw_rate in rad/s."""
        if dt <= 0.0:
            return
        self.charging = bool(charging)
        load = (self.p.idle_w
                + self.p.drive_w_per_mps * abs(speed)
                + self.p.turn_w_per_radps * abs(yaw_rate)
                + (self.p.lift_w if fork_moving else 0.0))
        self.power_w = load - (self.p.charge_w if self.charging else 0.0)
        energy_wh = self.power_w * dt * self.p.time_scale / 3600.0
        self.soc = min(1.0, max(0.0, self.soc - energy_wh / self.p.capacity_wh))
