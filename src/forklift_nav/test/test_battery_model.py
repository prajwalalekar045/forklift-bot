from forklift_nav.battery_model import BatteryModel, BatteryParams


def test_idle_drain_matches_energy():
    b = BatteryModel(BatteryParams(capacity_wh=1000.0, idle_w=100.0), soc=1.0)
    for _ in range(3600):                  # one hour at 100 W = 100 Wh = 10 % of 1 kWh
        b.step(1.0, 0.0, 0.0, False)
    assert abs(b.soc - 0.90) < 1e-6
    assert abs(b.power_w - 100.0) < 1e-9


def test_motion_and_lift_add_load():
    p = BatteryParams(idle_w=60.0, drive_w_per_mps=350.0, turn_w_per_radps=60.0, lift_w=450.0)
    b = BatteryModel(p, soc=0.5)
    b.step(1.0, 0.4, 0.5, True)
    assert abs(b.power_w - (60 + 140 + 30 + 450)) < 1e-9


def test_voltage_limits_and_current_sign():
    b = BatteryModel(soc=1.0)
    assert abs(b.voltage - b.p.v_full) < 1e-9
    b.soc = 0.0
    assert abs(b.voltage - b.p.v_empty) < 1e-9
    b.step(1.0, 0.0, 0.0, False)
    assert b.current < 0.0                 # discharging


def test_levels():
    b = BatteryModel(soc=0.5)
    assert b.level == 'ok'
    b.soc = 0.15
    assert b.level == 'low'
    b.soc = 0.05
    assert b.level == 'critical'


def test_charging_and_clamping():
    b = BatteryModel(BatteryParams(capacity_wh=100.0, charge_w=600.0), soc=0.99)
    b.step(60.0, 0.0, 0.0, False, charging=True)
    assert b.soc == 1.0 and b.current > 0.0
    b = BatteryModel(BatteryParams(capacity_wh=1.0), soc=0.01)
    b.step(3600.0, 1.0, 0.0, True)
    assert b.soc == 0.0


def test_time_scale_and_bad_dt():
    a = BatteryModel(BatteryParams(time_scale=1.0), soc=1.0)
    c = BatteryModel(BatteryParams(time_scale=60.0), soc=1.0)
    a.step(10.0, 0.0, 0.0, False)
    c.step(10.0, 0.0, 0.0, False)
    assert abs((1 - c.soc) - 60 * (1 - a.soc)) < 1e-12
    before = a.soc
    a.step(-1.0, 0.0, 0.0, False)
    assert a.soc == before
