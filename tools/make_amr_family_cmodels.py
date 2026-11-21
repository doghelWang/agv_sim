#!/usr/bin/env python3
"""Generate cmodels for the AMR families that have no real cmodel in the repo:

  models/latent_jacking_amr.cmodel  latent (submarine) jacking AMR, modelled on the real 0323/1234 cmodels:
                                    1476 x 1063 x 178 mm differential chassis, lift motor (gear 35, holding brake),
                                    up/down proximity limits, 2 down code readers, WLR-716MINI front laser, 600 kg load
  models/roller_amr.cmodel          roller-conveyor docking AMR: 1200 x 800 differential chassis, roller deck at 0.60 m,
                                    roller motor, tote edge/in-place photoelectrics, 2 safety lasers
  models/tugger_amr.cmodel          tugger AMR: 1100 x 650 x 450 mm, rear tow hook (pin) + hitch-locked sensor, front laser

Like the real cmodels, these only carry *module* information (motors, sensors, IO, limits).  The mechanism geometry
(stroke, tray, rollers, hitch) is NOT part of the cmodel schema; the simulator infers it (see docs/ASSEMBLY.md).
"""
import sys, uuid
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import make_pallet_truck_cmodel as M
from make_pallet_truck_cmodel import S, D, I, B, C, F

def chassis(name, head, tail, w, h, ws, vmax, load, weight, desc):
    return M.comp(name, "chassis", "diffChassis", [
        D("headOffset(Idle)", head * 1000), D("tailOffset(Idle)", tail * 1000), D("leftOffset(Idle)", w / 2 * 1000), D("rightOffset(Idle)", w / 2 * 1000),
        D("headOffset (Full Load)", head * 1000), D("tailOffset (Full Load)", tail * 1000), D("leftOffset (Full Load)", w / 2 * 1000), D("rightOffset (Full Load)", w / 2 * 1000),
        I("wheelsNum", 2), D("wheelSpace", ws * 1000),
        D("maxSpeed(Idle)", vmax * 1000), D("maxAcceleration(Idle)", 500), D("maxDeceleration(Idle)", 500),
        D("maxSpeed (Full Load)", vmax * 700), D("maxAcceleration (Full Load)", 300), D("maxDeceleration (Full Load)", 400),
        D("rotateMaxAngSpeed (Idle)", 90), D("rotateMaxAngAcceleration (Idle)", 90), D("rotateMaxAngDeceleration (Idle)", 90),
        D("rotateMaxAngSpeed (Full Load)", 45), D("rotateMaxAngAcceleration (Full Load)", 45), D("rotateMaxAngDeceleration (Full Load)", 60),
        D("avoidMaxDec (Idle)", 1000), D("avoidMaxDec (Full Load)", 700),
        D("selfWeight", weight), D("totalLoadWeight", load), D("rotateDiameter", max(head + tail, w) * 1000 * 1.05), D("maxClimbingAngle", 3),
    ], box=(head + tail, w, h), src=desc, desc=desc)

def drive(comps, ws, r, gear, x=0.0):
    for k, y in (("left", ws / 2), ("right", -ws / 2)):
        comps.append(M.comp(f"motor-{k}", "driver", "PMSMMotor", [C("ENCType", "ENCODER_INC"), I("RPM", 3000), B("bHbrake", True), D("gearRatio", gear)], loc=(x, y, r)))
        comps.append(M.comp(f"diffWheel-{k}", "driveWheel", "differentialWheel", [D("wheelRadius", r * 1000), F("relateMotor", f"motor-{k}")], loc=(x, y, 0.0), box=(r * 2, 0.05, r * 2)))

def common(comps, h, laser_x, laser_z):
    comps.append(M.comp("laser-front", "sensor", "2DLaser", [C("scanDirect", "SCAN_COUNTERCLOCKWISE"), C("usageMode", "PClOUD_LASER")], loc=(laser_x, 0, laser_z), cyl=(0.051, 0.079), src="WLR-716MINI"))
    comps.append(M.comp("gyro", "sensor", "gyro", [], loc=(0, 0, h * 0.5)))
    comps.append(M.comp("MainController", "mainCPU", "subMainCPU", [C("isWithGyro", "yes")], loc=(0, 0, h * 0.5), box=(0.12, 0.1, 0.04)))
    comps.append(M.comp("button-emc", "button", "subButton", [C("buttonType", "BUTTON_COMP_EMCY"), B("selfLock", True)], loc=(0, 0, h)))
    comps.append(M.comp("battery", "battery", "subBattery", [D("capacity", 60), D("voltage", 48), D("maxDischargeCurrent", 100), D("stdDischargeCurrent", 30), D("maxPower", 4800)], loc=(-0.2, 0, h * 0.4), box=(0.4, 0.4, 0.15)))
    comps.append(M.comp("charger", "extendedlnterface", "comDo", [], loc=(0, 0, 0.05)))

def jacking():
    head = tail = 0.738; w = 1.063; h = 0.178
    c = [chassis("chassis_diff", head, tail, w, h, 0.90, 1.0, 600, 0, "Latent jacking AMR chassis")]
    drive(c, 0.90, 0.065, 25.0)
    c.append(M.comp("motor-lift", "driver", "PMSMMotor", [C("ENCType", "ENCODER_INC"), I("RPM", 3000), B("bHbrake", True), D("gearRatio", 35.0)], loc=(0, 0, 0.1)))
    c.append(M.comp("up_sensor", "sensor", "proximitySensor", [], loc=(0, 0.1, 0.15)))
    c.append(M.comp("down_sensor", "sensor", "proximitySensor", [], loc=(0, -0.1, 0.1)))
    c.append(M.comp("smart camera", "sensor", "codeReader", [C("lensOrientation", "LENS_DIR_DOWN"), D("scanDistence", 100)], loc=(0, 0, 0.145), rpy=(0, 0, 180)))
    c.append(M.comp("smart camera0", "sensor", "codeReader", [C("lensOrientation", "LENS_DIR_UP"), D("scanDistence", 100)], loc=(0, 0, 0.1), rpy=(0, 0, 180)))
    common(c, h, 0.70, 0.08)
    return c

def roller():
    head = tail = 0.60; w = 0.80; h = 0.60
    c = [chassis("chassis_diff", head, tail, w, h, 0.56, 1.2, 300, 260, "Roller docking AMR chassis")]
    drive(c, 0.56, 0.075, 20.0)
    c.append(M.comp("roller_motor", "driver", "BDCMotor", [C("ENCType", "ENCODER_NULL"), I("ratedSpeed", 3000), C("bHbrake", "WITHOUT_HBRAKE"), D("ratedVolt", 24), D("powerVolt", 48)], loc=(0, 0, 0.45)))
    for k, y in (("roller_edge_l", 0.38), ("roller_edge_r", -0.38), ("roller_in_place", 0.0)):
        c.append(M.comp(k, "sensor", "proximitySensor", [], loc=(0, y, 0.62)))
    c.append(M.comp("code_reader_down", "sensor", "codeReader", [C("lensOrientation", "LENS_DIR_DOWN"), D("scanDistence", 100)], loc=(0.0, 0, 0.10)))
    c.append(M.comp("cam_front", "sensor", "camera", [C("lensOrientation", "LENS_DIR_FRONT")], loc=(head - 0.02, 0, 0.45), box=(0.04, 0.06, 0.04), src="RGB camera 640x480"))
    common(c, h, head - 0.02, 0.20)
    return c

def tugger():
    head = 0.55; tail = 0.55; w = 0.65; h = 0.45
    c = [chassis("chassis_diff", head, tail, w, h, 0.50, 1.2, 800, 220, "Tugger AMR chassis")]
    drive(c, 0.50, 0.10, 22.0)
    c.append(M.comp("tow_hook_motor", "driver", "BDCMotor", [C("ENCType", "ENCODER_NULL"), I("ratedSpeed", 2000), C("bHbrake", "WITHOUT_HBRAKE"), D("ratedVolt", 24), D("powerVolt", 48)], loc=(-tail + 0.1, 0, 0.3)))
    c.append(M.comp("hitch_locked", "sensor", "proximitySensor", [], loc=(-tail, 0, 0.3)))
    c.append(M.comp("code_reader_down", "sensor", "codeReader", [C("lensOrientation", "LENS_DIR_DOWN"), D("scanDistence", 100)], loc=(0.0, 0, 0.10)))
    c.append(M.comp("laser-rear", "sensor", "2DLaser", [C("scanDirect", "SCAN_COUNTERCLOCKWISE"), C("usageMode", "PClOUD_LASER")], loc=(-tail + 0.03, 0, 0.18), rpy=(0, 0, 180), cyl=(0.051, 0.079), src="WLR-716MINI"))
    common(c, h, head - 0.03, 0.18)
    return c

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "models"
    for name, fn in (("latent_jacking_amr", jacking), ("roller_amr", roller), ("tugger_amr", tugger)):
        M.UID = uuid.uuid5(uuid.NAMESPACE_DNS, name).hex
        M.comps = fn()
        M.desc = {"moreModuleInfo": [{"moduleGroupName": name, "moduleComponets": M.comps}]}
        M.build(f"{out}/{name}.cmodel")
