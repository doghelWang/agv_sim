#!/usr/bin/env python3
"""Generate models/pallet_truck_amr.cmodel — a low-lift pallet-truck AMR ("pallet jack" / 地牛式托盘搬运 AMR, 1.5 t class).

Modelled on the real units in the reference photo and typical products (Hikrobot / Geek+ / Seer / VisionNav pallet-truck AMRs):
  * tall slim housing (0.72 W x 0.85 L x 1.15 H) at the *rear*, two long fixed tines pointing forward (1150 x 140 x 60 mm,
    centre spacing 350 mm = EUR-pallet pocket spacing), no mast: the tines only lift ~0.2 m (just enough to lift the pallet off the floor)
  * differential drive: two Ø250 mm drive wheels under the housing, load wheels in the tine tips (not modelled)
  * slim pole above the housing with beacon + e-stop (1.85 m), touch screen + status light bar on the housing front-top
  * two diagonal 270° safety scanners + low front/back obstacle sensing, two pallet-pocket cameras at the tine root,
    fork-tip photoelectrics, down code reader, gyro, 48 V / 100 Ah Li battery
  * unladen 1.5 m/s, laden 1.0 m/s (laden turning slower), self weight 450 kg, rated load 1500 kg
Written as CompDesc.json inside a ZIP (accepted by the Rust/Python decoders).
"""
import json, sys, zipfile, hashlib, uuid
sys.path.insert(0, __file__.rsplit("/", 1)[0])

UID = uuid.uuid5(uuid.NAMESPACE_DNS, "pallet_truck_amr").hex

def S(key, v, desc=""): return {"key": key, "type": "DATA_STRING", "stringValue": v, "desc": desc, "boolParse": True}
def D(key, v): return {"key": key, "type": "DATA_DOUBLE", "doubleValue": float(v)}
def I(key, v): return {"key": key, "type": "DATA_INT32", "int32Value": int(v)}
def B(key, v): return {"key": key, "type": "DATA_BOOL", "boolValue": bool(v)}
def C(key, v): return {"key": key, "type": "DATA_COMBOX", "comboType": {"typeKey": v}}
def F(key, v): return {"key": key, "type": "DATA_FIXED_E", "stringValue": v}

def comp(name, main, sub, attrs=(), loc=(0, 0, 0), rpy=(0, 0, 0), box=None, cyl=None, src=None, desc=""):
    shape = {"shapeType": "ENUM_BOX", "box": {"sizeLen": box[0] * 1000, "sizeWidth": box[1] * 1000, "sizeHeight": box[2] * 1000}} if box else \
            ({"shapeType": "ENUM_CYLINDER", "cylinder": {"diameter": cyl[0] * 1000, "height": cyl[1] * 1000}} if cyl else {"shapeType": "ENUM_BOX", "box": {}})
    g = {"moduleName": S("module_name", name), "moduleDesc": S("module_desc", desc or name),
         "mainModuleType": C("main_module_type", main), "subModuleType": C("sub_module_type", sub), "moduleShape": shape, "extendParams": [S("module_srcname", src or name)]}
    st = [S("parentNodeUuid", UID)] + [D(k, v) for k, v in zip(("locCoordX", "locCoordY", "locCoordZ"), (loc[0] * 1000, loc[1] * 1000, loc[2] * 1000))] + \
         [D(k, v) for k, v in zip(("locCoordROLL", "locCoordPITCH", "locCoordYAW"), rpy)]
    return {"generalAttr": g, "structParam": {"extendParams": st}, "privateAttr": {"privateAttrs": [{"key": "attrs", "arrayBaseEle": list(attrs)}]}}

W = 0.72; HEAD = 0.40; TAIL = 0.45; H = 1.15        # housing: x in [-0.45, 0.40]
TINE_X = HEAD + 0.03; TINE_L = 1.15                 # tines start right in front of the housing
WS = 0.46; WR = 0.125; WXPOS = -0.05                # drive wheels (track 0.46 m, Ø250 mm)
POLE_H = 1.85

chassis = comp("chassis", "chassis", "diffChassis", [
    D("headOffset(Idle)", HEAD * 1000), D("tailOffset(Idle)", TAIL * 1000), D("leftOffset(Idle)", W / 2 * 1000), D("rightOffset(Idle)", W / 2 * 1000),
    D("headOffset (Full Load)", HEAD * 1000), D("tailOffset (Full Load)", TAIL * 1000), D("leftOffset (Full Load)", W / 2 * 1000), D("rightOffset (Full Load)", W / 2 * 1000),
    I("wheelsNum", 2), D("wheelSpace", WS * 1000),
    D("maxSpeed(Idle)", 1500), D("maxAcceleration(Idle)", 500), D("maxDeceleration(Idle)", 700),
    D("maxSpeed (Full Load)", 1000), D("maxAcceleration (Full Load)", 300), D("maxDeceleration (Full Load)", 500),
    D("rotateMaxAngSpeed (Idle)", 60), D("rotateMaxAngAcceleration (Idle)", 60), D("rotateMaxAngDeceleration (Idle)", 90),
    D("rotateMaxAngSpeed (Full Load)", 35), D("rotateMaxAngAcceleration (Full Load)", 30), D("rotateMaxAngDeceleration (Full Load)", 50),
    D("avoidMaxDec (Idle)", 1200), D("avoidMaxDec (Full Load)", 800),
    D("selfWeight", 450), D("totalLoadWeight", 1500), D("rotateDiameter", 1000), D("maxClimbingAngle", 3),
], box=(HEAD + TAIL, W, H), src="Pallet Truck AMR Chassis (differential)", desc="托盘搬运 AMR 底盘 (差速)")

comps = [chassis]
for k, y in (("left", WS / 2), ("right", -WS / 2)):
    comps.append(comp(f"walk-{k}", "driver", "PMSMMotor", [C("ENCType", "ENCODER_INC"), I("RPM", 3000), B("bHbrake", True), D("gearRatio", 18.0)], loc=(WXPOS, y, 0.15)))
    comps.append(comp(f"wheel_{k}", "driveWheel", "differentialWheel", [D("wheelRadius", WR * 1000), F("relateMotor", f"walk-{k}")], loc=(WXPOS, y, 0.0), box=(WR * 2, 0.07, WR * 2), desc=f"驱动轮 {k}"))

comps.append(comp("lift_motor", "driver", "BDCMotor", [C("ENCType", "ENCODER_NULL"), I("ratedSpeed", 3000), C("bHbrake", "WITH_HBRAKE"), D("ratedVolt", 24), D("powerVolt", 48)], loc=(HEAD - 0.1, 0, 0.2)))
# low-lift tines: stroke 0.20 m (lifts the pallet ~0.1 m off the ground), no mast
comps.append(comp("fork_mast", "lifter", "forkMast", [D("mastMaxHeight", 200), D("liftSpeed", 60), D("liftAccel", 120), D("tineLength", TINE_L * 1000), D("tineWidth", 140),
    D("tineThickness", 60), D("tineSpacing", 330), D("maxLoad", 1500), D("tiltRange", 0), D("loadedSpeedFactor", 0.67)], loc=(TINE_X, 0, 0.0), box=(0.06, 0.60, 0.20), src="Low-lift pallet fork", desc="低位货叉"))
comps.append(comp("lift_encoder", "sensor", "pullWireEncode", [D("wheelCirc", 230), C("absEncodeType", "CANOPEN_ENCODER")], loc=(HEAD - 0.1, 0, 0.3)))

comps.append(comp("lidar3d", "sensor", "3DLaser", [C("scanDirect", "SCAN_COUNTERCLOCKWISE"), C("usageMode", "PClOUD_LASER")], loc=(-0.30, 0, POLE_H + 0.03), cyl=(0.065, 0.06), src="Livox Mid-360S"))
comps.append(comp("safety_laser_fl", "sensor", "safetyLaser", [C("scanDirect", "SCAN_COUNTERCLOCKWISE"), C("usageMode", "SAFE_LASER")], loc=(HEAD - 0.02, W / 2 - 0.02, 0.15), rpy=(0, 0, 45), cyl=(0.1, 0.1), src="MR-LS-07H"))
comps.append(comp("safety_laser_rr", "sensor", "safetyLaser", [C("scanDirect", "SCAN_COUNTERCLOCKWISE"), C("usageMode", "SAFE_LASER")], loc=(-TAIL + 0.02, -(W / 2 - 0.02), 0.15), rpy=(0, 0, -135), cyl=(0.1, 0.1), src="MR-LS-07H"))
for side, y in (("left", 0.2), ("right", -0.2)):
    comps.append(comp(f"cam_{side}", "sensor", "camera", [C("lensOrientation", "LENS_DIR_FRONT")], loc=(HEAD - 0.02, y, 0.30), rpy=(0, 10, 0), box=(0.04, 0.06, 0.04), src="RGB camera 640x480"))
comps.append(comp("code_reader_down", "sensor", "codeReader", [C("lensOrientation", "LENS_DIR_DOWN"), D("scanDistence", 100)], loc=(0.0, 0, 0.10), box=(0.05, 0.05, 0.05)))
comps.append(comp("pallet_tip_l", "sensor", "proximitySensor", [], loc=(TINE_X + TINE_L, 0.165, 0.04), box=(0.03, 0.03, 0.03)))
comps.append(comp("pallet_tip_r", "sensor", "proximitySensor", [], loc=(TINE_X + TINE_L, -0.165, 0.04), box=(0.03, 0.03, 0.03)))
comps.append(comp("gyro", "sensor", "gyro", [], loc=(0, 0, 0.3)))
comps.append(comp("MainController", "mainCPU", "subMainCPU", [C("isWithGyro", "yes")], loc=(0, 0, 0.35), box=(0.12, 0.1, 0.04)))
comps.append(comp("IO module", "extendedlnterface", "IOModule", [S("chipPlatform", "R106")], loc=(0, 0, 0.4)))
comps.append(comp("button", "button", "subButton", [C("buttonType", "BUTTON_COMP_EMCY"), B("selfLock", True)], loc=(-0.30, 0, POLE_H - 0.02)))
comps.append(comp("battery", "battery", "subBattery", [D("capacity", 100), D("voltage", 48), D("maxDischargeCurrent", 150), D("stdDischargeCurrent", 50), D("maxPower", 7200)], loc=(-0.15, 0, 0.30), box=(0.4, 0.5, 0.3)))

desc = {"moreModuleInfo": [{"moduleGroupName": "pallet_truck", "moduleComponets": comps}]}

def build(path):
    cd = json.dumps(desc, ensure_ascii=False, indent=1).encode()
    mf = {"ModelFileDesc": [{"md5": hashlib.md5(cd).hexdigest(), "name": "CompDesc.json", "type": "MODEL_COMP", "version": "1.0"}]}
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("CompDesc.json", cd)
        z.writestr("ModelFileDesc.json", json.dumps(mf, indent=2))
    print("wrote", path, len(comps), "components")

if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else "models/pallet_truck_amr.cmodel")
