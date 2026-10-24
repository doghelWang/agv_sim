#!/usr/bin/env python3
"""Generate models/forklift_4steer.cmodel — a 4-steer-wheel counterbalance forklift AMR.

Chassis 1.7 x 0.9 m, four independent steer+drive wheels (holonomic / crab / spin), mast lifter (3 m) with two 1.1 m pallet
tines, Mid-360 3D lidar on the overhead guard, two corner safety lasers, dual front cameras, down-looking code reader, pallet
tip photoelectrics, gyro, battery, e-stop, IO module.  Written as CompDesc.json inside a ZIP (accepted by the Rust/Python decoders).
"""
import json, sys, zipfile, hashlib, uuid

UID = uuid.uuid5(uuid.NAMESPACE_DNS, "forklift_4steer").hex

def S(key, v, desc=""): return {"key": key, "type": "DATA_STRING", "stringValue": v, "desc": desc, "boolParse": True}
def D(key, v): return {"key": key, "type": "DATA_DOUBLE", "doubleValue": float(v)}
def I(key, v): return {"key": key, "type": "DATA_INT32", "int32Value": int(v)}
def B(key, v): return {"key": key, "type": "DATA_BOOL", "boolValue": bool(v)}
def C(key, v): return {"key": key, "type": "DATA_COMBOX", "comboType": {"typeKey": v}}
def F(key, v): return {"key": key, "type": "DATA_FIXED_E", "stringValue": v}

def comp(name, main, sub, attrs=(), loc=(0, 0, 0), rpy=(0, 0, 0), box=None, cyl=None, src=None, desc=""):
    """loc in metres, rpy in degrees (roll, pitch, yaw)."""
    shape = {"shapeType": "ENUM_BOX", "box": {"sizeLen": box[0] * 1000, "sizeWidth": box[1] * 1000, "sizeHeight": box[2] * 1000}} if box else \
            ({"shapeType": "ENUM_CYLINDER", "cylinder": {"diameter": cyl[0] * 1000, "height": cyl[1] * 1000}} if cyl else {"shapeType": "ENUM_BOX", "box": {}})
    ext = [S("module_srcname", src or name)]
    g = {"moduleName": S("module_name", name), "moduleDesc": S("module_desc", desc or name),
         "mainModuleType": C("main_module_type", main), "subModuleType": C("sub_module_type", sub), "moduleShape": shape, "extendParams": ext}
    st = [S("parentNodeUuid", UID)] + [D(k, v) for k, v in zip(("locCoordX", "locCoordY", "locCoordZ"), (loc[0] * 1000, loc[1] * 1000, loc[2] * 1000))] + \
         [D(k, v) for k, v in zip(("locCoordROLL", "locCoordPITCH", "locCoordYAW"), rpy)]
    return {"generalAttr": g, "structParam": {"extendParams": st}, "privateAttr": {"privateAttrs": [{"key": "attrs", "arrayBaseEle": list(attrs)}]}}

L = 1.7; W = 0.9; HEAD = 1.0; TAIL = 0.7
WX = {"fl": (0.62, 0.36), "fr": (0.62, -0.36), "rl": (-0.42, 0.36), "rr": (-0.42, -0.36)}

chassis = comp("chassis", "chassis", "steerChassis", [
    D("headOffset(Idle)", HEAD * 1000), D("tailOffset(Idle)", TAIL * 1000), D("leftOffset(Idle)", W / 2 * 1000), D("rightOffset(Idle)", W / 2 * 1000),
    D("headOffset (Full Load)", HEAD * 1000), D("tailOffset (Full Load)", TAIL * 1000), D("leftOffset (Full Load)", W / 2 * 1000), D("rightOffset (Full Load)", W / 2 * 1000),
    I("wheelsNum", 4),
    D("maxSpeed(Idle)", 1500), D("maxAcceleration(Idle)", 600), D("maxDeceleration(Idle)", 800),
    D("maxSpeed (Full Load)", 900), D("maxAcceleration (Full Load)", 350), D("maxDeceleration (Full Load)", 500),
    D("rotateMaxAngSpeed (Idle)", 45), D("rotateMaxAngAcceleration (Idle)", 60), D("rotateMaxAngDeceleration (Idle)", 90),
    D("rotateMaxAngSpeed (Full Load)", 30), D("rotateMaxAngAcceleration (Full Load)", 40), D("rotateMaxAngDeceleration (Full Load)", 60),
    D("avoidMaxDec (Idle)", 1200), D("avoidMaxDec (Full Load)", 800),
    D("selfWeight", 1800), D("totalLoadWeight", 1000), D("rotateDiameter", 2100), D("maxClimbingAngle", 3),
], box=(L, W, 2.0), src="4-Steer Forklift Chassis", desc="四舵轮叉车底盘")

comps = [chassis]
for k, (x, y) in WX.items():
    comps.append(comp(f"walk-{k}", "driver", "PMSMMotor", [C("ENCType", "ENCODER_INC"), I("RPM", 2560), B("bHbrake", True), D("gearRatio", 22.07)], loc=(x, y, 0.1)))
    comps.append(comp(f"steer-{k}", "driver", "PMSMMotor", [C("ENCType", "ENCODER_ABS"), I("RPM", 3000), B("bHbrake", False), D("gearRatio", 80.0)], loc=(x, y, 0.2)))
    comps.append(comp(f"wheel_{k}", "driveWheel", "verticalSteerWheel", [D("wheelRadius", 130), D("angleLmtPos", 120), D("angleLmtNeg", -120), D("rotOmgLmt", 120),
        C("angleSensorType", "GROUP_CALI_ABS_INTERNAL"), F("relateWalkMotor", f"walk-{k}"), F("relateRotMotor", f"steer-{k}")], loc=(x, y, 0.0), box=(0.26, 0.12, 0.26), desc=f"舵轮 {k}"))

comps.append(comp("lift_motor", "driver", "BDCMotor", [C("ENCType", "ENCODER_NULL"), I("ratedSpeed", 3000), C("bHbrake", "WITH_HBRAKE"), D("ratedVolt", 24), D("powerVolt", 48)], loc=(1.0, 0, 0.3)))
comps.append(comp("fork_mast", "lifter", "forkMast", [D("mastMaxHeight", 3000), D("liftSpeed", 300), D("liftAccel", 400), D("tineLength", 1100), D("tineWidth", 120),
    D("tineThickness", 60), D("tineSpacing", 350), D("maxLoad", 1000), D("tiltRange", 0), D("loadedSpeedFactor", 0.6)], loc=(HEAD + 0.15, 0, 0.0), box=(0.12, 0.8, 3.0), src="Mast + pallet fork", desc="门架+货叉"))
comps.append(comp("lift_encoder", "sensor", "pullWireEncode", [D("wheelCirc", 230), C("absEncodeType", "CANOPEN_ENCODER")], loc=(1.0, 0, 1.0)))

comps.append(comp("lidar3d", "sensor", "3DLaser", [C("scanDirect", "SCAN_COUNTERCLOCKWISE"), C("usageMode", "PClOUD_LASER")], loc=(0.35, 0, 2.12), cyl=(0.065, 0.06), src="Livox Mid-360S"))
comps.append(comp("safety_laser_fl", "sensor", "safetyLaser", [C("scanDirect", "SCAN_COUNTERCLOCKWISE"), C("usageMode", "SAFE_LASER")], loc=(0.92, 0.47, 0.32), rpy=(0, 0, 45), cyl=(0.1, 0.1), src="MR-LS-07H"))
comps.append(comp("safety_laser_rr", "sensor", "safetyLaser", [C("scanDirect", "SCAN_COUNTERCLOCKWISE"), C("usageMode", "SAFE_LASER")], loc=(-0.62, -0.47, 0.32), rpy=(0, 0, -135), cyl=(0.1, 0.1), src="MR-LS-07H"))
for side, y in (("left", 0.12), ("right", -0.12)):
    comps.append(comp(f"cam_{side}", "sensor", "camera", [C("lensOrientation", "LENS_DIR_FRONT")], loc=(0.95, y, 1.95), rpy=(0, 20, 0), box=(0.04, 0.06, 0.04), src="RGB camera 640x480"))
comps.append(comp("code_reader_down", "sensor", "codeReader", [C("lensOrientation", "LENS_DIR_DOWN"), D("scanDistence", 120)], loc=(0.3, 0, 0.18), box=(0.05, 0.05, 0.05)))
comps.append(comp("pallet_tip_l", "sensor", "proximitySensor", [], loc=(HEAD + 0.15 + 1.1, 0.175, 0.08), box=(0.03, 0.03, 0.03)))
comps.append(comp("pallet_tip_r", "sensor", "proximitySensor", [], loc=(HEAD + 0.15 + 1.1, -0.175, 0.08), box=(0.03, 0.03, 0.03)))
comps.append(comp("gyro", "sensor", "gyro", [], loc=(0, 0, 0.3)))
comps.append(comp("MainController", "mainCPU", "subMainCPU", [C("isWithGyro", "yes")], loc=(0, 0, 0.35), box=(0.12, 0.1, 0.04)))
comps.append(comp("IO module", "extendedlnterface", "IOModule", [S("chipPlatform", "R106")], loc=(0, 0, 0.4)))
comps.append(comp("button", "button", "subButton", [C("buttonType", "BUTTON_COMP_EMCY"), B("selfLock", True)], loc=(-0.5, 0.3, 1.2)))
comps.append(comp("battery", "battery", "subBattery", [D("capacity", 200), D("voltage", 48), D("maxDischargeCurrent", 300), D("stdDischargeCurrent", 120), D("maxPower", 14400)], loc=(-0.3, 0, 0.3), box=(0.6, 0.5, 0.4)))

desc = {"moreModuleInfo": [{"moduleGroupName": "forklift", "moduleComponets": comps}]}

def build(path):
    cd = json.dumps(desc, ensure_ascii=False, indent=1).encode()
    mf = {"ModelFileDesc": [{"md5": hashlib.md5(cd).hexdigest(), "name": "CompDesc.json", "type": "MODEL_COMP", "version": "1.0"}]}
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("CompDesc.json", cd)
        z.writestr("ModelFileDesc.json", json.dumps(mf, indent=2))
    print("wrote", path, len(comps), "components")

if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else "models/forklift_4steer.cmodel")
