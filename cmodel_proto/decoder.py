#!/usr/bin/env python3
"""
独立 .cmodel 解码器（不依赖 amr_studio_v4 后端、不绑定 protoc 版本）

.cmodel = ZIP 包，内含 Protobuf 二进制:
  CompDesc.model  -> AMR_MODEL_NSP.Message_Module_Info   (组件/结构/属性，核心)
  AbiSet.model    -> Controller_Ability                   (能力集)
  FuncDesc.model  -> Robot_Description                    (功能描述)

本模块直接加载序列化的 FileDescriptorProto (*.desc，从 amr_studio_v4 生成代码中提取)，
通过 descriptor_pool + message_factory 动态构造消息类，兼容 protobuf 3.12 ~ 6.x。
"""

import io
import json
import os
import zipfile
from typing import Dict, Optional

from google.protobuf import descriptor_pb2, descriptor_pool
from google.protobuf.json_format import MessageToDict

_HERE = os.path.dirname(os.path.abspath(__file__))

# (model 文件名, desc 文件, 完整消息名候选, 输出 json 名)
_MODEL_MAP = [
    ("CompDesc.model", "controller_model_comp_desc.desc", "Message_Module_Info", "CompDesc.json"),
    ("AbiSet.model", "controller_model_abi_set.desc", "Controller_Ability", "AbiSet.json"),
    ("FuncDesc.model", "controller_model_abi_desc.desc", "Robot_Description", "FuncDesc.json"),
]

_pool: Optional[descriptor_pool.DescriptorPool] = None
_files = {}


def _get_pool():
    global _pool
    if _pool is None:
        _pool = descriptor_pool.DescriptorPool()
        for _, desc_name, _, _ in _MODEL_MAP:
            with open(os.path.join(_HERE, desc_name), "rb") as f:
                fdp = descriptor_pb2.FileDescriptorProto.FromString(f.read())
            try:
                _pool.Add(fdp)
            except TypeError:
                # 已存在同名文件(重复加载)时忽略
                pass
            _files[desc_name] = fdp
    return _pool


def _message_class(desc_name: str, short_name: str):
    pool = _get_pool()
    fdp = _files[desc_name]
    full = f"{fdp.package}.{short_name}" if fdp.package else short_name
    descriptor = pool.FindMessageTypeByName(full)
    try:  # protobuf >= 4.21
        from google.protobuf import message_factory
        if hasattr(message_factory, "GetMessageClass"):
            return message_factory.GetMessageClass(descriptor)
        return message_factory.MessageFactory(pool).GetPrototype(descriptor)
    except Exception:
        from google.protobuf import reflection
        return reflection.message_factory.MessageFactory(pool).GetPrototype(descriptor)


def decode_bytes(model_name: str, data: bytes) -> dict:
    for mname, desc_name, msg_name, _ in _MODEL_MAP:
        if mname == model_name:
            cls = _message_class(desc_name, msg_name)
            msg = cls()
            msg.ParseFromString(data)
            return MessageToDict(msg)
    raise KeyError(model_name)


def decode_cmodel_to_dicts(cmodel_path: str) -> Dict[str, dict]:
    """返回 {"CompDesc": {...}, "AbiSet": {...}, "FuncDesc": {...}}（缺失的可选文件不返回）"""
    out = {}
    with zipfile.ZipFile(cmodel_path, "r") as z:
        names = {os.path.basename(n): n for n in z.namelist()}
        for mname, _, _, jname in _MODEL_MAP:
            key = jname.replace(".json", "")
            if mname in names:
                out[key] = decode_bytes(mname, z.read(names[mname]))
            elif jname in names:  # 已解码 JSON 形式的包
                out[key] = json.loads(z.read(names[jname]).decode("utf-8"))
    if "CompDesc" not in out:
        raise FileNotFoundError("cmodel 中缺少 CompDesc.model / CompDesc.json")
    return out


def decode_cmodel(cmodel_path: str, output_dir: str):
    """与 amr_studio_v4 cmodel_decoder.decode_cmodel 相同的接口：写出 *.json"""
    os.makedirs(output_dir, exist_ok=True)
    dicts = decode_cmodel_to_dicts(cmodel_path)
    for k, v in dicts.items():
        with open(os.path.join(output_dir, k + ".json"), "w", encoding="utf-8") as f:
            json.dump(v, f, ensure_ascii=False, indent=2)
    return list(dicts.keys())


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 3:
        print("Usage: python3 -m cmodel_proto.decoder <file.cmodel> <out_dir>")
        sys.exit(1)
    print(decode_cmodel(sys.argv[1], sys.argv[2]))
