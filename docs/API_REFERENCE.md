# 接口一览 (自动生成)

> 由 `tools/gen_api_docs.py` 从各服务的 OpenAPI 3.1 文档 (`docs/openapi/*.json`) 生成，请勿手改。
> 约定与规范依据见 [API.md](API.md)；运行中的服务可直接访问 `GET <base>/openapi.json`，平台网页「接口文档」可浏览。

## 资源平台 agv-hub

计算节点、车辆模型、仿真场景、软件程序包、仿真实例部署与记录、运维管理。接口在 /api/hub 下 (v1 契约)；/inst/<实例>/ 反向代理到实例的 Web 网关

OpenAPI: `docs/openapi/hub.json` · 契约版本 1.0.0

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/v1` | 列出全部接口 (响应头 Link 指向 OpenAPI 文档，RFC 8631) |
| GET | `/api/v1/openapi.json` | 本服务的 OpenAPI 3.1 文档 (由路由表生成) |
| GET | `/api/hub/health` | 健康检查 |
| GET | `/api/hub/overview` | 资源总览 |
| GET | `/api/hub/nodes` | 计算节点 |
| GET | `/api/hub/nodes/{nid}` | 节点详情 |
| PATCH | `/api/hub/nodes/{nid}` | 修改节点 {name, kind: controller\|hybrid\|sim, note, max_instances} |
| DELETE | `/api/hub/nodes/{nid}` | 移除节点 |
| POST | `/api/hub/nodes/enroll` | 生成节点接入令牌与命令 |
| POST | `/api/hub/nodes/add` | 添加计算节点 {kind,note,name,ip,ssh_port,username,password}: 验证 SSH 后保存 (首次部署时安装运行环境) |
| POST | `/api/hub/nodes/register` | 节点代理注册 {name, token, node_id, node_key, api_port, advertise_host, kind, hub_url, info} (节点代理调用) |
| POST | `/api/hub/nodes/{nid}/heartbeat` | 节点心跳 (请求体为节点信息 info；请求头 X-Node-Key) |
| GET | `/api/hub/nodes/{nid}/jobs/{jid}` | 节点任务进度 |
| POST | `/api/hub/nodes/probe` | 探测两节点之间的连通性 {from,to} |
| GET | `/api/hub/apidocs` | 接口文档目录 |
| GET | `/api/hub/apidocs/{svc}` | 某个服务的 OpenAPI 3.1 文档 (svc: hub\|agent\|sim\|nav\|gateway)；服务在运行时取实时文档，否则取仓库里的离线快照 |
| GET | `/.well-known/api-catalog` | API 目录 (RFC 9727) |
| GET | `/api/hub/admin/state` | 运维登录状态 |
| POST | `/api/hub/admin/setup` | 首次设置管理员密码 |
| POST | `/api/hub/admin/login` | 管理员登录 |
| POST | `/api/hub/admin/logout` | 退出登录 |
| POST | `/api/hub/admin/password` | 修改管理员密码 |
| GET | `/api/hub/admin/services` | 服务状态 |
| POST | `/api/hub/admin/restart` | 重启 {target: hub\|agent} |
| GET | `/api/hub/admin/logs` | 日志文件列表 |
| GET | `/api/hub/admin/logs/{name}` | 下载日志 |
| GET | `/api/hub/admin/updates` | 更新包列表 |
| POST | `/api/hub/admin/updates/upload` | 上传更新包 (请求体为 .tgz) |
| GET | `/api/hub/admin/updates/{uid}` | 更新包详情与预览 |
| DELETE | `/api/hub/admin/updates/{uid}` | 删除更新包 |
| POST | `/api/hub/admin/updates/{uid}/apply` | 应用更新 {restart_instances} |
| POST | `/api/hub/admin/updates/{uid}/rollback` | 回滚最近一次更新 {restart_instances} |
| GET | `/api/hub/admin/jobs` | 运维任务 |
| GET | `/api/hub/admin/jobs/{jid}` | 运维任务详情 |
| GET | `/api/hub/admin/apps` | App 列表与安装状态 |
| POST | `/api/hub/admin/apps/upload` | 上传 APK (请求体为文件) |
| DELETE | `/api/hub/admin/apps/{aid}` | 删除 APK |
| POST | `/api/hub/admin/apps/{aid}/install` | 安装 APK {mode: prompt\|adb, serial} |
| POST | `/api/hub/admin/adb/{action}` | 手机无线调试 adb (action: pair\|connect\|disconnect\|mdns) {port, code, serial} |
| POST | `/api/hub/admin/panel` | 外屏面板 open\|close |
| GET | `/api/hub/models` | 车辆模型 |
| GET | `/api/hub/models/{mid}` | 模型详情 (含生效参数) |
| PATCH | `/api/hub/models/{mid}` | 修改模型信息 {name, project, vtype, material_no, note} |
| DELETE | `/api/hub/models/{mid}` | 删除模型 |
| POST | `/api/hub/models/upload` | 上传 .cmodel (请求体为文件内容，?filename=&name=&project=&vtype=&material_no=) |
| GET | `/api/hub/models/{mid}/cmodel` | 下载 cmodel |
| GET | `/api/hub/models/{mid}/versions/{ver}/bundle` | 实例启动用模型包 (基线+补全) |
| GET | `/api/hub/models/{mid}/versions/{ver}/api/v1/model/editor` | 补全编辑器数据 |
| POST | `/api/hub/models/{mid}/versions/{ver}/api/v1/model/preview` | 试算补全 |
| PUT | `/api/hub/models/{mid}/versions/{ver}/api/v1/model/overrides` | 保存补全 |
| GET | `/api/hub/models/{mid}/versions/{ver}/api/v1/model/sensor_template` | 传感器模板 |
| GET | `/api/hub/models/{mid}/versions/{ver}/api/v1/model/urdf` | URDF |
| GET | `/api/hub/models/{mid}/versions/{ver}/api/v1/model` | 生效参数 |
| GET | `/api/hub/models/{mid}/versions/{ver}/api/v1/sensors/cameras` | 离线编辑无相机画面 |
| GET | `/api/hub/scenes` | 仿真场景 |
| GET | `/api/hub/scenes/{sid}` | 场景详情 (含定义) |
| PATCH | `/api/hub/scenes/{sid}` | 修改场景信息 {name, description, tags} |
| DELETE | `/api/hub/scenes/{sid}` | 删除场景 |
| POST | `/api/hub/scenes/upload` | 上传场景包 (.zip / scene.json) |
| GET | `/api/hub/scenes/{sid}/definition` | 场景定义 JSON |
| GET | `/api/hub/scenes/{sid}/map` | 栅格地图 ?part=pgm\|yaml |
| GET | `/api/hub/scenes/{sid}/package` | 下载场景包 |
| GET | `/api/hub/scenes/{sid}/taskflows` | 场景任务流 |
| PUT | `/api/hub/scenes/{sid}/taskflows` | 保存任务流 |
| GET | `/api/hub/packages` | 软件程序包 |
| PATCH | `/api/hub/packages/{pid}` | 修改程序包 {version, note, tag} |
| DELETE | `/api/hub/packages/{pid}` | 删除程序包 |
| GET | `/api/hub/packages/{pid}/image` | 下载镜像包 (节点代理导入用) |
| POST | `/api/hub/packages/from_node` | 节点镜像入库 (导出上传到平台) {node_id, ref, kind, version, note} |
| POST | `/api/hub/deployments/check` | 部署前校验 {sim_node, nav_node, model_id, model_ver, scene_id, sim_pkg, nav_pkg} |
| GET | `/api/hub/instances` | 仿真实例 |
| POST | `/api/hub/instances` | 部署新实例 {name, sim_node, nav_node, model_id, model_ver, scene_id, sim_pkg, nav_pkg, options} (先用 /deployments/check 校验) |
| POST | `/api/hub/instances/attach` | 接入外部实例 {name, web_url} |
| GET | `/api/hub/instances/{iid}` | 实例详情 |
| DELETE | `/api/hub/instances/{iid}` | 删除实例记录 |
| POST | `/api/hub/instances/{iid}/stop` | 终止仿真 |
| POST | `/api/hub/instances/{iid}/restart` | 重启实例 |
| GET | `/api/hub/instances/{iid}/logs` | 容器日志 |
| GET | `/api/hub/instances/{iid}/logs/file` | 下载实例日志文件 ?svc=sim\|nav&tail=20000 |
| POST | `/api/hub/records` | 归档仿真记录 (sim_bundle) |
| GET | `/api/hub/records` | 仿真记录 |
| GET | `/api/hub/records/{rid}` | 下载记录 sim_bundle |
| DELETE | `/api/hub/records/{rid}` | 删除记录 |

## 节点代理 agv-agent

在设备上分配端口、导入镜像、启停实例进程/容器、读取日志。除 health / openapi 外都要请求头 X-Node-Key

OpenAPI: `docs/openapi/agent.json` · 契约版本 1.0.0

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/v1` | 列出全部接口 (响应头 Link 指向 OpenAPI 文档，RFC 8631) |
| GET | `/api/v1/openapi.json` | 本服务的 OpenAPI 3.1 文档 (由路由表生成) |
| GET | `/api/v1/health` | 健康检查 |
| GET | `/api/v1/info` | 节点信息 |
| POST | `/api/v1/ports/allocate` | 分配端口 {instance, names[]} |
| POST | `/api/v1/ports/release` | 释放端口 |
| POST | `/api/v1/images/ensure` | 确保镜像存在 (缺失时从平台下载并导入) |
| POST | `/api/v1/images/export` | 导出本机镜像并上传到平台 |
| GET | `/api/v1/jobs/{jid}` | 任务进度 |
| GET | `/api/v1/containers` | 托管容器 |
| POST | `/api/v1/containers/run` | 启动容器 {name,image,role,instance,env} |
| GET | `/api/v1/containers/{name}` | 容器状态 |
| DELETE | `/api/v1/containers/{name}` | 删除 |
| POST | `/api/v1/containers/{name}/stop` | 停止 |
| GET | `/api/v1/containers/{name}/logs` | 日志 |
| POST | `/api/v1/probe` | 探测 URL 连通性 {url} |
| POST | `/api/v1/admin/restart` | 重启节点代理 (平台运维) |
| POST | `/api/v1/admin/role` | 切换节点角色 {role: monitor\|full} (平台部署前启用运行环境) |
| DELETE | `/api/v1/instances/{iid}` | 释放实例资源 |

## 仿真进程 sim_server

MuJoCo 物理仿真: 车型、场景、障碍物、传感器 (激光/相机/IMU/光电/触边/读码)、IO、速度指令、推送流

OpenAPI: `docs/openapi/sim.json` · 契约版本 1.0.0

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/v1` | 列出全部接口 (响应头 Link 指向 OpenAPI 文档，RFC 8631) |
| GET | `/api/v1/openapi.json` | 本服务的 OpenAPI 3.1 文档 (由路由表生成) |
| GET | `/api/v1/health` | 健康检查 |
| GET | `/api/v1/model` | 机器人模型 |
| GET | `/api/v1/model/urdf` | URDF |
| GET | `/api/v1/model/editor` | 补全编辑器数据: 完整度审计/传感器清单/当前覆盖 |
| GET | `/api/v1/model/audit` | 完整度审计 (参数来源/缺失) |
| GET | `/api/v1/model/overrides` | 人工补全覆盖 (model_overrides.json) |
| PUT | `/api/v1/model/overrides` | 保存并应用人工补全 (重建模型/URDF/仿真) |
| POST | `/api/v1/model/preview` | 试算补全结果 (不保存): spec/审计/URDF |
| GET | `/api/v1/model/sensor_template` | 新增传感器默认参数 ?type=lidar2d\|lidar3d\|camera\|stereo\|tof\|photoelectric\|codeReader |
| PUT | `/api/v1/model/chassis` | 切换车型 |
| POST | `/api/v1/model/reload` | 重新解析 cmodel 并重建仿真 |
| GET | `/api/v1/world` | 场景 |
| PUT | `/api/v1/world/scenario` | 切换场景 |
| GET | `/api/v1/world/obstacles` | 动态障碍物 |
| PUT | `/api/v1/world/obstacles` | 设置动态障碍物 (请求体为数组，元素 {x, y, w, h, z, yaw, motion}) |
| POST | `/api/v1/scene/load` | 加载场景定义 {scene} 或平台场景 {hub_scene_id} |
| POST | `/api/v1/model/load` | 从平台加载模型 {model_id, version} |
| GET | `/api/v1/instance` | 实例信息 (平台模型/场景) |
| GET | `/api/v1/world/map` | Nav2 栅格地图 ?part=pgm\|yaml&res=0.05 |
| GET | `/api/v1/sim` | 仿真状态/性能 |
| PUT | `/api/v1/sim` | 暂停/实时因子 |
| POST | `/api/v1/sim/reset` | 位姿复位 |
| GET | `/api/v1/state` | 真值/里程计/关节/碰撞 |
| GET | `/api/v1/sensors` | 传感器清单 |
| GET | `/api/v1/sensors/lidars/{name}` | 激光帧 (长轮询/二进制) |
| GET | `/api/v1/sensors/scan` | 融合扫描 |
| PUT | `/api/v1/sensors/lidar_config` | 激光配置 {beams, range_max, freq_hz, sensor_resolution_deg} |
| GET | `/api/v1/sensors/cameras` | 相机清单 (内参/流/帧序号) |
| GET | `/api/v1/sensors/cameras/{name}` | 相机帧 ?stream=rgb\|left\|right\|depth\|amplitude\|points&format=jpeg\|png\|raw |
| GET | `/api/v1/sensors/imu` | IMU |
| GET | `/api/v1/sensors/codes` | 读码/工位标签 |
| GET | `/api/v1/sensors/bumpers` | 防撞触边 (碰撞条) |
| GET | `/api/v1/sensors/photoelectric` | 光电传感器 |
| GET | `/api/v1/io` | IO 状态 |
| PUT | `/api/v1/io` | 批量写 IO |
| PUT | `/api/v1/io/{kind}/{name}` | 写单个 IO |
| GET | `/api/v1/control` | 当前指令/看门狗 |
| PUT | `/api/v1/control/cmd_vel` | 速度指令 (执行进程 → 仿真) |
| GET | `/api/v1/nav/feedback` | 导航回馈 |
| PUT | `/api/v1/nav/feedback` | 执行进程回馈导航状态 (请求体为导航回馈对象，字段同 GET /api/v1/nav/feedback) |
| GET | `/api/v1/events` | 仿真事件 |
| GET | `/api/v1/snapshot` | Web 汇总快照 |

## 执行进程 nav_runtime

定位 (SLAM)、拓扑路径规划、导航任务、任务流、安全防护、遥控

OpenAPI: `docs/openapi/nav.json` · 契约版本 1.0.0

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/v1` | 列出全部接口 (响应头 Link 指向 OpenAPI 文档，RFC 8631) |
| GET | `/api/v1/openapi.json` | 本服务的 OpenAPI 3.1 文档 (由路由表生成) |
| GET | `/api/v1/health` | 健康检查 |
| GET | `/api/v1/nav` | 执行状态 |
| PUT | `/api/v1/nav/planner` | 切换规划器 |
| GET | `/api/v1/missions` | 最近任务 |
| POST | `/api/v1/missions` | 下发任务 |
| GET | `/api/v1/missions/current` | 当前任务 |
| DELETE | `/api/v1/missions/current` | 取消当前任务 |
| GET | `/api/v1/missions/{id}` | 任务详情 |
| DELETE | `/api/v1/missions/{id}` | 取消任务 |
| POST | `/api/v1/teleop` | 手动遥控 |
| POST | `/api/v1/taskflows/run` | 执行任务流 {flow:{id,name,tid,loop,steps:[{type,target,speed,seconds}]}} |
| POST | `/api/v1/taskflows/stop` | 终止任务流 |
| GET | `/api/v1/taskflows/current` | 任务流进度 |
| POST | `/api/v1/plan` | 路线预览 (拓扑规划，不下发) |
| GET | `/api/v1/safety/params` | 保护空间 / 避障参数 |
| PUT | `/api/v1/safety/params` | 运行时调整保护空间 {protection:{...}} (兼容旧字段) |
| GET | `/api/v1/slam` | 定位状态: 模式/位姿/协方差/匹配得分/相对真值误差 |
| GET | `/api/v1/slam/map` | SLAM 地图 ?part=grid(默认, base64 0未知/1空闲/2占据)\|pgm\|yaml &step=N 下采样 |
| POST | `/api/v1/slam/mode` | 切换定位模式 {mode: slam\|localization\|odom\|ground_truth} |
| POST | `/api/v1/slam/save` | 保存当前场景的 SLAM 地图 (之后该场景自动进入定位模式) |
| POST | `/api/v1/slam/reset` | 清空 SLAM 地图重新建图 {delete_saved: bool} |
| POST | `/api/v1/slam/initialpose` | 设置初始位姿 {x, y, yaw} |
| GET | `/api/v1/events` | 执行事件 |
| GET | `/api/v1/ros` | ROS 图 |

## 设备端 Web 网关 v2

仿真实例的工作台接口: 控制权锁、任务流、注入、记录、回放 (经平台 /inst/<实例>/api/v2 代理访问)

OpenAPI: `docs/openapi/gateway.json` · 契约版本 1.0.0

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/v2` | 列出全部接口 (响应头 Link 指向 OpenAPI 文档，RFC 8631) |
| GET | `/api/v2/openapi.json` | 本服务的 OpenAPI 3.1 文档 (由路由表生成) |
| GET | `/api/v2/info` | 实例信息 |
| GET | `/api/v2/brief` | 轻量状态 |
| GET | `/api/v2/lock` | 控制权状态 |
| POST | `/api/v2/lock` | 获取控制权 {user, force} |
| PUT | `/api/v2/lock` | 续约 |
| DELETE | `/api/v2/lock` | 释放控制权 |
| GET | `/api/v2/events` | 带标签事件流 |
| POST | `/api/v2/pause` | 暂停/恢复 |
| POST | `/api/v2/reset` | 重置环境 |
| POST | `/api/v2/taskflow/run` | 执行任务流 |
| POST | `/api/v2/taskflow/stop` | 终止任务流 |
| GET | `/api/v2/inject/catalog` | 注入库 |
| POST | `/api/v2/inject` | 注入元素 {type, placement, x, y, w, h, z, motion, force} (类型见 GET /api/v2/inject/catalog) |
| DELETE | `/api/v2/inject/{oid}` | 移除注入元素 |
| GET | `/api/v2/records` | 仿真记录 |
| GET | `/api/v2/records/{rid}/bundle` | 下载 sim_bundle |
| GET | `/api/v2/records/bundle_all` | 打包下载全部记录 |
| POST | `/api/v2/records/archive` | 归档全部记录到平台 |
| POST | `/api/v2/recording` | 开关录制 |
| GET | `/api/v2/logs` | 下载仿真日志 RCD/EVT |
| GET | `/api/v2/safety` | 避障参数 |
| PUT | `/api/v2/safety` | 修改避障参数 {protection: {...}} |
| POST | `/api/v2/plan` | 路线预览 (执行进程规划器) {points: [{x, y}], obstacles} |
| GET | `/api/v2/slam` | 定位状态 |
| GET | `/api/v2/slam/map` | SLAM 地图 ?part=grid\|pgm\|yaml |
| POST | `/api/v2/slam/mode` | 切换定位模式 {mode: slam\|localization\|odom\|ground_truth} |
| POST | `/api/v2/slam/save` | 保存当前场景的 SLAM 地图 |
| POST | `/api/v2/slam/reset` | 重置定位/建图 |
| POST | `/api/v2/slam/initialpose` | 设置初始位姿 {x, y, yaw} |
| PUT | `/api/v2/planner` | 切换规划器 |
| PUT | `/api/v2/chassis` | 切换车型 (热切换) |
| POST | `/api/v2/scene` | 热切换场景 {hub_scene_id} |
| POST | `/api/v2/model` | 热切换车辆模型 {model_id, version} |
