# ============================================================================
# RK3588 单机全套 (对应手机单机模式): 资源平台 + 节点代理 (process 运行时) + 仿真 + Web 网关 + 执行 (ROS 2/Nav2)
# 在 agv-nav 镜像上补仿真侧依赖 (与 deploy/android/proot_setup.sh 相同) 和完整代码；不在板卡上跑 Docker，
# 由 pack.sh docker export 成目录树，板卡上 chroot 运行 (agv_ros.sh)
#   docker build -f deploy/rk3588_offline/Dockerfile -t agv-rk:latest .     (需先有 agv-nav:latest)
# ============================================================================
FROM agv-nav:latest
ARG PIP_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-pil python3-psutil python3-protobuf libopenblas0-pthread gcc libc6-dev procps iproute2 util-linux \
    && (pip install --no-cache-dir ${PIP_INDEX:+-i $PIP_INDEX} "mujoco>=3.3" "numpy<2" || pip install --no-cache-dir "mujoco>=3.3" "numpy<2") \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -c "import numpy, mujoco, PIL, psutil, google.protobuf; print('numpy', numpy.__version__, 'mujoco', mujoco.__version__)"
WORKDIR /opt/agv
# 完整代码 (.dockerignore 排除 .git/tests 等)；ros2/install 是 agv-nav 里编译好的 C++ 桥接，COPY 不会覆盖
COPY . .
ARG AGV_VERSION=local
RUN bash sim_core/native/build.sh gcc python3 && echo "$AGV_VERSION" > .agv_version && mkdir -p maps
ENTRYPOINT []
CMD ["bash"]
