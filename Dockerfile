FROM ubuntu:26.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONNOUSERSITE=1 \
    VIRTUAL_ENV=/opt/agent-ros-venv \
    PATH=/opt/agent-ros-venv/bin:$PATH

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        ca-certificates \
        curl \
        gnupg \
        locales \
        lsb-release \
        make \
        python3-pip \
        python3-venv \
        software-properties-common \
    && locale-gen C.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install --yes --no-install-recommends \
        ros-lyrical-desktop-full \
        ros-lyrical-gz-sim-vendor \
        ros-lyrical-ros-gz \
        ros-lyrical-turtlebot3-gazebo \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --shell /bin/bash ros \
    && mkdir --parents "$VIRTUAL_ENV" /workspace \
    && chown --recursive ros:ros "$VIRTUAL_ENV" /workspace

WORKDIR /workspace
COPY --chown=ros:ros . /workspace

USER ros
RUN source /opt/ros/lyrical/setup.bash \
    && python3 -m venv --system-site-packages "$VIRTUAL_ENV" \
    && "$VIRTUAL_ENV/bin/python" -m pip install --upgrade pip \
    && "$VIRTUAL_ENV/bin/python" -m pip install --no-cache-dir ".[dev]"

COPY --chown=ros:ros docker/ros-entrypoint.sh /usr/local/bin/ros-entrypoint
RUN chmod 0755 /usr/local/bin/ros-entrypoint

ENTRYPOINT ["/usr/local/bin/ros-entrypoint"]
CMD ["bash"]
