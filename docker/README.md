# Docker Images

This repo now uses split Dockerfiles by workflow.

## ROS 2 Humble

File: `docker/Dockerfile.humble`

Build ROS image with workspace prebuilt:

```bash
docker build -f docker/Dockerfile.humble --target ros-dev -t uav-controller:ros-humble .
docker run --rm -it --network=host --privileged -v "$PWD":/workspace uav-controller:ros-humble
```

## Python Simulation + Tools

File: `docker/Dockerfile.sim`

Includes `sim_py` virtualenv and Python deps for `tools/` (protocol monitors + GPS dashboard).

```bash
docker build -f docker/Dockerfile.sim -t uav-controller:sim .
docker run --rm -it -v "$PWD":/workspace uav-controller:sim
```

## Firmware Tooling (PlatformIO)

File: `docker/Dockerfile.firmware`

Includes PlatformIO and the whole `firmware/` tree (`espnow`, `elrs`, `lora`, `gps`).

```bash
docker build -f docker/Dockerfile.firmware -t uav-controller:firmware .
docker run --rm -it -v "$PWD":/workspace uav-controller:firmware
```

## Recommended default for VS Code devcontainer

`.devcontainer/devcontainer.json` is configured for ROS + full repo bootstrap from a bind-mounted workspace.
