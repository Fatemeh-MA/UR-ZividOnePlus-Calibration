# Isolated environment for the Zivid One+ (last supported SDK: 2.11.1).
# The host keeps its own (newer) Zivid SDK untouched.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates udev libatomic1 ocl-icd-libopencl1 clinfo \
        libfontconfig1 libgl1 libegl1 libxkbcommon-x11-0 libdbus-1-3 \
    && rm -rf /var/lib/apt/lists/*

# OpenCL ICD for the NVIDIA driver (the library itself is injected by the NVIDIA container runtime)
RUN mkdir -p /etc/OpenCL/vendors && echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

COPY *.deb /tmp/pkgs/
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        /tmp/pkgs/zivid-telicam-driver_3.0.1.1-3_amd64.deb \
        /tmp/pkgs/zivid_2.11.1+de9b5dae-1_amd64.deb \
        /tmp/pkgs/zivid-tools_2.11.1+de9b5dae-1_amd64.deb \
        /tmp/pkgs/zivid-studio_2.11.1+de9b5dae-1_amd64.deb \
    && rm -rf /tmp/pkgs /var/lib/apt/lists/*

# GUI libraries for Zivid Studio (Qt5 / X11)
RUN apt-get update && apt-get install -y --no-install-recommends \
        libglib2.0-0 libice6 libsm6 libxi6 libxrender1 libgtk-3-0 libpango-1.0-0 \
        libxcb-xinerama0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 \
        libxcb-render-util0 libxcb-shape0 libxcb-xfixes0 libxcb-xkb1 libxcb-glx0 \
        libx11-xcb1 libxext6 libxfixes3 libgl1-mesa-dri libglx-mesa0 fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display

CMD ["bash"]
