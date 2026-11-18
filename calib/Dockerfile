# Hand-eye calibration environment for the Zivid One+ (SDK 2.11.1 + zivid-python 2.11.1).
# Built on top of zivid-oneplus:2.11.1 (see ../Dockerfile). The scripts are NOT baked in:
# calib.sh mounts this folder as /work, so edits take effect without rebuilding.
FROM zivid-oneplus:2.11.1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-dev build-essential cmake ninja-build git \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir numpy==1.26.4 scipy==1.15.3 opencv-python-headless==4.11.0.86 pyyaml==6.0.3 \
    && python3 -m pip install --no-cache-dir -v "zivid==2.11.1.2.11.1"

COPY 3rdParty /opt/calib/3rdParty
RUN python3 -m pip install --no-cache-dir /opt/calib/3rdParty/rtde-2.3.6

# zividsamples.save_load_matrix helper used by the Zivid UR script
COPY save_load_matrix.py /usr/local/lib/python3.10/dist-packages/zividsamples/save_load_matrix.py
RUN touch /usr/local/lib/python3.10/dist-packages/zividsamples/__init__.py

WORKDIR /work
