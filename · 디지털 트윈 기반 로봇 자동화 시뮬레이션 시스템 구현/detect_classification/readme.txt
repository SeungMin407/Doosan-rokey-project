VSCode 설정 안내

VSCode에서 설치할 것:

Python

Python for VSCode

Python Extension Pack

Jupyter

Jupyter Keymap

설치 후 반드시 활성화(enable) 해야 합니다.
-------------------------------------------------------------
터미널에서 설치할 패키지:

# 시스템 패키지 업데이트 및 pip 설치
sudo apt update
sudo apt install -y python3-pip

# 필수 Python 패키지 설치
python3 -m pip install ultralytics roboflow

# PyTorch 설치
pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu121

# YOLO 관련 패키지 재설치
python3 -m pip install ultralytics

# NumPy 및 pandas 설치/업데이트
python3 -m pip uninstall -y numpy
python3 -m pip install "numpy>=1.26,<2"
python3 -m pip install pandas

# matplotlib 호환 버전 설치
pip uninstall matplotlib matplotlib-inline -y
pip install matplotlib==3.7.2 matplotlib-inline==0.1.6
-----------------------------------------------------------
--------------------------------------------------------------
1. execute sub.py

source /opt/ros/humble/setup.bash
python3 /home/rokey/Desktop/cooperate_3/detect_classification/sub.py
