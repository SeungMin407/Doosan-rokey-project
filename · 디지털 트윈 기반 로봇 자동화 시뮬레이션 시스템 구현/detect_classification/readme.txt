VSCode 설정 안내

VSCode에서 설치할 것:

Python

Python for VSCode

Python Extension Pack

Jupyter

Jupyter Keymap

SQLite Viewer
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
실행 가이드
--------------------------------------------------------------
1. 해당 폴더를 /home/roeky/Desktop/ 경로 아래 위치시킨다.

2. main 폴더 내부의 파일들을 /home/isaacsim/extension_examples/hello_world 폴더에 붙여넣는다.

3. 
cd ~/isaacsim
./post_install.sh
./isaac-sim.sh 실행한다.

4. execute sub_pub.py
다른 터미널을 열어
source /opt/ros/humble/setup.bash
python3 /home/rokey/Desktop/detect_classification/sub_pub.py
를 실행한다.

5.
isaacsim 내부에서 Window - Example - Robotics Examples를 누르고 Robotics Examples - ROKEY - conveyor를 클릭하고, load한다.
