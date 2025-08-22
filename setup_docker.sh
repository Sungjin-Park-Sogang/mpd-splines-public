# exeperiment_launcher 설치
cd deps/experiment_launcher
omni_python -m pip install -e .

# deps 설치
cd ../
omni_python -m pip install -e .


cd ../mpd/motion_planning_baselines
omni_python -m pip install -e .

cd ../torch_robotics
omni_python -m pip install -e .

cd ../
omni_python -m pip install -e .

omni_python -m pip install numpy==1.26 networkx==2.8.8 dotmap einops torchlie vendi_score torchkin