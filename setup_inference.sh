#!/bin/bash

# Setup script for inference-only environment (without isaac gym and pb_ompl)
# This script installs PyTorch 2.5+ with CUDA 12.8 support

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="${THIS_DIR}/deps"

echo "Setting up MPD inference-only environment..."

# Initialize submodules (we still need some dependencies)
git submodule update --init --recursive --progress

export SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL=True

# Activate conda
eval "$(~/miniconda3/bin/conda shell.bash hook)"

# Create environment from inference-specific yml file
conda env create -f environment_inference.yml

# Activate the environment
conda activate mpd-splines-inference

# Configure conda channels
conda config --add channels conda-forge
conda config --set channel_priority strict

pip install setuptools

conda install nvidia/label/cuda-12.8.0::cuda-toolkit
conda install conda-forge::cudnn

pip install torch torchvision

# Set CUDA environment variables
conda env config vars set CUDA_HOME="$CONDA_PREFIX"
conda activate mpd-splines-inference

# Install core dependencies (without pb_ompl and isaac gym)
echo "-------> Installing experiment_launcher"
cd ${DEPS_DIR}/experiment_launcher && pip install -e .

echo "-------> Installing theseus/torchkin"
cd ${DEPS_DIR}/theseus/torchkin && pip install -e .

echo "-------> Installing torch_robotics"
cd ${THIS_DIR}/mpd/torch_robotics && pip install -e .

echo "-------> Installing motion_planning_baselines"
cd ${THIS_DIR}/mpd/motion_planning_baselines && pip install -e .

# Skip pb_ompl installation
echo "-------> Skipping pybullet_ompl installation (using mock implementation)"

echo "-------> Installing pinocchio"
conda install pinocchio -c conda-forge --yes

echo "-------> Installing this library"
cd ${THIS_DIR} && pip install -e .

# Install additional dependencies
conda install -c "conda-forge/label/cf202003" gdown --yes
pip install numpy --upgrade
pip install networkx --upgrade
pip install torch_kmeans
conda install conda-forge::ffmpeg --yes
pip install dotmap
pip install vendi_score

# ROS dependencies (if needed for inference)
pip install empy==3.3.4
pip install rospkg catkin_pkg
pip install example-robot-data
pip install imutils
pip install opencv-contrib-python

# Git hooks
pre-commit install

echo ""
echo "========================================="
echo "Inference-only setup complete!"
echo "========================================="
echo ""
echo "Environment name: mpd-splines-inference"
echo "To activate: conda activate mpd-splines-inference"
echo ""
echo "Note: This environment uses mock implementations for:"
echo "- pb_ompl (for B-spline fitting and visualization)"
echo "- GenerateDataOMPL (for collision checking)"
echo ""
echo "For full functionality including isaac gym and pb_ompl,"
echo "use the original setup.sh script."
echo ""