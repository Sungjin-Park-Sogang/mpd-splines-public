#!/bin/bash

# Simplified setup script for inference-only environment
# Handles Linux-specific CUDA installation and externally-managed-environment issues

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="${THIS_DIR}/deps"

echo "Setting up MPD inference-only environment (Linux)..."

# Initialize submodules
echo "-------> Initializing submodules"
git submodule update --init --recursive --progress

export SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL=True

# Activate conda
eval "$(~/miniconda3/bin/conda shell.bash hook)"

# Update conda
echo "-------> Updating conda"
conda update -n base conda -y

# Remove existing environment if it exists
if conda env list | grep -q "mpd-splines-inference"; then
    echo "-------> Removing existing environment"
    conda env remove -n mpd-splines-inference -y
fi

# Create basic conda environment
echo "-------> Creating conda environment"
conda create -n mpd-splines-inference python=3.10 pip -y

# Activate the environment
conda activate mpd-splines-inference

# Configure conda channels
echo "-------> Configuring conda channels"
conda config --add channels conda-forge
conda config --add channels pytorch

# Install basic packages via conda (to avoid pip externally-managed issues)
echo "-------> Installing basic packages via conda"
conda install -y \
    numpy \
    scipy \
    matplotlib \
    plotly \
    scikit-learn \
    pillow \
    pandas \
    h5py \
    ffmpeg

# Install PyTorch 2.5+ with CUDA support
echo "-------> Installing PyTorch 2.5+ with CUDA 12.1 support (compatible with 12.8)"
conda install pytorch torchvision pytorch-cuda=12.1 -c pytorch -c nvidia -y

# Install additional packages via pip (within conda environment, this should work)
echo "-------> Installing additional packages via pip"
pip install \
    pyyaml \
    tensorboard \
    tqdm \
    ConfigArgParse \
    shapely \
    setuptools \
    joblib \
    sentence_transformers \
    wandb \
    trimesh \
    mesh_to_sdf \
    seaborn \
    tabulate \
    einops \
    torchdiffeq \
    torch-summary \
    pygame \
    gitpython \
    scikit-video \
    opencv-python \
    moviepy \
    hydra-core \
    urdfpy \
    pybullet \
    dirsync \
    colour \
    pre-commit \
    black \
    dotmap \
    vendi_score \
    torch_kmeans \
    networkx \
    "empy==3.3.4" \
    rospkg \
    catkin_pkg \
    example-robot-data \
    imutils \
    opencv-contrib-python

# Install pinocchio via conda
echo "-------> Installing pinocchio"
conda install pinocchio -c conda-forge -y

# Install core dependencies (without pb_ompl and isaac gym)
echo "-------> Installing experiment_launcher"
cd ${DEPS_DIR}/experiment_launcher && pip install -e .

echo "-------> Installing theseus/torchkin"
cd ${DEPS_DIR}/theseus/torchkin && pip install -e .

echo "-------> Installing torch_robotics"
cd ${THIS_DIR}/mpd/torch_robotics && pip install -e .

echo "-------> Installing motion_planning_baselines"
cd ${THIS_DIR}/mpd/motion_planning_baselines && pip install -e .

echo "-------> Installing this library (mpd)"
cd ${THIS_DIR} && pip install -e .

# Git hooks
echo "-------> Setting up pre-commit hooks"
pre-commit install

echo ""
echo "========================================="
echo "Inference-only setup complete!"
echo "========================================="
echo ""
echo "Environment name: mpd-splines-inference"
echo "To activate: conda activate mpd-splines-inference"
echo ""
echo "Testing PyTorch CUDA:"
python -c "import torch; print(f'PyTorch version: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}'); print(f'CUDA version: {torch.version.cuda if torch.cuda.is_available() else \"N/A\"}')"
echo ""
echo "Note: This environment uses mock implementations for:"
echo "- pb_ompl (for B-spline fitting and visualization)" 
echo "- GenerateDataOMPL (for collision checking)"
echo ""