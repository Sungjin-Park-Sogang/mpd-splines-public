# MPD Splines - Inference Only Setup

이 가이드는 RTX 5070Ti (CUDA 12.8)에서 PyTorch 2.5+를 사용하여 isaac gym과 pb_ompl 없이 inference만 실행하는 방법을 설명합니다.

## 문제 상황

- RTX 5070Ti → CUDA 12.8 필요
- CUDA 12.8 → PyTorch 2.5+ 필요  
- 기존 프로젝트는 Python 3.8 + isaac gym + pb_ompl 사용
- Inference만 실행하고 싶음

## 해결책

### 1. Inference 전용 환경 설정

```bash
# Inference 전용 환경 생성
./setup_inference.sh
```

또는 수동으로:

```bash
# 환경 생성
conda env create -f environment_inference.yml
conda activate mpd-splines-inference

# PyTorch 2.5+ 설치
pip install torch>=2.5.0 torchvision>=0.20.0 --index-url https://download.pytorch.org/whl/cu118
```

### 2. Mock 구현 사용

프로젝트에는 다음 mock 구현이 포함되어 있습니다:

- `mpd/inference/pb_ompl_mock.py`: pb_ompl의 `add_box`, `fit_bspline_to_path` 함수 대체
- `mpd/inference/generate_data_mock.py`: GenerateDataOMPL 클래스 대체

이러한 mock 구현은 자동으로 import됩니다 (원본이 없을 때).

### 3. Inference 실행

```python
from mpd.inference.inference import GenerativeOptimizationPlanner

# 정상적으로 inference 코드 실행
planner = GenerativeOptimizationPlanner(...)
results = planner.plan_trajectory(...)
```

## 제한사항

### Mock 구현의 제한사항:

1. **충돌 검사 (Collision Checking)**: 
   - 완전한 충돌 검사 대신 joint limit 검사만 수행
   - 더 관대한 상태 검증

2. **B-spline 피팅**:
   - scipy 기반의 간단한 구현
   - 원본 OMPL 구현보다 단순

3. **시각화**:
   - 기본적인 pybullet 시각화만 지원
   - isaac gym 시각화 기능 없음

### 정확한 결과가 필요한 경우:

전체 기능이 필요하면 원본 `setup.sh`를 사용하여 isaac gym과 pb_ompl을 설치하세요.

## 파일 구조

```
├── environment_inference.yml          # Inference 전용 conda 환경
├── setup_inference.sh                 # Inference 전용 설치 스크립트
├── mpd/inference/
│   ├── pb_ompl_mock.py               # pb_ompl mock 구현
│   └── generate_data_mock.py         # GenerateDataOMPL mock 구현
└── README_INFERENCE.md               # 이 문서
```

## 문제 해결

### CUDA 호환성 확인

```python
import torch
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"CUDA version: {torch.version.cuda}")
```

### Mock 사용 확인

```python
# pb_ompl mock 사용 여부 확인
try:
    from pb_ompl.pb_ompl import add_box
    print("Using original pb_ompl")
except ImportError:
    from mpd.inference.pb_ompl_mock import add_box
    print("Using pb_ompl mock")
```

이제 RTX 5070Ti에서 PyTorch 2.5+를 사용하여 inference를 실행할 수 있습니다!