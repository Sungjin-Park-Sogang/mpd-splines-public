# CuRobo Integration Test Guide

이 문서는 MPD에서 CuRobo 통합 기능을 테스트하는 방법을 설명합니다.

## 📋 테스트 스크립트 개요

Phase 3까지 구현된 CuRobo 통합 기능을 단계별로 테스트할 수 있는 4개의 스크립트가 제공됩니다.

## 🔧 테스트 실행 방법

### 1. 기본 Import 테스트
```bash
omni_python test_curobo_imports.py
```

**테스트 내용:**
- PyTorch, CuRobo 라이브러리 import 확인
- torch_robotics 모듈 가용성 확인
- MPD CuRobo 모듈 import 테스트
- CuRobo availability flags 확인
- Robot configuration 파일 접근 테스트

**예상 결과:**
```
✓ PyTorch: 2.x.x
✓ CUDA available: True
✓ CuRobo SDF World imports
✓ MPD CuRobo cost guides imports
🎉 All tests passed! CuRobo integration is ready.
```

### 2. 환경 변환 테스트
```bash
omni_python test_curobo_environment_conversion.py
```

**테스트 내용:**
- torch_robotics 환경 생성 (EnvSpheres3D 등)
- torch_robotics → CuRobo WorldConfig 변환
- CuRobo WorldCollision 초기화
- 다양한 환경 타입 변환 테스트
- CuRoboSDFWorldInterface 테스트

**예상 결과:**
```
✓ EnvSpheres3D created successfully
✓ Environment converted to CuRobo WorldConfig
  Spheres: 15, Cuboids: 0
✓ CuRobo world initialization
🎉 Environment conversion tests passed!
```

### 3. 기본 Inference 기능 테스트
```bash
omni_python test_curobo_basic_inference.py
```

**테스트 내용:**
- CuRobo configuration 파일 로딩
- CuRobo cost guide 생성
- CuRobo data generator 생성
- EvaluationSamplesGenerator with CuRobo 테스트
- CuRobo collision checking 기능

**예상 결과:**
```
✓ config_EnvSpheres3D-RobotPanda-CuRobo_00.yaml
✓ CuRobo collision cost created
✓ CuRobo data generator created
✓ EvaluationSamplesGenerator with CuRobo created
🎉 Basic inference tests passed!
```

### 4. 전체 Inference 통합 테스트
```bash
omni_python test_curobo_full_inference.py
```

**테스트 내용:**
- 완전한 inference pipeline 설정
- CuRobo cost guide manager 테스트
- Configuration 기반 inference 설정
- 실제 trajectory planning 테스트
- torch_robotics vs CuRobo 비교

**예상 결과:**
```
✓ Planning task created
✓ CostGuideManagerParametricTrajectory created
✓ Config-based inference working
✓ Trajectory planning test
🎉 Full inference testing mostly successful!
```

## 📊 테스트 결과 해석

### ✅ 성공 시나리오
모든 테스트가 통과하면 다음과 같이 실제 inference를 실행할 수 있습니다:

```bash
cd scripts/inference

# CuRobo 기반 inference
omni_python inference.py --cfg_inference_path ./cfgs/config_EnvSpheres3D-RobotPanda-CuRobo_00.yaml

# 기존 torch_robotics 기반 inference (비교용)
omni_python inference.py --cfg_inference_path ./cfgs/config_EnvSpheres3D-RobotPanda_00.yaml
```

### ⚠️ 실패 시나리오별 대응

#### 1. Import 실패
```
✗ CuRobo SDF World import failed
```
**해결책:** CuRobo가 올바르게 설치되지 않았습니다. Isaac Sim 환경에서 CuRobo 설치를 확인하세요.

#### 2. 환경 변환 실패
```
✗ Environment conversion failed
```
**해결책:** torch_robotics 환경이 올바르게 로드되지 않았거나, CuRobo WorldConfig 변환 로직에 문제가 있습니다.

#### 3. 기본 Inference 실패
```
✗ Cost guide creation failed
```
**해결책:** CuRobo MotionGen 초기화나 collision detection 설정에 문제가 있습니다.

#### 4. 전체 Inference 실패
```
✗ Trajectory planning test failed
```
**해결책:** GPU 메모리 부족이나 CuRobo 설정 문제일 수 있습니다.

## 🔍 디버깅 팁

### Verbose 로깅 활성화
```bash
# CuRobo 로깅 활성화하여 실행
CUROBO_LOG_LEVEL=debug omni_python test_curobo_basic_inference.py
```

### CUDA 메모리 확인
```bash
nvidia-smi  # GPU 메모리 사용량 확인
```

### 단계별 디버깅
1. **Step 1**: `test_curobo_imports.py` 먼저 실행
2. **Step 2**: Import가 성공하면 `test_curobo_environment_conversion.py` 실행
3. **Step 3**: 환경 변환이 성공하면 `test_curobo_basic_inference.py` 실행
4. **Step 4**: 기본 기능이 작동하면 `test_curobo_full_inference.py` 실행

## 📁 생성된 CuRobo 통합 파일들

### 핵심 구현
- `mpd/inference/curobo_cost_guides.py` - CuRobo SDF 인터페이스 및 collision cost
- `scripts/generate_data/generate_trajectories_curobo.py` - CuRobo 기반 데이터 생성기
- `mpd/inference/cost_guides.py` (수정됨) - CuRobo 통합 지원

### Configuration 파일
- `scripts/inference/cfgs/config_EnvSpheres3D-RobotPanda-CuRobo_00.yaml`
- `scripts/inference/cfgs/config_EnvWarehouse-RobotPanda-CuRobo_00.yaml`

### 테스트 스크립트
- `test_curobo_imports.py`
- `test_curobo_environment_conversion.py`
- `test_curobo_basic_inference.py`
- `test_curobo_full_inference.py`

## 🎯 다음 단계

테스트가 성공적으로 완료되면:

1. **성능 비교**: torch_robotics vs CuRobo inference 성능 측정
2. **Multi-robot 확장**: CuRobo의 multi-robot 지원 활용
3. **Production 배포**: 실제 로봇 시스템에서 CuRobo 기반 planning 사용

## 🆘 지원

문제가 발생하면:
1. 테스트 스크립트 출력의 상세 에러 메시지 확인
2. Isaac Sim과 CuRobo 설치 상태 점검
3. GPU 메모리 및 CUDA 환경 확인