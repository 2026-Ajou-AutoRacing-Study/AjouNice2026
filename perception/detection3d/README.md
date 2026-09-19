# 3D Object Detection

이 디렉터리는 MORAI 데이터로 학습·평가하는 3D object detection 모델과 모델 간
공통 코드를 관리한다. 원본 모델 저장소는 `third_party/`, 데이터 생성·선별·변환은
`dataset/morai_od/`, checkpoint와 실행 로그는 Git에서 제외된
`artifacts/detection3d/`에 둔다.

```text
detection3d/
├── bevfusion/    # 현재 production 학습·평가·배포 코드
├── common/       # 모델 중립 평가·좌표계·시각화 코드
├── fastbev/      # legacy Fast-BEV 재현 코드
└── tests/        # 공통 3DOD 단위 테스트
```

현재 문서 진입점은 [`../../docs/bevfusion/README.md`](../../docs/bevfusion/README.md)다.
Fast-BEV와 PointPainting의 과거 결과를 production 기본값으로 사용하지 않는다.
