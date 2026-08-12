# 3D Object Detection

이 디렉터리는 MORAI 데이터로 학습·평가하는 3D object detection 모델과 모델 간
공통 코드를 관리한다. 원본 모델 저장소는 `third_party/`, 데이터 생성·선별·변환은
`dataset/morai_od/`, checkpoint와 실행 로그는 Git에서 제외된
`artifacts/detection3d/`에 둔다.

```text
detection3d/
├── common/       # 모델 중립 평가·좌표계·시각화 코드
├── fastbev/      # Fast-BEV adapter, config, 실행 도구와 문서
└── tests/        # 공통 3DOD 단위 테스트
```

현재 공통 MORAI evaluator는 center-distance AP, TP error 및 3D IoU AP를 제공한다.
향후 StreamPETR도 예측 결과를 같은 ego-frame box 형식으로 변환해 동일 evaluator를
사용한다.
