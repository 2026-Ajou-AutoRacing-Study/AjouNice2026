# AjouNice2026
2026 대학생 AI·S/W 모빌리티 경진대회 AI융합자율주행 부문 - 아주대학교 AjouNice 팀  

## Target Environment
- Ubuntu 20.04
- ROS1 Noetic
- MORAI / K-City Competition simulator
- Desktop PC runtime

## Main Pipeline
Sensors  
→ Localization  
→ Perception  
→ Tracking / Prediction  
→ Planning  
→ Control  
→ Diagnostics

## Quick Start
```bash
git clone https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026.git

```

## Repository Structure
- `docs/`: 협업을 위한 문서들
- `configs/` : 차량, 센서, tf 구성에 사용할 파라미터들
- `scripts/`: setup/build/run/check에 사용할 스크립트들  
- `maps/`: map 파일들
- `base_ws/`: ros workspace 폴더
- `third_party/`: 개발에 참고할 외부 코드들 -> 추후 우리 workspace에 통합할 대상