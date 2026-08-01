# 모델 결정: CTRV vs CTRA (2026-07-30 확정)

**이 문서는 코드/다른 문서에서 계속 참조되고 있었지만 실제로는 누락돼 있던 파일이라, `docs/tp_final_report.md` 1절의 내용을 원본으로 복원함(2026-08-02).** `prediction_node.py`, `prediction.launch`, `eval_entry_zone_ctrv_ctra.py`, `docs/tp_pipeline_architecture.md`에서 이 파일을 근거로 인용하고 있음.

## 비교 조건

같은 조건(OD 10Hz + 실측 PointPainting 검출오차)에서 두 모델을 비교:

- **CTRV**: 등속 + 등회전속도 (가속도 없음)
- **CTRA(최선 버전)**: 감쇠가속도 λ=0.4 + 스무딩(5프레임 이동평균) + 가속도 clip 4m/s²

원본 실험: `eval_model_ablation.py`(모델×스무딩 2x2 분리 비교), `eval_final_horizon_decision_ctra.py`(CTRA 최종 버전 vs CTRV horizon 결정표).

## 결과 (단위: %, +는 CTRA가 CTRV보다 나쁨/악화, -는 CTRA가 더 좋음/개선)

| horizon | mean | p90 (안전마진) | p95 (안전마진) | 중앙값(typical) |
| --- | --- | --- | --- | --- |
| 0.5s | +1.6% 악화 | +0.2% 악화 | +0.3% 악화 | -4.3% 개선 |
| 1.0s | +2.4% 악화 | +0.5% 악화 | +0.7% 악화 | -19.8% 개선 |
| 1.5s | +3.4% 악화 | +2.5% 악화 | +1.5% 악화 | -19.9% 개선 |
| 2.0s | +4.6% 악화 | +5.8% 악화 | +2.5% 악화 | -17.6% 개선 |
| 3.0s | +2.9% 악화 | +9.6% 악화 | +4.0% 악화 | -13.2% 개선 |

**CTRA는 중앙값(전형적 상황)에서만 개선되고, mean/p90/p95(최악의 경우, 안전마진과 직결)는 모든 horizon에서 CTRV보다 나쁨.**

## 결론

예측 결과는 planning의 안전마진 산정에 쓰이므로 "평균적으로 정확함"보다 "최악의 경우에도 덜 위험함"이 우선 → **CTRV를 기본 모델로 채택**. CTRA는 `prediction_node.py`에 `~model=ctra`로 전환 가능하도록 코드만 보존(재조율 대비).

이후 로터리 진입구간 좁혀서 본 재평가(`docs/entry_zone_ctrv_ctra_summary.md`)에서도 CTRA는 p90 기준 CTRV 대비 수십~수백 배 나쁜 것으로 재확인되어 이 결론이 더 강하게 뒷받침됨(예: 2.0s에서 CTRV p90=22.7m vs CTRA p90=963.6m — omega가 노이즈 수준으로 작을 때 CTRA의 1/omega² 항이 CTRV의 1/omega 항보다 노이즈를 훨씬 크게 증폭시키기 때문).
