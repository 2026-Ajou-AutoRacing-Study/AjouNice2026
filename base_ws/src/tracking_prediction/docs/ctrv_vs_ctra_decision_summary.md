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

**2026-08-02 정정:** 이전에는 로터리 진입구간 재평가(`docs/entry_zone_ctrv_ctra_summary.md`)에서 CTRA가 p90 기준 수십~수백 배 나쁘다고 적었으나(2.0s에서 CTRV p90=22.7m vs CTRA p90=963.6m), 이 수치는 그 평가 스크립트(`eval_entry_zone_ctrv_ctra.py`)의 가속도 추정 버그(직전/현재 프레임 속도 부호 처리가 비대칭이라 가속도가 순간적으로 수백~수천 배 부풀려짐)로 오염된 값이었음(팀원이 Codex로 발견, 본 저장소에도 반영해 수정함). 버그 수정 후 재평가하면 CTRA는 mean에서만 CTRV보다 나쁘고(2.0s 기준 8.16m vs 5.84m) p90/p95는 CTRV와 거의 같거나 근소하게 더 나음 — "1/omega² 증폭" 설명도 틀렸음(최악 표본의 omega는 0.466 rad/s로 작지 않았음). 위 표(`eval_model_ablation.py`/`eval_final_horizon_decision_ctra.py` 기준, +0.2~+9.6% 수준)는 이 버그와 무관한 스크립트로 계산된 값이라 그대로 유효하며, CTRV 채택 결론의 실제 근거는 이 표임. 자세한 내용은 `docs/entry_zone_ctrv_ctra_summary.md` 참고.
