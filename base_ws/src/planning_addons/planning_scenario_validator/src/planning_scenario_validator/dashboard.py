"""ROS 없이 저장된 campaign_state.json만 읽어 진행 상황을 보여준다."""
import argparse
import json
import time
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description="planning validator 실시간 대시보드")
    parser.add_argument("campaign_dir", type=Path)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args(argv)
    state_path = args.campaign_dir.expanduser() / "campaign_state.json"
    try:
        while True:
            if not state_path.is_file():
                print("상태 파일 대기: {}".format(state_path))
            else:
                state = json.loads(state_path.read_text(encoding="utf-8"))
                results = state.get("results", {})
                counts = {key: sum(row.get("status") == key for row in results.values())
                          for key in ("PASS", "FAIL", "ERROR")}
                current = state.get("current") or {}
                metric = current.get("metrics") or {}
                print("\033[2J\033[H", end="")
                print("{}  {}/{} 완료".format(state.get("campaign", "?"), len(results), state.get("total", "?")))
                print("성공 {}  실패 {}  시스템 오류 {}  대기 {}".format(
                    counts["PASS"], counts["FAIL"], counts["ERROR"],
                    max(0, state.get("total", 0) - len(results) - bool(current))))
                if current:
                    print("현재: {} | {} | 시도 {}".format(current.get("scenario"), current.get("phase"), current.get("attempt")))
                    print("주행 {:.1f}초 | 목적지 {:.1f}m | 속도 {:.1f}m/s | 충돌 {}".format(
                        metric.get("elapsed_sec", 0), metric.get("goal_distance_m", 0),
                        metric.get("speed_mps", 0), metric.get("collision_count", 0)))
                else:
                    print("현재 실행 중인 시나리오 없음")
                for name, row in list(results.items())[-8:]:
                    detail = row.get("failure_category") or row.get("reason", "")
                    stage = row.get("suspected_stage", "")
                    print("  {:6s} {:34s} {}{}".format(row.get("status", "?"), name[:34],
                        detail, " [{}]".format(stage) if stage and stage != "undetermined" else ""))
                if state.get("upload_errors"):
                    print("업로드 오류 {}개 (로컬 결과는 보존됨)".format(len(state["upload_errors"])))
            if args.once:
                return 0
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
