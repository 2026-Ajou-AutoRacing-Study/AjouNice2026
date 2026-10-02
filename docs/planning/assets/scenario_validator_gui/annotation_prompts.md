# GUI 안내 이미지 제작 프롬프트

2026-10-03 사용자 제공 GUI 캡처 5장에 대해 built-in imagegen의 편집 모드를 사용했다. 원본은 대화에 첨부된 화면이며, 새 UI 디자인이나 실험 성능 이미지를 생성한 것이 아니다. 최종 파일은 이 폴더의 01–05 PNG이다. 3·4·5번은 아래의 후속 교정까지 적용했다. 제작 프롬프트 원문을 재편집 참고용으로 보관한다. 실제 실행값은 GUI/YAML과 사용자 가이드를 우선한다.

## 1. 기본 설정

```text
Use case: precise-object-edit. Asset type: Korean user guide annotated screenshot, image 1 of 5. EDIT TARGET: ONLY the FIRST of the five attached GUI screenshots, the tab labeled '기본 설정'. Other screenshots are context, not targets. Preserve every original screenshot pixel, all existing Korean text, fields, checkboxes, values and controls; do not redesign or invent GUI controls. Add a WHITE margin on the RIGHT about 420px wide for annotation notes, retaining the FULL original screenshot uncropped. Add thin RED rectangular outlines and numbered red circle badges around these existing areas, then red leader lines to Korean notes in right margin, large clear 22px red text. Labels EXACTLY: '1 먼저 예제 선택 → 작업 폴더 복사' [outline top source selector and 베이스 → 내 작업 폴더 복사 button]; '2 이름 · 시나리오 폴더 · 파일 패턴' [outline first 3 basic form rows]; '3 목적지는 처음엔 그대로' [outline four goal rows]; '4 제한시간 · 측위 · 객체 탐지' [outline next 3 rows]; '5 영상 ON / 센서 녹화 OFF 권장' [outline video and bag sensors checkboxes, do not change their current values]; '6 저장 → 검사 → 새 실험 시작' [outline top three action buttons]. Put notes vertically next to corresponding areas, avoid crossing leaders, no text covering any original UI. Subtle pale red box fills only outside UI if useful. This is annotation of an actual supplied screenshot, NOT a mockup; no UI changes, no invented buttons, no spelling changes, no extra controls. Retain all tab labels and original text, readable sharp high resolution. Do not change checkbox states or parameter values.
```

## 2. 고급 / 네트워크

```text
Use case: precise-object-edit. Asset type: annotated Korean GUI manual screenshot 2/5. EDIT TARGET: the unannotated attached GUI screenshot whose SELECTED tab is '고급 / 네트워크'. Ignore the annotated 기본 설정 screenshot and the other tabs. Keep ALL original GUI controls, Korean text, existing values, title and tab layout unchanged and uncropped. Add WHITE margin at RIGHT about 420 px wide. Match the previous annotated screenshot: thin red outlines, red circled numbers and red leader lines to large clear Korean red notes in margin. Do not write annotations over controls. Highlight these existing controls and label exactly: '1 토픽은 basic 유지 권장' [row rosbag 토픽 JSON]; '2 MORAI에 로드한 JSON과 맞추기' [Sensor JSON + Ego Network JSON rows together]; '3 화면 배치 split / 바 높이 auto' [녹화 화면 배치 + GUI 하단 높이 rows]; '4 Drive 공유할 때만 경로 확인' [Drive rclone 경로 row]; '5 실행 전 환경 검사' [환경 검사 button]; '6 처음 세팅이면 실제 GPU 검사' [실제 GPU 검사 button]; '7 연결 파일 경로 복사' [연결 JSON 경로 복사 button]. Use bottom/right free white margin to list button notes spaced vertically with non-overlapping leaders. No fake controls, no field changes, no text translation. SceneLoad IP 127.0.0.1 and goal tolerance 2.0 remain visible and unmodified. All original UI content must be readable and faithful. Clear high-resolution annotation only.
```

## 3. 신호등 환경

```text
Use case: precise-object-edit. Asset type: annotated Korean GUI manual screenshot 3/5. EDIT TARGET: the ORIGINAL unannotated attached GUI screenshot whose selected tab is '신호등 환경'. Do NOT edit any annotated image or other tab. Preserve every original field, gray disabled control state, Korean text, existing parameter values, title and tabs. Add WHITE margin RIGHT about 420px for notes, retaining FULL uncropped screenshot. Match previous annotation style: thin RED rectangles, red circled numbers, clear large Korean RED notes, leaders that don't cover controls. In right margin place notes exactly: '1 기본 탭에서 신호등 ON 먼저' [red box around '기본 설정' tab + small arrow, this is an instruction not toggle change]; '2 설치된 실행 환경 선택' [outline 실행 환경 row, system or virtualenv]; '3 가상환경이면 이 Python 경로 사용' [outline 가상환경 Python row]; '4 GPU는 0 / CPU 시험은 cpu' [outline 신호등 장치 row]; '5 환경 설치가 필요할 때만' [outline 공용 가상환경 설치 명령 복사 button]. Below annotations include red footer note '신호등 OFF면 이 탭은 건드리지 않아도 됩니다'. Use free whitespace for readable placement. The screenshot has controls disabled because detection is OFF: keep that disabled appearance, don't invent ON controls on this tab. No new controls, no values changed, no GUI redesign. All original text remains readable. Annotation only.
```

## 3-1. 신호등 문구 교정

```text
Edit the referenced annotated GUI screenshot. Make ONLY two precise typography corrections, preserve everything else (UI geometry, all controls, all other labels, red boxes and lines) unchanged. Correction 1: the BLACK label to the LEFT of the FIRST environment dropdown currently reads '시스템 Python'; restore it to EXACT '실행 환경'. The GRAY dropdown value '시스템 Python' inside the control must stay unchanged. Correction 2: the large RED annotation numbered 3 currently runs into the right image edge. Wrap its text into EXACTLY TWO lines: line 1 '가상환경이면', line 2 '이 Python 경로 사용'. Fit both lines fully within existing white right margin with at least 20px right padding. No other text changes, no fake GUI controls, no cropping. Every note fully readable.
```

## 4. 실행 목록 / 배치

```text
Use case: precise-object-edit. Asset type: Korean GUI manual annotated screenshot 4/5. EDIT TARGET: the ORIGINAL unannotated screenshot with selected tab '실행 목록 / 배치' and EMPTY batch table. Other images are references only. Keep ALL original UI text, rows, buttons, values and layout faithful, do not invent or remove controls. Add WHITE margin to RIGHT approximately 440 pixels, FULL original screenshot uncropped. Match earlier style of red outline boxes, circled red numbers and large 22px red Korean labels. Avoid all text clipping: wrap annotations at 16 Korean characters per line when needed. Place notes in white margin or existing large blank bottom area without covering controls. Add these annotations: '1 현재 YAML 또는 여러 파일 추가' [outline 현재 YAML 추가 + YAML 여러 개 추가 buttons]; '2 실행 순서 바꾸기' [outline up/down arrow buttons]; '3 전체 검사 → 새 배치 시작' [outline these two existing buttons]; '4 중단한 배치만 재개' [outline 배치 재개 button]; '5 이전 목록 열기로 이어가기' [outline 이전 목록 열기 button]; '6 결과는 배치 CSV로 확인' [outline 배치 CSV + 배치 결과 폴더 buttons]; '7 오래 돌릴 때 모니터 모드' [outline top right 모니터 모드 button]. Red footer warning in free lower space: '목록 초기화는 YAML·녹화 결과를 지우지 않습니다'. Leader lines must not cross each other or any control labels; use ample blank bottom and right margin. Preserve selected batch tab, table headers, total count and size estimate exactly. No GUI redesign, no invented entries. Sharply readable high resolution.
```

## 4-1. 배치 설명 교정

```text
Edit this existing annotated batch-tab screenshot. Preserve ALL underlying GUI controls, their text, values, button positions and table exactly. Correct ONLY annotation layout. Important: note 6 currently has a circled badge but its explanatory text is missing. Add a clear large RED note in the empty WHITE RIGHT margin, with EXACT two-line text '6 결과는 배치 CSV로' / '확인'. Connect it to the red outlined '배치 CSV' button with a thin red leader that routes below the button area, avoids all control labels and avoids other note text. Keep the existing circled 6 badge or move only that annotation into the white margin, not the actual button. Also the RED leader to note 1 must point to the existing highlighted '현재 YAML 추가 / YAML 여러 개 추가' button pair, NOT to '이전 목록 열기'. Correct its start point; route outside controls. Keep all other notes 2,3,4,5,7 and the footer. Ensure every red text is fully visible with no clipping. Do not create or remove actual GUI buttons. Annotation correction only.
```

## 5. 실험 기록 / 재개

```text
Use case: precise-object-edit. Asset type: annotated Korean user guide screenshot 5/5. EDIT TARGET: the ORIGINAL unannotated screenshot with selected '실험 기록 / 재개' tab and a table of earlier release/static experiments. Ignore annotated images for other tabs. Preserve FULL original screenshot, all field values, original table entries, P/F/E counts, button labels, title and layout EXACTLY; no inventing data, don't add or remove rows. Add a WHITE margin RIGHT about 480 pixels for annotations. Thin RED rectangular outlines with numbered red circle badges and non-crossing red leader lines. All red note text goes in the right margin, big readable 22px Korean text with line wrapping to avoid clipping. Add exactly these five labels: '1 시간 · 진행 · P/F/E로 기록 선택' [outline history table INCLUDING column headers]; '2 중단한 실험은 선택 실험 재개' [outline 선택 실험 재개 button]; '3 기록이 안 보이면 새로고침' [outline 새로고침 button]; '4 다른 결과 폴더도 열 수 있음' [outline 다른 폴더 기록 열기 button]; '5 처음부터 다시는 새 실험 시작' [outline top 새 실험 시작 button]. Put label5 near top right aligned to the top button, label1 by the table, remaining button labels vertically in right margin below table. Footer red text in blank lower-right white margin: '재개는 현재 화면이 아니라 당시 입력 사본을 사용합니다'. Keep existing '(이전)' status labels and UTC/local times as in screenshot; never imply previous records gained snapshots. Do not change any original UI text. No controls added, no redesign. Annotation-only, high resolution.
```

## 5-1. 재개 버튼 연결과 누락 설명 교정

```text
Edit the referenced annotated history-tab screenshot. Make only annotation corrections, keep every underlying GUI control, data row, P/F/E count and Korean UI label unchanged. Fix note 2: its red leader currently incorrectly starts at the third button ('다른 폴더 기록 열기'). Remove that incorrect leader and connect note 2 ONLY to the SECOND button '선택 실험 재개' via its existing circled 2 badge below the button, then route through blank area below badges to right note 2 without covering UI text. Leave note 4 pointing to third button. Add missing explanatory red text for badge 3 in the empty WHITE RIGHT margin BELOW note4, at roughly y=750: EXACT text '기록이 안 보이면 새로고침'. Connect it ONLY to first button '새로고침' / existing circled3 badge, routing through blank lower gray space with no UI label obscured. Also wrap note1 to two readable lines '시간 · 진행 · P/F/E로' / '기록 선택' so no letter clips the image edge. Preserve notes5 and footer. All five note numbers must have their explanations and refer to the correct button. Do not add or remove GUI widgets or change history entries. No cropping.
```

## 5-2. 잘못 연결된 긴 선 제거

```text
undefined
```
