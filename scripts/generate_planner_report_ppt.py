#!/usr/bin/env python3
"""Generate the three-slide Planner development-status presentation.

Run while LibreOffice is listening on the local UNO socket:

  soffice --headless --accept='socket,host=127.0.0.1,port=2002;urp;StarOffice.ComponentContext'
  python3 scripts/generate_planner_report_ppt.py
"""

from pathlib import Path
import time

import uno
from com.sun.star.awt import Point, Size
from com.sun.star.beans import PropertyValue
from com.sun.star.style.ParagraphAdjust import CENTER, LEFT


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "docs" / "planner_development_report.pptx"
PDF_OUTPUT = ROOT / "docs" / "planner_development_report.pdf"

# Impress uses 1/100 mm. This is 13.333 x 7.5 in (16:9).
WIDTH = 33867
HEIGHT = 19050

FONT = "Noto Sans CJK KR"
BLACK = 0x101010
MUTED = 0x626B78
BLUE = 0x1879BD
BLUE_LIGHT = 0xF1F7FC
GREEN = 0x15955C
GREEN_LIGHT = 0xF2FAF6
PURPLE = 0x7951B3
PURPLE_LIGHT = 0xF7F3FC
RED = 0xD94B43
RED_LIGHT = 0xFFF4F2
ORANGE = 0xD78220
ORANGE_LIGHT = 0xFFF8EC
GRAY = 0x697586
GRAY_LIGHT = 0xF5F7FA
WHITE = 0xFFFFFF


def prop(name, value):
    item = PropertyValue()
    item.Name = name
    item.Value = value
    return item


def connect():
    local_context = uno.getComponentContext()
    resolver = local_context.ServiceManager.createInstanceWithContext(
        "com.sun.star.bridge.UnoUrlResolver", local_context
    )
    for _ in range(30):
        try:
            return resolver.resolve(
                "uno:socket,host=127.0.0.1,port=2002;urp;StarOffice.ComponentContext"
            )
        except Exception:
            time.sleep(0.3)
    raise RuntimeError("Could not connect to the LibreOffice UNO listener")


def add_shape(doc, page, shape_type, x, y, w, h, *, fill=WHITE, line=None, width=0):
    shape = doc.createInstance(shape_type)
    shape.Position = Point(x, y)
    shape.Size = Size(w, h)
    if hasattr(shape, "FillStyle"):
        shape.FillStyle = 1  # SOLID
        shape.FillColor = fill
    if hasattr(shape, "LineStyle"):
        shape.LineStyle = 0 if line is None else 1
    if line is not None:
        shape.LineColor = line
        shape.LineWidth = width
    page.add(shape)
    return shape


def add_text(
    doc,
    page,
    text,
    x,
    y,
    w,
    h,
    *,
    size=22,
    color=BLACK,
    bold=False,
    align=LEFT,
    fill=None,
    margin=0,
):
    shape = add_shape(
        doc,
        page,
        "com.sun.star.drawing.TextShape",
        x,
        y,
        w,
        h,
        fill=WHITE if fill is None else fill,
        line=None,
    )
    shape.String = text
    shape.CharFontName = FONT
    shape.CharHeight = size
    shape.CharColor = color
    shape.CharWeight = 150.0 if bold else 100.0
    shape.ParaAdjust = align
    shape.TextLeftDistance = margin
    shape.TextRightDistance = margin
    shape.TextUpperDistance = margin
    shape.TextLowerDistance = margin
    shape.TextVerticalAdjust = 0  # top
    return shape


def add_card(doc, page, x, y, w, h, color, light, heading, bullets, *, note=None):
    card = add_shape(
        doc,
        page,
        "com.sun.star.drawing.RectangleShape",
        x,
        y,
        w,
        h,
        fill=light,
        line=color,
        width=55,
    )
    card.CornerRadius = 640
    card.Shadow = True
    card.ShadowColor = 0xD9DEE6
    card.ShadowXDistance = 70
    card.ShadowYDistance = 70
    add_shape(
        doc,
        page,
        "com.sun.star.drawing.RectangleShape",
        x - 260,
        y,
        210,
        h,
        fill=color,
        line=None,
    )
    add_text(
        doc,
        page,
        heading,
        x + 360,
        y + 270,
        w - 720,
        900,
        size=15.5,
        color=color,
        bold=True,
        align=CENTER,
        fill=light,
    )
    line_y = y + 1050
    for bullet in bullets:
        add_shape(
            doc,
            page,
            "com.sun.star.drawing.EllipseShape",
            x + 400,
            line_y + 190,
            90,
            90,
            fill=BLACK,
            line=None,
        )
        add_text(
            doc,
            page,
            bullet,
            x + 580,
            line_y,
            w - 960,
            620,
            size=11.5,
            color=BLACK,
            fill=light,
        )
        line_y += 680
    if note:
        add_text(
            doc,
            page,
            note,
            x + 420,
            y + h - 570,
            w - 760,
            330,
            size=10.5,
            color=MUTED,
            align=CENTER,
            fill=light,
        )


def add_issue_card(doc, page, x, y, w, h, color, light, heading, issue, response):
    card = add_shape(
        doc,
        page,
        "com.sun.star.drawing.RectangleShape",
        x,
        y,
        w,
        h,
        fill=light,
        line=color,
        width=55,
    )
    card.CornerRadius = 640
    card.Shadow = True
    card.ShadowColor = 0xD9DEE6
    card.ShadowXDistance = 70
    card.ShadowYDistance = 70
    add_shape(
        doc,
        page,
        "com.sun.star.drawing.RectangleShape",
        x - 260,
        y,
        210,
        h,
        fill=color,
        line=None,
    )
    add_text(doc, page, heading, x + 300, y + 270, w - 600, 850,
             size=16.5, color=color, bold=True, align=CENTER, fill=light)
    add_shape(doc, page, "com.sun.star.drawing.EllipseShape",
              x + 420, y + 1510, 100, 100, fill=color, line=None)
    add_text(doc, page, issue, x + 620, y + 1340, w - 1020, 1150,
             size=11.5, color=BLACK, fill=light)
    add_shape(doc, page, "com.sun.star.drawing.RectangleShape",
              x + 380, y + 2900, w - 760, 26, fill=0xD7DEE8, line=None)
    add_text(doc, page, "→", x + 380, y + 3240, 280, 440,
             size=15, color=color, bold=True, align=CENTER, fill=light)
    add_text(doc, page, response, x + 700, y + 3160, w - 1080, 1350,
             size=11.5, color=BLACK, fill=light)


def add_timeline_card(doc, page, x, y, w, h, color, light, phase, period, items):
    card = add_shape(doc, page, "com.sun.star.drawing.RectangleShape", x, y, w, h,
                     fill=light, line=color, width=55)
    card.CornerRadius = 640
    card.Shadow = True
    card.ShadowColor = 0xD9DEE6
    card.ShadowXDistance = 70
    card.ShadowYDistance = 70
    add_shape(doc, page, "com.sun.star.drawing.EllipseShape", x + 400, y + 310, 720, 720,
              fill=color, line=None)
    add_text(doc, page, phase, x + 475, y + 480, 570, 300,
             size=10.5, color=WHITE, bold=True, align=CENTER, fill=color)
    add_text(doc, page, period, x + 1300, y + 345, w - 1600, 430,
             size=13, color=color, bold=True, fill=light)
    line_y = y + 1260
    for item in items:
        add_shape(doc, page, "com.sun.star.drawing.EllipseShape",
                  x + 480, line_y + 180, 90, 90, fill=BLACK, line=None)
        add_text(doc, page, item, x + 670, line_y, w - 1070, 620,
                 size=11.5, color=BLACK, fill=light)
        line_y += 700


def add_header(doc, page, title, section, number):
    if " - " in title:
        prefix, suffix = title.split(" - ", 1)
        add_text(doc, page, prefix + " -", 1350, 820, 9000, 1200,
                 size=24, color=BLACK, bold=True)
        add_text(doc, page, suffix, 10350, 820, 15000, 1200,
                 size=24, color=BLACK, bold=True)
    else:
        add_text(doc, page, title, 1350, 820, 24500, 1200,
                 size=24, color=BLACK, bold=True)
    add_text(doc, page, section, 1350, 1980, 18000, 650, size=14, color=0x246BFD)
    add_text(doc, page, number, 1350, 3060, 1200, 500, size=15, color=BLACK, bold=True)
    # MORAI-style wordmark (text-only so it remains editable in PowerPoint).
    add_text(doc, page, "MOR", 28700, 980, 2050, 650, size=20, color=BLACK, bold=True)
    add_text(doc, page, "AI", 30650, 980, 1400, 650, size=20, color=0x4C7EFF, bold=True)


def add_footer(doc, page, page_number):
    add_text(doc, page, str(page_number), 32300, 17580, 450, 300,
             size=9, color=MUTED, align=CENTER)


def algorithm_slide(doc, page):
    add_header(doc, page, "개발현황 리포트 - Planner 알고리즘", "1. Planner 개발 현황", "I.")
    add_text(doc, page, "Planner 알고리즘", 2450, 2960, 8000, 570,
             size=17, color=BLACK, bold=True)

    y, w, h = 4500, 9200, 6000
    xs = (1600, 12400, 23200)
    add_card(doc, page, xs[0], y, w, h, BLUE, BLUE_LIGHT, "Behavior Path Planner", [
        "Lanelet2 기반 차선 추종",
        "Route 기반 차선변경 후보 생성",
        "정적 장애물 회피·차선 복귀",
        "동적 장애물 주행영역 제어",
    ], note="Output  |  PathWithLaneId")
    add_card(doc, page, xs[1], y, w, h, GREEN, GREEN_LIGHT, "Behavior Velocity Planner", [
        "신호등·정지선 정지 계획",
        "횡단보도·보행자 양보 판단",
        "교차로·블라인드스팟 안전 판단",
        "과속방지턱·제한구역 감속",
    ], note="Output  |  Velocity-modified Path")
    add_card(doc, page, xs[2], y, w, h, PURPLE, PURPLE_LIGHT, "Motion Velocity Planner", [
        "Trajectory 기반 최종 속도 계획",
        "정적 장애물 안전 정지",
        "동적 장애물 TTC·예측경로 대응",
        "선행차 추종·jerk 제한 감속",
    ], note="Output  |  Safe Trajectory")
    add_text(doc, page, "→", 10840, 6650, 1250, 620, size=29, color=MUTED, bold=True, align=CENTER)
    add_text(doc, page, "→", 21640, 6650, 1250, 620, size=29, color=MUTED, bold=True, align=CENTER)
    add_footer(doc, page, 1)


def issue_slide(doc, page):
    add_header(doc, page, "개발현황 리포트 - 개발 이슈", "2. 개발 이슈", "II.")
    add_text(doc, page, "주요 이슈 및 대응", 2450, 2960, 8000, 570,
             size=17, color=BLACK, bold=True)
    y, w, h = 4500, 9200, 6100
    add_issue_card(doc, page, 1600, y, w, h, ORANGE, ORANGE_LIGHT,
                   "Map · Route 정합성",
                   "차선 경계·고도 차이 → false collision / 차선변경 오차",
                   "K-City 경계 보정 + Route 기반 후보 경로 생성")
    add_issue_card(doc, page, 12400, y, w, h, RED, RED_LIGHT,
                   "장애물 회피 안정성",
                   "회피·복귀 경로와 동적 객체 누락 시 정지 판단 불안정",
                   "회피 방향·복귀 경로 튜닝 + predicted path·stop latch")
    add_issue_card(doc, page, 23200, y, w, h, PURPLE, PURPLE_LIGHT,
                   "MORAI 검증 자동화",
                   "ScenarioLoad 후 ROS 연결 해제로 무인 배치 검증 제한",
                   "대표 closed-loop + ROS 입력 재생 회귀 검증")
    add_footer(doc, page, 2)


def schedule_slide(doc, page):
    add_header(doc, page, "개발현황 리포트 - 향후 개발 일정", "3. 향후 개발 일정", "III.")
    add_text(doc, page, "안정화 → 통합 검증 → 릴리스", 2450, 2960, 11000, 570,
             size=17, color=BLACK, bold=True)
    y, w, h = 4700, 9200, 5200
    add_timeline_card(doc, page, 1600, y, w, h, BLUE, BLUE_LIGHT,
                      "01", "9월 1–2주", [
                          "BPP 회피·복귀 corner case 보완",
                          "BVP 신호·정지선 판단 재검증",
                          "MVP 동적 정지·재출발 안정화",
                      ])
    add_timeline_card(doc, page, 12400, y, w, h, GREEN, GREEN_LIGHT,
                      "02", "9월 3–4주", [
                          "보행자·NPC·바리케이트 시나리오 확대",
                          "MORAI closed-loop·제어기 연동",
                          "실패 케이스 bag·KPI 축적",
                      ])
    add_timeline_card(doc, page, 23200, y, w, h, PURPLE, PURPLE_LIGHT,
                      "03", "10월 1주", [
                          "Open-loop 회귀 gate·parameter sweep",
                          "성공 시나리오 baseline 승격",
                          "대회 주행 파라미터 freeze",
                      ])
    add_text(doc, page, "→", 10840, 6750, 1250, 620, size=29, color=MUTED, bold=True, align=CENTER)
    add_text(doc, page, "→", 21640, 6750, 1250, 620, size=29, color=MUTED, bold=True, align=CENTER)
    add_footer(doc, page, 3)


def main():
    context = connect()
    desktop = context.ServiceManager.createInstanceWithContext(
        "com.sun.star.frame.Desktop", context
    )
    document = desktop.loadComponentFromURL("private:factory/simpress", "_blank", 0, ())

    pages = document.getDrawPages()
    while pages.getCount() < 3:
        pages.insertNewByIndex(pages.getCount())
    for index in range(pages.getCount()):
        pages.getByIndex(index).Width = WIDTH
        pages.getByIndex(index).Height = HEIGHT

    algorithm_slide(document, pages.getByIndex(0))
    issue_slide(document, pages.getByIndex(1))
    schedule_slide(document, pages.getByIndex(2))

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    document.storeAsURL(
        uno.systemPathToFileUrl(str(OUTPUT)),
        (prop("FilterName", "Impress MS PowerPoint 2007 XML"),),
    )
    document.storeToURL(
        uno.systemPathToFileUrl(str(PDF_OUTPUT)),
        (prop("FilterName", "impress_pdf_Export"),),
    )
    document.close(True)
    print(OUTPUT)
    print(PDF_OUTPUT)


if __name__ == "__main__":
    main()
