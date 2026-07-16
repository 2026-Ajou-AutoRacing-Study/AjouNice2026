#!/usr/bin/env python3
"""Strict loader for the coordinate-resolved competition route catalog."""

import csv
import os


class RouteLoadError(RuntimeError):
    pass


def _as_int(value, field, filename):
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        raise RouteLoadError("%s: invalid %s=%r" % (filename, field, value))


def _as_float(value, field, filename):
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        raise RouteLoadError("%s: invalid %s=%r" % (filename, field, value))


def parse_centerline(value, lanelet_id, filename):
    points = []
    for token in str(value).split(";"):
        xyz = token.split(":")
        if len(xyz) != 3:
            raise RouteLoadError("%s: lanelet %s has malformed centerline" % (filename, lanelet_id))
        points.append(tuple(float(v) for v in xyz))
    if len(points) < 2:
        raise RouteLoadError("%s: lanelet %s has fewer than two centerline points" % (filename, lanelet_id))
    return points


def load_catalog(filename):
    if not filename or not os.path.isfile(filename):
        raise RouteLoadError("catalog CSV does not exist: %r" % filename)
    preferred_orders = set()
    ordered = []
    with open(filename, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"order", "lanelet_id", "role", "speed_limit_kph", "centerline_xy"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise RouteLoadError("%s: required columns are %s" % (filename, sorted(required)))
        for row_number, raw in enumerate(reader, start=2):
            role = str(raw["role"]).strip()
            # The full v44 catalog intentionally uses labels such as C01 for
            # candidate corridors.  Only preferred_course has the numeric route order.
            if role == "preferred_course":
                order = _as_int(raw["order"], "order", filename)
                if order in preferred_orders:
                    raise RouteLoadError("%s: duplicate preferred-course order %d" % (filename, order))
                preferred_orders.add(order)
                sort_order = order
            else:
                order = str(raw["order"]).strip()
                sort_order = 1000000 + row_number
            lanelet_id = _as_int(raw["lanelet_id"], "lanelet_id", filename)
            row = {
                "order": order,
                "sort_order": sort_order,
                "lanelet_id": lanelet_id,
                "role": role,
                "section": str(raw.get("section", "")).strip(),
                "speed_limit_kph": _as_float(raw["speed_limit_kph"], "speed_limit_kph", filename),
                "centerline": parse_centerline(raw["centerline_xy"], lanelet_id, filename),
            }
            ordered.append(row)
    return sorted(ordered, key=lambda item: item["sort_order"])


def _parse_bool(value, filename, row_number):
    text = str(value or "false").strip().lower()
    if text in ("", "false", "0", "no"):
        return False
    if text in ("true", "1", "yes"):
        return True
    raise RouteLoadError("%s row %d: reverse must be true or false" % (filename, row_number))


def load_selection(filename):
    if not filename or not os.path.isfile(filename):
        raise RouteLoadError("selection CSV does not exist: %r" % filename)
    result, orders = [], set()
    with open(filename, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"order", "lanelet_id"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise RouteLoadError("%s: required columns are order,lanelet_id" % filename)
        for number, raw in enumerate(reader, start=2):
            order = _as_int(raw["order"], "order", filename)
            if order in orders:
                raise RouteLoadError("%s: duplicate selection order %d" % (filename, order))
            orders.add(order)
            target = str(raw.get("target_speed_kph", "")).strip()
            catalog_order = str(raw.get("catalog_order", "")).strip()
            result.append({
                "order": order,
                "lanelet_id": _as_int(raw["lanelet_id"], "lanelet_id", filename),
                "catalog_order": None if not catalog_order else _as_int(catalog_order, "catalog_order", filename),
                "reverse": _parse_bool(raw.get("reverse", "false"), filename, number),
                "segment_name": str(raw.get("segment_name", "")).strip(),
                "target_speed_kph": None if not target else _as_float(target, "target_speed_kph", filename),
                "note": str(raw.get("note", "")).strip(),
            })
    if not result:
        raise RouteLoadError("%s: selection is empty" % filename)
    return sorted(result, key=lambda item: item["order"])


def select_route(catalog_rows, selection_filename):
    by_id = {}
    for row in catalog_rows:
        by_id.setdefault(row["lanelet_id"], []).append(row)
    if not selection_filename:
        selected = []
        for row in catalog_rows:
            if row["role"] == "preferred_course":
                copy = dict(row)
                copy.update(reverse=False, target_speed_kph=None)
                selected.append(copy)
        if not selected:
            raise RouteLoadError("catalog contains no preferred_course rows")
        return selected
    selected = []
    for item in load_selection(selection_filename):
        candidates = by_id.get(item["lanelet_id"], [])
        if not candidates:
            raise RouteLoadError("selection lanelet %s is absent from catalog" % item["lanelet_id"])
        preferred = [row for row in candidates if row["role"] == "preferred_course"]
        if item["catalog_order"] is not None:
            preferred = [row for row in preferred if row["order"] == item["catalog_order"]]
            if not preferred:
                raise RouteLoadError("selection lanelet %s has no preferred_course catalog_order %s" %
                                     (item["lanelet_id"], item["catalog_order"]))
        if not preferred:
            raise RouteLoadError("selection lanelet %s is not preferred_course" % item["lanelet_id"])
        if len(preferred) > 1:
            raise RouteLoadError("selection lanelet %s is ambiguous in catalog; add catalog_order" % item["lanelet_id"])
        row = dict(preferred[0])
        row.update(item)
        selected.append(row)
    return selected
