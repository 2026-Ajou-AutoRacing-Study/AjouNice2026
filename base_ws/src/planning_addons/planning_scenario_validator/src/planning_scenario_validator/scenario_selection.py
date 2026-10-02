"""Small filename checklist. Selection belongs to the campaign YAML, not GUI settings."""
from fnmatch import fnmatchcase
from pathlib import Path

from .config import scenario_sources


def matching_names(names, query):
    """Substring or shell wildcard search; comma-separated alternatives are OR."""
    terms = [term.strip().casefold() for term in query.split(",") if term.strip()]
    return [name for name in sorted(names) if not terms or any(
        fnmatchcase(name.casefold(), term) if any(char in term for char in "*?[")
        else term in name.casefold() for term in terms)]


class Selection:
    def __init__(self, names, selected):
        self.available = set(names)
        self.selected = set(selected)
        # Keep missing saved names visible until explicitly unchecked.
        self.names = sorted(self.available | self.selected)

    def visible(self, query=""):
        return matching_names(self.names, query)

    def set_visible(self, query, checked):
        visible = set(self.visible(query))
        if checked:
            self.selected.update(visible & self.available)
        else:
            self.selected.difference_update(visible)

    def toggle(self, names):
        names = set(names)
        if names <= self.selected:
            self.selected.difference_update(names)
        else:
            self.selected.update(names & self.available)

    def result(self):
        if not self.selected:
            raise ValueError("최소 한 장면을 체크하세요. 전체 실행은 ‘전체 실행(패턴)’을 누르세요")
        missing = self.selected - self.available
        if missing:
            raise ValueError("없어진 파일을 체크 해제하거나 복원하세요: " + ", ".join(sorted(missing)))
        return sorted(self.selected)


def choose_scenarios(parent, directory, pattern, include, *, on_ready=None):
    """Return (applied, include). None means pattern mode; cancel preserves form."""
    import tkinter as tk
    from tkinter import ttk, messagebox

    directory = Path(directory).resolve()
    if not directory.is_dir():
        raise ValueError("시나리오 폴더가 없습니다: {}".format(directory))
    names = [p.name for p in directory.iterdir()
             if p.is_file() and p.suffix == ".json" and p.resolve().parent == directory]
    if not names:
        raise ValueError("이 폴더에 시나리오 .json 파일이 없습니다")
    # Starting with every file checked would hide unfiltered checks and make
    # "search baseline -> check search results" unexpectedly run the whole folder.
    selected = include if include is not None else []
    model = Selection(names, selected)
    dialog = tk.Toplevel(parent)
    dialog.title("실행할 시나리오 선택")
    dialog.transient(parent)
    dialog.geometry("800x550")
    dialog.minsize(650, 440)
    frame = ttk.Frame(dialog, padding=10)
    frame.pack(fill="both", expand=True)
    ttk.Label(frame, text="처음은 체크 없이 시작합니다. 기존 선택은 유지됩니다. 적용 후 개인 YAML을 저장하세요.").pack(anchor="w")
    ttk.Label(frame, text=str(directory), wraplength=760).pack(anchor="w", pady=(0, 6))
    query = tk.StringVar(value="" if include is not None or pattern == "*.json" else pattern)
    search = ttk.Frame(frame)
    search.pack(fill="x")
    ttk.Label(search, text="이름 검색 / 패턴").pack(side="left")
    entry = ttk.Entry(search, textvariable=query)
    entry.pack(side="left", fill="x", expand=True, padx=6)
    ttk.Label(frame, text="예: cargobox / high_normal_*.json / baseline,stress (쉼표는 OR)").pack(anchor="w", pady=4)
    presets = ttk.Frame(frame)
    presets.pack(fill="x")
    ttk.Button(presets, text="모두 보기", command=lambda: query.set("")).pack(side="left", padx=2)
    for token, label in (("baseline", "기본 장면"), ("normal", "일반 장면"), ("stress", "어려운 장면")):
        if any(token in name.casefold() for name in names):
            ttk.Button(presets, text=label, command=lambda value=token: query.set(value)).pack(side="left", padx=2)
    count = tk.StringVar()
    ttk.Label(frame, textvariable=count).pack(anchor="w", pady=6)
    listing = ttk.Frame(frame)
    listing.pack(fill="both", expand=True)
    tree = ttk.Treeview(listing, columns=("checked", "name"), show="headings", selectmode="extended")
    tree.heading("checked", text="체크")
    tree.column("checked", width=55, stretch=False, anchor="center")
    tree.heading("name", text="시나리오 JSON")
    tree.column("name", width=620)
    tree.tag_configure("missing", foreground="#b22")
    scroll = ttk.Scrollbar(listing, orient="vertical", command=tree.yview)
    tree.configure(yscrollcommand=scroll.set)
    tree.pack(side="left", fill="both", expand=True)
    scroll.pack(side="right", fill="y")
    result = {"applied": False, "include": include}
    view = {"query": None}

    def refresh(*_):
        highlighted = set(tree.selection())
        focused = tree.focus()
        position = tree.yview()[0]
        changed_query = view['query'] != query.get()
        view['query'] = query.get()
        tree.delete(*tree.get_children())
        visible = model.visible(query.get())
        for name in visible:
            missing = name not in model.available
            tree.insert("", "end", iid=name, values=("☑" if name in model.selected else "☐",
                        name + (" [파일 없음]" if missing else "")), tags=("missing",) if missing else ())
        tree.selection_set([name for name in visible if name in highlighted])
        if focused in visible:
            tree.focus(focused)
        tree.yview_moveto(0 if changed_query else position)
        count.set("폴더 {}개 · 검색 결과 {}개 · 총 체크 {}개 (검색을 바꿔도 체크 유지)".format(
            len(model.available), len(visible), len(model.selected)))

    def bulk(checked):
        model.set_visible(query.get(), checked)
        refresh()

    def toggle_highlighted(_event=None):
        model.toggle(tree.selection())
        refresh()
        return "break"

    def click(event):
        row = tree.identify_row(event.y)
        if row and tree.identify_column(event.x) == "#1":
            tree.selection_set(row)
            tree.focus(row)
            model.toggle([row])
            refresh()
            return "break"

    def apply():
        try:
            chosen = model.result()
            # Revalidate against actual files before accepting a potentially stale listing.
            scenario_sources({"directory": str(directory), "include": chosen})
        except ValueError as error:
            messagebox.showerror("장면 선택 확인", str(error), parent=dialog)
            return
        result.update(applied=True, include=chosen)
        dialog.destroy()

    def use_pattern():
        # Allow returning to pattern mode even if its old glob no longer matches.
        # The user can edit the re-enabled field; run preflight validates it.
        result.update(applied=True, include=None)
        dialog.destroy()

    query.trace_add("write", refresh)
    tree.bind("<Button-1>", click)
    tree.bind("<space>", toggle_highlighted)
    tree.bind("<Control-a>", lambda _event: (tree.selection_set(tree.get_children()), "break")[-1])
    ttk.Label(frame, text="체크칸 클릭 / Shift·Ctrl로 여러 행 강조 후 Space로 체크·해제").pack(anchor="w", pady=5)
    tools = ttk.Frame(frame)
    tools.pack(fill="x", pady=3)
    for label, action in (("검색 결과 전체 체크", lambda: bulk(True)),
                          ("검색 결과 체크 해제", lambda: bulk(False)),
                          ("모든 체크 지우기", lambda: (model.selected.clear(), refresh()))):
        ttk.Button(tools, text=label, command=action).pack(side="left", padx=2)
    buttons = ttk.Frame(frame)
    buttons.pack(fill="x", pady=(6, 0))
    ttk.Button(buttons, text="전체 실행(패턴)", command=use_pattern).pack(side="left")
    ttk.Button(buttons, text="취소", command=dialog.destroy).pack(side="right", padx=2)
    ttk.Button(buttons, text="체크한 장면 적용", command=apply).pack(side="right", padx=2)
    refresh()
    dialog.bind("<Escape>", lambda _event: dialog.destroy())
    dialog.grab_set()
    entry.focus_set()
    if on_ready is not None:
        on_ready({"dialog": dialog, "model": model, "query": query, "tree": tree,
                  "refresh": refresh, "bulk": bulk, "apply": apply, "all": use_pattern})
    parent.wait_window(dialog)
    return result["applied"], result["include"]
