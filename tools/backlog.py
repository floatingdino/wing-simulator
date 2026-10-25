#!/usr/bin/env python3
"""Backlog status: `tools/backlog.py` prints tasks; `--ready` lists todo tasks whose deps are done."""
import re, sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent / "backlog" / "tasks"
tasks = {}
for p in sorted(root.glob("T*.md")):
    h = p.read_text().split("---")[1]
    f = dict(re.findall(r"^(\w+): (.*)$", h, re.M))
    f["depends"] = [d.strip() for d in f["depends"].strip("[]").split(",") if d.strip()]
    tasks[f["id"]] = f
ready = lambda t: t["status"] == "todo" and all(tasks.get(d, {}).get("status") == "done" for d in t["depends"])
for t in tasks.values():
    if "--ready" in sys.argv and not ready(t):
        continue
    print(f'{t["id"]:5} {t["stage"]} {t["status"]:11} {t["owner"]:12} {t["title"]}')
