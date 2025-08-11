#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re
import json
import csv
import sys
import argparse
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Any, Optional

# ========== 工具：ANSI 颜色转义去除 ==========
ANSI_ESCAPE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")

# ========== 关键行正则 ==========
re_basic   = re.compile(r"== Basic Report == index:(\d+), timestamp:(\d+)")
re_role    = re.compile(r"role=(\w+), n_ap=(\d+)")
re_rtt     = re.compile(r"rtt_accumulated_half_ns=([+-]?\d+), rtt_count=(\d+)")
re_ap      = re.compile(r"-- Antenna Path (\d+) --")
re_tone    = re.compile(r"Tone=(\w+)\s+Dist\(ifft/phase_slope/rtt/best\)=([^/]+)/([^/]+)/([^/]+)/([^/\s]+)")

# IQ 片段：ch:<idx>:<il>,<ql>,<ir>,<qr>;
NUM = r"[+-]?(?:\d+(?:\.\d+)?|nan|inf|-inf)"
re_iq_line_has_prefix = re.compile(r"\bIQ:\s*(.*)")
re_iq_tokens = re.compile(rf"ch:(\d+):({NUM}),({NUM}),({NUM}),({NUM});")

def to_float(x: str) -> float:
    try:
        return float(x)
    except Exception:
        return float('nan')

def flush_current_report(reports: List[Dict[str, Any]], current_report: Dict[str, Any]):
    if not current_report:
        return
    for ap in current_report.get("aps", []):
        iq_map = ap.get("iq", {})
        if iq_map:
            max_ch = max(iq_map.keys())
            ap["iq"] = [iq_map.get(ch, [float('nan')]*4) for ch in range(max_ch + 1)]
        else:
            ap["iq"] = []
    reports.append(current_report)

def parse_log(path: str):
    reports: List[Dict[str, Any]] = []
    current_report: Dict[str, Any] = {}
    current_ap: Optional[Dict[str, Any]] = None

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = ANSI_ESCAPE.sub("", raw).strip()
            if not line:
                continue

            m = re_basic.search(line)
            if m:
                flush_current_report(reports, current_report)
                current_report = {
                    "index": int(m.group(1)),
                    "timestamp_ms": int(m.group(2)),
                    "role": None,
                    "n_ap": None,
                    "rtt_accumulated_half_ns": None,
                    "rtt_count": None,
                    "aps": []
                }
                current_ap = None
                continue

            m = re_role.search(line)
            if m and current_report:
                current_report["role"] = m.group(1)
                current_report["n_ap"] = int(m.group(2))
                continue

            m = re_rtt.search(line)
            if m and current_report:
                current_report["rtt_accumulated_half_ns"] = int(m.group(1))
                current_report["rtt_count"] = int(m.group(2))
                continue

            m = re_ap.search(line)
            if m and current_report:
                ap_idx = int(m.group(1))
                current_ap = {
                    "ap_index": ap_idx,
                    "tone_quality": None,
                    "distance": {"ifft": None, "phase_slope": None, "rtt": None, "best": None},
                    "iq": {}
                }
                current_report["aps"].append(current_ap)
                continue

            m = re_tone.search(line)
            if m and current_ap is not None:
                current_ap["tone_quality"] = m.group(1)
                current_ap["distance"]["ifft"] = to_float(m.group(2))
                current_ap["distance"]["phase_slope"] = to_float(m.group(3))
                current_ap["distance"]["rtt"] = to_float(m.group(4))
                current_ap["distance"]["best"] = to_float(m.group(5))
                continue

            if current_ap is not None:
                payload = None
                m = re_iq_line_has_prefix.search(line)
                if m:
                    payload = m.group(1)
                else:
                    if "ch:" in line:
                        payload = line

                if payload:
                    for t in re_iq_tokens.finditer(payload):
                        ch = int(t.group(1))
                        il = to_float(t.group(2))
                        ql = to_float(t.group(3))
                        ir = to_float(t.group(4))
                        qr = to_float(t.group(5))
                        current_ap["iq"][ch] = [il, ql, ir, qr]
                    continue

    flush_current_report(reports, current_report)
    return reports

def save_jsonl(reports: List[Dict[str, Any]], out_path: str):
    with open(out_path, "w", encoding="utf-8") as w:
        for rpt in reports:
            w.write(json.dumps(rpt, ensure_ascii=False) + "\n")

def save_iq_flat_csv(reports: List[Dict[str, Any]], out_path: str):
    with open(out_path, "w", newline="", encoding="utf-8") as w:
        writer = csv.writer(w)
        writer.writerow([
            "index", "timestamp_ms", "role",
            "ap_index", "tone_quality",
            "distance_ifft", "distance_phase_slope", "distance_rtt", "distance_best",
            "ch", "il", "ql", "ir", "qr"
        ])
        for rpt in reports:
            for ap in rpt.get("aps", []):
                iq = ap.get("iq", [])
                for ch, iqv in enumerate(iq):
                    if not iqv or len(iqv) != 4:
                        iqv = [float('nan')]*4
                    writer.writerow([
                        rpt.get("index"), rpt.get("timestamp_ms"), rpt.get("role"),
                        ap.get("ap_index"), ap.get("tone_quality"),
                        ap["distance"].get("ifft"), ap["distance"].get("phase_slope"),
                        ap["distance"].get("rtt"), ap["distance"].get("best"),
                        ch, iqv[0], iqv[1], iqv[2], iqv[3]
                    ])

# ========== 输出文件名前缀（含时间戳） ==========
def build_out_prefix(log_path: str, use_mtime: bool = False, fmt: str = "%Y%m%d_%H%M%S") -> str:
    p = Path(log_path)
    ts = datetime.fromtimestamp(p.stat().st_mtime) if use_mtime else datetime.now()
    stamp = ts.strftime(fmt)
    return f"{p.stem}_{stamp}"

# ========== 自动选择 .log 文件 ==========
def human_size(n: int) -> str:
    for unit in ["B", "KB", "MB", "GB"]:
        if n < 1024.0:
            return f"{n:.1f}{unit}"
        n /= 1024.0
    return f"{n:.1f}TB"

def pick_log_file(
    directory: Path,
    pattern: str = "*.log",
    strategy: str = "latest",
    interactive: bool = False
) -> Path:
    """
    在 directory 下用 pattern 查找日志文件。
    - 若无：抛异常
    - 若单个：返回
    - 若多个：
        - interactive=True：列出并让用户选择
        - 否则按 strategy 选择：
            latest(默认)=修改时间最新
            earliest=修改时间最早
            largest=文件最大
            name=按文件名倒序（字典序）
    """
    candidates = sorted(directory.glob(pattern))
    if not candidates:
        # 扩展：尝试 *.log*（有的工具会生成 .log.1/.log.2025-08-11）
        candidates = sorted(directory.glob("*.log*"))
    if not candidates:
        raise FileNotFoundError(f"目录 {directory} 下未找到匹配 {pattern} 的日志文件")

    if len(candidates) == 1 and not interactive:
        return candidates[0]

    if interactive:
        print("检测到多个日志文件，请选择：")
        rows = []
        for i, p in enumerate(candidates):
            try:
                st = p.stat()
                mtime = datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
                size = human_size(st.st_size)
            except Exception:
                mtime, size = "N/A", "N/A"
            rows.append((i, p, mtime, size))
        for i, p, mtime, size in rows:
            print(f"[{i}] {p.name:40s}  {mtime}  {size}")
        while True:
            sel = input("输入编号选择文件：").strip()
            if sel.isdigit():
                idx = int(sel)
                if 0 <= idx < len(candidates):
                    return candidates[idx]
            print("输入无效，请重试。")
    else:
        # 非交互：按策略选择
        keyfunc = None
        reverse = True
        if strategy == "latest":
            keyfunc = lambda p: p.stat().st_mtime
            reverse = True
        elif strategy == "earliest":
            keyfunc = lambda p: p.stat().st_mtime
            reverse = False
        elif strategy == "largest":
            keyfunc = lambda p: p.stat().st_size
            reverse = True
        elif strategy == "name":
            keyfunc = lambda p: p.name
            reverse = True
        else:
            raise ValueError(f"未知策略: {strategy}")
        chosen = sorted(candidates, key=keyfunc, reverse=reverse)[0]
        return chosen

# ========== CLI ==========
def parse_args():
    ap = argparse.ArgumentParser(description="解析 UWB/BLE CS 日志中的 report 数据")
    ap.add_argument("log_path", nargs="?", default=None,
                    help="日志文件路径；若不提供则自动在当前目录查找 *.log")
    ap.add_argument("--pattern", default="*.log",
                    help="自动查找时使用的通配模式（默认 *.log）")
    ap.add_argument("--dir", dest="search_dir", default=".",
                    help="自动查找的目录（默认当前目录）")
    ap.add_argument("--strategy", choices=["latest", "earliest", "largest", "name"],
                    default="latest", help="多文件时的自动选择策略（默认 latest）")
    ap.add_argument("--interactive", action="store_true",
                    help="多文件时进入交互选择")
    ap.add_argument("--use-mtime", action="store_true",
                    help="使用日志文件修改时间生成输出时间戳（默认用当前时间）")
    ap.add_argument("--outdir", default=None,
                    help="输出目录（默认与日志文件同目录）")
    ap.add_argument("--stamp-fmt", default="%Y%m%d_%H%M%S",
                    help="时间戳格式（默认 %%Y%%m%%d_%%H%%M%%S）")
    return ap.parse_args()

def main():
    args = parse_args()

    # 选择日志文件
    if args.log_path:
        log_path = Path(args.log_path)
        if not log_path.exists():
            print(f"指定的日志不存在：{log_path}", file=sys.stderr)
            sys.exit(1)
    else:
        log_path = pick_log_file(
            directory=Path(args.search_dir),
            pattern=args.pattern,
            strategy=args.strategy,
            interactive=args.interactive
        )
        print(f"自动选择日志：{log_path}")

    # 解析
    reports = parse_log(str(log_path))
    print(f"Parsed reports: {len(reports)}")

    # 输出路径
    out_dir = Path(args.outdir) if args.outdir else log_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    prefix = f"{log_path.stem}_{(datetime.fromtimestamp(log_path.stat().st_mtime) if args.use_mtime else datetime.now()).strftime(args.stamp_fmt)}"
    jsonl_path = out_dir / f"{prefix}_reports.jsonl"
    csv_path   = out_dir / f"{prefix}_iq_flat.csv"

    # 保存
    save_jsonl(reports, str(jsonl_path))
    save_iq_flat_csv(reports, str(csv_path))
    print(f"Saved: {jsonl_path}")
    print(f"Saved: {csv_path}")

if __name__ == "__main__":
    main()