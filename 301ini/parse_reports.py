#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re
import json
import csv
import sys
from typing import List, Dict, Any
from pathlib import Path
from datetime import datetime
import argparse

# 用于去掉 ANSI 转义色彩码（日志里类似 [0m）
ANSI_ESCAPE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")

# 关键行的正则
re_basic   = re.compile(r"== Basic Report == index:(\d+), timestamp:(\d+)")
re_role    = re.compile(r"role=(\w+), n_ap=(\d+)")
re_rtt     = re.compile(r"rtt_accumulated_half_ns=([+-]?\d+), rtt_count=(\d+)")
re_ap      = re.compile(r"-- Antenna Path (\d+) --")
re_tone    = re.compile(r"Tone=(\w+)\s+Dist\(ifft/phase_slope/rtt/best\)=([^/]+)/([^/]+)/([^/]+)/([^/\s]+)")

# IQ 片段：ch:<idx>:<il>,<ql>,<ir>,<qr>;
# 允许负号/小数/nan/inf
NUM = r"[+-]?(?:\d+(?:\.\d+)?|nan|inf|-inf)"
re_iq_line_has_prefix = re.compile(r"\bIQ:\s*(.*)")
re_iq_tokens = re.compile(rf"ch:(\d+):({NUM}),({NUM}),({NUM}),({NUM});")

def to_float(x: str) -> float:
    # Python 的 float('nan') / float('inf') 都支持
    try:
        return float(x)
    except Exception:
        return float('nan')

def flush_current_report(reports: List[Dict[str, Any]], current_report: Dict[str, Any]):
    if not current_report:
        return
    # 将每个 AP 的 IQ 映射由 {ch: [il,ql,ir,qr]} 规范化为按通道排序的列表
    for ap in current_report.get("aps", []):
        iq_map = ap.get("iq", {})
        if iq_map:
            max_ch = max(iq_map.keys())
            # 填充缺失通道为 [nan,nan,nan,nan]
            ap["iq"] = [iq_map.get(ch, [float('nan')]*4) for ch in range(max_ch + 1)]
        else:
            ap["iq"] = []
    reports.append(current_report)

def parse_log(path: str):
    reports: List[Dict[str, Any]] = []
    current_report: Dict[str, Any] = {}
    current_ap: Dict[str, Any] = None

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = ANSI_ESCAPE.sub("", raw).strip()
            if not line:
                continue

            # 起一个新 report
            m = re_basic.search(line)
            if m:
                # 收尾上一份
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

            # 基本信息
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

            # AP 起始
            m = re_ap.search(line)
            if m and current_report:
                ap_idx = int(m.group(1))
                current_ap = {
                    "ap_index": ap_idx,
                    "tone_quality": None,
                    "distance": {"ifft": None, "phase_slope": None, "rtt": None, "best": None},
                    "iq": {}  # 暂存为 {ch: [il,ql,ir,qr]}
                }
                current_report["aps"].append(current_ap)
                continue

            # tone + distance
            m = re_tone.search(line)
            if m and current_ap is not None:
                current_ap["tone_quality"] = m.group(1)
                current_ap["distance"]["ifft"] = to_float(m.group(2))
                current_ap["distance"]["phase_slope"] = to_float(m.group(3))
                current_ap["distance"]["rtt"] = to_float(m.group(4))
                current_ap["distance"]["best"] = to_float(m.group(5))
                continue

            # IQ 首行（含 IQ: 前缀）或续行
            if current_ap is not None:
                payload = None
                m = re_iq_line_has_prefix.search(line)
                if m:
                    payload = m.group(1)  # 去掉 IQ: 前缀
                else:
                    # 也可能是直接以 ch: 开头的续行
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

    # 文件结束，收尾
    flush_current_report(reports, current_report)
    return reports

def save_jsonl(reports: List[Dict[str, Any]], out_path: str):
    with open(out_path, "w", encoding="utf-8") as w:
        for rpt in reports:
            w.write(json.dumps(rpt, ensure_ascii=False) + "\n")

def save_iq_flat_csv(reports: List[Dict[str, Any]], out_path: str):
    """
    扁平化导出：每行 = 一个(report, ap, channel)
    列：index, timestamp_ms, role, ap_index, tone_quality, distance_*, ch, il, ql, ir, qr
    """
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
                        # 填充缺失
                        iqv = [float('nan')]*4
                    writer.writerow([
                        rpt.get("index"), rpt.get("timestamp_ms"), rpt.get("role"),
                        ap.get("ap_index"), ap.get("tone_quality"),
                        ap["distance"].get("ifft"), ap["distance"].get("phase_slope"),
                        ap["distance"].get("rtt"), ap["distance"].get("best"),
                        ch, iqv[0], iqv[1], iqv[2], iqv[3]
                    ])

def build_out_prefix(log_path: str, use_mtime: bool = False, fmt: str = "%Y%m%d_%H%M%S") -> str:
    """
    根据日志文件名构造输出前缀：<stem>_<时间戳>
    use_mtime=True 时使用日志文件的修改时间；否则使用当前时间。
    """
    p = Path(log_path)
    ts = datetime.fromtimestamp(p.stat().st_mtime) if use_mtime else datetime.now()
    stamp = ts.strftime(fmt)
    return f"{p.stem}_{stamp}"

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("log_path", nargs="?", default="UartAssist.log")
    ap.add_argument("--use-mtime", action="store_true",
                    help="使用日志文件的修改时间作为时间戳（默认用当前时间）")
    ap.add_argument("--outdir", default=None,
                    help="输出目录（默认与日志文件同目录）")
    return ap.parse_args()

def main():
    args = parse_args()
    log_path = args.log_path

    reports = parse_log(log_path)
    print(f"Parsed reports: {len(reports)}")

    out_dir = Path(args.outdir) if args.outdir else Path(log_path).parent
    out_dir.mkdir(parents=True, exist_ok=True)

    prefix = build_out_prefix(log_path, use_mtime=args.use_mtime)
    jsonl_path = out_dir / f"{prefix}_reports.jsonl"
    csv_path   = out_dir / f"{prefix}_iq_flat.csv"

    save_jsonl(reports, str(jsonl_path))
    save_iq_flat_csv(reports, str(csv_path))
    print(f"Saved: {jsonl_path}")
    print(f"Saved: {csv_path}")

if __name__ == "__main__":
    main()