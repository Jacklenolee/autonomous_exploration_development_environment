#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import html
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


THRESHOLD_PERCENT = 120.0
OPTIMIZATION_THRESHOLD = 100.0 / THRESHOLD_PERCENT * 100.0


@dataclass
class PathMetricRecord:
    time_duration: float
    start_x: float
    start_y: float
    start_z: float
    goal_x: float
    goal_y: float
    goal_z: float
    actual_path_length: float
    shortest_path_length: float
    actual_to_shortest_percent: float
    shortest_to_actual_percent: float


@dataclass
class PathMetricSegment:
    source_file: Path
    segment_index: int
    records: List[PathMetricRecord]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze path optimization logs and generate plots + HTML report."
    )
    parser.add_argument(
        "--log-dir",
        default="src/vehicle_simulator/log",
        help="Directory containing path_metrics_*.txt files.",
    )
    parser.add_argument(
        "--pattern",
        default="path_metrics_*.txt",
        help="Filename pattern to match inside the log directory.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Analyze every matching log file instead of only the latest one.",
    )
    parser.add_argument(
        "--outdir",
        default="",
        help="Output directory for the generated report. Defaults to a timestamped folder inside the log dir.",
    )
    return parser.parse_args()


def load_metric_rows(file_path: Path) -> List[PathMetricRecord]:
    rows: List[PathMetricRecord] = []
    with file_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) != 11:
                continue

            values = list(map(float, parts))
            rows.append(
                PathMetricRecord(
                    time_duration=values[0],
                    start_x=values[1],
                    start_y=values[2],
                    start_z=values[3],
                    goal_x=values[4],
                    goal_y=values[5],
                    goal_z=values[6],
                    actual_path_length=values[7],
                    shortest_path_length=values[8],
                    actual_to_shortest_percent=values[9],
                    shortest_to_actual_percent=values[10],
                )
            )
    return rows


def same_pose(a: PathMetricRecord, b: PathMetricRecord, tol: float = 1e-3) -> bool:
    fields = (
        "start_x",
        "start_y",
        "start_z",
        "goal_x",
        "goal_y",
        "goal_z",
        "shortest_path_length",
    )
    return all(abs(getattr(a, field) - getattr(b, field)) <= tol for field in fields)


def split_segments(rows: List[PathMetricRecord], source_file: Path) -> List[PathMetricSegment]:
    if not rows:
        return []

    segments: List[PathMetricSegment] = []
    current: List[PathMetricRecord] = [rows[0]]
    for row in rows[1:]:
        prev = current[-1]
        reset = row.actual_path_length + 1e-6 < prev.actual_path_length
        changed = not same_pose(prev, row)
        if reset or changed:
            segments.append(
                PathMetricSegment(
                    source_file=source_file,
                    segment_index=len(segments) + 1,
                    records=current,
                )
            )
            current = [row]
        else:
            current.append(row)

    segments.append(
        PathMetricSegment(
            source_file=source_file,
            segment_index=len(segments) + 1,
            records=current,
        )
    )
    return segments


def final_metrics(segment: PathMetricSegment) -> dict:
    row = segment.records[-1]
    actual = row.actual_path_length
    shortest = row.shortest_path_length
    ratio = row.actual_to_shortest_percent
    optimization = row.shortest_to_actual_percent
    passed = math.isfinite(ratio) and ratio <= THRESHOLD_PERCENT
    if actual <= 0 or shortest <= 0:
        passed = False
    return {
        "file": segment.source_file.name,
        "segment": segment.segment_index,
        "samples": len(segment.records),
        "start_time": segment.records[0].time_duration,
        "end_time": row.time_duration,
        "start_x": row.start_x,
        "start_y": row.start_y,
        "start_z": row.start_z,
        "goal_x": row.goal_x,
        "goal_y": row.goal_y,
        "goal_z": row.goal_z,
        "actual_path_length": actual,
        "shortest_path_length": shortest,
        "actual_to_shortest_percent": ratio,
        "shortest_to_actual_percent": optimization,
        "passed": passed,
    }


def plot_overview(summary_rows: List[dict], output_path: Path) -> None:
    if not summary_rows:
        return

    labels = [f"{row['file']}\n#{row['segment']}" for row in summary_rows]
    ratio_values = [row["actual_to_shortest_percent"] for row in summary_rows]
    opt_values = [row["shortest_to_actual_percent"] for row in summary_rows]
    colors = ["#2ca02c" if row["passed"] else "#d62728" for row in summary_rows]

    fig, axes = plt.subplots(2, 1, figsize=(max(10, len(summary_rows) * 0.95), 8), constrained_layout=True)

    axes[0].bar(range(len(summary_rows)), ratio_values, color=colors)
    axes[0].axhline(THRESHOLD_PERCENT, color="#111111", linestyle="--", linewidth=1.2, label="120% limit")
    axes[0].set_ylabel("Actual / shortest (%)")
    axes[0].set_title("Final path ratio by segment")
    axes[0].legend(loc="upper right")
    axes[0].grid(axis="y", alpha=0.25)

    axes[1].bar(range(len(summary_rows)), opt_values, color=colors)
    axes[1].axhline(OPTIMIZATION_THRESHOLD, color="#111111", linestyle="--", linewidth=1.2, label="83.33% target")
    axes[1].set_ylabel("Shortest / actual (%)")
    axes[1].set_title("Final path optimization by segment")
    axes[1].set_xticks(range(len(summary_rows)))
    axes[1].set_xticklabels(labels, rotation=25, ha="right")
    axes[1].legend(loc="upper right")
    axes[1].grid(axis="y", alpha=0.25)

    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def segment_plot_title(segment: PathMetricSegment, metrics: dict) -> str:
    status = "PASS" if metrics["passed"] else "FAIL"
    return (
        f"{segment.source_file.name} #{segment.segment_index} | {status} | "
        f"ratio={metrics['actual_to_shortest_percent']:.2f}% | "
        f"optimization={metrics['shortest_to_actual_percent']:.2f}%"
    )


def plot_segment(segment: PathMetricSegment, output_path: Path) -> None:
    rows = segment.records
    metrics = final_metrics(segment)

    start_time = rows[0].time_duration
    times = [row.time_duration - start_time for row in rows]
    actual = [row.actual_path_length for row in rows]
    shortest = [row.shortest_path_length for row in rows]
    ratio = [row.actual_to_shortest_percent if math.isfinite(row.actual_to_shortest_percent) else float("nan") for row in rows]
    optimization = [row.shortest_to_actual_percent if math.isfinite(row.shortest_to_actual_percent) else float("nan") for row in rows]

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)

    axes[0].plot(times, actual, color="#1f77b4", linewidth=2.0, label="Actual path length")
    axes[0].plot(times, shortest, color="#ff7f0e", linewidth=2.0, linestyle="--", label="Shortest path length")
    axes[0].set_ylabel("Length (m)")
    axes[0].set_title("Path length over time")
    axes[0].legend(loc="upper left")
    axes[0].grid(alpha=0.25)

    axes[1].plot(times, ratio, color="#d62728", linewidth=2.0, label="Actual / shortest (%)")
    axes[1].plot(times, optimization, color="#2ca02c", linewidth=2.0, label="Shortest / actual (%)")
    axes[1].axhline(THRESHOLD_PERCENT, color="#d62728", linestyle="--", linewidth=1.2, label="120% limit")
    axes[1].axhline(OPTIMIZATION_THRESHOLD, color="#2ca02c", linestyle="--", linewidth=1.2, label="83.33% target")
    axes[1].set_ylabel("Percent (%)")
    axes[1].set_xlabel("Time since segment start (s)")
    axes[1].set_title("Path optimization over time")
    axes[1].legend(loc="upper left", ncol=2)
    axes[1].grid(alpha=0.25)

    fig.suptitle(segment_plot_title(segment, metrics), fontsize=11)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def write_summary_csv(summary_rows: List[dict], output_path: Path) -> None:
    fieldnames = [
        "file",
        "segment",
        "samples",
        "start_time",
        "end_time",
        "start_x",
        "start_y",
        "start_z",
        "goal_x",
        "goal_y",
        "goal_z",
        "actual_path_length",
        "shortest_path_length",
        "actual_to_shortest_percent",
        "shortest_to_actual_percent",
        "passed",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in summary_rows:
            writer.writerow(row)


def render_html(summary_rows: List[dict], overview_png: str, segment_pngs: List[tuple], output_path: Path) -> None:
    total = len(summary_rows)
    passed = sum(1 for row in summary_rows if row["passed"])
    failed = total - passed
    mean_ratio = sum(row["actual_to_shortest_percent"] for row in summary_rows) / total if total else float("nan")

    rows_html = []
    for row in summary_rows:
        status = "PASS" if row["passed"] else "FAIL"
        status_class = "pass" if row["passed"] else "fail"
        rows_html.append(
            "<tr>"
            f"<td>{html.escape(row['file'])}</td>"
            f"<td>{row['segment']}</td>"
            f"<td>{row['samples']}</td>"
            f"<td>{row['actual_path_length']:.3f}</td>"
            f"<td>{row['shortest_path_length']:.3f}</td>"
            f"<td>{row['actual_to_shortest_percent']:.2f}</td>"
            f"<td>{row['shortest_to_actual_percent']:.2f}</td>"
            f"<td class='{status_class}'>{status}</td>"
            "</tr>"
        )

    segment_blocks = []
    for image_name, row in segment_pngs:
        segment_blocks.append(
            f"<section class='segment'>"
            f"<h2>{html.escape(row['file'])} #{row['segment']}</h2>"
            f"<p>start=({row['start_x']:.3f}, {row['start_y']:.3f}, {row['start_z']:.3f}) "
            f"goal=({row['goal_x']:.3f}, {row['goal_y']:.3f}, {row['goal_z']:.3f})</p>"
            f"<img src='{html.escape(image_name)}' alt='segment plot'>"
            "</section>"
        )

    html_text = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Path Optimization Report</title>
  <style>
    body {{
      font-family: Arial, Helvetica, sans-serif;
      margin: 24px;
      color: #1a1a1a;
      background: #f7f7f7;
    }}
    h1, h2 {{
      margin: 0 0 12px 0;
    }}
    .card {{
      background: #fff;
      border: 1px solid #ddd;
      border-radius: 6px;
      padding: 16px;
      margin-bottom: 18px;
      box-shadow: 0 1px 3px rgba(0, 0, 0, 0.04);
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 14px;
      background: #fff;
    }}
    th, td {{
      border: 1px solid #ddd;
      padding: 8px 10px;
      text-align: left;
      vertical-align: top;
    }}
    th {{
      background: #f0f0f0;
    }}
    tr:nth-child(even) td {{
      background: #fbfbfb;
    }}
    .pass {{
      color: #1b7f3a;
      font-weight: bold;
    }}
    .fail {{
      color: #b42318;
      font-weight: bold;
    }}
    img {{
      max-width: 100%;
      height: auto;
      display: block;
      margin-top: 12px;
    }}
    .segment {{
      page-break-inside: avoid;
      margin-bottom: 24px;
    }}
    .meta {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 12px;
      margin-bottom: 16px;
    }}
    .stat {{
      background: #fff;
      border: 1px solid #ddd;
      border-radius: 6px;
      padding: 12px;
    }}
    .stat strong {{
      display: block;
      font-size: 20px;
      margin-top: 6px;
    }}
  </style>
</head>
<body>
  <h1>Path Optimization Report</h1>
  <div class="meta">
    <div class="stat">Segments<strong>{total}</strong></div>
    <div class="stat">Passed<strong>{passed}</strong></div>
    <div class="stat">Failed<strong>{failed}</strong></div>
    <div class="stat">Mean ratio<strong>{mean_ratio:.2f}%</strong></div>
  </div>
  <div class="card">
    <h2>Overview</h2>
    <p>Pass condition: actual / shortest <= {THRESHOLD_PERCENT:.0f}% and shortest / actual >= {OPTIMIZATION_THRESHOLD:.2f}%.</p>
    <img src="{html.escape(overview_png)}" alt="overview plot">
  </div>
  <div class="card">
    <h2>Summary Table</h2>
    <table>
      <thead>
        <tr>
          <th>File</th>
          <th>Segment</th>
          <th>Samples</th>
          <th>Actual</th>
          <th>Shortest</th>
          <th>Actual / Shortest (%)</th>
          <th>Shortest / Actual (%)</th>
          <th>Status</th>
        </tr>
      </thead>
      <tbody>
        {''.join(rows_html)}
      </tbody>
    </table>
  </div>
  <div class="card">
    <h2>Segment Details</h2>
    {''.join(segment_blocks)}
  </div>
</body>
</html>
"""

    output_path.write_text(html_text, encoding="utf-8")


def collect_input_files(log_dir: Path, pattern: str, analyze_all: bool) -> List[Path]:
    files = sorted(log_dir.glob(pattern), key=lambda p: p.stat().st_mtime)
    if not files:
        return []
    if analyze_all:
        return files
    return [files[-1]]


def main() -> int:
    args = parse_args()
    log_dir = Path(args.log_dir).resolve()
    if not log_dir.exists():
        print(f"Log directory not found: {log_dir}")
        return 1

    files = collect_input_files(log_dir, args.pattern, args.all)
    if not files:
        print(f"No log files matched {args.pattern} in {log_dir}")
        return 1

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.outdir:
      out_dir = Path(args.outdir).resolve()
    else:
      out_dir = log_dir / f"path_optimization_report_{timestamp}"
    out_dir.mkdir(parents=True, exist_ok=True)

    segments: List[PathMetricSegment] = []
    for file_path in files:
        rows = load_metric_rows(file_path)
        segments.extend(split_segments(rows, file_path))

    summary_rows = [final_metrics(segment) for segment in segments]
    summary_csv = out_dir / "summary.csv"
    overview_png = out_dir / "summary.png"
    report_html = out_dir / "report.html"

    write_summary_csv(summary_rows, summary_csv)
    plot_overview(summary_rows, overview_png)

    segment_pngs = []
    for index, segment in enumerate(segments, start=1):
        image_name = f"segment_{index:02d}.png"
        plot_segment(segment, out_dir / image_name)
        segment_pngs.append((image_name, final_metrics(segment)))

    render_html(summary_rows, overview_png.name, segment_pngs, report_html)

    print(f"Report written to: {report_html}")
    print(f"Summary CSV: {summary_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
