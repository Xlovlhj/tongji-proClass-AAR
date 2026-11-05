#!/usr/bin/env python3
"""Audit L0-L4 Markdown reports and build a traceable asset manifest.

The checker is intentionally limited to the Python standard library so it can
run outside Isaac Sim.  It verifies that locally embedded images exist, that
stage reports do not embed images from another stage, and that the Markdown
formula delimiters match the previewer's ``$``/``$$`` convention.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import unquote


STAGES = tuple(f"L{index}" for index in range(5))
PRIMARY_REPORTS = {
    "L0": "L0验收报告.md",
    "L1": "L1源码分析报告.md",
    "L2": "L2验收报告.md",
    "L3": "L3理论联系报告.md",
    "L4": "L4验收报告.md",
}
# These counts encode the current report contract: L0 has four formal final
# frames plus three input-weight tuning plots, L2 has one framework plus
# twelve experiment plots, and L4 contains the four framework figures plus
# all five challenge tasks' formal plots.
MINIMUM_PRIMARY_IMAGES = {
    "L0": 7,
    "L1": 1,
    "L2": 13,
    "L3": 1,
    "L4": 59,
}
EVIDENCE_SUFFIXES = {".csv", ".json", ".mp4", ".png", ".jpg", ".jpeg"}
MARKDOWN_LINK_RE = re.compile(r"(?P<image>!)?\[[^\]]*\]\((?P<target>[^)]+)\)")
BACKTICK_RE = re.compile(r"`([^`\n]+)`")


@dataclass
class ReportAudit:
    path: Path
    images: list[Path] = field(default_factory=list)
    evidence: list[Path] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="检查 Pro2/output/L0-L4 报告并生成总报告素材清单。"
    )
    parser.add_argument(
        "--write-manifest",
        action="store_true",
        help="写入 Pro2/output/report_asset_manifest.json。",
    )
    return parser.parse_args()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def display_path(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def strip_link_title(target: str) -> str:
    value = unquote(target.strip())
    if value.startswith("<") and ">" in value:
        return value[1 : value.index(">")]
    # Markdown permits an optional quoted title after the URL/path.
    match = re.match(r"^(.*?)(?:\s+[\"'][^\"']*[\"'])?$", value)
    return match.group(1).strip() if match else value


def resolve_local_reference(raw: str, report: Path, root: Path) -> Path | None:
    target = strip_link_title(raw)
    if not target or target.startswith(("http://", "https://", "#", "mailto:")):
        return None
    target = target.split("#", 1)[0]
    candidate = Path(target)
    if candidate.is_absolute():
        return candidate.resolve()
    if target.startswith("Pro2/"):
        return (root / candidate).resolve()
    return (report.parent / candidate).resolve()


def content_without_code(text: str) -> str:
    kept: list[str] = []
    in_fence = False
    for line in text.splitlines():
        if line.lstrip().startswith("```"):
            in_fence = not in_fence
            continue
        if not in_fence:
            kept.append(BACKTICK_RE.sub("", line))
    return "\n".join(kept)


def check_formula_syntax(text: str) -> list[str]:
    errors: list[str] = []
    prose = content_without_code(text)
    forbidden = {r"\(": "\\(", r"\)": "\\)", r"\[": "\\[", r"\]": "\\]"}
    for token, label in forbidden.items():
        if token in prose:
            errors.append(f"使用了不兼容的公式分隔符 {label}")

    display_lines = [
        (number, line)
        for number, line in enumerate(prose.splitlines(), start=1)
        if "$$" in line
    ]
    if len(display_lines) % 2:
        errors.append("行间公式的 $$ 数量为奇数")
    for number, line in display_lines:
        if line.strip() != "$$":
            errors.append(f"第 {number} 行的 $$ 未独占一行")
    return errors


def check_fences(text: str) -> list[str]:
    fence_count = sum(
        1 for line in text.splitlines() if line.lstrip().startswith("```")
    )
    return [] if fence_count % 2 == 0 else ["Markdown 代码围栏数量为奇数"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def audit_report(report: Path, stage_dir: Path, root: Path) -> ReportAudit:
    result = ReportAudit(path=report)
    text = report.read_text(encoding="utf-8")
    result.errors.extend(check_formula_syntax(text))
    result.errors.extend(check_fences(text))

    for match in MARKDOWN_LINK_RE.finditer(text):
        resolved = resolve_local_reference(match.group("target"), report, root)
        if resolved is None:
            continue
        if not resolved.exists():
            result.errors.append(
                f"本地链接不存在: {display_path(resolved, root)}"
            )
            continue
        if match.group("image"):
            result.images.append(resolved)
            try:
                resolved.relative_to(stage_dir.resolve())
            except ValueError:
                result.errors.append(
                    "嵌入图片不在本阶段目录内: "
                    f"{display_path(resolved, root)}"
                )
        elif resolved.is_file() and resolved.suffix.lower() in EVIDENCE_SUFFIXES:
            result.evidence.append(resolved)

    # Reports also use backticks for explicit project-root-relative evidence paths.
    for raw in BACKTICK_RE.findall(text):
        if " " in raw and not raw.startswith("Pro2/"):
            continue
        resolved = resolve_local_reference(raw, report, root)
        if (
            resolved is not None
            and resolved.exists()
            and resolved.is_file()
            and resolved.suffix.lower() in EVIDENCE_SUFFIXES
        ):
            result.evidence.append(resolved)

    result.images = sorted(set(result.images))
    result.evidence = sorted(set(result.evidence) | set(result.images))
    return result


def build_manifest(root: Path) -> tuple[dict[str, object], list[str]]:
    output_root = root / "Pro2" / "output"
    all_errors: list[str] = []
    stage_entries: dict[str, object] = {}

    for stage in STAGES:
        stage_dir = output_root / stage
        primary = stage_dir / PRIMARY_REPORTS[stage]
        reports = sorted(stage_dir.rglob("*.md"))
        if not primary.exists():
            all_errors.append(
                f"{stage}: 缺少主报告 {display_path(primary, root)}"
            )

        report_entries: list[dict[str, object]] = []
        for report in reports:
            audit = audit_report(report, stage_dir, root)
            for error in audit.errors:
                all_errors.append(f"{display_path(report, root)}: {error}")
            report_entries.append(
                {
                    "path": display_path(report, root),
                    "primary": report == primary,
                    "embedded_images": [
                        display_path(path, root) for path in audit.images
                    ],
                    "referenced_evidence": [
                        display_path(path, root) for path in audit.evidence
                    ],
                    "errors": audit.errors,
                }
            )

        primary_entry = next(
            (entry for entry in report_entries if entry["primary"]), None
        )
        primary_image_count = (
            len(primary_entry["embedded_images"]) if primary_entry else 0
        )
        minimum_images = MINIMUM_PRIMARY_IMAGES[stage]
        if primary_image_count < minimum_images:
            message = (
                f"{stage}: 主报告只嵌入 {primary_image_count} 张不同图片，"
                f"当前成果合同至少需要 {minimum_images} 张"
            )
            all_errors.append(message)
            if primary_entry is not None:
                primary_entry["errors"].append(message)

        asset_paths = sorted(
            {
                root / asset
                for entry in report_entries
                for key in ("embedded_images", "referenced_evidence")
                for asset in entry[key]
                if (root / asset).is_file()
            }
        )
        stage_entries[stage] = {
            "primary_report": display_path(primary, root),
            "minimum_required_primary_images": minimum_images,
            "primary_image_count": primary_image_count,
            "report_count": len(reports),
            "reports": report_entries,
            "referenced_asset_count": len(asset_paths),
            "referenced_assets": [
                {
                    "path": display_path(path, root),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
                for path in asset_paths
            ],
        }

    manifest: dict[str, object] = {
        "schema_version": 1,
        "scope": "Pro2 L0-L4 Markdown reports",
        "stages": stage_entries,
        "error_count": len(all_errors),
        "errors": all_errors,
    }
    return manifest, all_errors


def main() -> int:
    args = parse_args()
    root = repo_root()
    manifest, errors = build_manifest(root)
    if args.write_manifest:
        destination = root / "Pro2" / "output" / "report_asset_manifest.json"
        destination.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"已写入 {display_path(destination, root)}")

    for stage in STAGES:
        entry = manifest["stages"][stage]
        primary_entry = next(
            (report for report in entry["reports"] if report["primary"]), None
        )
        image_count = len(primary_entry["embedded_images"]) if primary_entry else 0
        print(
            f"{stage}: reports={entry['report_count']}, "
            f"primary_images={image_count}, assets={entry['referenced_asset_count']}"
        )
    if errors:
        print("\n检查失败：")
        for error in errors:
            print(f"- {error}")
        return 1
    print("\n检查通过：图片链接、阶段边界、公式分隔符和代码围栏均正常。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
