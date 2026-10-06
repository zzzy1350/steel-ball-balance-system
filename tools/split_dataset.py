#!/usr/bin/env python3
"""Copy validated source data into deterministic YOLO train/val/test splits."""

from __future__ import annotations

import argparse
import math
import random
import re
import shutil
import sys
from collections import defaultdict
from pathlib import Path

from check_dataset import ValidationReport, default_dataset_dir, print_report, validate_dataset


SPLIT_NAMES = ("train", "val", "test")
IGNORED_OUTPUT_NAMES = {".gitkeep"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="检查并划分钢球 YOLO 数据集；原始文件只复制，不移动。"
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=default_dataset_dir(),
        help="数据集根目录（默认：项目中的 steel_ball_dataset）",
    )
    parser.add_argument("--train", type=float, default=0.8, help="训练集比例（默认：0.8）")
    parser.add_argument("--val", type=float, default=0.1, help="验证集比例（默认：0.1）")
    parser.add_argument("--test", type=float, default=0.1, help="测试集比例（默认：0.1）")
    parser.add_argument("--seed", type=int, default=42, help="随机种子（默认：42）")
    parser.add_argument(
        "--group-regex",
        type=str,
        default=None,
        help=(
            "可选：从文件名中提取拍摄序列分组的正则表达式；同组图片不会跨子集。"
            "有捕获组时使用第一个捕获组，否则使用完整匹配。"
        ),
    )
    return parser.parse_args()


def validate_ratios(ratios: tuple[float, float, float]) -> str | None:
    if not all(math.isfinite(value) for value in ratios):
        return "划分比例必须是有限数值"
    if any(value < 0.0 for value in ratios):
        return "划分比例不能小于 0"
    if ratios[0] <= 0.0 or ratios[1] <= 0.0:
        return "训练集和验证集比例必须大于 0"
    if not math.isclose(sum(ratios), 1.0, rel_tol=0.0, abs_tol=1e-9):
        return f"train、val、test 比例之和必须为 1，当前为 {sum(ratios):.12g}"
    return None


def allocate_counts(total: int, ratios: tuple[float, float, float]) -> tuple[int, int, int]:
    """Use largest-remainder apportionment, with non-empty train and val."""
    if total < 2:
        raise ValueError("至少需要 2 张已标注图片，才能保证训练集和验证集都有数据")

    exact = [total * ratio for ratio in ratios]
    counts = [math.floor(value) for value in exact]
    remaining = total - sum(counts)
    order = sorted(
        range(3),
        key=lambda index: (exact[index] - counts[index], ratios[index], -index),
        reverse=True,
    )
    for index in order[:remaining]:
        counts[index] += 1

    for required_index in (0, 1):
        if counts[required_index] > 0:
            continue
        donor_candidates = [
            index
            for index in range(3)
            if counts[index] > (1 if index in (0, 1) else 0)
        ]
        if not donor_candidates:
            raise ValueError("图片数量不足，无法同时创建训练集和验证集")
        donor = max(donor_candidates, key=lambda index: counts[index] - exact[index])
        counts[donor] -= 1
        counts[required_index] += 1

    return counts[0], counts[1], counts[2]


def _compile_group_regex(pattern: str | None) -> re.Pattern[str] | None:
    if pattern is None:
        return None
    try:
        return re.compile(pattern)
    except re.error as exc:
        raise ValueError(f"分组正则表达式无效：{exc}") from exc


def _group_key(image_path: Path, pattern: re.Pattern[str] | None) -> str:
    if pattern is None:
        return image_path.stem.casefold()
    match = pattern.search(image_path.stem)
    if match is None:
        return f"__single__:{image_path.stem.casefold()}"
    value = match.group(1) if match.lastindex else match.group(0)
    return f"__group__:{value.casefold()}"


def _plain_split(
    images: list[Path],
    counts: tuple[int, int, int],
    rng: random.Random,
) -> dict[str, list[Path]]:
    shuffled = sorted(images, key=lambda path: path.name.casefold())
    rng.shuffle(shuffled)
    train_count, val_count, _ = counts
    return {
        "train": shuffled[:train_count],
        "val": shuffled[train_count : train_count + val_count],
        "test": shuffled[train_count + val_count :],
    }


def _grouped_split(
    images: list[Path],
    target_counts: tuple[int, int, int],
    pattern: re.Pattern[str],
    rng: random.Random,
) -> dict[str, list[Path]]:
    grouped: dict[str, list[Path]] = defaultdict(list)
    for image_path in sorted(images, key=lambda path: path.name.casefold()):
        grouped[_group_key(image_path, pattern)].append(image_path)

    groups = list(grouped.values())
    if len(groups) < 2:
        raise ValueError(
            "分组后少于 2 个独立序列，无法保证训练集和验证集都有数据；"
            "请调整 --group-regex 或增加不同序列"
        )

    rng.shuffle(groups)
    groups.sort(key=len, reverse=True)
    assignments: dict[str, list[Path]] = {name: [] for name in SPLIT_NAMES}
    current_counts = [0, 0, 0]

    first_two = groups[:2]
    direct_cost = abs(len(first_two[0]) - target_counts[0]) + abs(
        len(first_two[1]) - target_counts[1]
    )
    swapped_cost = abs(len(first_two[1]) - target_counts[0]) + abs(
        len(first_two[0]) - target_counts[1]
    )
    if swapped_cost < direct_cost:
        first_two.reverse()

    assignments["train"].extend(first_two[0])
    assignments["val"].extend(first_two[1])
    current_counts[0] += len(first_two[0])
    current_counts[1] += len(first_two[1])

    for group in groups[2:]:
        best_index = min(
            range(3),
            key=lambda candidate: (
                sum(
                    abs(
                        current_counts[index]
                        + (len(group) if index == candidate else 0)
                        - target_counts[index]
                    )
                    for index in range(3)
                ),
                current_counts[candidate] / max(target_counts[candidate], 1),
                candidate,
            ),
        )
        assignments[SPLIT_NAMES[best_index]].extend(group)
        current_counts[best_index] += len(group)

    for paths in assignments.values():
        paths.sort(key=lambda path: path.name.casefold())
    return assignments


def _ensure_output_is_empty(dataset_dir: Path) -> list[str]:
    existing: list[str] = []
    for kind in ("images", "labels"):
        for split_name in SPLIT_NAMES:
            directory = dataset_dir / kind / split_name
            if not directory.is_dir():
                existing.append(f"缺少目标目录：{directory}")
                continue
            for path in directory.iterdir():
                if path.name.lower() not in IGNORED_OUTPUT_NAMES:
                    existing.append(f"目标目录已有内容：{path}")
    return existing


def _find_label(source_labels: Path, image_path: Path) -> Path:
    exact_path = source_labels / f"{image_path.stem}.txt"
    if exact_path.is_file():
        return exact_path
    candidates = [
        path
        for path in source_labels.glob("*.txt")
        if path.name.casefold() != "classes.txt"
        and path.stem.casefold() == image_path.stem.casefold()
    ]
    if len(candidates) != 1:
        raise RuntimeError(f"复制时无法唯一定位标签：{image_path.name}")
    return candidates[0]


def _copy_assignments(
    dataset_dir: Path,
    report: ValidationReport,
    assignments: dict[str, list[Path]],
) -> dict[str, tuple[int, int]]:
    source_labels = dataset_dir / "source" / "labels"
    summary: dict[str, tuple[int, int]] = {}

    for split_name in SPLIT_NAMES:
        image_target = dataset_dir / "images" / split_name
        label_target = dataset_dir / "labels" / split_name
        split_boxes = 0

        for image_path in assignments[split_name]:
            normalized_stem = image_path.stem.casefold()
            label_path = _find_label(source_labels, image_path)
            shutil.copy2(image_path, image_target / image_path.name)
            shutil.copy2(label_path, label_target / f"{image_path.stem}.txt")
            split_boxes += report.boxes_by_stem.get(normalized_stem, 0)

        summary[split_name] = (len(assignments[split_name]), split_boxes)

    return summary


def main() -> int:
    args = parse_args()
    dataset_dir = args.dataset.resolve()
    ratios = (args.train, args.val, args.test)

    ratio_error = validate_ratios(ratios)
    if ratio_error:
        print(f"错误：{ratio_error}", file=sys.stderr)
        return 2

    try:
        group_pattern = _compile_group_regex(args.group_regex)
    except ValueError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2

    report = validate_dataset(dataset_dir)
    print_report(report)
    if not report.is_valid:
        print("\n数据集检查失败，未执行划分。", file=sys.stderr)
        return 1

    if report.image_count < 2:
        print("\n错误：至少需要 2 张已标注图片才能划分训练集和验证集。", file=sys.stderr)
        return 2

    existing_output = _ensure_output_is_empty(dataset_dir)
    if existing_output:
        print("\n为避免覆盖已有数据，未执行划分：", file=sys.stderr)
        for item in existing_output:
            print(f"  - {item}", file=sys.stderr)
        return 3

    try:
        target_counts = allocate_counts(report.image_count, ratios)
        images = list(report.images_by_stem.values())
        rng = random.Random(args.seed)
        if group_pattern is None:
            assignments = _plain_split(images, target_counts, rng)
        else:
            assignments = _grouped_split(images, target_counts, group_pattern, rng)
        summary = _copy_assignments(dataset_dir, report, assignments)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"\n划分失败：{exc}", file=sys.stderr)
        return 4


    print("\n数据集划分完成")
    print(f"  随机种子：{args.seed}")
    print(f"  请求比例：train={args.train:g}, val={args.val:g}, test={args.test:g}")
    if group_pattern is not None:
        print(f"  序列分组：{args.group_regex}")
    for split_name in SPLIT_NAMES:
        image_count, box_count = summary[split_name]
        print(f"  {split_name:<5} 图片：{image_count:>5}，钢球框：{box_count:>5}")
    print("\n原始 source 文件保持不变。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
