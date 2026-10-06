#!/usr/bin/env python3
"""Validate the source portion of the steel-ball YOLO dataset."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

try:
    from PIL import Image, UnidentifiedImageError
except ImportError:  # pragma: no cover - handled explicitly in main
    Image = None  # type: ignore[assignment]
    UnidentifiedImageError = OSError  # type: ignore[assignment,misc]


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}
IGNORED_FILENAMES = {".gitkeep", "classes.txt"}


@dataclass
class ValidationReport:
    image_count: int = 0
    labeled_image_count: int = 0
    negative_image_count: int = 0
    box_count: int = 0
    errors: list[str] = field(default_factory=list)
    boxes_by_stem: dict[str, int] = field(default_factory=dict)
    images_by_stem: dict[str, Path] = field(default_factory=dict)

    @property
    def is_valid(self) -> bool:
        return not self.errors


def default_dataset_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "steel_ball_dataset"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="检查钢球 YOLO 源图片与标签是否完整、合法。"
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=default_dataset_dir(),
        help="数据集根目录（默认：项目中的 steel_ball_dataset）",
    )
    return parser.parse_args()


def _visible_files(directory: Path) -> list[Path]:
    return sorted(
        (
            path
            for path in directory.iterdir()
            if path.is_file() and path.name.lower() not in IGNORED_FILENAMES
        ),
        key=lambda path: path.name.lower(),
    )


def _check_image(image_path: Path) -> str | None:
    assert Image is not None
    try:
        with Image.open(image_path) as image:
            image.verify()
        # verify() checks structure; loading also catches truncated image data.
        with Image.open(image_path) as image:
            image.load()
            if image.width <= 0 or image.height <= 0:
                return "图片宽度或高度无效"
    except (UnidentifiedImageError, OSError, ValueError) as exc:
        return f"图片损坏或无法读取：{exc}"
    return None


def _parse_label(label_path: Path) -> tuple[int, list[str]]:
    errors: list[str] = []
    box_count = 0

    try:
        text = label_path.read_text(encoding="utf-8-sig")
    except (OSError, UnicodeError) as exc:
        return 0, [f"{label_path}: 标签无法读取：{exc}"]

    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue

        fields = line.split()
        location = f"{label_path}:{line_number}"
        if len(fields) != 5:
            errors.append(f"{location}: 应有 5 个字段，实际为 {len(fields)} 个")
            continue

        class_token, *coordinate_tokens = fields
        try:
            class_id = int(class_token)
        except ValueError:
            errors.append(f"{location}: 类别编号必须是整数 0，实际为 {class_token!r}")
            continue

        if class_id != 0:
            errors.append(f"{location}: 只允许类别 0（steel_ball），实际为 {class_id}")

        try:
            x_center, y_center, width, height = map(float, coordinate_tokens)
        except ValueError:
            errors.append(f"{location}: 坐标和宽高必须是数字")
            continue

        values = (x_center, y_center, width, height)
        if not all(math.isfinite(value) for value in values):
            errors.append(f"{location}: 坐标和宽高必须是有限数值")
            continue

        if not 0.0 <= x_center <= 1.0:
            errors.append(f"{location}: x_center 必须位于 0～1，实际为 {x_center}")
        if not 0.0 <= y_center <= 1.0:
            errors.append(f"{location}: y_center 必须位于 0～1，实际为 {y_center}")
        if not 0.0 < width <= 1.0:
            errors.append(f"{location}: width 必须大于 0 且不超过 1，实际为 {width}")
        if not 0.0 < height <= 1.0:
            errors.append(f"{location}: height 必须大于 0 且不超过 1，实际为 {height}")

        left = x_center - width / 2.0
        right = x_center + width / 2.0
        top = y_center - height / 2.0
        bottom = y_center + height / 2.0
        tolerance = 1e-9
        if (
            left < -tolerance
            or top < -tolerance
            or right > 1.0 + tolerance
            or bottom > 1.0 + tolerance
        ):
            errors.append(f"{location}: 矩形框边界超出图片归一化范围 0～1")

        box_count += 1

    return box_count, errors


def validate_dataset(dataset_dir: Path) -> ValidationReport:
    dataset_dir = dataset_dir.resolve()
    images_dir = dataset_dir / "source" / "images"
    labels_dir = dataset_dir / "source" / "labels"
    report = ValidationReport()

    if Image is None:
        report.errors.append("缺少 Pillow，无法验证图片。请运行：pip install Pillow")
        return report

    if not images_dir.is_dir():
        report.errors.append(f"图片目录不存在：{images_dir}")
    if not labels_dir.is_dir():
        report.errors.append(f"标签目录不存在：{labels_dir}")
    if report.errors:
        return report

    image_files: list[Path] = []
    for path in _visible_files(images_dir):
        if path.suffix.lower() not in IMAGE_EXTENSIONS:
            report.errors.append(f"{path}: 不支持的图片格式")
            continue
        image_files.append(path)

    stem_to_images: dict[str, list[Path]] = {}
    for image_path in image_files:
        stem_to_images.setdefault(image_path.stem.casefold(), []).append(image_path)

    for normalized_stem, matching_images in stem_to_images.items():
        if len(matching_images) > 1:
            names = ", ".join(path.name for path in matching_images)
            report.errors.append(f"图片主文件名重复（扩展名不同）：{names}")
            continue
        report.images_by_stem[normalized_stem] = matching_images[0]

    visible_label_files = _visible_files(labels_dir)
    label_files = [
        path for path in visible_label_files if path.suffix.lower() == ".txt"
    ]
    for path in visible_label_files:
        if path.suffix.lower() != ".txt":
            report.errors.append(f"{path}: 标签目录中只允许 .txt 标签文件")

    stem_to_labels: dict[str, list[Path]] = {}
    for label_path in label_files:
        stem_to_labels.setdefault(label_path.stem.casefold(), []).append(label_path)

    for matching_labels in stem_to_labels.values():
        if len(matching_labels) > 1:
            names = ", ".join(path.name for path in matching_labels)
            report.errors.append(f"标签主文件名重复：{names}")

    report.image_count = len(image_files)

    for image_path in image_files:
        normalized_stem = image_path.stem.casefold()
        image_error = _check_image(image_path)
        if image_error:
            report.errors.append(f"{image_path}: {image_error}")

        matching_labels = stem_to_labels.get(normalized_stem, [])
        if not matching_labels:
            report.errors.append(f"{image_path}: 缺少同名标签 {image_path.stem}.txt")
            continue
        if len(matching_labels) > 1:
            continue

        report.labeled_image_count += 1
        box_count, label_errors = _parse_label(matching_labels[0])
        report.boxes_by_stem[normalized_stem] = box_count
        report.box_count += box_count
        report.errors.extend(label_errors)
        if box_count == 0 and not label_errors:
            report.negative_image_count += 1

    image_stems = set(stem_to_images)
    for normalized_stem, matching_labels in stem_to_labels.items():
        if normalized_stem not in image_stems:
            for label_path in matching_labels:
                report.errors.append(f"{label_path}: 找不到同名图片")

    return report


def print_report(report: ValidationReport) -> None:
    print("数据集检查结果")
    print(f"  图片数量：       {report.image_count}")
    print(f"  已有标签的图片： {report.labeled_image_count}")
    print(f"  负样本数量：     {report.negative_image_count}")
    print(f"  钢球框总数：     {report.box_count}")
    print(f"  错误数量：       {len(report.errors)}")

    if report.errors:
        print("\n错误详情：")
        for index, error in enumerate(report.errors, start=1):
            print(f"  {index}. {error}")
        print("\n检查未通过，请修复以上错误后重试。")
    else:
        print("\n检查通过，数据可以用于划分。")


def main() -> int:
    args = parse_args()
    report = validate_dataset(args.dataset)
    print_report(report)
    return 0 if report.is_valid else 1


if __name__ == "__main__":
    sys.exit(main())
