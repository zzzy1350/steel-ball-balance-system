#!/usr/bin/env python3
"""Compile the steel-ball YOLOv8 ONNX model for K230 with nncase 2.9 PTQ."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import random
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import onnx


SUPPORTED_IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a fixed-shape steel-ball YOLOv8 ONNX model to a K230 KModel using nncase 2.9."
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("k230_artifacts/steel_ball_yolov8n_416.onnx"),
        help="Input ONNX model.",
    )
    parser.add_argument(
        "--calib",
        type=Path,
        default=Path("steel_ball_dataset/images/train"),
        help="Calibration image directory.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("k230_artifacts/steel_ball_yolov8n_416.kmodel"),
        help="Output KModel path.",
    )
    parser.add_argument("--samples", type=int, default=56, help="Number of PTQ calibration images.")
    parser.add_argument("--seed", type=int, default=42, help="Calibration sampling seed.")
    parser.add_argument(
        "--camera-size",
        type=int,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        default=(640, 384),
        help="Camera frame size to emulate before model letterboxing (default: 640 384).",
    )
    parser.add_argument(
        "--camera-fit",
        choices=("crop", "stretch", "none"),
        default="crop",
        help="How non-camera-sized calibration images are converted to the camera frame.",
    )
    parser.add_argument(
        "--calibrate-method",
        choices=("Kld", "NoClip"),
        default="Kld",
        help="nncase activation calibration algorithm.",
    )
    parser.add_argument(
        "--finetune-weights",
        choices=("NoFineTuneWeights", "UseSquant"),
        default="UseSquant",
        help="nncase weight quantization refinement method.",
    )
    parser.add_argument(
        "--dump-dir",
        type=Path,
        default=Path("k230_artifacts/nncase_dump_416"),
        help="nncase diagnostic dump directory.",
    )
    parser.add_argument(
        "--dump",
        action="store_true",
        help="Enable nncase IR and assembly dumps (uses substantially more disk space).",
    )
    return parser.parse_args()


def package_version(name: str) -> str:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError as exc:
        raise SystemExit(f"Required package is not installed: {name}") from exc


def require_nncase_29():
    nncase_version = package_version("nncase")
    kpu_version = package_version("nncase-kpu")
    if not nncase_version.startswith("2.9.") or not kpu_version.startswith("2.9."):
        raise SystemExit(
            "K230 firmware uses nncase runtime 2.9.x, but the active environment has "
            f"nncase={nncase_version}, nncase-kpu={kpu_version}.\n"
            "Activate the dedicated environment first: conda activate k230_nncase29"
        )

    try:
        import nncase  # type: ignore
    except Exception as exc:
        raise SystemExit(f"Failed to import nncase 2.9: {exc}") from exc
    return nncase, nncase_version, kpu_version


def tensor_shape(value_info) -> list[int]:
    shape: list[int] = []
    for dimension in value_info.type.tensor_type.shape.dim:
        if not dimension.dim_value:
            raise SystemExit(
                f"Dynamic dimension found in tensor {value_info.name!r}; K230 export must be static."
            )
        shape.append(int(dimension.dim_value))
    return shape


def expected_yolov8_output_shape(input_shape: list[int]) -> list[int]:
    if len(input_shape) != 4 or input_shape[:2] != [1, 3]:
        raise SystemExit(f"Expected ONNX input shape [1, 3, height, width], got {input_shape}")

    height, width = input_shape[2:]
    if height <= 0 or width <= 0 or height % 32 or width % 32:
        raise SystemExit(
            f"YOLOv8 input height and width must be positive multiples of 32, got {height}x{width}"
        )

    grid_points = sum((height // stride) * (width // stride) for stride in (8, 16, 32))
    return [1, 5, grid_points]


def inspect_onnx(model_path: Path) -> tuple[str, list[int], str, list[int]]:
    if not model_path.is_file():
        raise SystemExit(f"ONNX model not found: {model_path}")

    model = onnx.load(str(model_path))
    onnx.checker.check_model(model)
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise SystemExit(
            f"Expected exactly one ONNX input and one output, got {len(model.graph.input)} and {len(model.graph.output)}."
        )

    input_info = model.graph.input[0]
    output_info = model.graph.output[0]
    input_shape = tensor_shape(input_info)
    output_shape = tensor_shape(output_info)
    expected_output_shape = expected_yolov8_output_shape(input_shape)
    if output_shape != expected_output_shape:
        raise SystemExit(
            f"Unexpected ONNX output shape: {output_shape}; expected {expected_output_shape} "
            f"for input {input_shape}"
        )
    return input_info.name, input_shape, output_info.name, output_shape


def choose_calibration_images(calib_dir: Path, samples: int, seed: int) -> list[Path]:
    if samples <= 0:
        raise SystemExit("--samples must be greater than zero")
    if not calib_dir.is_dir():
        raise SystemExit(f"Calibration directory not found: {calib_dir}")

    images = sorted(
        (
            path
            for path in calib_dir.iterdir()
            if path.is_file() and path.suffix.lower() in SUPPORTED_IMAGE_EXTENSIONS
        ),
        key=lambda path: path.name.casefold(),
    )
    if not images:
        raise SystemExit(f"No calibration images found in: {calib_dir}")

    rng = random.Random(seed)
    rng.shuffle(images)
    selected = images[: min(samples, len(images))]
    if not selected:
        raise SystemExit("No calibration images were selected")
    return selected


def fit_camera_frame(
    image: np.ndarray,
    camera_size: tuple[int, int],
    camera_fit: str,
) -> np.ndarray:
    """Convert arbitrary calibration imagery to the deployed camera frame."""
    camera_w, camera_h = camera_size
    height, width = image.shape[:2]
    if camera_w <= 0 or camera_h <= 0:
        raise SystemExit(f"Camera dimensions must be positive, got {camera_w}x{camera_h}")
    if (width, height) == camera_size:
        return image
    if camera_fit == "none":
        return image
    if camera_fit == "stretch":
        return cv2.resize(image, camera_size, interpolation=cv2.INTER_LINEAR)

    scale = max(camera_w / width, camera_h / height)
    resized_w = max(camera_w, int(round(width * scale)))
    resized_h = max(camera_h, int(round(height * scale)))
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    left = (resized_w - camera_w) // 2
    top = (resized_h - camera_h) // 2
    return resized[top : top + camera_h, left : left + camera_w]


def board_letterbox_geometry(
    source_size: tuple[int, int],
    target_size: tuple[int, int],
) -> dict[str, int | float | list[int]]:
    """Return the pad-before-resize geometry used by the K230 AI2D pipeline."""
    source_w, source_h = source_size
    target_w, target_h = target_size
    if min(source_w, source_h, target_w, target_h) <= 0:
        raise SystemExit(f"Invalid letterbox sizes: source={source_size}, target={target_size}")

    source_ratio = source_w / source_h
    target_ratio = target_w / target_h
    padded_w = source_w
    padded_h = source_h
    if source_ratio > target_ratio:
        padded_h = max(source_h, int(round(source_w / target_ratio)))
    elif source_ratio < target_ratio:
        padded_w = max(source_w, int(round(source_h * target_ratio)))

    pad_left = (padded_w - source_w) // 2
    pad_right = padded_w - source_w - pad_left
    pad_top = (padded_h - source_h) // 2
    pad_bottom = padded_h - source_h - pad_top
    return {
        "source_size": [source_w, source_h],
        "padded_source_size": [padded_w, padded_h],
        "target_size": [target_w, target_h],
        "source_pad": [pad_left, pad_right, pad_top, pad_bottom],
        "scale_x": target_w / padded_w,
        "scale_y": target_h / padded_h,
    }


def board_letterbox(image: np.ndarray, target_size: tuple[int, int]) -> np.ndarray:
    height, width = image.shape[:2]
    target_w, target_h = target_size
    geometry = board_letterbox_geometry((width, height), target_size)
    left, right, top, bottom = geometry["source_pad"]
    padded = cv2.copyMakeBorder(
        image,
        int(top),
        int(bottom),
        int(left),
        int(right),
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    return cv2.resize(padded, (target_w, target_h), interpolation=cv2.INTER_LINEAR)


def calibration_tensor(
    image_path: Path,
    input_shape: list[int],
    camera_size: tuple[int, int],
    camera_fit: str,
) -> np.ndarray:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"Failed to read calibration image: {image_path}")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = fit_camera_frame(image, camera_size, camera_fit)
    image = board_letterbox(image, (input_shape[3], input_shape[2]))
    image = np.transpose(image, (2, 0, 1))[None, ...]
    return np.ascontiguousarray(image, dtype=np.uint8)


def require_compile_option(options, name: str) -> None:
    if not hasattr(options, name):
        raise SystemExit(
            f"The active nncase package does not expose CompileOptions.{name}; "
            "verify that both nncase packages are the official 2.9.x builds."
        )


def compile_kmodel(
    nncase,
    model_path: Path,
    output_path: Path,
    tensors,
    input_shape: list[int],
    calibrate_method: str,
    finetune_weights: str,
    dump: bool,
    dump_dir: Path,
) -> None:
    import_options = nncase.ImportOptions()
    compile_options = nncase.CompileOptions()
    for option_name in (
        "target",
        "preprocess",
        "input_type",
        "input_shape",
        "input_layout",
        "output_layout",
        "input_range",
        "mean",
        "std",
        "swapRB",
    ):
        require_compile_option(compile_options, option_name)

    compile_options.target = "k230"
    compile_options.preprocess = True
    compile_options.input_type = "uint8"
    compile_options.input_shape = input_shape
    compile_options.input_layout = "NCHW"
    compile_options.output_layout = "NCHW"
    compile_options.input_range = [0, 255]
    compile_options.mean = [0.0, 0.0, 0.0]
    compile_options.std = [255.0, 255.0, 255.0]
    compile_options.swapRB = False
    if hasattr(compile_options, "letterbox_value"):
        compile_options.letterbox_value = 114.0

    dump_dir.mkdir(parents=True, exist_ok=True)
    compile_options.dump_ir = dump
    compile_options.dump_asm = dump
    compile_options.dump_dir = str(dump_dir.resolve())

    ptq_options = nncase.PTQTensorOptions()
    ptq_options.samples_count = len(tensors[0])
    ptq_options.quant_type = "uint8"
    ptq_options.w_quant_type = "uint8"
    ptq_options.calibrate_method = calibrate_method
    if hasattr(ptq_options, "finetune_weights_method"):
        ptq_options.finetune_weights_method = finetune_weights
    elif finetune_weights != "NoFineTuneWeights":
        raise SystemExit(
            f"This nncase build cannot use --finetune-weights {finetune_weights}"
        )
    ptq_options.set_tensor_data(tensors)

    compiler = nncase.Compiler(compile_options)
    compiler.import_onnx(model_path.read_bytes(), import_options)
    compiler.use_ptq(ptq_options)
    compiler.compile()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(compiler.gencode_tobytes())
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise SystemExit(f"nncase did not produce a valid KModel: {output_path}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    args = parse_args()
    nncase, nncase_version, kpu_version = require_nncase_29()
    model_path = args.model.resolve()
    calib_dir = args.calib.resolve()
    output_path = args.output.resolve()
    dump_dir = args.dump_dir.resolve()
    camera_size = (int(args.camera_size[0]), int(args.camera_size[1]))

    input_name, input_shape, output_name, output_shape = inspect_onnx(model_path)
    selected_images = choose_calibration_images(calib_dir, args.samples, args.seed)
    tensors = [[
        calibration_tensor(path, input_shape, camera_size, args.camera_fit)
        for path in selected_images
    ]]
    geometry = board_letterbox_geometry(
        camera_size,
        (input_shape[3], input_shape[2]),
    )

    print(f"nncase: {nncase_version}")
    print(f"nncase-kpu: {kpu_version}")
    print(f"ONNX: {model_path}")
    print(f"Input: {input_name} {input_shape} uint8 RGB NCHW")
    print(f"Output: {output_name} {output_shape}")
    print(f"Camera frame: {camera_size[0]}x{camera_size[1]} fit={args.camera_fit}")
    print(
        "Board letterbox: pad L=%d R=%d T=%d B=%d -> %dx%d"
        % (
            geometry["source_pad"][0],
            geometry["source_pad"][1],
            geometry["source_pad"][2],
            geometry["source_pad"][3],
            input_shape[3],
            input_shape[2],
        )
    )
    print(
        f"PTQ: calibrate_method={args.calibrate_method} "
        f"finetune_weights={args.finetune_weights}"
    )
    print(f"Calibration images: {len(selected_images)} (seed={args.seed})")
    for path in selected_images:
        print(f"  {path.name}")
    print(f"KModel output: {output_path}")

    compile_kmodel(
        nncase,
        model_path,
        output_path,
        tensors,
        input_shape,
        args.calibrate_method,
        args.finetune_weights,
        args.dump,
        dump_dir,
    )

    metadata = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "nncase": nncase_version,
        "nncase_kpu": kpu_version,
        "target": "k230",
        "onnx": str(model_path),
        "onnx_sha256": sha256(model_path),
        "onnx_input": {"name": input_name, "shape": input_shape, "type": "uint8_external"},
        "onnx_output": {"name": output_name, "shape": output_shape},
        "preprocess": {
            "layout": "NCHW",
            "color": "RGB",
            "input_range": [0, 255],
            "mean": [0.0, 0.0, 0.0],
            "std": [255.0, 255.0, 255.0],
            "letterbox_value": 114,
            "camera_size": list(camera_size),
            "camera_fit": args.camera_fit,
            "board_geometry": geometry,
        },
        "ptq": {
            "samples": len(selected_images),
            "seed": args.seed,
            "quant_type": "uint8",
            "weight_quant_type": "uint8",
            "calibrate_method": args.calibrate_method,
            "finetune_weights_method": args.finetune_weights,
            "images": [path.name for path in selected_images],
        },
        "kmodel": str(output_path),
        "kmodel_bytes": output_path.stat().st_size,
        "kmodel_sha256": sha256(output_path),
    }
    metadata_path = output_path.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"Saved KModel: {output_path} ({output_path.stat().st_size} bytes)")
    print(f"Saved metadata: {metadata_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
