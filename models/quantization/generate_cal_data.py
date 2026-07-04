#!/usr/bin/env python3
"""Generate float32 NCHW RGB calibration data for RDK X5 hb_mapper.

Preprocess modes:
  center_crop: Resize(256) + CenterCrop(224)  # strawberry, wheat
  resize:      Resize((224, 224))              # tomato field val

Output: .bin files, [1, 3, 224, 224] float32 NCHW RGB, value range [0, 1]
        (mean/std normalization is left to hb_mapper config.yaml)
"""
import argparse
import random
from pathlib import Path

import numpy as np
from PIL import Image


def preprocess_center_crop(image_path: str, input_size: int = 224) -> np.ndarray:
    """Resize(256) + CenterCrop(224), ToTensor -> float32 NCHW [0,1]."""
    img = Image.open(image_path).convert("RGB")
    img = img.resize((256, 256), Image.BILINEAR)
    left = (256 - input_size) // 2
    top = (256 - input_size) // 2
    img = img.crop((left, top, left + input_size, top + input_size))
    arr = np.array(img, dtype=np.float32)  # HWC [0,255]
    arr = np.transpose(arr, (2, 0, 1))  # CHW
    return arr


def preprocess_resize(image_path: str, input_size: int = 224) -> np.ndarray:
    """Resize((224,224)), ToTensor -> float32 NCHW [0,255]."""
    img = Image.open(image_path).convert("RGB")
    img = img.resize((input_size, input_size), Image.BILINEAR)
    arr = np.array(img, dtype=np.float32)  # HWC [0,255]
    arr = np.transpose(arr, (2, 0, 1))  # CHW
    return arr


def main():
    parser = argparse.ArgumentParser(description="Generate calibration data for hb_mapper")
    parser.add_argument("--src", required=True, help="Source directory with jpg/png images (recursive)")
    parser.add_argument("--dist", required=True, help="Output directory for .bin files")
    parser.add_argument("--mode", choices=["center_crop", "resize"], default="center_crop",
                        help="Preprocess mode: center_crop (strawberry/wheat) or resize (tomato)")
    parser.add_argument("--size", type=int, default=224, help="Input size (default: 224)")
    parser.add_argument("--max", type=int, default=200, help="Max number of images")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for sample selection")
    args = parser.parse_args()

    src_dir = Path(args.src)
    dist_dir = Path(args.dist)
    dist_dir.mkdir(parents=True, exist_ok=True)

    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    image_paths = sorted([p for p in src_dir.rglob("*") if p.suffix.lower() in exts])
    random.seed(args.seed)
    random.shuffle(image_paths)
    image_paths = image_paths[:args.max]

    if len(image_paths) < 20:
        print(f"WARNING: only {len(image_paths)} images found; hb_mapper recommends >= 20")

    preprocess_fn = preprocess_center_crop if args.mode == "center_crop" else preprocess_resize

    for i, img_path in enumerate(image_paths):
        try:
            tensor = preprocess_fn(str(img_path), args.size)
            out_path = dist_dir / f"calib_{i:04d}.bin"
            tensor.tofile(out_path)
            if i < 5:
                print(f"Wrote {out_path} shape={tensor.shape} dtype={tensor.dtype} min={tensor.min():.3f} max={tensor.max():.3f}")
        except Exception as e:
            print(f"Skip {img_path}: {e}")

    print(f"\nDone. Total calibration samples: {len(list(dist_dir.glob('*.bin')))}")


if __name__ == "__main__":
    main()
