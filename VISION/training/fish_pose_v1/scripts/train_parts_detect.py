"""Train semantic fish/rear/head boxes as an alternative to Pose."""

from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
DATA_CONFIG = PROJECT / "dataset_parts_v1/dataset.yaml"
INITIAL_WEIGHTS = ROOT / "VISION/models/best.pt"
EXPERIMENT = PROJECT / "runs/yolo11n-parts-960-v1"


def main():
    model = YOLO(str(INITIAL_WEIGHTS))
    if model.task != "detect":
        raise RuntimeError(f"Expected detect initial weights, got {model.task!r}")
    model.train(
        data=str(DATA_CONFIG),
        epochs=300,
        patience=100,
        imgsz=960,
        batch=8,
        device=0,
        workers=4,
        optimizer="AdamW",
        lr0=0.0015,
        lrf=0.01,
        cls=1.0,
        degrees=15.0,
        translate=0.08,
        scale=0.25,
        fliplr=0.5,
        flipud=0.0,
        mosaic=0.5,
        close_mosaic=20,
        seed=51,
        deterministic=True,
        project=str(EXPERIMENT),
        name="train",
        exist_ok=False,
        plots=True,
        verbose=True,
    )


if __name__ == "__main__":
    main()
