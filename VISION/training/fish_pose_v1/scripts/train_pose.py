"""Train the first two-keypoint fish pose model without touching runtime weights."""

from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
MODEL_CONFIG = PROJECT / "configs/yolo11n_fish_pose.yaml"
DATA_CONFIG = PROJECT / "dataset_v1/dataset.yaml"
INITIAL_WEIGHTS = ROOT / "VISION/models/best.pt"
EXPERIMENT = PROJECT / "runs/yolo11n-pose-960-seed32-v1"


def main():
    model = YOLO(str(MODEL_CONFIG)).load(str(INITIAL_WEIGHTS))
    if model.task != "pose":
        raise RuntimeError(f"Expected pose model, got {model.task!r}")
    if list(model.model.yaml["kpt_shape"]) != [2, 3]:
        raise RuntimeError(f"Unexpected keypoint shape: {model.model.yaml['kpt_shape']}")

    model.train(
        data=str(DATA_CONFIG),
        epochs=300,
        patience=80,
        imgsz=960,
        batch=8,
        device=0,
        workers=4,
        optimizer="AdamW",
        lr0=0.002,
        lrf=0.01,
        degrees=15.0,
        translate=0.08,
        scale=0.25,
        fliplr=0.5,
        flipud=0.0,
        mosaic=0.5,
        close_mosaic=20,
        seed=31,
        deterministic=True,
        project=str(EXPERIMENT),
        name="train",
        exist_ok=False,
        plots=True,
        verbose=True,
    )


if __name__ == "__main__":
    main()
