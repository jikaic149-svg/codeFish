"""Fine-tune fish pose with strict keypoint localization penalties."""

from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[4]
PROJECT = ROOT / "VISION/training/fish_pose_v1"
DATA_CONFIG = PROJECT / "dataset_v1/dataset.yaml"
INITIAL_WEIGHTS = PROJECT / "runs/yolo11n-pose-960-seed32-v1/train/weights/best.pt"
EXPERIMENT = PROJECT / "runs/yolo11n-pose-960-strict-sigma-v2"


def main():
    model = YOLO(str(INITIAL_WEIGHTS))
    if model.task != "pose" or list(model.model.yaml["kpt_shape"]) != [2, 3]:
        raise RuntimeError("Initial weights are not the expected two-keypoint model")

    model.train(
        data=str(DATA_CONFIG),
        epochs=300,
        patience=100,
        imgsz=960,
        batch=8,
        device=0,
        workers=4,
        optimizer="AdamW",
        lr0=0.001,
        lrf=0.01,
        pose=20.0,
        degrees=15.0,
        translate=0.08,
        scale=0.25,
        fliplr=0.5,
        flipud=0.0,
        mosaic=0.5,
        close_mosaic=20,
        seed=41,
        deterministic=True,
        project=str(EXPERIMENT),
        name="train",
        exist_ok=False,
        plots=True,
        verbose=True,
    )


if __name__ == "__main__":
    main()
