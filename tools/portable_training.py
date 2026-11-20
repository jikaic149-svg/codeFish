"""Explicit CPU training/validation entry point; all output stays inside the delivery."""
from pathlib import Path
import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('train', 'validate', 'smoke'))
    parser.add_argument('--epochs', type=int, default=90)
    parser.add_argument('--imgsz', type=int, default=960)
    parser.add_argument('--batch', type=int, default=2)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    from ultralytics import YOLO
    from ultralytics.utils import SETTINGS
    settings = {'datasets_dir': str(root / 'datasets'), 'runs_dir': str(root / 'state/training'),
                'weights_dir': str(root / 'project/vision/models/fish_body_yolo'),
                'sync': False, 'hub': False, 'wandb': False, 'comet': False, 'mlflow': False}
    SETTINGS.update({key: value for key, value in settings.items() if key in SETTINGS})
    model = YOLO(str(root / 'project/vision/models/fish_body_yolo/best.pt'))
    data = root / 'datasets/fish/dataset.yaml'
    if args.action == 'validate':
        model.val(data=str(data), split='test', imgsz=args.imgsz, batch=args.batch, device='cpu',
                  workers=0, project=str(root / 'state/training'), name='validation', plots=False)
    else:
        smoke = args.action == 'smoke'
        model.train(data=str(data), epochs=1 if smoke else args.epochs,
                    imgsz=64 if smoke else args.imgsz, batch=2 if smoke else args.batch,
                    device='cpu', workers=0, amp=False, cache=False, plots=False,
                    fraction=.01 if smoke else 1., optimizer='AdamW', lr0=.0003,
                    project=str(root / 'state/training'), name='smoke' if smoke else 'train',
                    save=True, val=not smoke, pretrained=False)


if __name__ == '__main__':
    main()
