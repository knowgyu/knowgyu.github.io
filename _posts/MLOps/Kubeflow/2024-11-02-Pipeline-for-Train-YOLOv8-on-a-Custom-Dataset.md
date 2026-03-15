---
title: "Pipeline for Train YOLOv8 on a Custom Dataset"
author: knowgyu
description: " "
date: 2023-12-09 09:42:45 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

본 문서에서는 YOLOv8 커스텀 데이터셋 학습과 cfg파일을 통해 학습 설정을 변경하는 것에 대해 다루겠습니다.

# Customize Ultralytics

---

Ultralytics 라이브러리는 실험에 대한 미세한 제어를 가능하게 하는 설정 관리 시스템(Settings Management system)을 제공합니다. `ultralytics.utils` 모듈 내에 있는 `SettingsManager`를 사용함으로써 사용자는 쉽게 설정을 접근하고 변경할 수 있습니다.

CLI에서 간단한 명령어 하나로 확인하고, 업데이트할 수 있습니다.

```bash
# Check Settings
yolo settings

# Update a setting
yolo settings mlflow=False

# Reset settings to Default
yolo settings reset
```

https://docs.ultralytics.com/quickstart/#inspecting-settings

또한, YOLOv8 모델을 사용하며 최적의 성능을 찾기 위한 다양한 configuration도 제공합니다.

## Configuration

---

https://docs.ultralytics.com/usage/cfg/?h=configuration#modes

> YOLO 설정과 하이퍼파라미터는 모델의 성능, 속도, 정확도에 큰 영향을 끼칩니다.

- Docs example

```python
from ultralytics import YOLO

# Load a YOLOv8 model from a pre-trained weights file
model = YOLO('yolov8n.pt')

# Run MODE mode using the custom arguments ARGS (guess TASK)
model.MODE(ARGS)
```

> Where:
> 
> - `TASK` (optional) is one of ([detect](https://docs.ultralytics.com/tasks/detect/), [segment](https://docs.ultralytics.com/tasks/segment/), [classify](https://docs.ultralytics.com/tasks/classify/), [pose](https://docs.ultralytics.com/tasks/pose/))
> - `MODE` (required) is one of ([train](https://docs.ultralytics.com/modes/train/), [val](https://docs.ultralytics.com/modes/val/), [predict](https://docs.ultralytics.com/modes/predict/), [export](https://docs.ultralytics.com/modes/export/), [track](https://docs.ultralytics.com/modes/track/))
> - `ARGS` (optional) are `arg=value` pairs like `imgsz=640` that override defaults.

YOLOv8 모델을 학습시킬 때는 다양한 하이퍼파라미터와 설정을 다룰 수 있습니다. 이러한 설정은 `cfg/defaults.yaml`에 정의되어 있어 사용자가 직접 입력하지 않아도 기본값으로 학습이 진행됩니다. 필요하다면 별도 인자를 입력해 해당 기본값을 덮어쓸 수 있습니다.

이러한 사용자 지정 인자는 커스텀 데이터셋 사용, Epochs, Batch size, Resume 옵션 등 다양한 상황에서 활용할 수 있습니다.

### YOLOv8 모델 학습 예시

YOLOv8의 CLI 명령어는 아래와 같습니다

```bash
# YOLOv8 with DDP
yolo detect train model=yolov8m.pt data=/data1/dataset/custom_dataset/custom_data.yaml \
batch=48 workers=8 device=0,1 imgsz=640 \ 
project=custom_yolov8 name=Test1 \
save=True save_period=30
```

이를 Kubeflow Component 형식으로 나타내면 아래와 같습니다.

```python
@partial(
    create_component_from_func,
    base_image="nohgyu/test:v1.1",
    packages_to_install=["ultralytics","ray[tune]","opencv-python==4.8.0.74","mlflow", "boto3"],
)
def train(
    checker: bool,
    model_path: str,
    data: str,
    batch: int,
    workers: int,
    device: int,
    imgsz: int,
    project: str,
    name: str,
    save: bool,
    save_period: int,
        ) -> str:
    import os
    from ultralytics import YOLO
    import mlflow
    
    if not checker:
        print("CUDA is not available.\nPlease Check GPU Device")
        return None

    model = YOLO(model_path)    

    results = model.train(data=data, batch=batch, workers=workers,
                          device=device, imgsz=imgsz, project=project,
                          name=name, save=save, save_period=save_period)

    return os.path.join(project,name,'weights','last.pt')
```

위 예시처럼 모델 학습 과정에서 최적의 성능을 찾기 위해 다양한 인자 조합을 실험하는 것은 매우 중요합니다.

하지만 학습을 실행할 때마다 많은 인자를 직접 입력해야 합니다.

특히 Kubeflow 파이프라인을 **실행할 때마다** 많은 Config 값을 **직접 입력**해야 하므로, **휴먼 에러**가 발생하기 쉽습니다.

이러한 문제들을 해결하기 위해 `cfg`파일을 이용하겠습니다.

### YOLOv8 `cfg-custom.yaml`

- `cfg/defaults.yaml`
    
    ```python
    # Ultralytics YOLO 🚀, AGPL-3.0 license
    # Default training settings and hyperparameters for medium-augmentation COCO training
    
    task: detect  # (str) YOLO task, i.e. detect, segment, classify, pose
    mode: train  # (str) YOLO mode, i.e. train, val, predict, export, track, benchmark
    
    # Train settings -------------------------------------------------------------------------------------------------------
    model:  # (str, optional) path to model file, i.e. yolov8n.pt, yolov8n.yaml
    data:  # (str, optional) path to data file, i.e. coco128.yaml
    epochs: 300  # (int) number of epochs to train for
    patience: 50  # (int) epochs to wait for no observable improvement for early stopping of training
    batch: 16  # (int) number of images per batch (-1 for AutoBatch)
    imgsz: 640  # (int | list) input images size as int for train and val modes, or list[w,h] for predict and export modes
    save: True  # (bool) save train checkpoints and predict results
    save_period: -1 # (int) Save checkpoint every x epochs (disabled if < 1)
    cache: False  # (bool) True/ram, disk or False. Use cache for data loading
    device:  # (int | str | list, optional) device to run on, i.e. cuda device=0 or device=0,1,2,3 or device=cpu
    workers: 8  # (int) number of worker threads for data loading (per RANK if DDP)
    project:  # (str, optional) project name
    name:  # (str, optional) experiment name, results saved to 'project/name' directory
    exist_ok: False  # (bool) whether to overwrite existing experiment
    pretrained: True  # (bool | str) whether to use a pretrained model (bool) or a model to load weights from (str)
    optimizer: auto  # (str) optimizer to use, choices=[SGD, Adam, Adamax, AdamW, NAdam, RAdam, RMSProp, auto]
    verbose: True  # (bool) whether to print verbose output
    seed: 0  # (int) random seed for reproducibility
    deterministic: True  # (bool) whether to enable deterministic mode
    single_cls: False  # (bool) train multi-class data as single-class
    rect: False  # (bool) rectangular training if mode='train' or rectangular validation if mode='val'
    cos_lr: False  # (bool) use cosine learning rate scheduler
    close_mosaic: 10  # (int) disable mosaic augmentation for final epochs (0 to disable)
    resume: False  # (bool) resume training from last checkpoint
    amp: True  # (bool) Automatic Mixed Precision (AMP) training, choices=[True, False], True runs AMP check
    fraction: 1.0  # (float) dataset fraction to train on (default is 1.0, all images in train set)
    profile: False  # (bool) profile ONNX and TensorRT speeds during training for loggers
    freeze: None  # (int | list, optional) freeze first n layers, or freeze list of layer indices during training
    # Segmentation
    overlap_mask: True  # (bool) masks should overlap during training (segment train only)
    mask_ratio: 4  # (int) mask downsample ratio (segment train only)
    # Classification
    dropout: 0.0  # (float) use dropout regularization (classify train only)
    
    # Val/Test settings ----------------------------------------------------------------------------------------------------
    val: True  # (bool) validate/test during training
    split: val  # (str) dataset split to use for validation, i.e. 'val', 'test' or 'train'
    save_json: False  # (bool) save results to JSON file
    save_hybrid: False  # (bool) save hybrid version of labels (labels + additional predictions)
    conf:  # (float, optional) object confidence threshold for detection (default 0.25 predict, 0.001 val)
    iou: 0.7  # (float) intersection over union (IoU) threshold for NMS
    max_det: 300  # (int) maximum number of detections per image
    half: False  # (bool) use half precision (FP16)
    dnn: False  # (bool) use OpenCV DNN for ONNX inference
    plots: True  # (bool) save plots during train/val
    
    # Prediction settings --------------------------------------------------------------------------------------------------
    source:  # (str, optional) source directory for images or videos
    show: False  # (bool) show results if possible
    save_txt: False  # (bool) save results as .txt file
    save_conf: False  # (bool) save results with confidence scores
    save_crop: False  # (bool) save cropped images with results
    show_labels: True  # (bool) show object labels in plots
    show_conf: True  # (bool) show object confidence scores in plots
    vid_stride: 1  # (int) video frame-rate stride
    stream_buffer: False  # (bool) buffer all streaming frames (True) or return the most recent frame (False)
    line_width:   # (int, optional) line width of the bounding boxes, auto if missing
    visualize: False  # (bool) visualize model features
    augment: False  # (bool) apply image augmentation to prediction sources
    agnostic_nms: False  # (bool) class-agnostic NMS
    classes:  # (int | list[int], optional) filter results by class, i.e. classes=0, or classes=[0,2,3]
    retina_masks: False  # (bool) use high-resolution segmentation masks
    boxes: True  # (bool) Show boxes in segmentation predictions
    
    # Export settings ------------------------------------------------------------------------------------------------------
    format: torchscript  # (str) format to export to, choices at https://docs.ultralytics.com/modes/export/#export-formats
    keras: False  # (bool) use Kera=s
    optimize: False  # (bool) TorchScript: optimize for mobile
    int8: False  # (bool) CoreML/TF INT8 quantization
    dynamic: False  # (bool) ONNX/TF/TensorRT: dynamic axes
    simplify: False  # (bool) ONNX: simplify model
    opset:  # (int, optional) ONNX: opset version
    workspace: 4  # (int) TensorRT: workspace size (GB)
    nms: False  # (bool) CoreML: add NMS
    
    # Hyperparameters ------------------------------------------------------------------------------------------------------
    lr0: 0.01  # (float) initial learning rate (i.e. SGD=1E-2, Adam=1E-3)
    lrf: 0.01  # (float) final learning rate (lr0 * lrf)
    momentum: 0.937  # (float) SGD momentum/Adam beta1
    weight_decay: 0.0005  # (float) optimizer weight decay 5e-4
    warmup_epochs: 3.0  # (float) warmup epochs (fractions ok)
    warmup_momentum: 0.8  # (float) warmup initial momentum
    warmup_bias_lr: 0.1  # (float) warmup initial bias lr
    box: 7.5  # (float) box loss gain
    cls: 0.5  # (float) cls loss gain (scale with pixels)
    dfl: 1.5  # (float) dfl loss gain
    pose: 12.0  # (float) pose loss gain
    kobj: 1.0  # (float) keypoint obj loss gain
    label_smoothing: 0.0  # (float) label smoothing (fraction)
    nbs: 64  # (int) nominal batch size
    hsv_h: 0.015  # (float) image HSV-Hue augmentation (fraction)
    hsv_s: 0.7  # (float) image HSV-Saturation augmentation (fraction)
    hsv_v: 0.4  # (float) image HSV-Value augmentation (fraction)
    degrees: 0.0  # (float) image rotation (+/- deg)
    translate: 0.1  # (float) image translation (+/- fraction)
    scale: 0.5  # (float) image scale (+/- gain)
    shear: 0.0  # (float) image shear (+/- deg)
    perspective: 0.0  # (float) image perspective (+/- fraction), range 0-0.001
    flipud: 0.0  # (float) image flip up-down (probability)
    fliplr: 0.5  # (float) image flip left-right (probability)
    mosaic: 1.0  # (float) image mosaic (probability)
    mixup: 0.0  # (float) image mixup (probability)
    copy_paste: 0.0  # (float) segment copy-paste (probability)
    
    # Custom config.yaml ---------------------------------------------------------------------------------------------------
    cfg:  # (str, optional) for overriding defaults.yaml
    
    # Tracker settings ------------------------------------------------------------------------------------------------------
    tracker: botsort.yaml  # (str) tracker type, choices=[botsort.yaml, bytetrack.yaml]
    ```
    

위 파일은 기본으로 사용되는 `ARG`값들이 정의되어있는 파일입니다.  이 파일을 이용해 `cfg-custom.yaml` 을 만들겠습니다.

- `cfg-custom.yaml`

```
# Ultralytics YOLO 🚀, AGPL-3.0 license
# Default training settings and hyperparameters for medium-augmentation COCO training

task: detect  # (str) YOLO task, i.e. detect, segment, classify, pose
mode: train  # (str) YOLO mode, i.e. train, val, predict, export, track, benchmark

# Train settings -------------------------------------------------------------------------------------------------------
model: **yolov8m.pt**  # (str, optional) path to model file, i.e. yolov8n.pt, yolov8n.yaml
data: **/data1/dataset/custom_dataset/custom_data.yaml** # (str, optional) path to data file, i.e. coco128.yaml
epochs: **300**  # (int) number of epochs to train for
patience: 50  # (int) epochs to wait for no observable improvement for early stopping of training
batch: **48**  # (int) number of images per batch (-1 for AutoBatch)
imgsz: **640**  # (int | list) input images size as int for train and val modes, or list[w,h] for predict and export modes
...
...생략
...

```

이렇게 `cfg-custom.yaml` 파일을 생성했다면, 아래와 같이 실행할 수 있습니다.

```bash
# 수정 전 코드
yolo detect train model=yolov8m.pt data=/data1/dataset/custom_dataset/custom_data.yaml \
batch=48 workers=8 device=0,1 imgsz=640 \ 
project=custom_yolov8 name=Test \
save=True save_period=30

# 수정 후 코드
yolo detect train cfg=cfg-custom.yaml
```

이처럼 `cfg-custom.yaml` 파일을 이용하면 더욱 편리하게 YOLO 설정 및 하이퍼파라미터 조정이 가능합니다.

이를 활용해 Kubeflow Pipeline에 적용하도록 하겠습니다.

## Custom Dataset Training Pipeline

---

노드 **로컬** 볼륨에 있는 COCO128 데이터셋을 학습하는 파이프라인을 작성하겠습니다.

우선, 폴더 구조는 아래와 같습니다

```bash
kfcocotest/
├── cfg-custom.yaml
├── data-custom.yaml
├── datasets
│   ├── coco128  [256 entries exceeds filelimit, not opening dir]
│   ├── coco128test.txt
│   └── coco128train.txt
└── KFP_compiler.py
```

### 볼륨 마운트

이전 PV Mount 문서에서 다룬 방법대로 PV/PVC를 생성한 후 Pipeline에 연결하겠습니다.

- `cocopv.yaml`

```bash
apiVersion: v1
kind: PersistentVolume
metadata:
  name: local-cocodata
spec:
  capacity:
    storage: 3Gi
  volumeMode: Filesystem
  accessModes:
    - ReadWriteOnce
  persistentVolumeReclaimPolicy: Retain
  storageClassName: "train"
  hostPath:
    path: "/data/kfcocotest"
  nodeAffinity:
    required:
      nodeSelectorTerms:
      - matchExpressions:
        - key: kubernetes.io/hostname
          operator: In
          values:
          - blog-space
```

- `cocopvc.yaml`

```bash
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: pvc-local-cocodata
  namespace: blog-space
spec:
  accessModes:
  - ReadWriteOnce
  volumeMode: Filesystem
  resources:
    requests:
      storage: 3Gi
  storageClassName: "train"
```

- 생성 및 확인

```bash
kubectl apply -f cocopv.yaml 
kubectl apply -f cocopvc.yaml 

# 각각 아래와 같은 메세지가 출력됩니다.
persistentvolume/local-cocodata created
persistentvolumeclaim/pvc-local-cocodata created
```

![Kubeflow Dashboard에서 확인. pvc-local-cocodata 생성 확인 완료](/assets/img/kubeflow/kubeyolo201.png)

Kubeflow Dashboard에서 확인. pvc-local-cocodata 생성 확인 완료

![해당 PVC에 연결한 Jupyter Notebook 확인](/assets/img/kubeflow/kubeyolo202.png)

해당 PVC에 연결한 Jupyter Notebook 확인

### 학습 설정

정상적으로 마운트된 것을 확인했다면, Notebook에서 yaml파일들을 수정합니다.

![Untitled](/assets/img/kubeflow/kubeyolo203.png)

- `cfg-custom.yaml`
    
    ```yaml
    # Ultralytics YOLO 🚀, AGPL-3.0 license
    # Default training settings and hyperparameters for medium-augmentation COCO training
    
    task: detect  # (str) YOLO task, i.e. detect, segment, classify, pose
    mode: train  # (str) YOLO mode, i.e. train, val, predict, export, track, benchmark
    
    # Train settings -------------------------------------------------------------------------------------------------------
    model:  # (str, optional) path to model file, i.e. yolov8n.pt, yolov8n.yaml
    data:  # (str, optional) path to data file, i.e. coco128.yaml
    epochs: 2  # (int) number of epochs to train for
    patience: 50  # (int) epochs to wait for no observable improvement for early stopping of training
    batch: 4  # (int) number of images per batch (-1 for AutoBatch)
    imgsz: 320  # (int | list) input images size as int for train and val modes, or list[w,h] for predict and export modes
    save: True  # (bool) save train checkpoints and predict results
    save_period: 30 # (int) Save checkpoint every x epochs (disabled if < 1)
    cache: False  # (bool) True/ram, disk or False. Use cache for data loading
    device:  # (int | str | list, optional) device to run on, i.e. cuda device=0 or device=0,1,2,3 or device=cpu
    workers: 8  # (int) number of worker threads for data loading (per RANK if DDP)
    project: testprj  # (str, optional) project name
    name: testexp  # (str, optional) experiment name, results saved to 'project/name' directory
    exist_ok: False  # (bool) whether to overwrite existing experiment
    pretrained: True  # (bool | str) whether to use a pretrained model (bool) or a model to load weights from (str)
    optimizer: auto  # (str) optimizer to use, choices=[SGD, Adam, Adamax, AdamW, NAdam, RAdam, RMSProp, auto]
    verbose: True  # (bool) whether to print verbose output
    seed: 404  # (int) random seed for reproducibility
    deterministic: True  # (bool) whether to enable deterministic mode
    single_cls: False  # (bool) train multi-class data as single-class
...
...
생략
...
...
    mixup: 0.0  # (float) image mixup (probability)
    copy_paste: 0.0  # (float) segment copy-paste (probability)
    
    # Custom config.yaml ---------------------------------------------------------------------------------------------------
    cfg:  # (str, optional) for overriding defaults.yaml
    
    # Tracker settings ------------------------------------------------------------------------------------------------------
    tracker: botsort.yaml  # (str) tracker type, choices=[botsort.yaml, bytetrack.yaml]
    ```
    
- `data-custom.yaml`
    
    ```bash
    # train and val data as 1) directory: path/images/, 2) file: path/images.txt, or 3) list: [path1/images/, path2/images/]
    train: ./coco128train.txt # 6,331
    val: ./coco128test.txt # 1,118
    
    # number of classes
    nc: 80
    
    # class names
    names: [ 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
             'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
             'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
             'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
             'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
             'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
             'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
             'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
             'hair drier', 'toothbrush' ]
    ```
    

>`yolo settings` 명령어로 `datasets_dir` 경로를 잘 확인해야합니다.
>
>`coco128train.txt` 는 아래와 같은 형식으로 입력되어 있습니다.
>`./coco128/000000000357.jpg` 
>`./coco128/000000000450.jpg`
{: .prompt-tip }

### 파이프라인 작성 및 업로드

- `KFP_compiler.py`
    
    ```python
    from functools import partial
    
    import kfp
    from kfp.components import create_component_from_func, InputPath, OutputPath
    from kfp import dsl
    
    @partial(
        create_component_from_func,
        base_image="nohgyu/test:v1.1",
    )
    def verify_training(
        model_name: str,
        cfg: str,
        data: str,
        ) -> bool:
        import torch
    
        assert isinstance(model_name, str), "model_name must be a string"
        assert isinstance(cfg, str), "model_name must be a string"
        assert isinstance(data, str), "model_name must be a string"
        
        return torch.cuda.is_available()
        
        
    @partial(
        create_component_from_func,
        base_image="nohgyu/test:v1.1",
        packages_to_install=["ultralytics","opencv-python==4.8.0.74","mlflow", "boto3"],
    )
    def train(
        checker: bool,
        model_name: str,
        cfg: str,
        data: str,
            ) -> str:
        import os
        from ultralytics import YOLO
        import mlflow
        import yaml
        
        if not checker:
            print("CUDA is not available.\nPlease Check GPU Device")
            return None
    
        os.environ["MLFLOW_TRACKING_URI"] = "http://mlflow-server-service.mlflow-system.svc:5000"
        os.environ["MLFLOW_S3_ENDPOINT_URL"] = "http://minio-service.kubeflow.svc:9000"
        os.environ["AWS_ACCESS_KEY_ID"] = "minio"
        os.environ["AWS_SECRET_ACCESS_KEY"] = "minio123"
    
        model = YOLO(model_name)    
    
        if not cfg.endswith('.yaml'):
            cfg += '.yaml'
        if not data.endswith('.yaml'):
            data += '.yaml'    
        
        if not os.path.isfile(cfg):
            raise FileNotFoundError(f"FileNotFound : {cfg}")
        if not os.path.isfile(data):
            raise FileNotFoundError(f"FileNotFound : {data}")
        
        results = model.train(cfg=cfg,data=data)
    
        with open(cfg,'r',encoding='utf-8') as file:
            tmp = yaml.safe_load(file)
            
        project = tmp.get('project')
        name = tmp.get('name')
            
        return os.path.join(project,name,'weights','last.pt')
        
    @partial(
        create_component_from_func,
        base_image="nohgyu/test:v1.1",
        packages_to_install=["ultralytics","ray[tune]","opencv-python==4.8.0.74","mlflow", "boto3"],
    )
    def tune(
        model_path: str,
        cfg: str,
        data: str,
        ):
        from ultralytics import YOLO
        from ray import tune
        import os
        import yaml
    
        os.environ["MLFLOW_TRACKING_URI"] = "http://mlflow-server-service.mlflow-system.svc:5000"
        os.environ["MLFLOW_S3_ENDPOINT_URL"] = "http://minio-service.kubeflow.svc:9000"
        os.environ["AWS_ACCESS_KEY_ID"] = "minio"
        os.environ["AWS_SECRET_ACCESS_KEY"] = "minio123"
        
        model = YOLO(model_path)
    
        if not cfg.endswith('.yaml'):
            cfg += '.yaml'
        if not data.endswith('.yaml'):
            data += '.yaml'    
        
        with open(cfg,'r',encoding='utf-8') as file:
            tmp = yaml.safe_load(file)
            
        project = tmp.get('project')
        name = tmp.get('name')
        print(f"project : {project}, name : {name}")
        print(os.listdir())
        
        result = model.tune(
                    data=data,
                    space={"lr0": tune.uniform(1e-5, 1e-1)},
                    epochs=2,
                    grace_period=2,
                    gpu_per_trial=1,
                    iterations=1,
                    project=project,
                    batch=4,
                    use_ray=True
                    )
        
        print(os.listdir())
            
    @dsl.pipeline(name="pipelinename",
              description="MLpipline Description",
              )
    def train_pipeline(
        model_name: str,
        cfg: str,
        data: str,
        ):
        import os
        
        vop = dsl.VolumeOp(
            name="volume mount",
            resource_name="pvc-local-cocodata",
            storage_class='train',
            modes=dsl.VOLUME_MODE_RWO,
            size="3Gi",
            generate_unique_name=False,
            action='apply',
        )
        
        check = verify_training(model_name, cfg, data).add_pvolumes({"/data":vop.volume})
        trained = train(check.output, model_name, cfg, data).add_pvolumes({"/data":vop.volume})
        tune(trained.output, cfg, data).add_pvolumes({"/data":vop.volume})
    
    if __name__ == "__main__":
        kfp.compiler.Compiler().compile(train_pipeline, "pipeline-mount-test.yaml")
    ```
    

>파이프라인 Config로 `model_name`, `cfg`, `data` 를 입력받습니다. `cfg`와 `data`의 경우 `.yaml`을 붙이지 않아도 됩니다.
>
>Pipeline 컴파일 후 yaml파일 업로드는 Notebook이 아닌 **로컬**에서 수행합니다. (`kfp` 라이브러리 필요)
{: .prompt-info }

- 입력 예시

![cfg와 data의 경우 .yaml을 붙이지 않아도 됩니다.](/assets/img/kubeflow/kubeyolo204.png)

cfg와 data의 경우 .yaml을 붙이지 않아도 됩니다.

- 파이프라인 실행 결과

![Untitled](/assets/img/kubeflow/kubeyolo205.png)

- `train.log`
    
    ```bash
    /tmp/tmp.8YNyaqswau:44: DeprecationWarning: The distutils package is deprecated and slated for removal in Python 3.12. Use setuptools or check PEP 632 for potential alternatives
      from distutils.util import strtobool
    /usr/local/lib/python3.10/dist-packages/pydantic/_internal/_fields.py:127: UserWarning: Field "model_server_url" has conflict with protected namespace "model_".
    
    You may be able to resolve this warning by setting `model_config['protected_namespaces'] = ()`.
      warnings.warn(
    /usr/local/lib/python3.10/dist-packages/pydantic/_internal/_config.py:269: UserWarning: Valid config keys have changed in V2:
    * 'schema_extra' has been renamed to 'json_schema_extra'
      warnings.warn(message, UserWarning)
    
      0%|          | 0.00/755k [00:00<?, ?B/s]
     32%|â–ˆâ–ˆâ–ˆâ–      | 240k/755k [00:00<00:00, 2.37MB/s]
    100%|â–ˆâ–ˆâ–ˆâ–ˆâ–ˆâ–ˆâ–ˆâ–ˆâ–ˆâ–ˆ| 755k/755k [00:00<00:00, 4.90MB/s]
    
    [34m[1mtrain: [0mScanning /data/datasets/coco128...:   0%|          | 0/115 [00:00<?, ?it/s]
    [34m[1mtrain: [0mScanning /data/datasets/coco128... 8 images, 0 backgrounds, 0 corrupt:   7%|â–‹         | 8/115 [00:00<00:01, 79.12it/s]
    [34m[1mtrain: [0mScanning /data/datasets/coco128... 48 images, 1 backgrounds, 0 corrupt:  43%|â–ˆâ–ˆâ–ˆâ–ˆâ–Ž     | 49/115 [00:00<00:00, 261.80it/s]
...
...
...생략
...
...
              dining table         13          1      0.348          1      0.497      0.497
                    laptop         13          2          1          0      0.995      0.646
                cell phone         13          1          1          0          0          0
                     clock         13          2      0.678          1      0.995      0.504
                teddy bear         13         10       0.68        0.6      0.647      0.279
    Speed: 0.1ms preprocess, 0.9ms inference, 0.0ms loss, 0.5ms postprocess per image
    Results saved to [1mtestprj/testexp[0m
    [34m[1mMLflow: [0mresults logged to http://mlflow-server-service.mlflow-system.svc:5000
    [34m[1mMLflow: [0mdisable with 'yolo settings mlflow=False'
    ```
    

```yaml
engine/trainer: [0mtask=detect, mode=train, model=None, data=data-custom.yaml, epochs=2, patience=50, batch=4, imgsz=320, save=True, save_period=30, cache=False, device=None, workers=8, project=testprj, name=testexp, exist_ok=True, pretrained=True, optimizer=auto, verbose=True, seed=404, deterministic=True, single_cls=False, rect=False, cos_lr=False, close_mosaic=10, resume=False, amp=True, fraction=1.0, profile=False, freeze=10, overlap_mask=True, mask_ratio=4, dropout=0.0, val=True, split=val, save_json=False, save_hybrid=False, conf=None, iou=0.7, max_det=300, half=False, dnn=False, plots=True, source=None, vid_stride=1, stream_buffer=False, visualize=False, augment=False, agnostic_nms=False, classes=None, retina_masks=False, show=False, save_frames=False, save_txt=False, save_conf=False, save_crop=False, show_labels=True, show_conf=True, show_boxes=True, line_width=None, format=torchscript, keras=False, optimize=False, int8=False, dynamic=False, simplify=False, opset=None, workspace=4, nms=False, lr0=0.01, lrf=0.01, momentum=0.937, weight_decay=0.0005, warmup_epochs=3.0, warmup_momentum=0.8, warmup_bias_lr=0.1, box=7.5, cls=0.5, dfl=1.5, pose=12.0, kobj=1.0, label_smoothing=0.0, nbs=64, hsv_h=0.015, hsv_s=0.7, hsv_v=0.4, degrees=0.0, translate=0.1, scale=0.5, shear=0.0, perspective=0.0, flipud=0.0, fliplr=0.0, mosaic=1.0, mixup=0.0, copy_paste=0.0, cfg=cfg-custom.yaml, tracker=botsort.yaml, save_dir=testprj/testexp
```

위는 로그의 일부분입니다. 위 출력을 보면 `cfg-custom`에서 지정해준 `ARG`들이 정상적으로 적용된 것을 알 수 있습니다.

## Conclusion

---

위와 같은 방법을 통해 YOLO Setting과 Hyper params을 조정하고 Custom Dataset을 학습시킬 수 있습니다.


> 현재 컴포넌트들이 서로 주고받는 Config가 아직 유기적으로 연결되어 있지 않아 추가 수정이 필요합니다.
>
> 또한 현재 `Tune` 컴포넌트 실행 시 첫 이터레이션 이후 `data-custom.yaml` 을 찾지 못하는 에러가 발생합니다.
>
> `os.listdir()` 로 터미널 로그를 확인한 결과 같은 디렉터리 안에 파일이 존재하는데도 에러가 발생했습니다. 다른 프로세스 사용과 연관된 문제인지 추가 확인이 필요합니다.
{: .prompt-warning }
