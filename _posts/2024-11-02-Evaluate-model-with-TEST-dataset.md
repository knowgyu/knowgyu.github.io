---
title: "Evaluate model with TEST dataset"
author: knowgyu
description: " "
date: 2023-12-10 09:43:07 +0900
math: true
categories: [MLOps, 4-Kubeflow-Pipeline-YOLOv8]
tags: [MLOps, Kubeflow]
---

본 문서에서는 학습이 완료된 모델을 평가하는 컴포넌트를 작성하겠습니다.

학습이 완료된 모델을 테스트셋으로 평가하고, Precision, Recall, mAP 등 메트릭을 Kubeflow UI를 통해 표로 시각화하는 것을 다룹니다.

# Evaluate MLmodel

---

모델 학습을 완료한 후 테스트셋을 이용해 평가하는 것은 매우 중요합니다. 실제 환경에서 새로운 데이터에 대해 모델이 얼마나 잘 예측할 것인가를 예상하기 위해, 학습에 사용되지 않은 데이터에 대한 모델의 일반화 성능을 평가합니다.

YOLOv8 에서는 `model.val()` 를 이용해 모델을 평가할 수 있습니다. 

> [https://docs.ultralytics.com/modes/val/](https://docs.ultralytics.com/modes/val/)

Validation 설정은 아래와 같습니다.

| **Key**       | **Value** | **Description**                                                    |
| ------------- | --------- | ------------------------------------------------------------------ |
| `data`        | `None`    | path to data file, i.e. coco128.yaml                               |
| `imgsz`       | `640`     | size of input images as integer                                    |
| `batch`       | `16`      | number of images per batch (-1 for AutoBatch)                      |
| `save_json`   | `False`   | save results to JSON file                                          |
| `save_hybrid` | `False`   | save hybrid version of labels (labels + additional predictions)    |
| `conf`        | `0.001`   | object confidence threshold for detection                          |
| `iou`         | `0.6`     | intersection over union (IoU) threshold for NMS                    |
| `max_det`     | `300`     | maximum number of detections per image                             |
| `half`        | `True`    | use half precision (FP16)                                          |
| `device`      | `None`    | device to run on, i.e. cuda device=0/1/2/3 or device=cpu           |
| `dnn`         | `False`   | use OpenCV DNN for ONNX inference                                  |
| `plots`       | `False`   | save plots and images during train/val                             |
| `rect`        | `False`   | rectangular val with each batch collated for minimum padding       |
| `split`       | `val`     | dataset split to use for validation, i.e. 'val', 'test' or 'train' |

`split`을 보면, 기본값으로 `val`을 사용하고 있는 것을 알 수 있습니다. 하지만, 본 테스크에선 데이터셋을 train, val, test로 나눠 test 데이터셋을 이용해 평가를 하려합니다.

우선, 평가 이전 데이터셋을 확인하겠습니다.

## Dataset

---

### Directory Tree

```bash
.
├── cfg-custom.yaml
├── data-custom.yaml
├── data-custom.yaml
├── datasets
│   ├── custom_augment  [30000 entries exceeds filelimit, not opening dir]
│   ├── custom_test.txt
│   ├── custom_train.txt
│   ├── custom_val.txt
│   ├── coco128  [256 entries exceeds filelimit, not opening dir]
│   ├── coco128_test.txt
│   ├── coco128_train.txt
│   └── coco128_val.txt
├── yolov8n.pt
└── yolov8s.pt
```

> Jupyter 노트북과 마운트되어있는 폴더입니다.
> 

### Split Dataset

현재 데이터 관리를 폴더별로 구분지어 하는게 아닌, 데이터의 경로가 담긴 텍스트파일을 이용해 데이터를 불러오고 있습니다.

기존에 있던 스크립트를 사용해 train, val, test 세 개의 텍스트 파일을 생성합니다.

- 텍스트 파일 내용

```
./custom_augment/img0001.jpg
./custom_augment/img0002.jpg
./custom_augment/img0003.jpg
...
```

- `data-custom.yaml`

```yaml
# train and val data as 1) directory: path/images/, 2) file: path/images.txt, or 3) list: [path1/images/, path2/images/]
# Default path is '/data/datasets'
# To edit default path check this link "https://docs.ultralytics.com/quickstart/#inspecting-settings"
train: custom_train.txt
val: custom_val.txt
test: custom_test.txt

# number of classes
nc: 5

# class names
names: [ 'cat', 'dog', 'orange', 'octopus', 'person' ]
```

## Evaluate

---

이제, 파이프라인에 평가 단계를 추가하기 위한 컴포넌트를 작성하겠습니다.

### `test_op`

```python
@partial(
    create_component_from_func,
    base_image="nohgyu/test:v1.2",
)
def test_op(
    model_path: str,
    data: str,
    mlpipeline_ui_metadata_path: OutputPath("UI_Metadata"),
    ):
    import os
    from ultralytics import YOLO    
    import mlflow
    import json
    import pandas as pd

    # MLflow Setup
    os.environ["MLFLOW_TRACKING_URI"] = "http://mlflow-server-service.mlflow-system.svc:5000"
    os.environ["MLFLOW_S3_ENDPOINT_URL"] = "http://minio-service.kubeflow.svc:9000"
    os.environ["AWS_ACCESS_KEY_ID"] = "minio"
    os.environ["AWS_SECRET_ACCESS_KEY"] = "minio123"
    
    # Load and Test
    model = YOLO(model_path)
    
    metrics = model.val(data=data, split='test')
    
    precision, recall, map50, map50_95, fitness = list(metrics.results_dict.values())

    # Define table schema
    schema = [
        {"name": "Metrics", "type": "STRING"},
        {"name": "Value", "type": "NUMBER"},
    ]

    # Prepare table data
    prediction_results = [
        {"name": "Precision", "value": precision},
        {"name": "Recall", "value": recall},
        {"name": "mAP@0.5", "value": map50},
        {"name": "mAP@0.5-0.95", "value": map50_95},
        {"name": "Fitness", "value": fitness},
    ]
    
    # Convert to CSV
    df = pd.DataFrame(prediction_results)
    csvsource = df.to_csv(index=False, header=False)

    # Generate metadata for UI visualization
    metadata = {
        "outputs": [{
                "type": "table",
                "format": "csv",
                "header": [x["name"] for x in schema],
                "source": csvsource,
                "storage": "inline",
        }]
    }   
    # Write metadata to file
    with open(mlpipeline_ui_metadata_path, 'w', encoding='utf-8') as metadata_file:
        json.dump(metadata, metadata_file)
```

코드는 기존의 `train_op` , `tune_op` 와 매우 유사합니다.

`split='test'` 를 통해 `data-yaml`에서 정의한 `custom_test.txt` 파일에 대해 평가를 진행하게 됩니다.

평가가 끝난 후, MLflow에 로깅되지만, Pipeline UI에서 바로 확인할 수 있도록 Table을 만들어 출력하게 됩니다.

`metrics` 에 관한 자세한 내용은<br>
[https://docs.ultralytics.com/reference/utils/metrics/?h=detmetrics#ultralytics.utils.metrics.DetMetrics](https://docs.ultralytics.com/reference/utils/metrics/?h=detmetrics#ultralytics.utils.metrics.DetMetrics) 참고

### `train_pipeline`

```python
@dsl.pipeline(name="pipelinename",
          description="MLpipline Description",
          )
def train_pipeline(
    model_name: str = 'yolov8n.pt',
    cfg: str = 'cfg-custom',
    data: str = 'data-custom',
    bool_train: bool=True,
    bool_tune: bool=False,
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
    
    verifier = verify_pipeline_op(model_name, cfg, data)
    
    get_distribution = get_data_distribution_op(verifier.outputs['data_yaml'])
    
    plotting = plot_data_distribution_op(get_distribution.outputs["result_json"], get_distribution.outputs["cls_names_json"])
    
    with dsl.Condition(bool_train == True , "train"):
        trainer = train_op(model_name, verifier.outputs['cfg_yaml'], verifier.outputs['data_yaml'])
        tester = test_op(trainer.outputs['last_pt'], trainer.outputs['data_yaml'])
        
        with dsl.Condition(bool_tune == True , 'tune'):
            raytuner = tune_op(trainer.outputs['last_pt'],trainer.outputs['best_pt'],trainer.outputs['data_yaml'])
...
```

컴포넌트 작성을 완료했다면, 학습이 완료된 모델의 경로와 데이터 yaml파일을 입력받아 컴포넌트를 실행할 수 있도록 추가합니다.

- 실행 화면

![Untitled](/assets/img/kubeflow/kubeyolo401.png)

## Conclusion

---

ultralytics 패키지를 이용해 쉽게 평가를 진행할 수 있습니다.<br>
또한, 학습과 튜닝하는 컴포넌트와 유사한 코드로 작성할 수 있어 편리합니다.

Kubeflow Pipeline UI는 Visualization기능을 제공해 위와 같이 metrics를 받아 Table로 시각화할 수 있습니다.

현재는 모델의 Precision, Recall, mAP@0.5, mAP@0.5-0.95, Fitness를 표로 출력하고 있지만, 필요에 따라 클래스별 성능 지표를 추가할 수 있을 것으로 보입니다.

![Untitled](/assets/img/kubeflow/kubeyolo402.png)

> (23.12.12작성) <br>
> Kubeflow Piepelines v1SDK는 Table외에도 Confusion matrix, Markdown, ROC curve, TensorBoard, Web app을 지원합니다. 여기서 Web app을 사용하면, 이미지를 HTML형식으로 바꿔 시각화하는 것이 가능합니다.<br>
[Data Analysis Component + Visualization](https://www.notion.so/Data-Analysis-Component-Visualization-d18d8dba0c684e7cb868d3c2a9b7120b?pvs=21) 
>
>이를 이용해 YOLO 학습 및 평가 시 기본으로 생성되는 그래프들을 불러와 시각화하는 기능을 추가했습니다.
  - Train : `labels.jpg`, `labels_correlogram.jpg`, `train_batch0.jpg`, `confusion_matrix.png`, `results.png`
  - Test : `confusion_matrix.png`, `confusion_matrix_normalized.png`
{: .prompt-tip }
