---
title: "Data Analysis Component + Visualization"
author: knowgyu
description: " "
date: 2023-12-09 09:43:01 +0900
math: true
categories: [MLOps, 4-Kubeflow-Pipeline-YOLOv8]
tags: [MLOps, Kubeflow]
---

본 문서에서는 학습 전 데이터셋 분석을 위한 컴포넌트를 작성하겠습니다.

클래스별 이미지 파일 수와 인스턴스 수를 json형태로 저장하고, Kubeflow UI를 통해 시각화하는 것을 다룹니다.

# Data Analysis

---

ML모델을 학습할 때, 데이터셋은 매우 중요한 역할을 합니다. 데이터에 의해 ML모델의 성능 저하가 일어날 수 있는 문제점들은 아래와 같습니다.

- 작은 크기의 데이터셋
- 입력 이미지의 크기
- (상대적으로) 작은 인스턴스 : 입력 이미지 내에 라벨링되는 객체가 작은 경우
- 겹쳐있는 인스턴스
- 부정확한 바운딩 박스
- 클래스 불균형

위 문제들 외에도 데이터에 의한 ML모델 성능 저하는 다양한 원인으로 발생합니다.

이러한 데이터셋의 문제를 해결 혹은 데이터셋을 더욱 잘 이해하기 위해 CDA, EDA와 같은 데이터 분석 기법이 존재합니다.

무수히 많은 데이터를 분석하고 이해하기 위해서 시각화는 매우 중요한 사항입니다.

이를 위해 본 페이지에서는 파이프라인 Config로 입력받은 `data-custom.yaml` 에서 데이터셋의 위치와 클래스 이름을 확인하고, 해당 데이터셋의 클래스별 이미지 파일의 수와 인스턴스 수를 Kubeflow Dashboard UI를 이용해 시각화하겠습니다.

## Pipeline Visualization

---

Kubeflow 대시보드의 Runs에서 실행된 파이프라인의 진행사항과 결과를 볼 수 있습니다.

그래프에서 실행된 컴포넌트를 누르면 컴포넌트의 입출력, Logs, Visualization과 같은 실행 정보를 확인할 수 있습니다.

이 중, Visualization 기능을 사용해 컴포넌트에서 생성된 Plot을 확인하겠습니다.

> [https://mlops-for-all.github.io/docs/kubeflow/advanced-run](https://mlops-for-all.github.io/docs/kubeflow/advanced-run)
> 

### 시각화 예시

`mlpipeline_ui_metadata: OutputPath("UI_Metadata")` argument에 html 포맷으로 저장해 plot을 생성할 수 있습니다.

```python
@partial(
    create_component_from_func,
    packages_to_install=["matplotlib"],
)
def plot_linear(
    mlpipeline_ui_metadata: OutputPath("UI_Metadata")
):
    import base64
    import json
    from io import BytesIO

    import matplotlib.pyplot as plt

    plt.plot(x=[1, 2, 3], y=[1, 2,3])

    tmpfile = BytesIO()
    plt.savefig(tmpfile, format="png")
    encoded = base64.b64encode(tmpfile.getvalue()).decode("utf-8")

    html = f"<img src='data:image/png;base64,{encoded}'>"
    metadata = {
        "outputs": [
            {
                "type": "web-app",
                "storage": "inline",
                "source": html,
            },
        ],
    }
    with open(mlpipeline_ui_metadata, "w") as html_writer:
        json.dump(metadata, html_writer)
```

💡 Kubeflow Pipeline UI에서 시각화를 하기 위해 아티팩트의 이름은 반드시 `mlpipeline-ui-metadata` 로 지정해야합니다.


> For KFP v1, the pipeline component must write a JSON file specifying metadata for the output viewer(s) that you want to use for visualizing the results. The component must also export a file output artifact with an artifact name of `mlpipeline-ui-metadata`, or else the Kubeflow Pipelines UI will not render the visualization. In other words, the `.outputs.artifacts` setting for the generated pipeline component should show: `- {name: mlpipeline-ui-metadata, path: /mlpipeline-ui-metadata.json}`. The JSON filepath does not matter, although `/mlpipeline-ui-metadata.json` is used for consistency in the examples below.

- 실행 결과

![모두를 위한 MLOps https://mlops-for-all.github.io/docs/kubeflow/advanced-run](/assets/img/kubeflow/kubeyolo301.png)


## Data distribution

---

위 기능을 이용해 Plot하기 전, `data-custom.yaml`을 읽고, 클래스 이름과 데이터셋의 클래스별 분포를 분석하겠습니다.


- `data-custom.yaml`

```python
# train and val data as 1) directory: path/images/, 2) file: path/images.txt, or 3) list: [path1/images/, path2/images/]
# Default path is '/data/datasets'
# To edit default path check this link "https://docs.ultralytics.com/quickstart/#inspecting-settings"
train: custom_train.txt
val: custom_test.txt

# number of classes
nc: 5

# class names
names: [ 'cat', 'dog', 'orange', 'person', 'glasses' ]
```

### 데이터 분포 분석 컴포넌트

- `get_data_distribution_op`(1/2)

```python
def get_data_distribution_op(
    data_yaml: str,
    mlpipeline_metrics_path: OutputPath("Metrics"), # Unused
    result_json_path: OutputPath("json"),
    cls_names_json_path: OutputPath("json"),
    ):
    import os
    import yaml
    import json
    from collections import defaultdict
    from tqdm import tqdm
    from ultralytics import settings
    
    yolo_default_dir = settings['datasets_dir']
    
    # Read yaml and Get train.txt path
    with open(data_yaml, 'r', encoding='utf-8') as file:
        data = yaml.safe_load(file)
        train_txt_rel_path = data['train']
        train_txt_abs_path = yolo_default_dir + '/' + train_txt_rel_path
        
        # Get Class names from yaml file
        cls_names = data['names']
        print('cls_names:',cls_names)
        
        cls_dict = {i: cls_names[i] for i in range(len(cls_names))}

        
    # Read train.txt and Get dataset path
    with open(train_txt_abs_path, 'r', encoding='utf-8') as file:
        line = file.readline().strip()
        dir = os.path.dirname(line)
        
        # if folders are nested, dirname func may not work as desired
        folders = dir.split(os.path.sep)
        print(folders)
        
        dataset_dir = os.path.join(yolo_default_dir, folders[1])
```

위 함수의 Args는 아래와 같습니다.

- `data_yaml` : 파이프라인 실행 시 입력받은 데이터셋 구성 정보를 담은 yaml파일 경로입니다.
- `mlpipeline_metrics_path` : 아직 구현하지 않은 기능입니다.(metrics기능을 이용 표로 나타낼 예정)
- `result_json_path` : json형태의 파일을 plot 컴포넌트로 보내기 위한 인자입니다.
- `cls_names_json_path` : json형태의 파일을 plot 컴포넌트로 보내기 위한 인자입니다.

위 코드 스니펫은 입력받은 yaml파일을 읽어 학습하려는 파일들의 경로가 담긴 `train.txt` 파일의 위치를 찾고, `train.txt` 의 첫번째 줄을 읽어 데이터셋의 경로를 찾습니다. 또한, yaml파일로부터 클래스 이름을 딕셔너리 형태로 저장합니다.

> `train.txt` 는 `./coco128/025324.jpg` 와 같이 `./` 를 포함해야합니다.<br>
> `./animal/cat/siamesecat/029384.jpg` 와 같이 중첩된 폴더일 경우 `dir.split(os.path.sep)` 을 이용해 `./animal` 의 형태로 저장합니다.
{: .prompt-tip }

- `get_data_distribution_op`(2/2)

```python
# Search txt files and count imgs and instances per class
    cls_files = defaultdict(set)
    class_instances = defaultdict(int)
    
    for root, _, files in tqdm(os.walk(dataset_dir)):
        for filename in files:
            if filename.endswith('.txt'):
                with open(os.path.join(root, filename), 'r', encoding='utf-8') as f:
                    for line in f.readlines():
                        try:
                            cls_id = int(line.split()[0])
                        
                        except ValueError:
                            continue
                        
                        cls_files[cls_id].add(filename)
                        class_instances[cls_id] += 1
                        
    class_imgs = {k:len(cls_files[k]) for k in sorted(cls_files.keys())}
    class_instances = {k:v for k,v in sorted(class_instances.items())}
    
    result_dict = {key: {'img': class_imgs[key], 'instances': class_instances[key]} for key in class_instances}
    
    print('Result dict\n',result_dict)
    print('Class name dict\n',cls_dict)

    with open(result_json_path, 'w', encoding='utf-8') as f:
        json.dump(result_dict, f)
        
    with open(cls_names_json_path, 'w', encoding='utf-8') as f:
        json.dump(cls_dict, f)
    
    # Unused
    with open(mlpipeline_metrics_path, 'w', encoding='utf-8') as f:
        json.dump(result_dict, f)
```

데이터셋의 경로를 찾았다면, 해당 데이터셋을 순회하며 클래스 별 파일 수와 인스턴스 수를 저장합니다.

`result_dict` 와 `cls_dict` 의 경우 아래와 같이 저장됩니다.

```python
# result_dict
{"0": {"img": 62, "instances": 261}, "1": {"img": 3, "instances": 6}, "2": {"img": 13, "instances": 48}, "3": {"img": 5, "instances": 6}, ...

# cls_dict
{"0": "person", "1": "bicycle", "2": "car", "3": "motorcycle", "4": "airplane", "5": "bus", "6": "train", "7": "truck",
```

위와 같이 딕셔너리 형태로 저장했다면, Json형태로 저장해 다음 컴포넌트로 전달합니다.

- 실행 결과

![Untitled](/assets/img/kubeflow/kubeyolo302.png)

### 데이터 Plot 컴포넌트

- `plot_data_distribution_op` (1/2)

```python
def plot_data_distribution_op(
    result_json_path: InputPath("json"),
    cls_names_json_path: InputPath("json"),
    mlpipeline_ui_metadata: OutputPath("UI_Metadata"),
    ) -> bool:
    import os
    from collections import defaultdict
    from tqdm import tqdm
    import json
    
    import base64
    from io import BytesIO
    import matplotlib.pyplot as plt
    
    import pandas as pd
    import seaborn as sns
    from matplotlib.ticker import MultipleLocator

    # Load data from JSON
    with open(result_json_path, 'r', encoding='utf-8') as f:
        result_dict = json.load(f)
    
    with open(cls_names_json_path, 'r', encoding='utf-8') as f:
        cls_dict = json.load(f)
        
    result_dict = {int(key): value for key, value in result_dict.items()}
    cls_dict = {int(key): value for key, value in cls_dict.items()}
    print('result dict\n',result_dict)
    print('cls_dict\n',cls_dict)

    # Convert data to DataFrame
    df = pd.DataFrame(list(result_dict.items()), columns=['Classes', 'Values'], index=result_dict.keys())
    df_values = pd.json_normalize(df['Values'])
    df = pd.concat([df.drop('Values', axis=1), df_values], axis=1)
    df_melted = pd.melt(df, id_vars=['Classes'], value_vars=['img', 'instances'], var_name='Category', value_name='Values')
    df_melted['Classes'] = df_melted['Classes'].map(cls_dict)
```

위 함수의 Args들은 아래와 같습니다.

- `result_json_path` : 데이터 분석 컴포넌트에서 생성한 클래스 인덱스별 이미지수와 인스턴스 수를 담은 Json을 받아옵니다.
- `cls_names_json_path` : 데이터 분석 컴포넌트에서 생성한 클래스 인덱스별 클래스 이름을 담은 Json을 받아옵니다.
- `mlpipeline_ui_metadata` : Kubeflow Pipeline UI에 시각화하기위한 HTML을 반환합니다.

위 코드 스니펫은 데이터 분포 분석 컴포넌트에서 받은 Json파일을 통해 받아온 후, `result_dict`와 `cls_dict`형태로 저장합니다.

그 후, matplotlib로 Plot하기 전, `pandas` 라이브러리를 이용해 `DataFrame` 형태로 변환합니다.

<aside>
💡 `result_dict = {int(key): value for key, value in result_dict.items()}` 
위 코드의 경우 이전 컴포넌트에서 Json형태로 딕셔너리를 가져올 때, key값이 String으로 바뀌어 저장되었는데, Plot하는 코드에서 key값을 int로 처리하기에 형변환하였습니다.

</aside>

```python
# Plotting
...
    # Save plot as image and encode to base64
    tmpfile = BytesIO()
    plt.savefig(tmpfile, format='png')
    encoded = base64.b64encode(tmpfile.getvalue()).decode("utf-8")

    # Create HTML string
    html = f"<img src='data:image/png;base64,{encoded}'>"

    # Define metadata for the web app
    metadata = {
        "outputs": [
            {
                "type": "web-app",
                "storage": "inline",
                "source": html,
            },
        ],
    }

    # Save metadata to file
    with open(mlpipeline_ui_metadata, "w") as html_writer:
        json.dump(metadata, html_writer)
        
    return True
```

Plot하는 과정은 생략하였습니다.

Plot 후 ‘png’형태로 저장 → HTML로 변환해 `mlpipeline_ui_metadata`로 저장합니다.

- 실행 결과

![Untitled](/assets/img/kubeflow/kubeyolo303.png)

## Conclusion

---

전체 코드는 아래와 같습니다.

- `pipeline_compiler.py`
    
    ```python
    from functools import partial
    from typing import NamedTuple
    
    import kfp
    from kfp.components import create_component_from_func, InputPath, OutputPath
    from kfp import dsl
    
    @partial(
        create_component_from_func,
        base_image="nohgyu/test:v1.2",
    )
    def verify_pipeline_op(
        model_path: str,
        cfg_input: str,
        data_input: str,
        ) -> NamedTuple("Yamlfiles",[("cfg_yaml", str), ("data_yaml", str)]):
        import os
        import torch
        from collections import namedtuple
    
        if not cfg_input.endswith('.yaml'):
            cfg_input += '.yaml'
        if not data_input.endswith('.yaml'):
            data_input += '.yaml'    
        
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"FileNotFound : {model_path}")    
        if not os.path.isfile(cfg_input):
            raise FileNotFoundError(f"FileNotFound : {cfg_input}")
        if not os.path.isfile(data_input):
            raise FileNotFoundError(f"FileNotFound : {data_input}")
        
        verify_outputs = namedtuple(
            'Yamlfiles',
            ['cfg_yaml', 'data_yaml'],
        )
        
        return verify_outputs(cfg_input,data_input)
    
    @partial(
        create_component_from_func,
        base_image="nohgyu/test:v1.2",
    )    
    def get_data_distribution_op(
        data_yaml: str,
        mlpipeline_metrics_path: OutputPath("Metrics"), # Unused
        result_json_path: OutputPath("json"),
        cls_names_json_path: OutputPath("json"),
        ):
        import os
        import yaml
        import json
        from collections import defaultdict
        from tqdm import tqdm
        from ultralytics import settings
        
        yolo_default_dir = settings['datasets_dir']
        
        # Read yaml and Get train.txt path
        with open(data_yaml, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            train_txt_rel_path = data['train']
            train_txt_abs_path = yolo_default_dir + '/' + train_txt_rel_path
            
            # Get Class names from yaml file
            cls_names = data['names']
            print('cls_names:',cls_names)
            
            cls_dict = {i: cls_names[i] for i in range(len(cls_names))}
    
            
        # Read train.txt and Get dataset path
        with open(train_txt_abs_path, 'r', encoding='utf-8') as file:
            line = file.readline().strip()
            dir = os.path.dirname(line)
            
            # if folders are nested, dirname func may not work as desired
            folders = dir.split(os.path.sep)
            print(folders)
            
            dataset_dir = os.path.join(yolo_default_dir, folders[1])
            
        # Search txt files and count imgs and instances per class
        cls_files = defaultdict(set)
        class_instances = defaultdict(int)
        
        for root, _, files in tqdm(os.walk(dataset_dir)):
            for filename in files:
                if filename.endswith('.txt'):
                    with open(os.path.join(root, filename), 'r', encoding='utf-8') as f:
                        for line in f.readlines():
                            try:
                                cls_id = int(line.split()[0])
                            
                            except ValueError:
                                continue
                            
                            cls_files[cls_id].add(filename)
                            class_instances[cls_id] += 1
                            
        class_imgs = {k:len(cls_files[k]) for k in sorted(cls_files.keys())}
        class_instances = {k:v for k,v in sorted(class_instances.items())}
        
        result_dict = {key: {'img': class_imgs[key], 'instances': class_instances[key]} for key in class_instances}
        
        print('Result dict\n',result_dict)
        print('Class name dict\n',cls_dict)
    
        with open(result_json_path, 'w', encoding='utf-8') as f:
            json.dump(result_dict, f)
            
        with open(cls_names_json_path, 'w', encoding='utf-8') as f:
            json.dump(cls_dict, f)
        
        # Unused
        with open(mlpipeline_metrics_path, 'w', encoding='utf-8') as f:
            json.dump(result_dict, f)
    
    @partial(
        create_component_from_func,
        base_image="nohgyu/test:v1.2",
        packages_to_install=["seaborn","pandas"]
    )    
    def plot_data_distribution_op(
        result_json_path: InputPath("json"),
        cls_names_json_path: InputPath("json"),
        mlpipeline_ui_metadata: OutputPath("UI_Metadata"),
        ) -> bool:
        import os
        from collections import defaultdict
        from tqdm import tqdm
        import json
        
        import base64
        from io import BytesIO
        import matplotlib.pyplot as plt
        
        import pandas as pd
        import seaborn as sns
        from matplotlib.ticker import MultipleLocator
    
        # Load data from JSON
        with open(result_json_path, 'r', encoding='utf-8') as f:
            result_dict = json.load(f)
        
        with open(cls_names_json_path, 'r', encoding='utf-8') as f:
            cls_dict = json.load(f)
            
        result_dict = {int(key): value for key, value in result_dict.items()}
        cls_dict = {int(key): value for key, value in cls_dict.items()}
        print('result dict\n',result_dict)
        print('cls_dict\n',cls_dict)
    
        # Convert data to DataFrame
        df = pd.DataFrame(list(result_dict.items()), columns=['Classes', 'Values'], index=result_dict.keys())
        df_values = pd.json_normalize(df['Values'])
        df = pd.concat([df.drop('Values', axis=1), df_values], axis=1)
        df_melted = pd.melt(df, id_vars=['Classes'], value_vars=['img', 'instances'], var_name='Category', value_name='Values')
        df_melted['Classes'] = df_melted['Classes'].map(cls_dict)
    
        # Plotting
        sns.set(style='whitegrid')
        plt.figure(figsize=(14, 8))
    
        # Define display variable
        display = 'both'
    
        # Plot based on display value
        palette = 'Set2'
        title = '< Distribution of Img & GT per Class >'
    
        # Create bar plot
        ax = sns.barplot(data=df_melted,
                        x='Values',
                        y='Classes',
                        hue='Category',
                        palette=palette,
                        order=df_melted['Classes'].unique(),
                        orient='h')
    
        # Customize plot
        plt.xlabel('Count', rotation=0, fontsize=12)
        plt.ylabel('Classes', rotation=90, fontsize=12)
        plt.title(title)
        plt.gca().yaxis.set_major_locator(MultipleLocator(base=1))
    
        # Display values on the plot
        for p in ax.patches:
            ax.text(p.get_width() + 200,
                    p.get_y() + p.get_height() / 1.7,
                    f"{p.get_width():.0f}",
                    ha='left', va='center', fontsize=10)
    
            if display == 'img':
                ax.text(p.get_width() + 2500,
                        p.get_y() + p.get_height() / 1.7,
                        f"({round((int(p.get_width()) / sum(df_melted[df_melted['Category'] == 'img']['Values']) * 100), 1)}%)",
                        ha='left', va='center', fontsize=9, color='purple', alpha=1.0)
            elif display == 'instances':
                ax.text(p.get_width() + 5200,
                        p.get_y() + p.get_height() / 1.7,
                        f"({round((int(p.get_width()) / sum(df_melted[df_melted['Category'] == 'instances']['Values']) * 100), 1)}%)",
                        ha='left', va='center', fontsize=9, color='#004d97', alpha=1.0)
    
        # Save plot as image and encode to base64
        tmpfile = BytesIO()
        plt.savefig(tmpfile, format='png')
        encoded = base64.b64encode(tmpfile.getvalue()).decode("utf-8")
    
        # Create HTML string
        html = f"<img src='data:image/png;base64,{encoded}'>"
    
        # Define metadata for the web app
        metadata = {
            "outputs": [
                {
                    "type": "web-app",
                    "storage": "inline",
                    "source": html,
                },
            ],
        }
    
        # Save metadata to file
        with open(mlpipeline_ui_metadata, "w") as html_writer:
            json.dump(metadata, html_writer)
            
        return True
    
    @partial(
        create_component_from_func,
        base_image="nohgyu/test:v1.2",
    )
    def train_op(
        model_path: str,
        cfg: str,
        data: str,
        ) -> NamedTuple("trainOutputs",[("last_pt",str), ("best_pt",str), ("data_yaml",str), ("project",str), ("exp_name", str)]):
        import os
        from ultralytics import YOLO
        import mlflow
        import yaml
        from collections import namedtuple
    
        # MLflow Setup
        os.environ["MLFLOW_TRACKING_URI"] = "http://mlflow-server-service.mlflow-system.svc:5000"
        os.environ["MLFLOW_S3_ENDPOINT_URL"] = "http://minio-service.kubeflow.svc:9000"
        os.environ["AWS_ACCESS_KEY_ID"] = "minio"
        os.environ["AWS_SECRET_ACCESS_KEY"] = "minio123"
    
        # Train
        model = YOLO(model_path)    
        
        results = model.train(cfg=cfg,data=data)
    
        with open(cfg,'r',encoding='utf-8') as file:
            config = yaml.safe_load(file)
            
        train_outputs = namedtuple(
            'trainOutputs',
            ['last_pt','best_pt','data_yaml','project','exp_name'],
        )
    
        data_yaml = data
        project = config.get('project')
        exp_name = config.get('name')
        last_pt = os.path.join(project,exp_name,'weights','last.pt')
        best_pt = os.path.join(project,exp_name,'weights','best.pt')
    
        return train_outputs(last_pt, best_pt, data_yaml, project, exp_name)
    
        
    @partial(
        create_component_from_func,
        base_image="nohgyu/test:v1.2",
    )
    def tune_op(
        last_pt: str,
        best_pt: str,
        data_yaml: str,
        project: str,
        exp_name: str,
        ):
        from ultralytics import YOLO
        from ray import tune
        import os
        import yaml
    
        os.environ["MLFLOW_TRACKING_URI"] = "http://mlflow-server-service.mlflow-system.svc:5000"
        os.environ["MLFLOW_S3_ENDPOINT_URL"] = "http://minio-service.kubeflow.svc:9000"
        os.environ["AWS_ACCESS_KEY_ID"] = "minio"
        os.environ["AWS_SECRET_ACCESS_KEY"] = "minio123"
    
        model = YOLO(last_pt)
        
        result = model.tune(
                    data=data_yaml,
                    space={"lr0": tune.uniform(1e-5, 1e-1)},
                    epochs=2,
                    grace_period=2,
                    gpu_per_trial=1,
                    iterations=1,
                    project=project,
                    name="tune",
                    batch=4,
                    use_ray=True
                    )
                
    @dsl.pipeline(name="pipelinename",
              description="MLpipline Description",
              )
    def train_pipeline(
        model_name: str = 'yolov8n.pt',
        cfg: str = 'cfg-custom',
        data: str = 'data-custom',
        bool_train: bool=True,
        bool_tune: bool=True,
        ):
        import os
        
        vop = dsl.VolumeOp(
            name="volume mount",
            resource_name="pvc-local-cocodata",
            storage_class='train',
            modes=dsl.VOLUME_MODE_RWO,
            size="3Gi",
            generate_unique_name=False,
            action='apply',)
        
        verifier = verify_pipeline_op(model_name, cfg, data)
        
        get_distribution = get_data_distribution_op(verifier.outputs['data_yaml'])
        
        plotting = plot_data_distribution_op(get_distribution.outputs["result_json"], get_distribution.outputs["cls_names_json"])
        
        with dsl.Condition(plotting.outputs!= None and bool_train == True , "train"):
            trainer = train_op(model_name, verifier.outputs['cfg_yaml'], verifier.outputs['data_yaml'])
            
            with dsl.Condition(bool_tune == True , 'tune'):
                raytuner = tune_op(trainer.outputs['last_pt'],trainer.outputs['best_pt'],trainer.outputs['data_yaml'],trainer.outputs['project'],trainer.outputs['exp_name'])
        
        # Volume Mount
        verifier.add_pvolumes({"/data":vop.volume})
        get_distribution.add_pvolumes({"/data":vop.volume})
        plotting.add_pvolumes({"/data":vop.volume})
        trainer.add_pvolumes({"/data":vop.volume})
        raytuner.add_pvolumes({"/data":vop.volume})
        
        # Disable caching. Two Options 
        '''
        This Option doesn't work well
        Use `.execution_options.caching_strategy.max_cache_staleness = "P0D"` instead.
        
        # verifier.set_caching_options(False)
        # get_distribution.set_caching_options(False)
        # plotting.set_caching_options(False)
        # trainer.set_caching_options(False)
        # raytuner.set_caching_options(False)
        '''
        verifier.execution_options.caching_strategy.max_cache_staleness = "P0D"
        get_distribution.execution_options.caching_strategy.max_cache_staleness = "P0D"
        plotting.execution_options.caching_strategy.max_cache_staleness = "P0D"
        trainer.execution_options.caching_strategy.max_cache_staleness = "P0D"
        raytuner.execution_options.caching_strategy.max_cache_staleness = "P0D"
        
        # UI Component name settings (Default is Op name)
        '''
        vop.set_display_name("UI Component name settings")
        verifier.set_display_name("UI Component name settings")
        get_distribution.set_display_name("UI Component name settings")
        plotting.set_display_name("UI Component name settings")
        trainer.set_display_name("UI Component name settings")
        raytuner.set_display_name("UI Component name settings")
        '''
    
    if __name__ == "__main__":
        kfp.compiler.Compiler().compile(train_pipeline, "knowgyu_optimizeTest.yaml")
    ```
    

### Pipeline Graph

![Untitled](/assets/img/kubeflow/kubeyolo304.png)

### Pipeline Run parameters

| 항목       | 예시        | 설명                                                                                             |
| ---------- | ----------- | ------------------------------------------------------------------------------------------------ |
| model_name | yolov8n.pt  | 모델을 입력합니다.                                                                               |
| cfg        | cfg-custom  | YOLO설정 및 하이퍼파라미터 Configuration 파일을 입력합니다. .yaml 확장자를 붙이지 않아도 됩니다. |
| data       | data-custom | 데이터셋 yaml파일을 입력합니다. yaml 확장자를 붙이지 않아도 됩니다.                              |
| bool_train | True        | bool형으로 True로 입력할 시 학습을 진행합니다.                                                   |
| bool_tune  | False       | bool형으로 True로 입력할 시 하이퍼파라미터 튜닝을 진행합니다.                                    |

## + Misc

---

### Disable Caching

Kubeflow는 캐싱 기능을 지원하여 동일한 Config로 파이프라인을 실행할 시 캐시된 파일을 이용해 실행 속도를 빠르게 합니다.

![캐시파일을 이용해 출력할 경우 위와 같이 표시됨](/assets/img/kubeflow/kubeyolo305.png)

캐시파일을 이용해 출력할 경우 위와 같이 표시됨

하지만, `cfg-custom` 혹은 `data-custom` 파일 내부에서 변경이 될 경우 입력되는 Config는 동일하기에 오작동하는 상황이 발생합니다.

이러한 문제를 막기 위해 캐시 기능을 끄겠습니다.

```python
# Disable caching. Two Options 
    '''
    This Option doesn't work well
    Use `.execution_options.caching_strategy.max_cache_staleness = "P0D"` instead.
    
    # verifier.set_caching_options(False)
    # get_distribution.set_caching_options(False)
    # plotting.set_caching_options(False)
    # trainer.set_caching_options(False)
    # raytuner.set_caching_options(False)
    '''
    verifier.execution_options.caching_strategy.max_cache_staleness = "P0D"
    get_distribution.execution_options.caching_strategy.max_cache_staleness = "P0D"
    plotting.execution_options.caching_strategy.max_cache_staleness = "P0D"
    trainer.execution_options.caching_strategy.max_cache_staleness = "P0D"
    raytuner.execution_options.caching_strategy.max_cache_staleness = "P0D"
```

`set_caching_options(False)` 의 경우 정상적으로 작동하지 않는 경우가 발생해 다른 방법을 사용했습니다.

`.execution_options.caching_strategy.max_cache_staleness = "P0D"` 를 통해 캐시 기능을 종료할 수 있습니다.

> 만약, 실제 학습을 위한 파이프라인 실행이 아닌, 각 컴포넌트 기능을 테스트하기 위함이라면, 몇몇 컴포넌트는 캐시기능을 사용하는 것이 좋을 수 있습니다.<br>
ex) 하이퍼 파라미터 튜닝 컴포넌트를 테스트하고 싶을 때(`raytuner`) 이전 컴포넌트들에서 데이터를 분석하거나 모델을 학습하지 않아도 상관없습니다. <br>
이런 상황에서 `raytuner`를 제외한 나머지 컴포넌트들은 주석처리해 캐시 기능을 사용한다면, `raytuner` 컴포넌트를 제외한 나머지 컴포넌트들의 결과는 캐시에서 가져오기에 빠르게 진행할 수 있습니다.
{: .prompt-tip }


### Set UI display name

Kubeflow Pipeline Graph에 표시되는 컴포넌트 이름은 각 함수의 이름입니다.

본 예시에서는 동일한 컴포넌트를 사용하지 않기에, 서로 구분할 수 있지만, 만약 동일한 컴포넌트를 여러번 사용하게 될 경우 UI로 표시되는 이름이 동일해 구별이 힘들 수 있습니다.

 `.set_display_name`을 이용해 파이프라인 그래프에 표시되는 이름을 변경할 수 있습니다.

```python
# UI Component name settings (Default is Op name)
    vop.set_display_name("UI Component name settings")
    verifier.set_display_name("UI Component name settings")
    get_distribution.set_display_name("UI Component name settings")
    plotting.set_display_name("UI Component name settings")
    trainer.set_display_name("UI Component name settings")
    raytuner.set_display_name("UI Component name settings")
```
