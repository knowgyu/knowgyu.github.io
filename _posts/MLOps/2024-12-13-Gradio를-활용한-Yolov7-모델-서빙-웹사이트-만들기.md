---
title: "Gradio를 활용한 Yolov7 모델 서빙 웹사이트 만들기"
author: knowgyu
description: " "
date: 2023-10-12 08:36:26 +0900
math: true
categories: [AI & CV, Gradio를 활용한 웹 기반 서빙]
tags: [MLOps, Yolo, Gradio, Docker]
---

# Gradio를 활용한 Yolov7 모델 서빙 웹페이지 만들기

ML 모델을 실험하고 공유하는 데 있어서 큰 어려움은 **모델을 테스트하기 위한 환경을 구축**하는 것입니다.   
Gradio를 사용하면 몇 줄의 Python 코드만으로 **웹 기반 데모**를 빠르게 생성할 수 있습니다.

이 글에서는 **Yolov7 모델**을 Gradio를 활용해 모델을 테스트하는 환경을 만들겠습니다.  
**이미지와 비디오를 입력받아 추론 결과를 반환**하는 **웹페이지**를 만들고, 모델 파일을 유연하게 관리하는 확장 기능도 추가해보겠습니다.

## 시작하기 전

다음 게시글에서는 Docker를 활용한 방법을 소개합니다.  
만약, 로컬에 설치하는 것이 부담이 된다면, Docker를 활용한 방법을 추천드립니다.

> Docker로 수행할 경우, pytorch 2.0.1버전과 cuda11.7버전이 설치된 이미지를 사용합니다.
{: .prompt-tip }

---

## 1. Gradio 설치

먼저 Gradio 라이브러리를 설치합니다.

```bash
pip install gradio==3.50.2
```

> gradio 버전 업데이트로 인해 기존 기능이 동작하지 않습니다.
> 이를 방지하고자 3.X버전 중 가장 최신 버전을 설치합니다.
{: .prompt-warning }

---

## 2. Yolov7 모델 설치

Gradio에서 사용할 **Yolov7 모델**을 설치합니다.  
Yolov7을 GitHub에서 다운로드합니다.

```bash
git clone https://github.com/WongKinYiu/yolov7
```

---

## 3. 필요 라이브러리 설치

Yolov7을 실행하기 위해 필요한 라이브러리들을 설치합니다.

### PyTorch 설치

PyTorch 설치는 [공식 사이트](https://pytorch.org/get-started/locally/)를 참고합니다.  
아래는 CUDA 11.8 기반의 PyTorch 설치 예제입니다.

```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### Yolov7 requirements 설치

```bash
pip install -r requirements.txt
```

### OpenCV 설치

Gradio와 OpenCV를 함께 사용하기 위해 다음 명령어를 실행합니다.

```bash
pip install opencv-python
```

---

## 4. 기본 모델 서빙 코드 (main.py)

먼저, Yolov7 모델을 불러와서 **이미지와 비디오 추론**을 진행하는 Gradio 기반 웹 앱을 작성합니다.

### main.py

```python
import cv2
import torch
import gradio as gr
from PIL import Image
import atexit
import os

# Load Yolov7 Model
model = torch.hub.load('', source='local', model='yolov7', force_reload=True)

# 이미지 처리 함수
def yolo_image(im):
    if im is None:
        return None
    results = model(im)
    results.render()
    return Image.fromarray(results.imgs[0])

# 비디오 처리 함수
def yolo_video(video):
    if video is None:
        return None

    cap = cv2.VideoCapture(video)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width, height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    tmp_output_video = 'tmp_output.mp4'
    output_vw = cv2.VideoWriter(tmp_output_video, fourcc, fps, (width, height))

    def delete_tmp_video():
        if os.path.exists(tmp_output_video):
            os.remove(tmp_output_video)

    atexit.register(delete_tmp_video)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        results = model(frame)
        results.render()
        output_vw.write(results.imgs[0])

    cap.release()
    output_vw.release()
    return tmp_output_video

# Gradio 인터페이스
iface = gr.Interface(
    fn=lambda img, vid, conf: (yolo_image(img), yolo_video(vid)),
    inputs=["image", "video", gr.Slider(0, 1, 0.25, label="Confidence Threshold")],
    outputs=["image", "video"],
    title="Yolov7 Gradio Demo",
    description="이미지와 비디오에 Yolov7 모델을 적용합니다."
)

if __name__ == "__main__":
    iface.launch(server_name="0.0.0.0", server_port=12345)
```
![alt text](/assets/img/Gradio/image1.png)
**실행 방법**:  
```bash
python main.py
```

실행 시 **yolov7.pt**를 자동으로 설치하고, 테스트할 수 있습니다.

---

## 5. 확장된 모델 서빙 UI (extend_main.py)

이제 사용자가 **다양한 모델을 업로드**하고 **Dropdown**에서 선택할 수 있는 확장 기능을 추가해봅니다.

### 주요 추가 기능

1. **모델 업로드 및 삭제**: 새로운 모델을 업로드하거나, 삭제할 수 있습니다.  
2. **Dropdown 모델 선택**: `weights` 폴더에 저장된 모델 파일 중 하나를 선택할 수 있습니다.  
3. **실시간 Confidence Threshold 조정**: Slider를 통해 모델의 예측 신뢰도 조정이 가능합니다.
4. **Inference Device 선택**: Inference할 Device로 CPU, cuda0, cuda1 등 선택할 수 있습니다.
  
우선, **4번 기능**을 추가하기 위해, `yolov7` 폴더에 있는 `hubconf.py`를 수정합니다.
> 이 중, 모델을 load하는 `custom` 함수를 수정하기 위함


### hubconf.py
```python
"""PyTorch Hub models

Usage:
    import torch
    model = torch.hub.load('repo', 'model')
"""

from pathlib import Path

import torch

from models.yolo import Model
from utils.general import check_requirements, set_logging
from utils.google_utils import attempt_download
from utils.torch_utils import select_device

dependencies = ['torch', 'yaml']
check_requirements(Path(__file__).parent / 'requirements.txt', exclude=('pycocotools', 'thop'))
set_logging()


def create(name, pretrained, channels, classes, autoshape):
    """Creates a specified model

    Arguments:
        name (str): name of model, i.e. 'yolov7'
        pretrained (bool): load pretrained weights into the model
        channels (int): number of input channels
        classes (int): number of model classes

    Returns:
        pytorch model
    """
    try:
        cfg = list((Path(__file__).parent / 'cfg').rglob(f'{name}.yaml'))[0]  # model.yaml path
        model = Model(cfg, channels, classes)
        if pretrained:
            fname = f'{name}.pt'  # checkpoint filename
            attempt_download(fname)  # download if not found locally
            ckpt = torch.load(fname, map_location=torch.device('cpu'))  # load
            msd = model.state_dict()  # model state_dict
            csd = ckpt['model'].float().state_dict()  # checkpoint state_dict as FP32
            csd = {k: v for k, v in csd.items() if msd[k].shape == v.shape}  # filter
            model.load_state_dict(csd, strict=False)  # load
            if len(ckpt['model'].names) == classes:
                model.names = ckpt['model'].names  # set class names attribute
            if autoshape:
                model = model.autoshape()  # for file/URI/PIL/cv2/np inputs and NMS
        device = select_device('0' if torch.cuda.is_available() else 'cpu')  # default to GPU if available
        return model.to(device)

    except Exception as e:
        s = 'Cache maybe be out of date, try force_reload=True.'
        raise Exception(s) from e


def custom(path_or_model='path/to/model.pt', autoshape=True, device=None):
    """custom mode

    Arguments (3 options):
        path_or_model (str): 'path/to/model.pt'
        path_or_model (dict): torch.load('path/to/model.pt')
        path_or_model (nn.Module): torch.load('path/to/model.pt')['model']

    Returns:
        pytorch model
    """
    model = torch.load(path_or_model, map_location=torch.device('cpu')) if isinstance(path_or_model, str) else path_or_model  # load checkpoint
    if isinstance(model, dict):
        model = model['ema' if model.get('ema') else 'model']  # load model

    hub_model = Model(model.yaml).to(next(model.parameters()).device)  # create
    hub_model.load_state_dict(model.float().state_dict())  # load state_dict
    hub_model.names = model.names  # class names
    if autoshape:
        hub_model = hub_model.autoshape()  # for file/URI/PIL/cv2/np inputs and NMS
    # device = select_device('0' if torch.cuda.is_available() else 'cpu')  # default to GPU if available
    device = select_device(device)
    return hub_model.to(device)


def yolov7(pretrained=True, channels=3, classes=80, autoshape=True):
    return create('yolov7', pretrained, channels, classes, autoshape)


if __name__ == '__main__':
    model = custom(path_or_model='yolov7.pt')  # custom example
    # model = create(name='yolov7', pretrained=True, channels=3, classes=80, autoshape=True)  # pretrained example

    # Verify inference
    import numpy as np
    from PIL import Image

    imgs = [np.zeros((640, 480, 3))]

    results = model(imgs)  # batched inference
    results.print()
    results.save()
```

이제, 기능을 추가한 `main.py`를 작성하겠습니다.

### extend_main.py
```python
import cv2
import torch
import gradio as gr
from PIL import Image
import os
import atexit
import shutil
from hubconf import custom

# Event Callback Function
def yolo_image(im):
    if im is None:
        return None
    
    model.conf = conf
    results = model(im)
    results.render()
    
    return Image.fromarray(results.imgs[0])
# Event Callback Function
def yolo_video(video):
    if video is None:
        return None
    
    # YOLO process
    def yolo(im):
        results = model(im)
        results.render()
        
        return results.imgs[0]
    
    # Delete tmp Video
    def delete_output_video():
        if os.path.exists(tmp_output_video):
            os.remove(tmp_output_video)

    atexit.register(delete_output_video)

    # Open Video
    cap = cv2.VideoCapture(video)

    # VideoWriter setting
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    tmp_output_video = 'tmp_processed_output.mp4'
    output_vw = cv2.VideoWriter(tmp_output_video, fourcc, fps,(width, height))
    
    try:
        while(cap.isOpened()):
            ret, frame = cap.read()
            if not ret:
                break

            model.conf = conf
            result = yolo(frame)
            output_vw.write(result)

    except Exception as e:
        print("Error Occured: ", str(e))
        # Delete tmp video
        delete_output_video()
    
    finally:
        cap.release()
        output_vw.release()
        
        return tmp_output_video

def slider_callback(value):
    global conf
    conf = value

def load_model(modeldropdown, devicedropdown):
    global model, device
    gr.Warning("Wait for Load Model")
    if torch.cuda.is_available() is not True and devicedropdown != "cpu":
        return gr.Warning("CUDA를 사용할 수 없습니다.")
    
    scrip_dir = os.path.dirname(os.path.abspath(__file__))
    if modeldropdown is None:
        return dropdown.update()
    model_path = scrip_dir + "/weights/" + modeldropdown
    model = custom(model_path, device=devicedropdown)
    
    
    if devicedropdown != "cpu":
        model.half()
    
    model.conf = conf

    return gr.Info(f"Device :{next(model.parameters()).device}")

def upload_save(file):
    
    file_name = os.path.basename(file.name)
    save_path = os.path.dirname(os.path.abspath(__file__)) + "/weights/" + file_name
    try:
        shutil.copy(file.name, save_path)
    except Exception as e:
        print("Error!!", e)
        return uploadbtn.update(label="Error Occured! Press F5")


    updated_list = [f for f in os.listdir("weights") if os.path.isfile(os.path.join("weights", f))]
    return dropdown.update(choices=updated_list)

def delete_callback(file_):
    
    file_path = os.path.dirname(os.path.abspath(__file__)) + "/weights/" + file_
    os.remove(file_path)


    updated_list = [f for f in os.listdir("weights") if os.path.isfile(os.path.join("weights", f))]

    return dropdown.update(choices=updated_list, value=None)

# Initial Setting
scrip_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(scrip_dir)
conf = 0.25
model_list = [f for f in os.listdir("weights") if os.path.isfile(os.path.join("weights", f))]
model = None
device = None

# Gradio Blocks Setting
KnowgyuBlock = gr.Blocks(theme="Soft", title="YOLOv7-Model_Test")
with KnowgyuBlock:
    gr.Markdown('''
                # [Gyu] Object Detection Model Test
                ### 사용법 :
                - 모델 선택 → 이미지(혹은 동영상) 업로드 → 'Run' 버튼 클릭
                - 'Upload model' 버튼을 통해 모델을 추가할 수 있습니다.
                ''')

    with gr.Tab("Image"):
        with gr.Row():
            image_input = gr.Image()
            image_output = gr.Image(label="Result")

        with gr.Row():
            slider_input1 = gr.Slider(minimum=0, maximum=1, value=0.25, label="Confidence Threshold", 
                                    interactive=True, container=True)
            image_button = gr.Button("Run")
            
            
    with gr.Tab("Video"):
        with gr.Row():
            video_input = gr.Video()
            video_output = gr.Video(label="Result")

        with gr.Row():
            slider_input2 = gr.Slider(minimum=0, maximum=1, value=0.25, label="Confidence Threshold",
                                    interactive=True, container=True)
            video_button = gr.Button("Run")

    with gr.Row():
        with gr.Column():
            dropdown = gr.Dropdown(label="Select Model", choices=model_list, 
                                   container=True,interactive=True)
            devicedropdown = gr.Dropdown(label="Select Device", choices=["cpu","0","1"], value="0")
        with gr.Column():
            uploadbtn = gr.UploadButton(label="Upload model", type='file'
                                        , file_types=[".pt"])
            deletebtn = gr.Button(value="Delete selected model", interactive=True)
            

    ## Event Callback Functions ##
    ##############################
    slider_input1.change(fn = slider_callback, inputs=slider_input1)
    slider_input2.change(fn = slider_callback, inputs=slider_input2)
    dropdown.change(fn = load_model, inputs = [dropdown,devicedropdown])
    devicedropdown.change(fn = load_model, inputs = [dropdown, devicedropdown])
    uploadbtn.upload(fn = upload_save, inputs = uploadbtn, outputs=dropdown)
    deletebtn.click(fn = delete_callback, inputs=dropdown, outputs=dropdown)

    with torch.no_grad():
        image_button.click(yolo_image,image_input,image_output)
        video_button.click(yolo_video,video_input,video_output)
    
    




if __name__ == "__main__":

    KnowgyuBlock.queue(max_size=10)
    KnowgyuBlock.launch(server_name="0.0.0.0", server_port=9999, show_error=True, inbrowser=True)

```
![alt text](/assets/img/Gradio/image2.png)

> 우측 상단을 보면, Device를 선택하고 그 결과를 확인할 수 있습니다.
{: .prompt-tip }

---

## 6. 실행 및 결과

1. **main.py**를 실행하면 기본 Gradio 웹 앱이 생성되어 이미지/비디오 추론이 가능합니다.  
2. **extend_main.py**를 실행하면 사용자가 모델을 업로드하고 선택할 수 있는 확장된 UI가 실행됩니다.  

> `main.py`에 기능을 추가한 코드가 `extend_main.py`입니다.

**실행 예시**:  
```bash
python extend_main.py
```

---

## 결론

이 글에서는 **Yolov7 모델**을 Gradio를 활용해 웹페이지 형태로 서빙하는 방법을 소개했습니다.  
기본 서빙부터 **모델 업로드, 선택, Confidence 조정**까지 확장된 기능을 구현함으로써 **실험과 배포**를 손쉽게 할 수 있습니다.  

다음 게시글에서는 DockerFile을 작성하고, Docker compose로 웹페이지를 배포해보겠습니다.
