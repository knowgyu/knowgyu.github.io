---
title: "Pipeline-Write"
author: knowgyu
description: " "
date: 2023-12-01 08:05:33 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

## Kubeflow Pipeline 작성

> Kubeflow를 사용하려면 **컴포넌트**와 **파이프라인**을 함께 작성해야 합니다.

컴포넌트는 독립적으로 실행되기보다, 파이프라인의 구성 요소로 연결되어 실행됩니다.

즉 컴포넌트를 실제로 실행해 보려면 파이프라인 정의가 먼저 필요합니다.

## Pipeline

> 파이프라인은 여러 컴포넌트의 집합과, 각 컴포넌트를 어떤 순서로 실행할지 정의한 흐름으로 구성됩니다.<br>
> 이 흐름은 방향 순환이 없는 그래프(DAG)이며, 간단한 조건문도 포함할 수 있습니다.

이 글에서는 숫자를 입력받아 출력하는 컴포넌트와, 두 숫자를 입력받아 합을 출력하는 컴포넌트를 묶어 간단한 파이프라인을 구성합니다.

## Component Set

1. 숫자를 입력받아 출력하고 반환하는 컴포넌트
    
```python
@create_component_from_func
def print_and_return_number(number: int) -> int:
    print(number)
    return number
```
    
2. 두 개의 숫자를 입력받아 합을 출력하는 컴포넌트
    
```python
@create_component_from_func
def sum_and_print_numbers(number_1: int, number_2: int) -> int:
    sum_num = number_1 + number_2
    print(sum_num)
    return sum_num
```
    

## Python 코드로 파이프라인 작성

```python
def example_pipeline(number_1: int, number_2: int):
	number_1_result = print_and_return_number(number_1)
	number_2_result = print_and_return_number(number_2)

	sum_result = sum_and_print_numbers(
			number_1 = number_1_result.output, number_2 = number_2_result.output
	)
```

## Kubeflow 형식으로 변환

마지막으로 Kubeflow에서 실행할 수 있는 형식으로 변환합니다.

```python
from kfp.dsl import pipeline

@pipeline(name='example_pipeline')
def example_pipeline(number_1: int, number_2: int):
	number_1_result = print_and_return_number(number_1)
	number_2_result = print_and_return_number(number_2)

	sum_result = sum_and_print_numbers(
			number_1 = number_1_result.output, number_2 = number_2_result.output
	)

if __name__ == "__main__":
	import kfp
	kfp.compiler.Compiler().compile(example_pipeline, "example_pipeline.yaml")
```

<aside>
💡 Kubeflow에서 파이프라인 실행은 YAML 형식으로 진행합니다.
`kfp.compiler.Compiler().compile(example_pipeline, "example_pipeline.yaml")` 로 컴파일합니다.

</aside>

## 전체 예제

```python
import kfp
from kfp.components import create_component_from_func
from kfp.dsl import pipeline

@create_component_from_func
def print_and_return_number(number: int) -> int:
    print(number)
    return number

@create_component_from_func
def sum_and_print_numbers(number_1: int, number_2: int):
    print(number_1 + number_2)

@pipeline(name="example_pipeline")
def example_pipeline(number_1: int, number_2: int):
    number_1_result = print_and_return_number(number_1)
    number_2_result = print_and_return_number(number_2)

    sum_result = sum_and_print_numbers(
        number_1=number_1_result.output, number_2=number_2_result.output
    )

if __name__ == "__main__":
    kfp.compiler.Compiler().compile(example_pipeline, "example_pipeline.yaml")
```

샘플 YAML
- `example_pipeline.yaml`
    
    ```yaml
    apiVersion: argoproj.io/v1alpha1
    kind: Workflow
    metadata:
      generateName: example-pipeline-
      annotations: {pipelines.kubeflow.org/kfp_sdk_version: 1.8.9, pipelines.kubeflow.org/pipeline_compilation_time: '2023-11-23T16:06:24.189360',
        pipelines.kubeflow.org/pipeline_spec: '{"inputs": [{"name": "number_1", "type":
          "Integer"}, {"name": "number_2", "type": "Integer"}], "name": "example_pipeline"}'}
      labels: {pipelines.kubeflow.org/kfp_sdk_version: 1.8.9}
    spec:
      entrypoint: example-pipeline
      templates:
      - name: example-pipeline
        inputs:
          parameters:
          - {name: number_1}
          - {name: number_2}
        dag:
          tasks:
          - name: print-and-return-number
            template: print-and-return-number
            arguments:
              parameters:
              - {name: number_1, value: '{{inputs.parameters.number_1}}'}
          - name: print-and-return-number-2
            template: print-and-return-number-2
            arguments:
              parameters:
              - {name: number_2, value: '{{inputs.parameters.number_2}}'}
          - name: sum-and-print-numbers
            template: sum-and-print-numbers
            dependencies: [print-and-return-number, print-and-return-number-2]
            arguments:
              parameters:
              - {name: print-and-return-number-2-Output, value: '{{tasks.print-and-return-number-2.outputs.parameters.print-and-return-number-2-Output}}'}
              - {name: print-and-return-number-Output, value: '{{tasks.print-and-return-number.outputs.parameters.print-and-return-number-Output}}'}
      - name: print-and-return-number
        container:
          args: [--number, '{{inputs.parameters.number_1}}', '----output-paths', /tmp/outputs/Output/data]
          command:
          - sh
          - -ec
          - |
            program_path=$(mktemp)
            printf "%s" "$0" > "$program_path"
            python3 -u "$program_path" "$@"
          - |
            def print_and_return_number(number):
                print(number)
                return number
    
            def _serialize_int(int_value: int) -> str:
                if isinstance(int_value, str):
                    return int_value
                if not isinstance(int_value, int):
                    raise TypeError('Value "{}" has type "{}" instead of int.'.format(
                        str(int_value), str(type(int_value))))
                return str(int_value)
    
            import argparse
            _parser = argparse.ArgumentParser(prog='Print and return number', description='')
            _parser.add_argument("--number", dest="number", type=int, required=True, default=argparse.SUPPRESS)
            _parser.add_argument("----output-paths", dest="_output_paths", type=str, nargs=1)
            _parsed_args = vars(_parser.parse_args())
            _output_files = _parsed_args.pop("_output_paths", [])
    
            _outputs = print_and_return_number(**_parsed_args)
    
            _outputs = [_outputs]
    
            _output_serializers = [
                _serialize_int,
    
            ]
    
            import os
            for idx, output_file in enumerate(_output_files):
                try:
                    os.makedirs(os.path.dirname(output_file))
                except OSError:
                    pass
                with open(output_file, 'w') as f:
                    f.write(_output_serializers[idx](_outputs[idx]))
          image: python:3.7
        inputs:
          parameters:
          - {name: number_1}
        outputs:
          parameters:
          - name: print-and-return-number-Output
            valueFrom: {path: /tmp/outputs/Output/data}
          artifacts:
          - {name: print-and-return-number-Output, path: /tmp/outputs/Output/data}
        metadata:
          labels:
            pipelines.kubeflow.org/kfp_sdk_version: 1.8.9
            pipelines.kubeflow.org/pipeline-sdk-type: kfp
            pipelines.kubeflow.org/enable_caching: "true"
          annotations: {pipelines.kubeflow.org/component_spec: '{"implementation": {"container":
              {"args": ["--number", {"inputValue": "number"}, "----output-paths", {"outputPath":
              "Output"}], "command": ["sh", "-ec", "program_path=$(mktemp)\nprintf \"%s\"
              \"$0\" > \"$program_path\"\npython3 -u \"$program_path\" \"$@\"\n", "def
              print_and_return_number(number):\n    print(number)\n    return number\n\ndef
              _serialize_int(int_value: int) -> str:\n    if isinstance(int_value, str):\n        return
              int_value\n    if not isinstance(int_value, int):\n        raise TypeError(''Value
              \"{}\" has type \"{}\" instead of int.''.format(\n            str(int_value),
              str(type(int_value))))\n    return str(int_value)\n\nimport argparse\n_parser
              = argparse.ArgumentParser(prog=''Print and return number'', description='''')\n_parser.add_argument(\"--number\",
              dest=\"number\", type=int, required=True, default=argparse.SUPPRESS)\n_parser.add_argument(\"----output-paths\",
              dest=\"_output_paths\", type=str, nargs=1)\n_parsed_args = vars(_parser.parse_args())\n_output_files
              = _parsed_args.pop(\"_output_paths\", [])\n\n_outputs = print_and_return_number(**_parsed_args)\n\n_outputs
              = [_outputs]\n\n_output_serializers = [\n    _serialize_int,\n\n]\n\nimport
              os\nfor idx, output_file in enumerate(_output_files):\n    try:\n        os.makedirs(os.path.dirname(output_file))\n    except
              OSError:\n        pass\n    with open(output_file, ''w'') as f:\n        f.write(_output_serializers[idx](_outputs[idx]))\n"],
              "image": "python:3.7"}}, "inputs": [{"name": "number", "type": "Integer"}],
              "name": "Print and return number", "outputs": [{"name": "Output", "type":
              "Integer"}]}', pipelines.kubeflow.org/component_ref: '{}', pipelines.kubeflow.org/arguments.parameters: '{"number":
              "{{inputs.parameters.number_1}}"}'}
      - name: print-and-return-number-2
        container:
          args: [--number, '{{inputs.parameters.number_2}}', '----output-paths', /tmp/outputs/Output/data]
          command:
          - sh
          - -ec
          - |
            program_path=$(mktemp)
            printf "%s" "$0" > "$program_path"
            python3 -u "$program_path" "$@"
          - |
            def print_and_return_number(number):
                print(number)
                return number
    
            def _serialize_int(int_value: int) -> str:
                if isinstance(int_value, str):
                    return int_value
                if not isinstance(int_value, int):
                    raise TypeError('Value "{}" has type "{}" instead of int.'.format(
                        str(int_value), str(type(int_value))))
                return str(int_value)
    
            import argparse
            _parser = argparse.ArgumentParser(prog='Print and return number', description='')
            _parser.add_argument("--number", dest="number", type=int, required=True, default=argparse.SUPPRESS)
            _parser.add_argument("----output-paths", dest="_output_paths", type=str, nargs=1)
            _parsed_args = vars(_parser.parse_args())
            _output_files = _parsed_args.pop("_output_paths", [])
    
            _outputs = print_and_return_number(**_parsed_args)
    
            _outputs = [_outputs]
    
            _output_serializers = [
                _serialize_int,
    
            ]
    
            import os
            for idx, output_file in enumerate(_output_files):
                try:
                    os.makedirs(os.path.dirname(output_file))
                except OSError:
                    pass
                with open(output_file, 'w') as f:
                    f.write(_output_serializers[idx](_outputs[idx]))
          image: python:3.7
        inputs:
          parameters:
          - {name: number_2}
        outputs:
          parameters:
          - name: print-and-return-number-2-Output
            valueFrom: {path: /tmp/outputs/Output/data}
          artifacts:
          - {name: print-and-return-number-2-Output, path: /tmp/outputs/Output/data}
        metadata:
          labels:
            pipelines.kubeflow.org/kfp_sdk_version: 1.8.9
            pipelines.kubeflow.org/pipeline-sdk-type: kfp
            pipelines.kubeflow.org/enable_caching: "true"
          annotations: {pipelines.kubeflow.org/component_spec: '{"implementation": {"container":
              {"args": ["--number", {"inputValue": "number"}, "----output-paths", {"outputPath":
              "Output"}], "command": ["sh", "-ec", "program_path=$(mktemp)\nprintf \"%s\"
              \"$0\" > \"$program_path\"\npython3 -u \"$program_path\" \"$@\"\n", "def
              print_and_return_number(number):\n    print(number)\n    return number\n\ndef
              _serialize_int(int_value: int) -> str:\n    if isinstance(int_value, str):\n        return
              int_value\n    if not isinstance(int_value, int):\n        raise TypeError(''Value
              \"{}\" has type \"{}\" instead of int.''.format(\n            str(int_value),
              str(type(int_value))))\n    return str(int_value)\n\nimport argparse\n_parser
              = argparse.ArgumentParser(prog=''Print and return number'', description='''')\n_parser.add_argument(\"--number\",
              dest=\"number\", type=int, required=True, default=argparse.SUPPRESS)\n_parser.add_argument(\"----output-paths\",
              dest=\"_output_paths\", type=str, nargs=1)\n_parsed_args = vars(_parser.parse_args())\n_output_files
              = _parsed_args.pop(\"_output_paths\", [])\n\n_outputs = print_and_return_number(**_parsed_args)\n\n_outputs
              = [_outputs]\n\n_output_serializers = [\n    _serialize_int,\n\n]\n\nimport
              os\nfor idx, output_file in enumerate(_output_files):\n    try:\n        os.makedirs(os.path.dirname(output_file))\n    except
              OSError:\n        pass\n    with open(output_file, ''w'') as f:\n        f.write(_output_serializers[idx](_outputs[idx]))\n"],
              "image": "python:3.7"}}, "inputs": [{"name": "number", "type": "Integer"}],
              "name": "Print and return number", "outputs": [{"name": "Output", "type":
              "Integer"}]}', pipelines.kubeflow.org/component_ref: '{}', pipelines.kubeflow.org/arguments.parameters: '{"number":
              "{{inputs.parameters.number_2}}"}'}
      - name: sum-and-print-numbers
        container:
          args: [--number-1, '{{inputs.parameters.print-and-return-number-Output}}', --number-2,
            '{{inputs.parameters.print-and-return-number-2-Output}}']
          command:
          - sh
          - -ec
          - |
            program_path=$(mktemp)
            printf "%s" "$0" > "$program_path"
            python3 -u "$program_path" "$@"
          - |
            def sum_and_print_numbers(number_1, number_2):
                print(number_1 + number_2)
    
            import argparse
            _parser = argparse.ArgumentParser(prog='Sum and print numbers', description='')
            _parser.add_argument("--number-1", dest="number_1", type=int, required=True, default=argparse.SUPPRESS)
            _parser.add_argument("--number-2", dest="number_2", type=int, required=True, default=argparse.SUPPRESS)
            _parsed_args = vars(_parser.parse_args())
    
            _outputs = sum_and_print_numbers(**_parsed_args)
          image: python:3.7
        inputs:
          parameters:
          - {name: print-and-return-number-2-Output}
          - {name: print-and-return-number-Output}
        metadata:
          labels:
            pipelines.kubeflow.org/kfp_sdk_version: 1.8.9
            pipelines.kubeflow.org/pipeline-sdk-type: kfp
            pipelines.kubeflow.org/enable_caching: "true"
          annotations: {pipelines.kubeflow.org/component_spec: '{"implementation": {"container":
              {"args": ["--number-1", {"inputValue": "number_1"}, "--number-2", {"inputValue":
              "number_2"}], "command": ["sh", "-ec", "program_path=$(mktemp)\nprintf \"%s\"
              \"$0\" > \"$program_path\"\npython3 -u \"$program_path\" \"$@\"\n", "def
              sum_and_print_numbers(number_1, number_2):\n    print(number_1 + number_2)\n\nimport
              argparse\n_parser = argparse.ArgumentParser(prog=''Sum and print numbers'',
              description='''')\n_parser.add_argument(\"--number-1\", dest=\"number_1\",
              type=int, required=True, default=argparse.SUPPRESS)\n_parser.add_argument(\"--number-2\",
              dest=\"number_2\", type=int, required=True, default=argparse.SUPPRESS)\n_parsed_args
              = vars(_parser.parse_args())\n\n_outputs = sum_and_print_numbers(**_parsed_args)\n"],
              "image": "python:3.7"}}, "inputs": [{"name": "number_1", "type": "Integer"},
              {"name": "number_2", "type": "Integer"}], "name": "Sum and print numbers"}',
            pipelines.kubeflow.org/component_ref: '{}', pipelines.kubeflow.org/arguments.parameters: '{"number_1":
              "{{inputs.parameters.print-and-return-number-Output}}", "number_2": "{{inputs.parameters.print-and-return-number-2-Output}}"}'}
      arguments:
        parameters:
        - {name: number_1}
        - {name: number_2}
      serviceAccountName: pipeline-runner
    ```
