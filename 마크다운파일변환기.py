#!/usr/bin/env python3

import os
import re

def clean_markdown_filenames(root_dir):
    """_posts 디렉토리 내의 모든 md 파일 이름에서 하이퍼링크에 사용할 수 없는 문자 제거"""
    processed_count = 0
    renamed_count = 0

    print(f"'{root_dir}' 디렉토리 내 마크다운 파일명 처리 중...")

    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith('.md'):
                file_path = os.path.join(root, file)
                processed_count += 1

                # 파일명에서 특수 문자 제거
                new_filename = re.sub(r'[\[\]\*]', '', file)

                # 파일명이 변경되었다면 파일 이름 변경
                if new_filename != file:
                    new_path = os.path.join(root, new_filename)
                    try:
                        os.rename(file_path, new_path)
                        renamed_count += 1
                        print(f"이름 변경: {file} -> {new_filename}")
                    except Exception as e:
                        print(f"파일 이름 변경 오류 ({file_path}): {e}")

    return processed_count, renamed_count

if __name__ == "__main__":
    posts_dir = "_posts"

    if not os.path.isdir(posts_dir):
        print(f"오류: '{posts_dir}' 디렉토리를 찾을 수 없습니다")
    else:
        total_processed, total_renamed = clean_markdown_filenames(posts_dir)
        print(f"\n처리 완료:")
        print(f"- 총 처리된 파일: {total_processed}개")
        print(f"- 이름이 변경된 파일: {total_renamed}개")
