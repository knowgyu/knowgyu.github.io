import os
import json
import re
from datetime import datetime
import yaml

def extract_frontmatter(content):
    """마크다운 파일에서 frontmatter와 콘텐츠 추출"""
    pattern = r'^---\s*$(.*?)^---\s*$(.*)'
    match = re.search(pattern, content, re.MULTILINE | re.DOTALL)

    if not match:
        return {}, content

    frontmatter_text = match.group(1)
    content_text = match.group(2)

    # YAML 파싱 시도
    try:
        frontmatter = yaml.safe_load(frontmatter_text)
        if frontmatter is None:
            frontmatter = {}
    except Exception as e:
        print(f"YAML 파싱 오류: {e}")
        # 수동 파싱 시도
        frontmatter = {}
        for line in frontmatter_text.strip().split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                frontmatter[key.strip()] = value.strip()

    return frontmatter, content_text.strip()

def process_post_file(file_path, category_from_path=''):
    """포스트 파일 처리"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        frontmatter, post_content = extract_frontmatter(content)

        filename = os.path.basename(file_path)
        post_id = os.path.splitext(filename)[0]

        # 파일명에서 날짜 추출 (YYYY-MM-DD 형식)
        date_match = re.match(r'(\d{4}-\d{2}-\d{2})', post_id)
        date_from_filename = date_match.group(1) if date_match else None

        # 프론트매터나 파일명에서 날짜 처리
        date_str = ''
        if 'date' in frontmatter:
            try:
                # 다양한 날짜 형식 처리 시도
                date_formats = [
                    '%Y-%m-%d %H:%M:%S %z',  # 2025-02-12 13:46:04 +0900
                    '%Y-%m-%d',              # 2025-02-12
                    '%Y-%m-%d %H:%M:%S'      # 2025-02-12 13:46:04
                ]

                for fmt in date_formats:
                    try:
                        date_obj = datetime.strptime(str(frontmatter['date']), fmt)
                        date_str = date_obj.isoformat()
                        break
                    except ValueError:
                        continue
            except Exception as e:
                print(f"날짜 형식 오류 ({file_path}): {e}")
                # 파일명에서 날짜 사용
                if date_from_filename:
                    date_str = f"{date_from_filename}T00:00:00"
        elif date_from_filename:
            date_str = f"{date_from_filename}T00:00:00"

        # 카테고리 처리
        categories = []
        if 'categories' in frontmatter:
            # 프론트매터의 카테고리 처리
            if isinstance(frontmatter['categories'], list):
                categories = frontmatter['categories']
            elif isinstance(frontmatter['categories'], str):
                # 문자열을 리스트로 변환 (쉼표로 구분된 경우 처리)
                if ',' in frontmatter['categories']:
                    categories = [c.strip() for c in frontmatter['categories'].split(',')]
                else:
                    categories = [frontmatter['categories']]

        # 파일 경로에서 카테고리 추출 (프론트매터에 없는 경우)
        if not categories and category_from_path:
            # 경로를 슬래시로 구분된 카테고리로 변환
            path_categories = category_from_path.replace('_', ' ').split('/')
            categories = path_categories

        # 태그 처리
        tags = []
        if 'tags' in frontmatter:
            if isinstance(frontmatter['tags'], list):
                tags = frontmatter['tags']
            elif isinstance(frontmatter['tags'], str):
                # 문자열을 리스트로 변환
                if ',' in frontmatter['tags']:
                    tags = [t.strip() for t in frontmatter['tags'].split(',')]
                # [tag1, tag2] 형식 처리
                elif frontmatter['tags'].startswith('[') and frontmatter['tags'].endswith(']'):
                    tags_str = frontmatter['tags'][1:-1]
                    tags = [t.strip() for t in tags_str.split(',')]
                else:
                    tags = [frontmatter['tags']]

        # 최종 URL 경로 생성
        # Jekyll 규칙에 따라 날짜와 제목으로 URL 형성
        # 파일명에서 날짜 부분 제거
        slug = re.sub(r'^\d{4}-\d{2}-\d{2}-', '', post_id)
        url_path = f"/{slug}/"

        # 카테고리 경로가 있는 경우 URL에 포함
        if category_from_path:
            # _posts/ 이후의 경로에서 파일명을 제외한 부분
            category_path = '/'.join(category_from_path.split('/'))
            if category_path:
                url_path = f"/{category_path.lower()}/{slug}/"

        # 최종 결과 반환
        return {
            'id': post_id,
            'title': frontmatter.get('title', ''),
            'path': url_path,
            'categories': categories,
            'content': post_content,
            'date': date_str,
            'tags': tags,
            'author': frontmatter.get('author', '')
        }
    except Exception as e:
        print(f"파일 처리 중 오류 ({file_path}): {e}")
        return None

def walk_posts_directory(posts_dir, base_path=''):
    """포스트 디렉토리 재귀적 탐색"""
    all_posts = []

    try:
        for item in os.listdir(posts_dir):
            full_path = os.path.join(posts_dir, item)

            # 숨김 파일 및 디렉토리 건너뛰기
            if item.startswith('.'):
                continue

            # 파일인 경우 처리
            if os.path.isfile(full_path) and item.endswith('.md'):
                category_path = base_path if base_path else ''
                post = process_post_file(full_path, category_path)
                if post:
                    all_posts.append(post)

            # 디렉토리인 경우 재귀 호출
            elif os.path.isdir(full_path):
                # 현재 디렉토리 이름을 카테고리 경로에 추가
                new_base_path = os.path.join(base_path, item) if base_path else item
                sub_posts = walk_posts_directory(full_path, new_base_path)
                all_posts.extend(sub_posts)
    except Exception as e:
        print(f"디렉토리 탐색 중 오류 ({posts_dir}): {e}")

    return all_posts

# 메인 실행 부분
if __name__ == "__main__":
    posts_directory = os.path.join(os.getcwd(), '_posts')
    output_path = os.path.join(os.getcwd(), 'assets/js/chatbot/posts-data.json')

    # 모든 포스트 재귀적으로 처리
    posts = walk_posts_directory(posts_directory)

    # 날짜별로 정렬 (최신 글 먼저)
    posts.sort(key=lambda x: x.get('date', ''), reverse=True)

    # JSON 파일 저장
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(posts, f, ensure_ascii=False, indent=2)

    print(f"성공적으로 {len(posts)}개의 포스트를 처리하여 {output_path}에 저장했습니다.")
