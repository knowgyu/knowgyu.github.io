import os
import json
import re
import time
import gc
from datetime import datetime
from pathlib import Path
import yaml
import openai
from dotenv import load_dotenv

# 환경 변수 로드
load_dotenv()
openai.api_key = os.getenv('OPENAI_API_KEY')

def extract_frontmatter(content):
    """마크다운 파일에서 YAML 프론트매터 추출"""
    pattern = r'^---\s*\n(.*?)\n---\s*\n'
    match = re.search(pattern, content, re.DOTALL)
    if not match:
        return {}, content

    frontmatter_text = match.group(1)
    rest_content = content[match.end():]

    try:
        frontmatter = yaml.safe_load(frontmatter_text)
        return frontmatter, rest_content
    except Exception as e:
        print(f"YAML 파싱 오류: {e}")
        return {}, content

def chunk_text(text, chunk_size=800, overlap=100, max_chunks=5):
    """텍스트를 일정 크기의 청크로 나눔"""
    chunks = []
    start = 0

    # 텍스트가 너무 짧으면 그대로 반환
    if len(text) <= chunk_size:
        return [text]

    while start < len(text) and len(chunks) < max_chunks:
        end = min(start + chunk_size, len(text))

        # 문장 경계 찾기
        if end < len(text):
            sentence_end = max(
                text.rfind('. ', start, end),
                text.rfind('? ', start, end),
                text.rfind('! ', start, end)
            )

            if sentence_end > start:
                end = sentence_end + 2

        chunks.append(text[start:end].strip())
        start = end - overlap

    return chunks

def create_embedding(text):
    """OpenAI API를 사용해 텍스트 임베딩 생성"""
    if not text.strip():
        return None

    try:
        # 텍스트가 너무 길면 잘라내기 (OpenAI 제한)
        if len(text) > 8000:
            print(f"텍스트 길이 초과 ({len(text)}자), 8000자로 잘라냅니다.")
            text = text[:8000]

        response = openai.embeddings.create(  # 수정: openai.Embedding -> openai.embeddings
            input=[text],  # input은 리스트로 전달
            model="text-embedding-3-large"
        )
        return response.data[0].embedding
    except Exception as e:
        print(f"임베딩 생성 오류: {e}")
        return None

def process_markdown_file(md_file_path):
    # 파일 내용 읽기
    with open(md_file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    frontmatter, body = extract_frontmatter(content)

    # 프론트매터의 date가 있으면 사용, 없으면 파일 생성 시간 사용
    fm_date = frontmatter.get('date', None)
    if fm_date:
        post_date = fm_date  # 프론트매터에 정의된 date
    else:
        # 파일 생성 시각으로 대체
        timestamp = os.path.getmtime(md_file_path)
        post_date = datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')

    post_data = {
        "title": frontmatter.get("title", os.path.splitext(os.path.basename(md_file_path))[0]),
        "date": post_date,
        "categories": frontmatter.get("categories", []),
        "tags": frontmatter.get("tags", []),
        "content": body,
        "path": str(md_file_path),
    }

    return post_data

def process_post_file(file_path, category_from_path=''):
    """포스트 파일 처리 및 청크 생성"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # 프론트매터와 본문 분리
        frontmatter, post_content = extract_frontmatter(content)

        # 파일명에서 날짜와 제목 추출 (yyyy-mm-dd-title.md)
        file_name = os.path.basename(file_path)
        match = re.match(r'(\d{4}-\d{2}-\d{2})-(.*?)\.md', file_name)

        if match:
            date_str = match.group(1)
            title_from_filename = match.group(2).replace('-', ' ')
        else:
            date_str = frontmatter.get('date', datetime.now().strftime('%Y-%m-%d'))
            title_from_filename = os.path.splitext(file_name)[0]

        # 카테고리 처리 (프론트매터와 경로 모두 고려)
        categories = frontmatter.get('categories', [])
        if isinstance(categories, str):
            categories = [categories]

        # 경로 기반 카테고리 추가
        if category_from_path and category_from_path not in categories:
            categories.append(category_from_path)

        # 태그 처리
        tags = frontmatter.get('tags', [])
        if isinstance(tags, str):
            tags = [tags]

        # 포스트 URL 경로 생성
        url_path = frontmatter.get('permalink', f"/{date_str.replace('-', '/')}/{title_from_filename.replace(' ', '-')}/")

        # 포스트 ID 생성
        post_id = re.sub(r'[^a-zA-Z0-9]', '-', url_path).strip('-')

        # 결과 객체 생성
        result = {
            'id': post_id,
            'title': frontmatter.get('title', title_from_filename),
            'path': url_path,
            'categories': categories,
            'content': post_content,
            'date': date_str,
            'tags': tags,
            'author': frontmatter.get('author', '')
        }

        # 청크 생성 (제목 + 내용에서)
        full_text = f"{result['title']}\n\n{post_content}"
        chunks = chunk_text(full_text)

        # 청크 정보 저장
        result['chunks'] = []
        for i, chunk_content in enumerate(chunks):
            result['chunks'].append({
                'id': f"{post_id}_chunk_{i}",
                'text': chunk_content,
                'index': i
            })

        return result
    except Exception as e:
        print(f"파일 처리 중 오류 ({file_path}): {e}")
        return None

def walk_posts_directory(posts_dir, base_path=''):
    """포스트 디렉토리 재귀적 탐색"""
    print(f"디렉토리 탐색 중: {posts_dir}")
    all_posts = []

    for item in os.listdir(posts_dir):
        path = os.path.join(posts_dir, item)

        if os.path.isdir(path):
            # 디렉토리인 경우 재귀 탐색
            # 디렉토리명을 카테고리로 사용
            category = item if not base_path else f"{base_path}/{item}"
            sub_posts = walk_posts_directory(path, category)
            all_posts.extend(sub_posts)
        elif item.endswith('.md'):
            # 마크다운 파일인 경우 처리
            post_data = process_post_file(path, base_path)
            if post_data:
                all_posts.append(post_data)

    return all_posts

def save_embedding_batch(embeddings, batch_id, output_dir):
    """임베딩 데이터를 분할 저장"""
    os.makedirs(output_dir, exist_ok=True)
    batch_path = os.path.join(output_dir, f"embeddings_batch_{batch_id}.json")
    with open(batch_path, 'w', encoding='utf-8') as f:
        json.dump(embeddings, f, ensure_ascii=False)
    print(f"임베딩 배치 {batch_id} 저장 완료 ({len(embeddings)}개)")
    return batch_path

if __name__ == "__main__":
    print("RAG를 위한 포스트 데이터 및 임베딩 생성 시작...")

    # API 키 확인
    if not openai.api_key:
        print("OpenAI API 키가 설정되지 않았습니다. .env 파일을 확인하세요.")
        exit(1)
    else:
        print("OpenAI API 키가 설정되었습니다.")

    # 디렉토리 및 파일 경로 설정
    posts_directory = os.path.join(os.getcwd(), '_posts')
    output_path = os.path.join(os.getcwd(), 'assets/js/chatbot/posts-data.json')
    embedding_dir = os.path.join(os.getcwd(), 'assets/js/chatbot/embeddings')
    index_path = os.path.join(embedding_dir, 'index.json')

    # 디렉토리 생성
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    os.makedirs(embedding_dir, exist_ok=True)

    # 포스트 디렉토리 확인
    if not os.path.exists(posts_directory):
        print(f"포스트 디렉토리를 찾을 수 없습니다: {posts_directory}")
        exit(1)
    else:
        print(f"포스트 디렉토리 확인됨: {posts_directory}")

    # 모든 포스트 처리
    print("포스트 파일 수집 및 처리 중...")
    posts = walk_posts_directory(posts_directory)

    # 날짜순 정렬 (최신 포스트 우선)
    posts.sort(key=lambda x: x.get('date', ''), reverse=True)

    # 너무 긴 콘텐츠 자르기
    for post in posts:
        if len(post['content']) > 20000:  # 약 20KB로 제한
            post['content'] = post['content'][:20000] + "..."

    print(f"{len(posts)}개 포스트 처리 완료, 저장 중...")
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(posts, f, ensure_ascii=False, indent=2)

    # 배치 처리를 위한 준비
    batch_size = 50  # 한 번에 처리할 청크 수
    batch_id = 0
    current_batch = {}
    processed_chunks = set()
    embedding_batches = []

    # 기존 인덱스 파일 로드 (있으면)
    if os.path.exists(index_path):
        try:
            with open(index_path, 'r', encoding='utf-8') as f:
                index_data = json.load(f)
                embedding_batches = index_data.get('batches', [])
                processed_chunks = set(index_data.get('processed_chunks', []))
                print(f"인덱스 로드: {len(processed_chunks)}개 청크 이미 처리됨")
        except Exception as e:
            print(f"인덱스 로드 실패: {e}")

    # 포스트를 배치로 나눠 처리
    total_chunks = sum(len(post.get('chunks', [])) for post in posts)
    processed_count = 0

    print(f"임베딩 생성 시작: 총 {total_chunks}개 청크 처리 예정")

    for post in posts:
        if 'chunks' not in post:
            continue

        for chunk in post['chunks']:
            chunk_id = chunk['id']

            # 이미 처리한 청크는 건너뛰기
            if chunk_id in processed_chunks:
                continue

            # 텍스트가 너무 길면 잘라내기
            if len(chunk['text']) > 8000:
                chunk['text'] = chunk['text'][:8000]

            print(f"임베딩 생성 중: {chunk_id} ({processed_count+1}/{total_chunks})")

            try:
                # API 요청 제한 방지
                if processed_count > 0 and processed_count % 5 == 0:
                    print("API 요청 제한 방지 위해 대기...")
                    time.sleep(2)

                embedding = create_embedding(chunk['text'])
                if embedding:
                    current_batch[chunk_id] = embedding
                    processed_chunks.add(chunk_id)
                    processed_count += 1

                    # 배치가 다 차면 저장하고 메모리 비우기
                    if len(current_batch) >= batch_size:
                        batch_path = save_embedding_batch(current_batch, batch_id, embedding_dir)
                        embedding_batches.append({
                            'id': batch_id,
                            'path': os.path.basename(batch_path),
                            'chunks': list(current_batch.keys())
                        })

                        # 인덱스 업데이트
                        with open(index_path, 'w', encoding='utf-8') as f:
                            json.dump({
                                'batches': embedding_batches,
                                'processed_chunks': list(processed_chunks),
                                'last_updated': datetime.now().isoformat()
                            }, f, ensure_ascii=False, indent=2)

                        # 메모리 비우기
                        current_batch = {}
                        gc.collect()  # 가비지 컬렉션 강제 실행
                        batch_id += 1
                        print(f"메모리 정리 및 배치 {batch_id-1} 저장 완료")
            except Exception as e:
                print(f"청크 {chunk_id} 임베딩 실패: {e}")
                continue

    # 남은 배치 저장
    if current_batch:
        batch_path = save_embedding_batch(current_batch, batch_id, embedding_dir)
        embedding_batches.append({
            'id': batch_id,
            'path': os.path.basename(batch_path),
            'chunks': list(current_batch.keys())
        })

    # 최종 인덱스 업데이트
    with open(index_path, 'w', encoding='utf-8') as f:
        json.dump({
            'batches': embedding_batches,
            'processed_chunks': list(processed_chunks),
            'last_updated': datetime.now().isoformat()
        }, f, ensure_ascii=False, indent=2)

    print(f"처리 완료: {len(posts)}개 포스트, {len(processed_chunks)}개 임베딩")

    # 클라이언트용 인덱스 파일 생성
    index_js_path = os.path.join(os.getcwd(), 'assets/js/chatbot/embeddings-index.js')
    with open(index_js_path, 'w', encoding='utf-8') as f:
        f.write(f"// 자동 생성된 파일 - {datetime.now().isoformat()}\n")
        f.write("const embeddingIndex = ")
        json.dump({
            'batches': embedding_batches,
            'count': len(processed_chunks),
            'last_updated': datetime.now().isoformat()
        }, f, ensure_ascii=False, indent=2)
        f.write(";\n\nexport default embeddingIndex;")
