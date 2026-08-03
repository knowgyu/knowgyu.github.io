#!/usr/bin/env python3
"""Report the current content drift from the approved Tistory shell contract."""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
POSTS = ROOT / "_posts"
SIDEBAR = ROOT / "_includes" / "sidebar.html"
HOME = ROOT / "_layouts" / "home.html"
POSTS_TAB = ROOT / "_tabs" / "posts.md"
CATEGORY_LAYOUT = ROOT / "_layouts" / "category.html"
TAXONOMY = ROOT / "_data" / "taxonomy.yml"
POST_TAIL = ROOT / "_includes" / "post-tail.html"
POST_ROW = ROOT / "_includes" / "post-row.html"
POST_LAYOUT = ROOT / "_layouts" / "post.html"
SITE = ROOT / "_site"
EMOJI = re.compile(
    "[\U0001f300-\U0001faff\U00002700-\U000027bf\U00002600-\U000026ff]"
)
ENDING_PUNCTUATION = re.compile(r"[.!?,;:。！？、]+$")
PLACEHOLDERS = {"", " ", "...", "todo", "tbd", "placeholder"}
PRIVATE_POST = POSTS / "얘이제내려2000-11-30-First-Posting.md"
PRIVATE_TOKENS = ("얘이제내려", "First-Posting", "안녕하세요 !")


def parse_front_matter(path: Path) -> dict[str, str]:
    text = path.read_text(encoding="utf-8")
    if not text.startswith("---\n"):
        return {}
    end = text.find("\n---", 4)
    if end == -1:
        return {}
    values: dict[str, str] = {}
    for line in text[4:end].splitlines():
        if ":" not in line or line.lstrip().startswith("#"):
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def scalar(value: str | None) -> str:
    if value is None:
        return ""
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        return value[1:-1]
    return value


def inline_list(value: str | None) -> list[str]:
    value = scalar(value)
    if not value:
        return []
    if value.startswith("[") and value.endswith("]"):
        value = value[1:-1]
    return [item.strip().strip("\"'") for item in value.split(",") if item.strip()]


def fallback_description(text: str, categories: list[str]) -> str:
    if "Hybrid" in text:
        return "ROS 하이브리드 경로 계획"
    if "Docker" in text:
        return "Docker 설치 기록"
    if "MLflow" in text:
        return "MLflow 서버 설정"
    if "Prometheus" in text or "Grafana" in text:
        return "Prometheus Grafana 설정"
    if "Data-Analysis" in text:
        return "데이터 분석 시각화"
    if "Evaluate" in text:
        return "테스트 데이터셋 평가"
    if "Change-password" in text or "Add-User" in text:
        return "Kubeflow 계정 설정"
    if "PV-Mount" in text or "HyperParameter" in text:
        return "PV 마운트와 튜닝"
    if "YOLOv8" in text:
        return "YOLOv8 학습 파이프라인"
    if "float" in text or "fuse" in text or "eval" in text:
        return "PyTorch 함수 정리"
    for category in reversed(categories):
        if re.search(r"[가-힣]", category):
            return clean_description(f"{category} 기록", [])
    return "기술 학습 기록"


def clean_description(text: str, categories: list[str]) -> str:
    text = re.sub(r"^BOJ[-\s]+(\d+)번?[-\s]+(.+)$", r"BOJ \1 \2 풀이", text)
    text = re.sub(r"^Setup[-\s]+", "", text)
    text = text.replace("State_dict란", "state_dict 정리")
    text = text.replace("float(),-fuse(),-eval()", "float fuse eval 정리")
    text = text.replace("Ubuntu-22.04-(nvidia)-Docker-설치하기", "Ubuntu Docker 설치")
    text = text.replace("Ubuntu-22.04-딥러닝-환경-구축", "Ubuntu 딥러닝 환경 구축")
    text = text.replace("Error-Docker-CLI-Context-에러", "Docker CLI Context 오류")
    text = text.replace("Pipeline-for-Train-YOLOv8-on-a-Custom-Dataset", "YOLOv8 학습 파이프라인")
    text = text.replace("Evaluate-model-with-TEST-dataset", "테스트 데이터셋 평가")
    text = text.replace("PV-Mount-&-HyperParameter-Tuning", "PV 마운트와 튜닝")
    text = text.replace("Data-Analysis-Component-+-Visualization", "데이터 분석 컴포넌트")
    text = text.replace("Misc-Kubeflow-Change-password-&-Add-User", "Kubeflow 계정 설정")
    text = text.replace("Misc-MetalLB-Settings", "MetalLB 설정")
    text = text.replace("Misc.-Pipeline-설명", "파이프라인 설명")
    text = text.replace("Component-Environment", "컴포넌트 환경 구성")
    text = text.replace("Component-Write", "컴포넌트 작성")
    text = text.replace("Pipeline-Run", "파이프라인 실행")
    text = text.replace("Pipeline-Upload", "파이프라인 업로드")
    text = text.replace("Pipeline-Write", "파이프라인 작성")
    text = text.replace("Yolov8-학습pp", "YOLOv8 학습 ")
    text = text.replace("-", " ")
    text = re.sub(r"[\"'`]", "", text)
    text = re.sub(r"\s+", " ", text).strip()
    text = ENDING_PUNCTUATION.sub("", text).strip()
    if not re.search(r"[가-힣]", text):
        text = fallback_description(text, categories)
    text = text[:28].rstrip()
    if not re.search(r"[가-힣]", text):
        text = fallback_description(text, categories)
    return text[:28].rstrip()


def title_from_front_matter(fm: dict[str, str], post: Path) -> str:
    title = scalar(fm.get("title")).strip()
    if title:
        return title
    return re.sub(r"^\d{4}-\d{2}-\d{2}-", "", post.stem)


def normalize_post(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    if not text.startswith("---\n"):
        return False
    end = text.find("\n---", 4)
    if end == -1:
        return False

    fm_lines = text[4:end].splitlines()
    fm = parse_front_matter(path)
    rel_body = text[end:]
    changed = False

    if path == PRIVATE_POST:
        if scalar(fm.get("published")).lower() != "false":
            insert_at = next((i + 1 for i, line in enumerate(fm_lines) if line.startswith("date:")), len(fm_lines))
            fm_lines.insert(insert_at, "published: false")
            changed = True
    else:
        description = scalar(fm.get("description")).strip()
        generated = clean_description(title_from_front_matter(fm, path), inline_list(fm.get("categories")))
        needs_description = (
            description.lower() in PLACEHOLDERS
            or len(description) > 28
            or bool(ENDING_PUNCTUATION.search(description))
            or bool(EMOJI.search(description))
            or not re.search(r"[가-힣]", description)
        )
        if needs_description:
            for i, line in enumerate(fm_lines):
                if line.startswith("description:"):
                    fm_lines[i] = f'description: "{generated}"'
                    changed = True
                    break

    if path.name == "2024-12-05-당구공-경로-생성-프로젝트.md":
        for i, line in enumerate(fm_lines):
            if line.startswith("categories:") and "Project" in line:
                fm_lines[i] = "categories: [AI & CV, Computer Vision]"
                changed = True
            if line.startswith("tags:") and "Project" not in line:
                fm_lines[i] = line.rstrip("]") + ", Project]"
                changed = True

    if changed:
        path.write_text("---\n" + "\n".join(fm_lines) + rel_body, encoding="utf-8")
    return changed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--strict", action="store_true", help="fail when drift remains")
    parser.add_argument("--fix", action="store_true", help="normalize front matter in place")
    parser.add_argument("--max-description", type=int, default=28)
    args = parser.parse_args()

    posts = sorted(POSTS.rglob("*.md"))
    if args.fix:
        changed = sum(1 for post in posts if normalize_post(post))
        print(f"content baseline fix: changed={changed}")

    findings: list[str] = []
    published = 0
    unpublished = 0

    for post in posts:
        fm = parse_front_matter(post)
        rel = post.relative_to(ROOT)
        is_published = scalar(fm.get("published")).lower() != "false"
        if is_published:
            published += 1
        else:
            unpublished += 1
            continue

        description = scalar(fm.get("description")).strip()
        categories = inline_list(fm.get("categories"))

        if description.lower() in PLACEHOLDERS:
            findings.append(f"{rel}: blank or placeholder description")
        if description and len(description) > args.max_description:
            findings.append(f"{rel}: description over {args.max_description} chars")
        if description and ENDING_PUNCTUATION.search(description):
            findings.append(f"{rel}: description ends with punctuation")
        if description and EMOJI.search(description):
            findings.append(f"{rel}: description contains emoji")
        if description and not re.search(r"[가-힣]", description):
            findings.append(f"{rel}: description lacks Korean wording")
        if not categories:
            findings.append(f"{rel}: missing public categories")
        if len(categories) > 2:
            findings.append(f"{rel}: category depth {len(categories)} exceeds 2")

    structural_errors: list[str] = []
    sidebar = SIDEBAR.read_text(encoding="utf-8") if SIDEBAR.exists() else ""
    home = HOME.read_text(encoding="utf-8") if HOME.exists() else ""
    posts_tab = POSTS_TAB.read_text(encoding="utf-8") if POSTS_TAB.exists() else ""
    category_layout = CATEGORY_LAYOUT.read_text(encoding="utf-8") if CATEGORY_LAYOUT.exists() else ""
    taxonomy = TAXONOMY.read_text(encoding="utf-8") if TAXONOMY.exists() else ""
    post_tail = POST_TAIL.read_text(encoding="utf-8") if POST_TAIL.exists() else ""
    post_row = POST_ROW.read_text(encoding="utf-8") if POST_ROW.exists() else ""
    post_layout = POST_LAYOUT.read_text(encoding="utf-8") if POST_LAYOUT.exists() else ""

    for label, path, text in (
        ("sidebar", SIDEBAR, sidebar),
        ("home", HOME, home),
        ("posts tab", POSTS_TAB, posts_tab),
        ("category layout", CATEGORY_LAYOUT, category_layout),
        ("taxonomy", TAXONOMY, taxonomy),
        ("post tail", POST_TAIL, post_tail),
        ("post row", POST_ROW, post_row),
        ("post layout", POST_LAYOUT, post_layout),
    ):
        if not text:
            structural_errors.append(f"{label}: missing {path.relative_to(ROOT)}")

    if "search-input" in sidebar or "search-trigger" in sidebar:
        structural_errors.append("sidebar: duplicates topbar search runtime")
    if "site.data.taxonomy" not in sidebar or "taxonomy-tree" not in sidebar:
        structural_errors.append("sidebar: taxonomy disclosure tree missing")
    if "taxonomy-toggle" not in sidebar or "aria-expanded" not in sidebar:
        structural_errors.append("sidebar: accessible taxonomy buttons missing")
    if "sidebar-collapse-toggle" not in sidebar or "knowgyu:sidebar-collapsed" not in sidebar:
        structural_errors.append("sidebar: persisted collapse control missing")
    if "profile-avatar" not in sidebar or "site.avatar" not in sidebar:
        structural_errors.append("sidebar: configured profile avatar missing")
    if "knowgyu:sidebar-scroll" not in sidebar or "restoreScroll" not in sidebar:
        structural_errors.append("sidebar: category navigation scroll persistence missing")
    if "Categories</span>" in sidebar:
        structural_errors.append("sidebar: redundant Categories navigation remains")
    if "taxonomy-icon" not in sidebar or "branch.icon" not in sidebar or "child.icon" not in sidebar:
        structural_errors.append("sidebar: taxonomy icon metadata missing")
    if "aria-expanded=\"true\"" not in sidebar or "{% unless expanded %} hidden{% endunless %}" in sidebar:
        structural_errors.append("sidebar: taxonomy roots must be expanded by default")
    if "전체 글" not in sidebar or "/posts/" not in sidebar:
        structural_errors.append("sidebar: posts route missing")
    if "paginator" in home or "post-paginator" in home:
        structural_errors.append("home: still depends on paginator")
    if "전체 글 보기" not in home or "latest_posts limit: 5" not in home:
        structural_errors.append("home: curated gateway/latest block missing")
    if "주요 카테고리" not in home or "작업 흐름" in home or "featured_categories" not in home:
        structural_errors.append("home: taxonomy-sourced major category paths missing")
    if "permalink:" in posts_tab:
        structural_errors.append("posts tab: should use Chirpy tab default permalink")
    if "data-catalog-item" not in posts_tab or "forloop.index > 15" not in posts_tab:
        structural_errors.append("posts tab: fixed 15-item catalog missing")
    if "| escape" not in posts_tab and "| escape" not in post_row:
        structural_errors.append("posts tab: catalog output must escape titles and categories")
    if "{% include post-row.html" not in posts_tab or "{% include post-row.html" not in category_layout:
        structural_errors.append("post rows: posts tab and category layout must share post-row include")
    if "data-post-row" not in post_row or "data-post-title" not in post_row or "data-post-date" not in post_row:
        structural_errors.append("post row: semantic row fields missing")
    if "post-row-excerpt" not in post_row or "post-description.html" not in post_row:
        structural_errors.append("post row: body preview missing")
    if 'class="dash' in category_layout or "<ul class=\"content" in category_layout:
        structural_errors.append("category layout: dashed category list grammar remains")
    if "15/30" in posts_tab or "density" in posts_tab.lower():
        structural_errors.append("posts tab: density selector leaked in")
    if "좁게" not in post_layout or "기본" not in post_layout or "넓게" not in post_layout:
        structural_errors.append("post layout: readable width labels missing")
    if ">760</button>" in post_layout or ">900</button>" in post_layout or ">1100</button>" in post_layout:
        structural_errors.append("post layout: raw width labels still visible")
    if "Embedded System" not in taxonomy or "AI & CV" not in taxonomy or "Computer Science" not in taxonomy:
        structural_errors.append("taxonomy: top-level owners missing")
    if "icon:" not in taxonomy:
        structural_errors.append("taxonomy: Font Awesome icon metadata missing")
    if (
        "post-tail-section--sequence" not in post_tail
        or "post-tail-section--latest" not in post_tail
        or "latest_count == 5" not in post_tail
    ):
        structural_errors.append("post tail: sequence/latest two-section shell missing")

    if SITE.exists():
        feed = SITE / "feed.xml"
        feed_text = feed.read_text(encoding="utf-8") if feed.exists() else ""
        if not feed_text:
            structural_errors.append("site output: missing _site/feed.xml")
        elif any(token in feed_text for token in PRIVATE_TOKENS):
            structural_errors.append("site output: private post leaked into feed.xml")

        posts_index = SITE / "posts" / "index.html"
        posts_index_text = posts_index.read_text(encoding="utf-8") if posts_index.exists() else ""
        if posts_index_text:
            if 'catalog-item' not in posts_index_text:
                structural_errors.append("site output: posts catalog items rendered as escaped markdown")
            if 'class="language-plaintext highlighter-rouge"' in posts_index_text:
                structural_errors.append("site output: posts catalog rendered as a code block")
            if "data-post-row" not in posts_index_text:
                structural_errors.append("site output: shared posts row marker missing")
        else:
            structural_errors.append("site output: missing _site/posts/index.html")

        category_ros = SITE / "categories" / "ros" / "index.html"
        category_ros_text = category_ros.read_text(encoding="utf-8") if category_ros.exists() else ""
        if category_ros_text and "data-post-row" not in category_ros_text:
            structural_errors.append("site output: category detail shared row marker missing")

        post_pages = sorted((SITE / "posts").glob("*/index.html")) if (SITE / "posts").exists() else []
        sample_post = next((path for path in post_pages if path.parent.name != "index"), None)
        sample_text = sample_post.read_text(encoding="utf-8") if sample_post else ""
        if not sample_text:
            structural_errors.append("site output: missing built sample post")
        elif (
            sample_text.count("post-tail-section") < 2
            or "post-tail-section--sequence" not in sample_text
            or "post-tail-section--latest" not in sample_text
        ):
            structural_errors.append("site output: post tail two-section shell missing")

    print(f"content baseline: posts={len(posts)} published={published} unpublished={unpublished}")
    if structural_errors:
        print(f"tistory structure findings: {len(structural_errors)}")
        for finding in structural_errors:
            print(f"- {finding}")
        return 1
    if findings:
        print(f"content baseline findings: {len(findings)}")
        for finding in findings[:40]:
            print(f"- {finding}")
        if len(findings) > 40:
            print(f"- ... {len(findings) - 40} more")
        return 1 if args.strict else 0

    print("content baseline: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
