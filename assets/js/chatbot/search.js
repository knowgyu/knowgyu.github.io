const ChatbotSearch = (function () {
  // 프라이빗 변수
  let postsData = [];

  // 초기화
  async function init() {
    try {
      const response = await fetch('/assets/js/chatbot/posts-data.json');
      postsData = await response.json();
      return true;
    } catch (error) {
      console.error('포스트 데이터 로드 실패:', error);
      return false;
    }
  }

  // 토큰화 함수
  function tokenizeQuery(query) {
    return query
      .toLowerCase()
      .split(/\s+/)
      .filter((k) => k.length > 1);
  }

  // 관련 포스트 찾기
  function findRelevantPosts(query) {
    const keywords = tokenizeQuery(query);

    const timeRelatedQuery = isTimeRelatedQuery(query);

    // 포스트별 점수 계산
    let posts = postsData
      .map((post) => {
        const contentLower = post.content.toLowerCase();
        const titleLower = post.title.toLowerCase();

        const contentMatches = keywords.filter((k) =>
          contentLower.includes(k)
        ).length;
        const titleMatches = keywords.filter((k) =>
          titleLower.includes(k)
        ).length;

        // 날짜 정보 추출 (가능한 경우)
        const dateInfo = extractDateFromPath(post.path);

        return {
          ...post,
          score: contentMatches + titleMatches * 2,
          date: dateInfo,
        };
      })
      .filter((post) => post.score > 0);

    // 시간 관련 쿼리인 경우 날짜 기반 정렬 우선
    if (timeRelatedQuery) {
      posts.sort((a, b) => {
        // 날짜 정보가 있는 경우 날짜순 정렬 (최신순)
        if (a.date && b.date) return b.date - a.date;
        return b.score - a.score;
      });
    } else {
      // 일반 쿼리는 점수 기반 정렬
      posts.sort((a, b) => b.score - a.score);
    }

    return posts.slice(0, ChatbotConfig.ui.maxResults);
  }

  // 경로에서 날짜 추출
  function extractDateFromPath(path) {
    if (!path) return null;

    // YYYY-MM-DD 형식 찾기
    const dateMatch = path.match(/(\d{4}-\d{2}-\d{2})/);
    if (dateMatch) {
      return new Date(dateMatch[1]);
    }

    return null;
  }

  // 시간 관련 쿼리인지 확인
  function isTimeRelatedQuery(query) {
    const timeKeywords = ['최근', '최신', '요즘', '근래', '새로운', '신규'];
    const queryLower = query.toLowerCase();

    return timeKeywords.some((keyword) => queryLower.includes(keyword));
  }

  // 관련 텍스트 추출
  function extractRelevantText(content, query) {
    const keywords = tokenizeQuery(query);
    const contentLower = content.toLowerCase();

    for (const keyword of keywords) {
      const index = contentLower.indexOf(keyword);
      if (index !== -1) {
        const start = Math.max(0, index - 120);
        const end = Math.min(content.length, index + 120);
        let text = content.substring(start, end);

        // 마크다운 서식 제거
        text = text.replace(/\*\*/g, '').replace(/\*/g, '').replace(/\n/g, ' ');

        if (start > 0) text = '...' + text;
        if (end < content.length) text = text + '...';

        return text;
      }
    }

    return content.substring(0, 200).replace(/\n/g, ' ') + '...';
  }

  // 현재 페이지가 포스트인지 확인
  function isPostPage() {
    return window.location.pathname.includes('/posts/');
  }

  // 현재 페이지 컨텍스트 가져오기
  function getCurrentPageContext() {
    if (!isPostPage()) return null;

    try {
      const postTitle =
        document.querySelector('h1.post-title')?.textContent || '';
      const postContent =
        document.querySelector('.post-content')?.textContent || '';

      if (postTitle && postContent) {
        return {
          title: postTitle,
          content: postContent.trim(),
          path: window.location.pathname,
          isCurrentPage: true,
        };
      }
    } catch (error) {
      console.error('현재 페이지 컨텍스트 추출 실패:', error);
    }

    return null;
  }

  // 인터페이스 노출
  return {
    init,
    findRelevantPosts,
    extractRelevantText,
    getCurrentPageContext,
    isPostPage,
  };
})();

window.ChatbotSearch = ChatbotSearch;
