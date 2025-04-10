/**
 * GyuPT 챗봇 검색 모듈 (폴백용)
 *
 * 이 모듈은 서버 RAG 시스템이 실패할 경우를 대비한 폴백 검색과
 * 현재 페이지 컨텍스트 추출 기능을 담당합니다.
 * - 블로그 포스트 데이터 로드
 * - 현재 페이지 컨텍스트 추출
 *
 * 사용 예시:
 * await ChatbotSearch.init(); // 검색 모듈 초기화
 * const currentPageContext = ChatbotSearch.getCurrentPageContext(); // 현재 페이지 컨텍스트 가져오기
 */

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

  // 간단한 폴백 검색 (서버 RAG가 실패했을 때만 사용)
  function findRelevantPosts(query) {
    const keywords = query
      .toLowerCase()
      .split(/\s+/)
      .filter((k) => k.length > 1);

    // 포스트별 점수 계산 (간소화됨)
    let posts = postsData
      .map((post) => {
        const contentLower = post.content.toLowerCase();
        const titleLower = post.title.toLowerCase();

        // 단순 키워드 매칭
        const score = keywords.filter(
          (k) => contentLower.includes(k) || titleLower.includes(k)
        ).length;

        return {
          ...post,
          score,
        };
      })
      .filter((post) => post.score > 0)
      .sort((a, b) => b.score - a.score);

    return posts.slice(0, ChatbotConfig.ui.maxResults);
  }

  // 관련 텍스트 추출 (폴백용)
  function extractRelevantText(content, query) {
    if (!content) return '';

    // 단순화된 컨텍스트 추출
    const previewLength = 200;
    return content.substring(0, previewLength).replace(/\n/g, ' ') + '...';
  }

  // 현재 페이지가 포스트인지 확인
  function isPostPage() {
    return window.location.pathname.includes('/posts/');
  }

  // 현재 페이지 컨텍스트 가져오기 (계속 필요함)
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
