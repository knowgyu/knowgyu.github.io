/**
 * GyuPT 챗봇 검색 모듈 (폴백용)
 *
 * 이 모듈은 블로그 포스트 데이터를 로드합니다.
 * 현재 페이지 컨텍스트 추출 및 폴백 검색 기능은 제거되었습니다.
 *
 * 사용 예시:
 * await ChatbotSearch.init(); // 검색 모듈 초기화
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

  // 인터페이스 노출 (최소화)
  return {
    init,
  };
})();

window.ChatbotSearch = ChatbotSearch;
