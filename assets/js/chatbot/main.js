/**
 * GyuPT 챗봇 메인 모듈
 *
 * 이 모듈은 챗봇의 모든 기능을 통합하고 초기화합니다.
 * - UI, 검색, API 모듈 초기화
 * - 이벤트 리스너 설정
 * - 사용자 입력 처리
 * - 각 모듈 간 조율
 *
 * 모든 모듈(config.js, ui.js, search.js, api.js)이
 * 먼저 로드된 후에 이 모듈이 실행되어야 합니다.
 */

const Chatbot = (function () {
  // 초기화
  async function init() {
    // UI 초기화
    if (!ChatbotUI.init()) {
      console.error('챗봇 UI 초기화 실패');
      return;
    }

    // 검색 초기화
    if (!(await ChatbotSearch.init())) {
      ChatbotUI.addMessage('포스트 데이터 로드 실패', 'bot');
      return;
    }

    // 이전 대화 불러오기
    ChatbotUI.loadConversation();

    // 이벤트 리스너 설정
    setupEventListeners();
  }

  // 이벤트 리스너 설정
  function setupEventListeners() {
    document
      .getElementById('chatbot-submit')
      .addEventListener('click', handleUserInput);
    document
      .getElementById('chatbot-input')
      .addEventListener('keypress', (e) => {
        if (e.key === 'Enter') handleUserInput();
      });
    document
      .getElementById('chatbot-toggle')
      .addEventListener('click', ChatbotUI.toggle);
  }

  // 사용자 입력 처리
  async function handleUserInput() {
    const userQuery = ChatbotUI.getInput();
    if (!userQuery) return;

    // 사용자 메시지 표시
    ChatbotUI.addMessage(userQuery, 'user');
    ChatbotUI.clearInput();

    // 로딩 표시
    const loadingId = ChatbotUI.addMessage(
      '<div class="chatbot-loading"></div>',
      'bot'
    );

    try {
      // 서버 호출 (빈 컨텍스트로)
      const response = await ChatbotAPI.getChatResponse(userQuery, '');

      // 로딩 메시지 업데이트
      ChatbotUI.updateMessage(loadingId, '');

      // 스트리밍 효과 시작
      await ChatbotUI.simulateStreaming(
        response.chunks || [response.answer],
        loadingId
      );

      // 서버에서 받은 참고 문서 정보가 있는 경우만 표시
      if (response.sourcePosts && response.sourcePosts.length > 0) {
        ChatbotUI.addSourceLinks(response.sourcePosts, loadingId);
      }
    } catch (error) {
      console.error('답변 생성 중 오류:', error);
      ChatbotUI.updateMessage(
        loadingId,
        '죄송합니다, 응답 생성 중 오류가 발생했습니다.'
      );
    }
  }

  // 인터페이스 노출
  return {
    init,
  };
})();

// 페이지 로드 시 챗봇 초기화
document.addEventListener('DOMContentLoaded', Chatbot.init);
