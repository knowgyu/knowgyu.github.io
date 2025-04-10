/**
 * GyuPT 챗봇 설정 파일
 *
 * 이 파일은 챗봇의 모든 주요 설정값을 관리합니다.
 * - API 엔드포인트 URL (로컬 및 프로덕션)
 * - UI 관련 설정 (타이핑 속도, 검색 결과 수 등)
 * - 스토리지 키 설정
 *
 * 챗봇의 동작을 변경하려면 이 파일의 값을 수정하세요.
 */

const ChatbotConfig = {
  api: {
    localUrl: 'http://localhost:8888/.netlify/functions/chat',
    productionUrl:
      'https://knowgyu-ai-functions.netlify.app/.netlify/functions/chat',
  },
  ui: {
    maxResults: 3,
    typingSpeed: { min: 10, max: 10 },
    chunkDelay: 100,
  },
  storage: {
    keys: {
      history: 'chatHistory',
      collapsed: 'chatbotCollapsed',
    },
  },
};

// 모듈로 내보내기
window.ChatbotConfig = ChatbotConfig;
