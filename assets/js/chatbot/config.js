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
