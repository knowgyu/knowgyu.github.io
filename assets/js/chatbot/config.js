(() => {
  const namespace = (window.Gyupt = window.Gyupt || {});

  namespace.config = {
    api: {
      localUrl: 'http://localhost:8888/.netlify/functions/chat',
      productionUrl:
        'https://knowgyu-ai-functions.netlify.app/.netlify/functions/chat',
      queryParam: 'chatbotApi',
      localModeValue: 'local',
      requestTimeoutMs: 20000,
    },
    ui: {
      initialCollapsed: true,
      maxStoredMessages: 24,
      maxMessagesForRequest: 10,
      maxConversationCharactersForRequest: 3600,
      maxDraftLength: 1200,
      postSuggestionPrompts: [
        '현재 글 요약해주세요.',
        '현재 글과 관련된 다른 포스트들 알려주세요.',
      ],
      suggestionPrompts: [
        'ROS 글을 읽기 좋은 순서로 정리해줘',
        'MLOps와 Kubeflow 기록을 연결해서 보여줘',
      ],
    },
    copy: {
      welcome: '안녕하세요. 블로그에 남긴 기술 기록을 근거로 답변합니다.',
      ready: '블로그 내용을 근거로 답변합니다.',
      postReady: '현재 글 맥락까지 함께 봅니다.',
      loading: '관련 기록을 확인하고 있습니다.',
      copied: '답변을 복사했습니다.',
      reset: '새 대화를 시작했습니다.',
      unavailable:
        '현재 챗봇 서버에 연결할 수 없습니다. Netlify Functions를 다시 켠 뒤 시도해 주세요.',
    },
    storage: {
      collapsedKey: 'gyupt:collapsed',
      conversationKey: 'gyupt:conversation',
    },
    selectors: {
      container: '#chatbot-container',
      toggle: '#chatbot-toggle',
      status: '#chatbot-status',
      body: '#chatbot-body',
      messages: '#chatbot-messages',
      suggestions: '#chatbot-suggestions',
      form: '#chatbot-form',
      input: '#chatbot-input',
      submit: '#chatbot-submit',
      reset: '#chatbot-reset',
    },
  };
})();
