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
        '최근에 정리한 ROS 관련 글을 추천해줘',
        'Kubeflow 관련 글 흐름을 처음부터 읽는 순서로 알려줘',
      ],
    },
    copy: {
      welcome: '안녕하세요. 블로그 글을 바탕으로 답변해 드립니다.',
      ready: '블로그 내용을 바탕으로 답변합니다.',
      postReady: '현재 글 기준으로도 답변합니다.',
      loading: '답변을 정리하고 있습니다.',
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
