/**
 * GyuPT 챗봇 UI 모듈
 *
 * 이 모듈은 챗봇의 시각적 요소와 사용자 인터페이스를 관리합니다.
 * - 메시지 추가 및 업데이트
 * - 챗봇 토글 (접기/펼치기) 기능
 * - 타이핑 효과 시뮬레이션
 * - 대화 내역 저장 및 로드
 *
 * 사용 예시:
 * ChatbotUI.init(); // UI 초기화
 * const messageId = ChatbotUI.addMessage('안녕하세요', 'bot'); // 메시지 추가
 * ChatbotUI.toggle(); // 챗봇 접기/펼치기
 */

const ChatbotUI = (function () {
  // 프라이빗 변수
  let container;
  let messagesContainer;
  let input;
  let submitButton;
  let toggleButton;

  // 초기화
  function init() {
    container = document.getElementById('chatbot-container');
    messagesContainer = document.getElementById('chatbot-messages');
    input = document.getElementById('chatbot-input');
    submitButton = document.getElementById('chatbot-submit');
    toggleButton = document.getElementById('chatbot-toggle');

    if (!container) {
      console.error('챗봇 컨테이너를 찾을 수 없습니다.');
      return false;
    }

    // 초기 UI 상태 설정
    initState();

    return true;
  }

  // 상태 초기화
  function initState() {
    const isCollapsed = sessionStorage.getItem(
      ChatbotConfig.storage.keys.collapsed
    );

    if (isCollapsed === null) {
      sessionStorage.setItem(ChatbotConfig.storage.keys.collapsed, 'false');
    }

    if (isCollapsed === 'true') {
      container.classList.add('collapsed');
    } else {
      container.classList.remove('collapsed');
    }

    container.classList.add('visible');

    if (isCollapsed !== 'true') {
      setTimeout(() => scrollToBottom(), 100);
    }
  }

  // 토글 기능
  function toggle() {
    container.classList.toggle('collapsed');
    const isCollapsed = container.classList.contains('collapsed');
    sessionStorage.setItem(
      ChatbotConfig.storage.keys.collapsed,
      isCollapsed ? 'true' : 'false'
    );

    if (!isCollapsed) {
      scrollToBottom();
    }
  }

  // 메시지 추가
  function addMessage(text, sender) {
    const id = Date.now().toString();
    const messageElement = document.createElement('div');
    messageElement.classList.add('message', sender);
    messageElement.dataset.id = id; // ID를 데이터 속성으로 저장
    messageElement.innerHTML = text;

    messagesContainer.appendChild(messageElement);
    scrollToBottom();
    saveConversation();

    return id;
  }

  // 메시지 업데이트
  function updateMessage(id, text) {
    const messageElement = messagesContainer.querySelector(
      `.message[data-id="${id}"]`
    );
    if (messageElement) {
      messageElement.innerHTML = text;
      scrollToBottom();
      saveConversation();
    }
  }

  // 스트리밍 효과
  async function simulateStreaming(chunks, messageId) {
    const messageElement = messagesContainer.querySelector(
      `.message[data-id="${messageId}"]`
    );
    if (!messageElement) return;

    let currentText = '';

    // 문자열을 배열로 변환
    if (typeof chunks === 'string') {
      chunks = [chunks];
    }

    for (const chunk of chunks) {
      for (let i = 0; i < chunk.length; i++) {
        currentText += chunk[i];
        messageElement.innerHTML = currentText;
        scrollToBottom();

        await new Promise((resolve) =>
          setTimeout(
            resolve,
            Math.floor(
              Math.random() *
                (ChatbotConfig.ui.typingSpeed.max -
                  ChatbotConfig.ui.typingSpeed.min)
            ) + ChatbotConfig.ui.typingSpeed.min
          )
        );
      }

      await new Promise((resolve) =>
        setTimeout(resolve, ChatbotConfig.ui.chunkDelay)
      );

      if (chunk[chunk.length - 1] !== ' ') {
        currentText += ' ';
        messageElement.innerHTML = currentText;
      }
    }

    messageElement.innerHTML = currentText;
    saveConversation();
    return currentText;
  }

  // 링크 목록 추가
  function addSourceLinks(posts, messageId) {
    if (posts.length === 0) return;

    const messageElement = messagesContainer.querySelector(
      `.message[data-id="${messageId}"]`
    );
    if (!messageElement) return;

    const sourcesList = document.createElement('div');
    sourcesList.className = 'sources';
    sourcesList.innerHTML =
      '참고한 글: ' +
      posts
        .map((post) => {
          const pathParts = post.path.split('/').filter((part) => part !== '');
          const lastPart = pathParts[pathParts.length - 1] || '';
          return `<a href="/posts/${lastPart}/" target="_blank">${post.title}</a>`;
        })
        .join(', ');

    messageElement.appendChild(sourcesList);
    saveConversation();
    scrollToBottom();

    setTimeout(() => scrollToBottom(), 100);
  }

  // 자동 스크롤
  function scrollToBottom() {
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  // 대화 내역 저장
  function saveConversation() {
    const messages = messagesContainer.innerHTML;
    sessionStorage.setItem(ChatbotConfig.storage.keys.history, messages);
  }

  // 대화 내역 불러오기
  function loadConversation() {
    const history = sessionStorage.getItem(ChatbotConfig.storage.keys.history);
    if (history) {
      messagesContainer.innerHTML = history;
      scrollToBottom();
    }
  }

  // 인터페이스 노출
  return {
    init,
    toggle,
    addMessage,
    updateMessage,
    simulateStreaming,
    addSourceLinks,
    scrollToBottom,
    loadConversation,
    getInput: () => input.value.trim(),
    clearInput: () => {
      input.value = '';
    },
  };
})();

window.ChatbotUI = ChatbotUI;
