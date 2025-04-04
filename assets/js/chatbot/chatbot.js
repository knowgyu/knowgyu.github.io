class BlogChatbot {
  constructor() {
    console.log('챗봇 생성자 호출됨');
    this.postsData = [];
    this.container = document.getElementById('chatbot-container');

    if (!this.container) {
      console.error(
        '챗봇 컨테이너를 찾을 수 없습니다. HTML이 제대로 로드되었는지 확인하세요.'
      );
      return;
    }

    this.messagesContainer = document.getElementById('chatbot-messages');
    this.input = document.getElementById('chatbot-input');
    this.submitButton = document.getElementById('chatbot-submit');
    this.toggleButton = document.getElementById('chatbot-toggle');

    console.log('모든 DOM 요소 참조 완료:', {
      container: !!this.container,
      messagesContainer: !!this.messagesContainer,
      input: !!this.input,
      submitButton: !!this.submitButton,
      toggleButton: !!this.toggleButton,
    });

    this.init();
  }

  async init() {
    try {
      console.log('포스트 데이터 로드 시도...');
      const response = await fetch('/assets/js/chatbot/posts-data.json');
      this.postsData = await response.json();
      console.log(`${this.postsData.length}개의 포스트 데이터 로드됨`);

      // 이전 대화 불러오기
      this.loadConversation();

      // 이벤트 리스너 설정
      this.submitButton.addEventListener('click', () => this.handleUserInput());
      this.input.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') this.handleUserInput();
      });
      this.toggleButton.addEventListener('click', () => this.toggleChatbot());

      console.log('블로그 챗봇이 초기화되었습니다.');
    } catch (error) {
      console.error('챗봇 초기화 중 오류 발생:', error);
      this.addMessage(
        '시스템 초기화 중 오류가 발생했습니다. 나중에 다시 시도해주세요.',
        'bot'
      );
    }
  }

  toggleChatbot() {
    this.container.classList.toggle('collapsed');

    // 접힌 상태에 따라 처리
    if (this.container.classList.contains('collapsed')) {
      // 여기서는 아이콘 변경 로직이 필요 없음 (CSS로 처리)
      console.log('챗봇이 접혔습니다.');
    } else {
      console.log('챗봇이 열렸습니다.');
    }

    // 자동 스크롤 (열린 경우)
    if (!this.container.classList.contains('collapsed')) {
      this.scrollToBottom();
    }
  }

  // 수정된 handleUserInput 메서드
  async handleUserInput() {
    const userQuery = this.input.value.trim();
    if (userQuery === '') return;

    // 사용자 메시지 표시
    this.addMessage(userQuery, 'user');
    this.input.value = '';

    // 챗봇 응답 생성 (로딩 표시)
    const loadingId = this.addMessage(
      '<div class="chatbot-loading"></div>',
      'bot'
    );

    try {
      // API URL 설정 (기존과 동일)
      let apiUrl;
      const isLocalhost =
        window.location.hostname === 'localhost' ||
        window.location.hostname === '127.0.0.1';
      const netlifyDevPort = 8888;

      if (isLocalhost) {
        apiUrl = `http://localhost:${netlifyDevPort}/.netlify/functions/chat`;
      } else {
        apiUrl =
          'https://knowgyu-ai-functions.netlify.app/.netlify/functions/chat';
      }

      // 관련 포스트 찾기 (기존과 동일)
      const relevantPosts = this.findRelevantPosts(userQuery);
      let context = '관련 정보를 찾을 수 없습니다.';

      if (relevantPosts.length > 0) {
        context = relevantPosts
          .map(
            (post) =>
              `제목: ${post.title}\n내용: ${this.extractRelevantText(
                post.content,
                userQuery
              )}`
          )
          .join('\n\n');
      }

      // API 호출
      const response = await fetch(apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: userQuery,
          context: context,
        }),
      });

      if (!response.ok) {
        throw new Error('API 응답 오류: ' + response.status);
      }

      const data = await response.json();

      // 로딩 메시지를 빈 메시지로 업데이트
      this.updateMessage(loadingId, '', 'bot');

      // 스트리밍 시뮬레이션 시작
      await this.simulateStreaming(data.chunks || [data.answer], loadingId);

      // 참고 링크 추가
      if (relevantPosts.length > 0) {
        const sourcesList = document.createElement('div');
        sourcesList.className = 'sources';
        sourcesList.innerHTML =
          '참고한 글: ' +
          relevantPosts
            .map((post) => {
              const pathParts = post.path
                .split('/')
                .filter((part) => part !== '');
              const lastPart = pathParts[pathParts.length - 1] || '';
              return `<a href="/posts/${lastPart}/" target="_blank">${post.title}</a>`;
            })
            .join(', ');

        const botMessage = this.messagesContainer.querySelector(
          `.message.bot:last-child`
        );
        botMessage.appendChild(sourcesList);

        // 참고 링크 추가 후 대화 내역 저장
        this.saveConversation();

        // 참고 링크 추가 후 스크롤 조정
        this.scrollToBottom();

        // DOM 업데이트를 위해 약간의 지연 후 한 번 더 스크롤
        setTimeout(() => this.scrollToBottom(), 100);
      }
    } catch (error) {
      console.error('답변 생성 중 오류:', error);
      this.updateMessage(
        loadingId,
        '죄송합니다, 응답 생성 중 오류가 발생했습니다.',
        'bot'
      );
    }
  }

  // 스트리밍 효과를 시뮬레이션하는 메서드 - 속도 조절
  async simulateStreaming(chunks, messageId) {
    const botMessage = this.messagesContainer.querySelector(
      `.message.bot:last-child`
    );
    let currentText = '';

    // chunks가 문자열이면 배열로 변환
    if (typeof chunks === 'string') {
      chunks = [chunks];
    }

    for (const chunk of chunks) {
      // 한 번에 청크 전체가 아닌 문자 단위로 표시
      for (let i = 0; i < chunk.length; i++) {
        currentText += chunk[i];
        botMessage.innerHTML = currentText;
        this.scrollToBottom();

        // 문자별 타이핑 딜레이 (20-40ms 사이의 랜덤 값)
        await new Promise((resolve) =>
          setTimeout(resolve, Math.floor(Math.random() * 10) + 10)
        );
      }

      // 각 청크(문장) 사이의 추가 딜레이
      await new Promise((resolve) => setTimeout(resolve, 100));

      // 문장 끝에 공백 추가
      if (chunk[chunk.length - 1] !== ' ') {
        currentText += ' ';
        botMessage.innerHTML = currentText;
      }
    }

    // 모든 청크를 표시한 뒤 대화 내역 저장
    botMessage.innerHTML = currentText;
    this.saveConversation();

    return currentText;
  }

  addMessage(text, sender) {
    const messageElement = document.createElement('div');
    messageElement.classList.add('message', sender);
    messageElement.innerHTML = text;

    this.messagesContainer.appendChild(messageElement);
    this.scrollToBottom(); // 자동 스크롤 추가
    this.saveConversation(); // 대화 내역 저장

    return Date.now().toString(); // 메시지 ID 반환
  }

  updateMessage(id, text, sender) {
    // 실제 구현에서는 ID를 활용하여 특정 메시지 업데이트
    const messages = this.messagesContainer.querySelectorAll(
      `.message.${sender}`
    );
    const lastMessage = messages[messages.length - 1];
    if (lastMessage) {
      // 사용자 검색어로 키워드 하이라이팅 적용
      const userQuery = this.input.value.trim();
      const keywords = userQuery.split(' ').filter((k) => k.length > 1);
      const highlightedText = this.highlightKeywords(text, keywords);

      lastMessage.innerHTML = highlightedText;
      this.scrollToBottom(); // 자동 스크롤
      this.saveConversation(); // 대화 내역 저장
    }
  }

  findRelevantPosts(query) {
    // 검색어를 토큰화
    const keywords = query
      .toLowerCase()
      .split(/\s+/)
      .filter((k) => k.length > 1);

    return this.postsData
      .map((post) => {
        const contentLower = post.content.toLowerCase();
        const titleLower = post.title.toLowerCase();

        // 키워드 매칭 점수 계산
        const contentMatches = keywords.filter((k) =>
          contentLower.includes(k)
        ).length;
        const titleMatches = keywords.filter((k) =>
          titleLower.includes(k)
        ).length;

        return {
          ...post,
          score: contentMatches + titleMatches * 2, // 제목 매칭에 가중치 부여
        };
      })
      .filter((post) => post.score > 0)
      .sort((a, b) => b.score - a.score)
      .slice(0, 3); // 상위 3개 포스트만 반환
  }

  extractRelevantText(content, query) {
    // 키워드 주변 텍스트 추출
    const keywords = query
      .toLowerCase()
      .split(/\s+/)
      .filter((k) => k.length > 1);
    const contentLower = content.toLowerCase();

    for (const keyword of keywords) {
      const index = contentLower.indexOf(keyword);
      if (index !== -1) {
        // 키워드 주변 텍스트 250자 추출
        const start = Math.max(0, index - 120);
        const end = Math.min(content.length, index + 120);
        let text = content.substring(start, end);

        // 마크다운 서식 제거 (간단한 처리)
        text = text.replace(/\*\*/g, '').replace(/\*/g, '').replace(/\n/g, ' ');

        // 문장 경계에서 자르기
        if (start > 0) text = '...' + text;
        if (end < content.length) text = text + '...';

        return text;
      }
    }

    // 키워드를 찾지 못한 경우 앞부분 반환
    return content.substring(0, 200).replace(/\n/g, ' ') + '...';
  }

  // 이전 대화 내역 저장
  saveConversation() {
    const messages = this.messagesContainer.innerHTML;
    // localStorage 대신 sessionStorage 사용
    sessionStorage.setItem('chatHistory', messages);
    console.log('대화 내역이 세션에 저장되었습니다.');
  }

  // 저장된 대화 내역 불러오기
  loadConversation() {
    // localStorage 대신 sessionStorage에서 불러오기
    const history = sessionStorage.getItem('chatHistory');
    if (history) {
      this.messagesContainer.innerHTML = history;
      console.log('이전 대화 내역을 불러왔습니다.');
      this.scrollToBottom();
    }
  }

  // 키워드 하이라이팅
  highlightKeywords(text, keywords) {
    if (!keywords || keywords.length === 0) return text;

    keywords.forEach((keyword) => {
      if (keyword.length < 2) return; // 너무 짧은 키워드는 제외
      const regex = new RegExp(`(${keyword})`, 'gi');
      text = text.replace(regex, '<span class="highlight">$1</span>');
    });
    return text;
  }

  // 자동 스크롤
  scrollToBottom() {
    this.messagesContainer.scrollTop = this.messagesContainer.scrollHeight;
  }
}

// 페이지 로드 시 챗봇 초기화
document.addEventListener('DOMContentLoaded', () => {
  console.log('DOM이 로드되었습니다. 챗봇 초기화 중...');
  const chatbot = new BlogChatbot();
});

// 로드 실패 시 추가 확인을 위한 백업 핸들러
window.addEventListener('load', () => {
  console.log('window.load 이벤트 발생');
  if (!document.getElementById('chatbot-container')) {
    console.error(
      'window.load 이벤트 후에도 챗봇 컨테이너를 찾을 수 없습니다.'
    );
  }
});
