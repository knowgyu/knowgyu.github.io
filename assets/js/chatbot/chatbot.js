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
      // 데이터 로드
      console.log('포스트 데이터 로드 시도...');
      const response = await fetch('/assets/js/chatbot/posts-data.json');
      this.postsData = await response.json();
      console.log(`${this.postsData.length}개의 포스트 데이터 로드됨`);

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
    const icon = this.toggleButton.querySelector('i');
    if (this.container.classList.contains('collapsed')) {
      icon.classList.remove('fa-chevron-down');
      icon.classList.add('fa-chevron-up');
    } else {
      icon.classList.remove('fa-chevron-up');
      icon.classList.add('fa-chevron-down');
    }
  }

  async handleUserInput() {
    const userQuery = this.input.value.trim();
    if (userQuery === '') return;

    // 사용자 메시지 표시
    this.addMessage(userQuery, 'user');
    this.input.value = '';

    // 챗봇 응답 생성 (로딩 표시)
    const loadingId = this.addMessage(
      '<div class="chatbot-loading"></div> 답변을 생성 중입니다...',
      'bot'
    );

    try {
      const answer = this.generateAnswer(userQuery);
      // 로딩 메시지 업데이트
      this.updateMessage(loadingId, answer, 'bot');
    } catch (error) {
      console.error('답변 생성 중 오류:', error);
      this.updateMessage(
        loadingId,
        '죄송합니다, 답변을 생성하는 중 오류가 발생했습니다.',
        'bot'
      );
    }
  }

  addMessage(text, sender) {
    const messageElement = document.createElement('div');
    messageElement.classList.add('message', sender);
    messageElement.innerHTML = text;

    this.messagesContainer.appendChild(messageElement);
    this.messagesContainer.scrollTop = this.messagesContainer.scrollHeight;

    return Date.now().toString(); // 메시지 ID 반환
  }

  updateMessage(id, text, sender) {
    // 실제 구현에서는 ID를 활용하여 특정 메시지 업데이트
    // 이 간단한 예제에서는 마지막 메시지를 업데이트
    const messages = this.messagesContainer.querySelectorAll(
      `.message.${sender}`
    );
    const lastMessage = messages[messages.length - 1];
    if (lastMessage) {
      lastMessage.innerHTML = text;
      this.messagesContainer.scrollTop = this.messagesContainer.scrollHeight;
    }
  }

  generateAnswer(query) {
    // 관련 포스트 찾기
    const relevantPosts = this.findRelevantPosts(query);

    if (relevantPosts.length === 0) {
      return '죄송합니다, 질문과 관련된 내용을 찾을 수 없습니다.';
    }

    // 답변 생성
    let answer = `<strong>질문과 관련된 내용을 찾았습니다:</strong><br><br>`;

    relevantPosts.forEach((post) => {
      // 관련 부분 추출
      const relevantText = this.extractRelevantText(post.content, query);

      answer += `<div class="post-result">
        <strong>${post.title}</strong>에서:<br>
        ${relevantText}<br>
        <div class="source">출처: <a href="${post.path}" target="_blank">자세히 보기</a></div>
      </div><br>`;
    });

    return answer;
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
