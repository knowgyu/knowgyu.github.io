const ChatbotAPI = (function () {
  // API URL 가져오기
  function getApiUrl() {
    const isLocalhost =
      window.location.hostname === 'localhost' ||
      window.location.hostname === '127.0.0.1';

    return isLocalhost
      ? ChatbotConfig.api.localUrl
      : ChatbotConfig.api.productionUrl;
  }

  // 챗봇 응답 요청
  async function getChatResponse(question, context) {
    const url = getApiUrl();

    try {
      const response = await fetch(url, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: question,
          context: context,
        }),
      });

      if (!response.ok) {
        throw new Error('API 응답 오류: ' + response.status);
      }

      return await response.json();
    } catch (error) {
      console.error('API 요청 중 오류:', error);
      throw error;
    }
  }

  // 인터페이스 노출
  return {
    getChatResponse,
  };
})();

window.ChatbotAPI = ChatbotAPI;
