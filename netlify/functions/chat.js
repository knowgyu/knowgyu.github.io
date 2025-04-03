const { Configuration, OpenAIApi } = require('openai');

exports.handler = async (event) => {
  try {
    // POST 요청만 처리
    if (event.httpMethod !== 'POST') {
      return { statusCode: 405, body: 'Method Not Allowed' };
    }

    // 요청 데이터 파싱
    const { question, context } = JSON.parse(event.body);

    // OpenAI API 설정
    const configuration = new Configuration({
      apiKey: process.env.OPENAI_API_KEY,
    });
    const openai = new OpenAIApi(configuration);

    // 프롬프트 구성
    const prompt = `
블로그 내용을 기반으로 다음 질문에 답변해주세요:

질문: ${question}

관련 블로그 내용:
${context}

답변을 한국어로 명확하게 제공해주세요. 블로그 내용에 관련 정보가 없다면 솔직하게 모른다고 답변해주세요.
`;

    // API 호출
    const response = await openai.createChatCompletion({
      model: 'gpt-3.5-turbo',
      messages: [
        {
          role: 'system',
          content:
            '당신은 블로그 내용을 기반으로 질문에 답변하는 도우미입니다.',
        },
        { role: 'user', content: prompt },
      ],
      max_tokens: 500,
      temperature: 0.7,
    });

    // 응답 반환
    return {
      statusCode: 200,
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        answer: response.data.choices[0].message.content.trim(),
      }),
    };
  } catch (error) {
    console.error('Error:', error);
    return {
      statusCode: 500,
      body: JSON.stringify({ error: '내부 서버 오류가 발생했습니다.' }),
    };
  }
};
