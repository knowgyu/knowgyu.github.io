(() => {
  const namespace = (window.Gyupt = window.Gyupt || {});
  const { config } = namespace;

  function resolveApiUrl(locationObject = window.location) {
    const apiMode = readApiMode(locationObject);

    return apiMode === config.api.localModeValue
      ? config.api.localUrl
      : config.api.productionUrl;
  }

  function readApiMode(locationObject) {
    try {
      const params = new URLSearchParams(locationObject.search || '');
      return params.get(config.api.queryParam);
    } catch (error) {
      return null;
    }
  }

  async function streamAnswer(
    question,
    conversation = [],
    pageContext = null,
    handlers = {}
  ) {
    const controller = new AbortController();
    const timeoutId = window.setTimeout(() => {
      controller.abort();
    }, config.api.requestTimeoutMs);

    try {
      const response = await fetch(resolveApiUrl(), {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question, conversation, pageContext }),
        signal: controller.signal,
      });

      if (!response.ok) {
        const payload = await response.json().catch(() => ({}));
        throw new Error(payload.error || `API 응답 오류 (${response.status})`);
      }

      const contentType = response.headers.get('content-type') || '';
      if (!contentType.includes('text/event-stream') || !response.body) {
        const payload = await response.json().catch(() => ({}));
        handlers.onDone?.(payload);
        return payload;
      }

      return consumeSseStream(response.body, handlers);
    } catch (error) {
      if (error.name === 'AbortError') {
        throw new Error('응답 시간이 초과되었습니다.');
      }

      throw error;
    } finally {
      window.clearTimeout(timeoutId);
    }
  }

  async function requestAnswer(question, conversation = [], pageContext = null) {
    const controller = new AbortController();
    const timeoutId = window.setTimeout(() => {
      controller.abort();
    }, config.api.requestTimeoutMs);

    try {
      const response = await fetch(resolveApiUrl(), {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question, conversation, pageContext }),
        signal: controller.signal,
      });

      const payload = await response.json().catch(() => ({}));

      if (!response.ok) {
        throw new Error(payload.error || `API 응답 오류 (${response.status})`);
      }

      return payload;
    } catch (error) {
      if (error.name === 'AbortError') {
        throw new Error('응답 시간이 초과되었습니다.');
      }

      throw error;
    } finally {
      window.clearTimeout(timeoutId);
    }
  }

  async function consumeSseStream(stream, handlers) {
    const reader = stream.getReader();
    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) {
        break;
      }

      buffer += decoder.decode(value, { stream: true });
      const events = buffer.split('\n\n');
      buffer = events.pop() || '';

      events.forEach((eventBlock) => {
        const parsed = parseSseEvent(eventBlock);
        if (!parsed) {
          return;
        }

        const { event, data } = parsed;

        if (event === 'delta') {
          handlers.onDelta?.(data.content || '');
        } else if (event === 'sources') {
          handlers.onSources?.(data.sourcePosts || []);
        } else if (event === 'done') {
          handlers.onDone?.(data);
        } else if (event === 'error') {
          throw new Error(data.error || '스트리밍 중 오류가 발생했습니다.');
        }
      });
    }
  }

  function parseSseEvent(eventBlock) {
    const lines = eventBlock.split('\n');
    let event = 'message';
    let data = '';

    for (const line of lines) {
      if (line.startsWith('event:')) {
        event = line.slice(6).trim();
      } else if (line.startsWith('data:')) {
        data += line.slice(5).trim();
      }
    }

    if (!data) {
      return null;
    }

    return {
      event,
      data: JSON.parse(data),
    };
  }

  namespace.api = {
    requestAnswer,
    streamAnswer,
    readApiMode,
    resolveApiUrl,
  };
})();
