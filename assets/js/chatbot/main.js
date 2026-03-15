(() => {
  const namespace = (window.Gyupt = window.Gyupt || {});
  const { api, config, pageContext, ui } = namespace;

  let state;
  let statusTimer = null;

  function init() {
    if (!ui.init()) {
      return;
    }

    state = createInitialState();
    persistConversation();
    ui.bindHandlers({
      onSubmit: handleSubmit,
      onToggle: handleToggle,
      onReset: handleReset,
      onPrompt: handlePrompt,
      onCopy: handleCopy,
      onDraftChange: handleDraftChange,
    });
    render();
  }

  function createInitialState() {
    const currentPageContext = pageContext.readCurrentPage();

    return {
      isBusy: false,
      isCollapsed: readCollapsedState(),
      draft: '',
      pageContext: currentPageContext,
      status: resolveReadyCopy(currentPageContext),
      statusTone: 'neutral',
      messages: loadConversation(currentPageContext),
    };
  }

  function loadConversation(currentPageContext) {
    const storageValue = readSessionValue(config.storage.conversationKey);
    const welcomeMessage = createWelcomeMessage(currentPageContext);

    if (!storageValue) {
      return [welcomeMessage];
    }

    try {
      const parsed = JSON.parse(storageValue);
      const normalized = normalizeStoredConversation(parsed);
      const currentPageKey = resolveConversationPageKey(currentPageContext);

      if (normalized.pageKey !== currentPageKey) {
        return [welcomeMessage];
      }

      const sanitized = normalized.messages
        .filter((message) => message && typeof message.text === 'string')
        .map((message) => ({
          id: createMessageId(),
          role: message.role === 'user' ? 'user' : 'assistant',
          text: message.text,
          pending: false,
          sources: sanitizeSources(message.sources),
        }));

      return sanitized.length > 0
        ? sanitized.slice(-config.ui.maxStoredMessages)
        : [welcomeMessage];
    } catch (error) {
      console.error('대화 내역 복원 실패:', error);
      return [welcomeMessage];
    }
  }

  function normalizeStoredConversation(parsed) {
    if (Array.isArray(parsed)) {
      return {
        pageKey: '__global__',
        messages: parsed,
      };
    }

    if (!parsed || typeof parsed !== 'object') {
      return {
        pageKey: '__global__',
        messages: [],
      };
    }

    return {
      pageKey:
        typeof parsed.pageKey === 'string' ? parsed.pageKey : '__global__',
      messages: Array.isArray(parsed.messages) ? parsed.messages : [],
    };
  }

  function sanitizeSources(sources) {
    if (!Array.isArray(sources)) {
      return [];
    }

    return sources
      .filter(
        (source) =>
          source &&
          typeof source.title === 'string' &&
          typeof source.path === 'string'
      )
      .map((source) => ({
        title: source.title,
        path: source.path,
      }));
  }

  function readCollapsedState() {
    const storageValue = readLocalValue(config.storage.collapsedKey);

    if (storageValue === null) {
      return config.ui.initialCollapsed;
    }

    return storageValue === 'true';
  }

  function handleToggle() {
    state.isCollapsed = !state.isCollapsed;
    writeLocalValue(config.storage.collapsedKey, String(state.isCollapsed));
    render();

    if (!state.isCollapsed) {
      ui.focusInput();
    }
  }

  function handleReset() {
    if (state.isBusy) {
      return;
    }

    state.messages = [createWelcomeMessage(state.pageContext)];
    state.draft = '';
    persistConversation();
    setTransientStatus(config.copy.reset, 'neutral');
    render();
    ui.focusInput();
  }

  function handlePrompt(prompt) {
    if (state.isBusy) {
      return;
    }

    state.draft = limitDraft(prompt);

    if (state.isCollapsed) {
      state.isCollapsed = false;
      writeLocalValue(config.storage.collapsedKey, 'false');
    }

    render();
    ui.focusInput();
  }

  async function handleCopy(messageId) {
    const message = state.messages.find((entry) => entry.id === messageId);
    if (!message || !message.text) {
      return;
    }

    try {
      await window.navigator.clipboard.writeText(message.text);
      setTransientStatus(config.copy.copied, 'neutral');
      render();
    } catch (error) {
      console.error('복사 실패:', error);
    }
  }

  function handleDraftChange(value) {
    state.draft = limitDraft(value);
    render();
  }

  async function handleSubmit() {
    const question = state.draft.trim();
    if (!question || state.isBusy) {
      return;
    }

    clearStatusTimer();

    const conversationForRequest = buildConversationForRequest();
    state.draft = '';
    state.messages.push(createMessage('user', question));
    state.messages.push(
      createMessage('assistant', config.copy.loading, { pending: true })
    );
    state.isBusy = true;
    state.status = config.copy.loading;
    state.statusTone = 'busy';
    persistConversation();
    render();

    try {
      let streamedText = '';
      let streamedSources = [];

      await api.streamAnswer(question, conversationForRequest, state.pageContext, {
        onDelta(delta) {
          streamedText += delta;
          updatePendingMessage({
            text: streamedText,
            pending: true,
          });
          render();
        },
        onSources(sourcePosts) {
          streamedSources = sanitizeSources(sourcePosts);
        },
        onDone(payload) {
          if (!streamedText) {
            streamedText = payload.answer || '';
          }

          if (Array.isArray(payload.sourcePosts)) {
            streamedSources = sanitizeSources(payload.sourcePosts);
          }
        },
      });

      replacePendingMessage({
        role: 'assistant',
        text: streamedText || '응답이 비어 있습니다.',
        pending: false,
        sources: streamedSources,
      });
      state.status = resolveReadyCopy(state.pageContext);
      state.statusTone = 'neutral';
    } catch (error) {
      console.error('챗봇 요청 실패:', error);
      const errorMessage =
        error instanceof Error && error.message
          ? error.message
          : config.copy.unavailable;

      replacePendingMessage({
        role: 'assistant',
        text: errorMessage,
        pending: false,
        sources: [],
      });
      state.status = errorMessage;
      state.statusTone = 'error';
    } finally {
      state.isBusy = false;
      trimConversation();
      persistConversation();
      render();
    }
  }

  function limitDraft(value) {
    return value.slice(0, config.ui.maxDraftLength);
  }

  function replacePendingMessage(nextMessage) {
    const pendingIndex = state.messages.findIndex((message) => message.pending);
    const normalizedMessage = {
      id: createMessageId(),
      role: nextMessage.role,
      text: nextMessage.text,
      pending: false,
      sources: nextMessage.sources || [],
    };

    if (pendingIndex === -1) {
      state.messages.push(normalizedMessage);
      return;
    }

    state.messages.splice(pendingIndex, 1, normalizedMessage);
  }

  function updatePendingMessage(nextMessage) {
    const pendingMessage = state.messages.find((message) => message.pending);
    if (!pendingMessage) {
      return;
    }

    if (typeof nextMessage.text === 'string') {
      pendingMessage.text = nextMessage.text;
    }

    if (typeof nextMessage.pending === 'boolean') {
      pendingMessage.pending = nextMessage.pending;
    }
  }

  function trimConversation() {
    if (state.messages.length <= config.ui.maxStoredMessages) {
      return;
    }

    state.messages = state.messages.slice(-config.ui.maxStoredMessages);
  }

  function buildConversationForRequest() {
    const candidates = state.messages
      .filter((message) => !message.pending)
      .slice(-config.ui.maxMessagesForRequest);
    const selected = [];
    let totalCharacters = 0;

    for (let index = candidates.length - 1; index >= 0; index -= 1) {
      const message = candidates[index];
      const messageLength = message.text.length;

      if (
        selected.length > 0 &&
        totalCharacters + messageLength >
          config.ui.maxConversationCharactersForRequest
      ) {
        break;
      }

      selected.unshift({
        role: message.role,
        text: message.text,
      });
      totalCharacters += messageLength;
    }

    return selected;
  }

  function createMessage(role, text, options = {}) {
    return {
      id: createMessageId(),
      role,
      text,
      pending: options.pending === true,
      sources: options.sources || [],
    };
  }

  function createMessageId() {
    return `gyupt-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
  }

  function createWelcomeMessage(currentPageContext) {
    return createMessage('assistant', buildWelcomeText(currentPageContext));
  }

  function buildWelcomeText(currentPageContext) {
    if (!currentPageContext) {
      return config.copy.welcome;
    }

    const lines = [
      `지금 보고 있는 글은 **${currentPageContext.title}** 입니다.`,
      '',
    ];

    if (currentPageContext.description) {
      lines.push(currentPageContext.description, '');
    }

    lines.push(
      '이 글 기준으로 바로 도와드릴 수 있어요.',
      '- 현재 글 요약해주세요.',
      '- 현재 글과 관련된 다른 포스트들 알려주세요.'
    );

    return lines.join('\n');
  }

  function resolveConversationPageKey(currentPageContext) {
    return currentPageContext ? currentPageContext.path : '__global__';
  }

  function persistConversation() {
    const serializableMessages = state.messages
      .filter((message) => !message.pending)
      .map((message) => ({
        role: message.role,
        text: message.text,
        sources: message.sources,
      }));

    writeSessionValue(
      config.storage.conversationKey,
      JSON.stringify({
        pageKey: resolveConversationPageKey(state.pageContext),
        messages: serializableMessages,
      })
    );
  }

  function setTransientStatus(text, tone) {
    state.status = text;
    state.statusTone = tone;
    clearStatusTimer();
    statusTimer = window.setTimeout(() => {
      state.status = resolveReadyCopy(state.pageContext);
      state.statusTone = 'neutral';
      render();
    }, 1800);
  }

  function clearStatusTimer() {
    if (statusTimer !== null) {
      window.clearTimeout(statusTimer);
      statusTimer = null;
    }
  }

  function render() {
    ui.render(state);
  }

  function resolveReadyCopy(currentPageContext) {
    return currentPageContext ? config.copy.postReady : config.copy.ready;
  }

  function readLocalValue(key) {
    try {
      return window.localStorage.getItem(key);
    } catch (error) {
      return null;
    }
  }

  function writeLocalValue(key, value) {
    try {
      window.localStorage.setItem(key, value);
    } catch (error) {
      console.warn('로컬 스토리지 저장 실패:', error);
    }
  }

  function readSessionValue(key) {
    try {
      return window.sessionStorage.getItem(key);
    } catch (error) {
      return null;
    }
  }

  function writeSessionValue(key, value) {
    try {
      window.sessionStorage.setItem(key, value);
    } catch (error) {
      console.warn('세션 스토리지 저장 실패:', error);
    }
  }

  document.addEventListener('DOMContentLoaded', init);
})();
