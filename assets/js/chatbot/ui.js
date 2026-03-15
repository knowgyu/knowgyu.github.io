(() => {
  const namespace = (window.Gyupt = window.Gyupt || {});
  const { config, markdown } = namespace;
  const refs = {};

  function init() {
    refs.container = document.querySelector(config.selectors.container);
    refs.toggle = document.querySelector(config.selectors.toggle);
    refs.status = document.querySelector(config.selectors.status);
    refs.body = document.querySelector(config.selectors.body);
    refs.messages = document.querySelector(config.selectors.messages);
    refs.suggestions = document.querySelector(config.selectors.suggestions);
    refs.form = document.querySelector(config.selectors.form);
    refs.input = document.querySelector(config.selectors.input);
    refs.submit = document.querySelector(config.selectors.submit);
    refs.reset = document.querySelector(config.selectors.reset);

    return Object.values(refs).every(Boolean);
  }

  function bindHandlers({
    onSubmit,
    onToggle,
    onReset,
    onPrompt,
    onCopy,
    onDraftChange,
  }) {
    refs.form.addEventListener('submit', (event) => {
      event.preventDefault();
      onSubmit();
    });

    refs.toggle.addEventListener('click', () => {
      onToggle();
    });

    refs.reset.addEventListener('click', () => {
      onReset();
    });

    refs.input.addEventListener('input', (event) => {
      autosizeInput();
      onDraftChange(event.target.value);
    });

    refs.input.addEventListener('keydown', (event) => {
      if (event.key === 'Enter' && !event.shiftKey) {
        event.preventDefault();
        onSubmit();
      }
    });

    refs.suggestions.addEventListener('click', (event) => {
      const button = event.target.closest('[data-prompt]');
      if (!button) {
        return;
      }

      onPrompt(button.dataset.prompt);
    });

    refs.messages.addEventListener('click', (event) => {
      const button = event.target.closest('[data-copy-message-id]');
      if (!button) {
        return;
      }

      onCopy(button.dataset.copyMessageId);
    });
  }

  function render(state) {
    refs.container.classList.remove('chatbot-hidden');
    refs.container.classList.toggle('collapsed', state.isCollapsed);
    refs.toggle.setAttribute('aria-expanded', String(!state.isCollapsed));
    refs.body.setAttribute('aria-hidden', String(state.isCollapsed));
    refs.status.textContent = state.status;
    refs.status.dataset.tone = state.statusTone;
    refs.input.disabled = state.isBusy;
    refs.reset.disabled = state.isBusy;
    renderDraft(state.draft);
    refs.submit.disabled = state.isBusy || !state.draft.trim();
    refs.messages.replaceChildren(...state.messages.map(createMessageElement));
    renderSuggestions(state);

    if (!state.isCollapsed) {
      scrollToBottom();
    }
  }

  function renderDraft(draft) {
    if (refs.input.value !== draft) {
      refs.input.value = draft;
    }

    autosizeInput();
  }

  function renderSuggestions(state) {
    const prompts = state.pageContext
      ? config.ui.postSuggestionPrompts
      : config.ui.suggestionPrompts;
    const shouldShowSuggestions =
      !state.isBusy &&
      state.messages.filter((message) => !message.pending).length <= 1 &&
      prompts.length > 0;

    refs.suggestions.hidden = !shouldShowSuggestions;
    refs.suggestions.replaceChildren();

    if (!shouldShowSuggestions) {
      return;
    }

    const fragment = document.createDocumentFragment();

    prompts.forEach((prompt) => {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'chatbot-chip';
      button.dataset.prompt = prompt;
      button.textContent = prompt;
      fragment.appendChild(button);
    });

    refs.suggestions.appendChild(fragment);
  }

  function createMessageElement(message) {
    const element = document.createElement('article');
    element.className = `message ${message.role}`;

    if (message.pending) {
      element.classList.add('pending');

      const spinner = document.createElement('span');
      spinner.className = 'chatbot-loading';
      spinner.setAttribute('aria-hidden', 'true');
      element.appendChild(spinner);

      const label = document.createElement('div');
      label.className = 'message-text';
      markdown.renderInto(label, message.text);
      element.appendChild(label);
      return element;
    }

    const body = document.createElement('div');
    body.className = 'message-text';
    markdown.renderInto(body, message.text);
    element.appendChild(body);

    if (message.role === 'assistant' && message.text) {
      const actions = document.createElement('div');
      actions.className = 'message-actions';

      const copyButton = document.createElement('button');
      copyButton.type = 'button';
      copyButton.className = 'message-copy-button';
      copyButton.dataset.copyMessageId = message.id;
      copyButton.textContent = '복사';
      actions.appendChild(copyButton);
      element.appendChild(actions);
    }

    if (message.sources && message.sources.length > 0) {
      const sources = document.createElement('div');
      sources.className = 'message-sources';

      const label = document.createElement('span');
      label.className = 'message-sources-label';
      label.textContent = '참고한 글';
      sources.appendChild(label);

      message.sources.forEach((source) => {
        const link = document.createElement('a');
        link.href = normalizeSourcePath(source.path);
        link.target = '_blank';
        link.rel = 'noreferrer';
        link.textContent = source.title;
        sources.appendChild(link);
      });

      element.appendChild(sources);
    }

    return element;
  }

  function normalizeSourcePath(path) {
    if (!path) {
      return '/';
    }

    const normalizedPath = path.startsWith('/') ? path : `/${path}`;

    if (!normalizedPath.startsWith('/posts/')) {
      return normalizedPath;
    }

    return normalizedPath
      .split('/')
      .map((segment, index) => (index <= 1 ? segment : segment.replace(/\s+/g, '-')))
      .join('/');
  }

  function autosizeInput() {
    refs.input.style.height = 'auto';
    refs.input.style.height = `${Math.min(refs.input.scrollHeight, 144)}px`;
  }

  function focusInput() {
    refs.input.focus();
  }

  function scrollToBottom() {
    refs.messages.scrollTop = refs.messages.scrollHeight;
  }

  namespace.ui = {
    init,
    bindHandlers,
    render,
    focusInput,
  };
})();
