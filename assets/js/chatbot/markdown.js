(() => {
  const namespace = (window.Gyupt = window.Gyupt || {});
  const SAFE_PROTOCOLS = new Set(['http:', 'https:', 'mailto:']);

  function render(text = '') {
    const fragment = document.createDocumentFragment();
    const normalizedText = normalizeText(text);
    const lines = normalizedText.split('\n');
    appendBlocks(fragment, lines);

    if (!fragment.hasChildNodes()) {
      fragment.appendChild(document.createElement('p'));
    }

    return fragment;
  }

  function renderInto(element, text = '') {
    element.replaceChildren(render(text));
  }

  function appendBlocks(parent, lines) {
    let index = 0;

    while (index < lines.length) {
      if (!lines[index].trim()) {
        index += 1;
        continue;
      }

      const fencedCodeMatch = lines[index].match(/^ {0,3}```([A-Za-z0-9_-]+)?\s*$/);
      if (fencedCodeMatch) {
        const { nextIndex, node } = consumeCodeBlock(lines, index, fencedCodeMatch[1] || '');
        parent.appendChild(node);
        index = nextIndex;
        continue;
      }

      const headingMatch = lines[index].match(/^(#{1,6})\s+(.+)$/);
      if (headingMatch) {
        const heading = document.createElement(`h${headingMatch[1].length}`);
        appendInlineContent(heading, headingMatch[2].trim());
        parent.appendChild(heading);
        index += 1;
        continue;
      }

      if (/^ {0,3}([-*_])(?:\s*\1){2,}\s*$/.test(lines[index])) {
        parent.appendChild(document.createElement('hr'));
        index += 1;
        continue;
      }

      if (/^>\s?/.test(lines[index])) {
        const { nextIndex, node } = consumeBlockquote(lines, index);
        parent.appendChild(node);
        index = nextIndex;
        continue;
      }

      if (/^ {0,3}[-+*]\s+/.test(lines[index]) || /^ {0,3}\d+[.)]\s+/.test(lines[index])) {
        const { nextIndex, node } = consumeList(lines, index);
        parent.appendChild(node);
        index = nextIndex;
        continue;
      }

      const { nextIndex, node } = consumeParagraph(lines, index);
      parent.appendChild(node);
      index = nextIndex;
    }
  }

  function consumeCodeBlock(lines, startIndex, language) {
    const pre = document.createElement('pre');
    const code = document.createElement('code');
    const normalizedLanguage = language.trim().replace(/[^A-Za-z0-9_-]/g, '');

    if (normalizedLanguage) {
      code.className = `language-${normalizedLanguage}`;
      code.dataset.language = normalizedLanguage;
    }

    const content = [];
    let index = startIndex + 1;

    while (index < lines.length && !/^ {0,3}```\s*$/.test(lines[index])) {
      content.push(lines[index]);
      index += 1;
    }

    code.textContent = content.join('\n');
    pre.appendChild(code);

    return {
      nextIndex: index < lines.length ? index + 1 : index,
      node: pre,
    };
  }

  function consumeBlockquote(lines, startIndex) {
    const collected = [];
    let index = startIndex;

    while (index < lines.length) {
      const line = lines[index];

      if (!line.trim()) {
        collected.push('');
        index += 1;
        continue;
      }

      const match = line.match(/^>\s?(.*)$/);
      if (!match) {
        break;
      }

      collected.push(match[1]);
      index += 1;
    }

    const blockquote = document.createElement('blockquote');
    appendBlocks(blockquote, collected);

    return {
      nextIndex: index,
      node: blockquote,
    };
  }

  function consumeList(lines, startIndex) {
    const ordered = /^ {0,3}\d+[.)]\s+/.test(lines[startIndex]);
    const list = document.createElement(ordered ? 'ol' : 'ul');
    let index = startIndex;
    let currentItemLines = [];

    while (index < lines.length) {
      const line = lines[index];
      const listItemMatch = ordered
        ? line.match(/^ {0,3}\d+[.)]\s+(.*)$/)
        : line.match(/^ {0,3}[-+*]\s+(.*)$/);

      if (listItemMatch) {
        if (currentItemLines.length > 0) {
          list.appendChild(createListItem(currentItemLines));
        }

        currentItemLines = [listItemMatch[1]];
        index += 1;
        continue;
      }

      if (!line.trim()) {
        break;
      }

      if (/^\s{2,}\S/.test(line) && currentItemLines.length > 0) {
        currentItemLines.push(line.replace(/^\s+/, ''));
        index += 1;
        continue;
      }

      break;
    }

    if (currentItemLines.length > 0) {
      list.appendChild(createListItem(currentItemLines));
    }

    return {
      nextIndex: index,
      node: list,
    };
  }

  function createListItem(lines) {
    const item = document.createElement('li');
    const body = lines.join('\n').trim();
    appendInlineContent(item, body);
    return item;
  }

  function consumeParagraph(lines, startIndex) {
    const collected = [];
    let index = startIndex;

    while (index < lines.length) {
      const line = lines[index];
      if (!line.trim()) {
        break;
      }

      if (isBlockBoundary(line) && collected.length > 0) {
        break;
      }

      collected.push(line);
      index += 1;
    }

    const paragraph = document.createElement('p');
    appendInlineContent(paragraph, collected.join('\n'));

    return {
      nextIndex: index,
      node: paragraph,
    };
  }

  function isBlockBoundary(line) {
    return (
      /^ {0,3}```/.test(line) ||
      /^(#{1,6})\s+/.test(line) ||
      /^ {0,3}([-*_])(?:\s*\1){2,}\s*$/.test(line) ||
      /^>\s?/.test(line) ||
      /^ {0,3}[-+*]\s+/.test(line) ||
      /^ {0,3}\d+[.)]\s+/.test(line)
    );
  }

  function appendInlineContent(parent, text) {
    const tokenPattern =
      /(\[([^\]\n]+)\]\(([^)\s]+)\)|`([^`\n]+)`|\*\*([^*]+)\*\*|\*([^*\n]+)\*)/g;
    let lastIndex = 0;
    let match;

    while ((match = tokenPattern.exec(text)) !== null) {
      appendTextWithBreaks(parent, text.slice(lastIndex, match.index));

      if (match[2] && match[3]) {
        const href = sanitizeUrl(match[3]);

        if (href) {
          const link = document.createElement('a');
          link.href = href;
          if (/^https?:\/\//.test(href)) {
            link.target = '_blank';
            link.rel = 'noreferrer';
          }
          appendInlineContent(link, match[2]);
          parent.appendChild(link);
        } else {
          appendTextWithBreaks(parent, match[0]);
        }
      } else if (match[4]) {
        const code = document.createElement('code');
        code.textContent = match[4];
        parent.appendChild(code);
      } else if (match[5]) {
        const strong = document.createElement('strong');
        appendInlineContent(strong, match[5]);
        parent.appendChild(strong);
      } else if (match[6]) {
        const emphasis = document.createElement('em');
        appendInlineContent(emphasis, match[6]);
        parent.appendChild(emphasis);
      }

      lastIndex = tokenPattern.lastIndex;
    }

    appendTextWithBreaks(parent, text.slice(lastIndex));
  }

  function appendTextWithBreaks(parent, text) {
    if (!text) {
      return;
    }

    const segments = text.split('\n');
    segments.forEach((segment, index) => {
      if (segment) {
        parent.appendChild(document.createTextNode(segment));
      }

      if (index < segments.length - 1) {
        parent.appendChild(document.createElement('br'));
      }
    });
  }

  function sanitizeUrl(rawUrl) {
    if (typeof rawUrl !== 'string') {
      return null;
    }

    const trimmed = rawUrl.trim();
    if (!trimmed) {
      return null;
    }

    if (/^[/?#.]/.test(trimmed)) {
      return trimmed;
    }

    try {
      const parsed = new URL(trimmed, window.location.origin);

      if (
        parsed.origin === window.location.origin ||
        SAFE_PROTOCOLS.has(parsed.protocol)
      ) {
        return parsed.href;
      }
    } catch (error) {
      return null;
    }

    return null;
  }

  function normalizeText(text) {
    if (typeof text !== 'string') {
      return '';
    }

    return text.replace(/\r\n?/g, '\n').trim();
  }

  namespace.markdown = {
    render,
    renderInto,
  };
})();
