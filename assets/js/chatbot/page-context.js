(() => {
  const namespace = (window.Gyupt = window.Gyupt || {});
  const MAX_CONTENT_CHARACTERS = 5000;
  const MAX_HEADING_COUNT = 10;
  const MAX_LIST_ITEMS = 8;

  function readCurrentPage(documentObject = document, locationObject = window.location) {
    const article = documentObject.querySelector('.post-article');
    const content = article?.querySelector('.content');
    const path = normalizePath(locationObject.pathname);

    if (!article || !content || !path.startsWith('/posts/')) {
      return null;
    }

    const title = normalizeText(article.querySelector('header h1')?.textContent || '');
    if (!title) {
      return null;
    }

    const description = normalizeText(
      article.querySelector('.post-desc')?.textContent || ''
    );
    const headings = collectTextList(
      content.querySelectorAll('h2, h3, h4'),
      MAX_HEADING_COUNT
    );
    const categories = collectTextList(
      article.querySelectorAll('.post-meta a[href*="/categories/"]'),
      MAX_LIST_ITEMS
    );
    const tags = collectTextList(
      article.querySelectorAll('.post-tags .post-tag'),
      MAX_LIST_ITEMS
    );
    const articleContent = normalizeText(content.innerText || '').slice(
      0,
      MAX_CONTENT_CHARACTERS
    );

    return {
      type: 'post',
      path,
      title,
      description,
      headings,
      categories,
      tags,
      content: articleContent,
    };
  }

  function collectTextList(nodes, limit) {
    return Array.from(nodes || [])
      .map((node) => normalizeText(node.textContent || ''))
      .filter(Boolean)
      .slice(0, limit);
  }

  function normalizeText(value) {
    return String(value)
      .replace(/\r\n?/g, '\n')
      .replace(/\s+\n/g, '\n')
      .replace(/\n{3,}/g, '\n\n')
      .replace(/[ \t]{2,}/g, ' ')
      .trim();
  }

  function normalizePath(pathname) {
    if (!pathname) {
      return '/';
    }

    try {
      const decoded = decodeURIComponent(pathname);
      return decoded.endsWith('/') ? decoded : `${decoded}/`;
    } catch (error) {
      return pathname.endsWith('/') ? pathname : `${pathname}/`;
    }
  }

  namespace.pageContext = {
    readCurrentPage,
  };
})();
