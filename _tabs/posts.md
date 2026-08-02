---
layout: page
title: 전체 글
icon: fas fa-list-ul
order: 1
---

<section class="posts-catalog" aria-labelledby="posts-catalog-title" markdown="0">
  <p class="catalog-kicker">All posts</p>
  <h2 id="posts-catalog-title">전체 글</h2>
  <div class="catalog-list" data-posts-catalog>
{% assign catalog_posts = site.posts | where_exp: 'item', 'item.hidden != true' %}
{% for post in catalog_posts %}
<article class="catalog-item" data-catalog-item{% if forloop.index > 15 %} hidden{% endif %}>
<a href="{{ post.url | relative_url | escape }}">
<span class="catalog-title">{{ post.title | escape }}</span>
<span class="catalog-meta">
<time datetime="{{ post.date | date_to_xmlschema | escape }}">{{ post.date | date: "%Y-%m-%d" }}</time>
{% if post.categories.size > 0 %}
<span>{% for category in post.categories limit: 2 %}{{ category | escape }}{% unless forloop.last %} / {% endunless %}{% endfor %}</span>
{% endif %}
</span>
</a>
</article>
{% endfor %}
  </div>

{% if catalog_posts.size > 15 %}
<nav class="catalog-pager" aria-label="Posts pages" data-posts-pager>
<button type="button" data-page-prev disabled>이전</button>
<span data-page-status>1 / {{ catalog_posts.size | divided_by: 15.0 | ceil }}</span>
<button type="button" data-page-next>다음</button>
</nav>
<script>
(() => {
  const items = [...document.querySelectorAll('[data-catalog-item]')];
  const pager = document.querySelector('[data-posts-pager]');
  if (!pager || items.length <= 15) return;

  const size = 15;
  const pages = Math.ceil(items.length / size);
  const prev = pager.querySelector('[data-page-prev]');
  const next = pager.querySelector('[data-page-next]');
  const status = pager.querySelector('[data-page-status]');
  let page = 0;

  const render = () => {
    items.forEach((item, index) => item.hidden = Math.floor(index / size) !== page);
    prev.disabled = page === 0;
    next.disabled = page === pages - 1;
    status.textContent = `${page + 1} / ${pages}`;
  };

  prev.addEventListener('click', () => {
    page = Math.max(0, page - 1);
    render();
  });
  next.addEventListener('click', () => {
    page = Math.min(pages - 1, page + 1);
    render();
  });
  render();
})();
</script>
{% endif %}
</section>
