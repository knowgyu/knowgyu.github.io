---
layout: page
title: Projects
icon: fas fa-project-diagram
order: 2
permalink: /projects/
---

{% assign project_posts = site.posts | where_exp: 'post', 'post.project' %}
{% assign project_names = '' | split: '' %}
{% for post in project_posts %}
  {% unless project_names contains post.project %}
    {% assign project_names = project_names | push: post.project %}
  {% endunless %}
{% endfor %}
{% assign project_names = project_names | sort %}

<section class="projects-index" aria-labelledby="projects-title" markdown="0">
  <p class="catalog-kicker">Projects</p>
  <h2 id="projects-title">Projects</h2>

  {% for project in project_names %}
    <section class="project-group" aria-labelledby="project-{{ forloop.index }}">
      {% assign group_posts = project_posts | where: 'project', project %}
      {% assign latest_post = group_posts | first %}
      <h3 id="project-{{ forloop.index }}">{{ project | replace: '-', ' ' | escape }}</h3>
      <p class="project-group-meta">
        <span>{{ group_posts | size }} post{% unless group_posts.size == 1 %}s{% endunless %}</span>
        {% if latest_post %}
          <span>
            Latest:
            <a href="{{ latest_post.url | relative_url | escape }}">{{ latest_post.title | escape }}</a>
            <time datetime="{{ latest_post.date | date_to_xmlschema | escape }}">{{ latest_post.date | date: "%Y-%m-%d" }}</time>
          </span>
        {% endif %}
      </p>
      <div class="post-row-list">
        {% for post in group_posts %}
          {% include post-row.html post=post class='project-post-row' %}
        {% endfor %}
      </div>
    </section>
  {% endfor %}
</section>
