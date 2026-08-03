const fs = require('node:fs')
const path = require('node:path')
const { chromium } = require('playwright')

const ROUTES = [
  { name: 'home', path: '/' },
  { name: 'posts-catalog', path: '/posts/' },
  { name: 'categories', path: '/categories/' },
  { name: 'category-ros', path: '/categories/ros/' },
  { name: 'archives', path: '/archives/' },
  { name: 'tags', path: '/tags/' },
  { name: 'search-open', path: '/' },
  {
    name: 'post',
    path: '/posts/ROS1-하이퍼파라미터-튜닝-및-마무리/',
  },
]
const MODES = ['light', 'dark']
const VIEWPORTS = [
  { name: 'wide-desktop', width: 2048, height: 1152 },
  { name: 'desktop', width: 1440, height: 900 },
  { name: 'tablet', width: 768, height: 1024 },
  { name: 'mobile', width: 390, height: 844 },
  { name: 'overflow-probe', width: 360, height: 844 },
]

const outputDir = path.resolve(
  process.env.VISUAL_QA_OUTPUT || path.join('.omx', 'artifacts', 'visual-qa'),
)
const screenshotPath = (route, mode, viewport) =>
  path.join(outputDir, `${route}-${mode}-${viewport}.png`)
const evidencePath = (route, mode, viewport) =>
  path.join(outputDir, `${route}-${mode}-${viewport}.json`)
const selected = (value, all) =>
  process.env[value]
    ? all.filter((entry) => process.env[value].split(',').includes(entry.name || entry))
    : all

function formatFailure(failure) {
  return [
    `route=${failure.route}`,
    `viewport=${failure.viewport}`,
    `selector=${failure.selector}`,
    `property=${failure.property}`,
    `observed=${failure.observed}`,
    `budget=${failure.budget}`,
    `screenshot=${failure.screenshot}`,
  ].join(' ')
}

async function collectEvidence(page, routeName, viewport, screenshot) {
  return page.evaluate(
    ({ route, viewportName, screenshotPath }) => {
      const failures = []
      const cssNumber = (value) => {
        const number = Number.parseFloat(value)
        return Number.isFinite(number) ? number : null
      }
      const root = document.documentElement
      const body = document.body
      const main = document.querySelector('#main-wrapper')
      const viewportLabel = `${innerWidth}x${innerHeight}`
      const fail = (selector, property, observed, budget) =>
        failures.push({
          route,
          viewport: viewportLabel,
          selector,
          property,
          observed: String(observed),
          budget,
          screenshot: screenshotPath,
        })
      const style = (element) => (element ? getComputedStyle(element) : null)
      const rect = (element) => element?.getBoundingClientRect() || null
      const first = (selector) => document.querySelector(selector)
      const count = (selector) => document.querySelectorAll(selector).length
      const hasText = (selector) => Boolean(first(selector)?.textContent.trim())
      const visible = (element) =>
        Boolean(element && !element.hidden && style(element)?.display !== 'none' && element.getClientRects().length)
      const assertTextFits = (selector) => {
        for (const element of document.querySelectorAll(selector)) {
          if (!visible(element)) continue
          if (!element.textContent.trim()) fail(selector, 'text', 'empty', 'non-empty')
          if (element.scrollWidth > element.clientWidth + 2)
            fail(selector, 'scrollWidth-clientWidth', `${element.scrollWidth - element.clientWidth}px`, '<=2px')
          const box = rect(element)
          if (box && box.height <= 0) fail(selector, 'height', `${box.height}px`, '>0')
        }
      }
      const assertReadableTaxonomyLabels = () => {
        for (const label of document.querySelectorAll('#sidebar .taxonomy-label')) {
          if (!visible(label)) continue
          const text = label.textContent.trim()
          const box = rect(label)
          const lineWidth = Math.max(0, ...[...label.getClientRects()].map((line) => line.width))
          if (text.length >= 5 && box && lineWidth < 32)
            fail('#sidebar .taxonomy-label', 'line width', `${Math.round(lineWidth)}px for ${text}`, '>=32px')
        }
      }
      const isDesktop = innerWidth >= 1024

      if (!body) fail('body', 'presence', 'missing', 'required')
      if (!main) fail('#main-wrapper', 'presence', 'missing', 'required')

      for (const [selector, element] of [
        ['html', root],
        ['body', body],
        ['#main-wrapper', main],
      ]) {
        if (!element) continue
        const overflow = element.scrollWidth - element.clientWidth
        if (overflow > 2) fail(selector, 'scrollWidth-clientWidth', `${overflow}px`, '<=2px')
      }

      const sidebar = first('#sidebar')
      if (isDesktop && sidebar) {
        const width = rect(sidebar)?.width
        if (width == null || width < 256 || width > 272)
          fail('#sidebar', 'width', `${width ?? 'missing'}px`, '256px-272px')
        if (!first('#sidebar .sidebar-collapse-toggle'))
          fail('#sidebar .sidebar-collapse-toggle', 'presence', 'missing', 'required')
        const avatar = first('#sidebar .profile-avatar')
        if (!avatar || !avatar.getAttribute('src') || avatar.naturalWidth <= 0)
          fail('#sidebar .profile-avatar', 'image', avatar ? 'not loaded' : 'missing', 'loaded profile image')
        if (count('#sidebar .utility-links a[href*="/categories/"]'))
          fail('#sidebar .utility-links a[href*="/categories/"]', 'count', count('#sidebar .utility-links a[href*="/categories/"]'), '0')
        for (const toggle of document.querySelectorAll('#sidebar .taxonomy-toggle')) {
          if (toggle.textContent.trim())
            fail('#sidebar .taxonomy-toggle', 'visible text', toggle.textContent.trim(), 'empty')
          if (toggle.getAttribute('aria-expanded') !== 'true')
            fail('#sidebar .taxonomy-toggle', 'aria-expanded', toggle.getAttribute('aria-expanded'), 'true by default')
          const toggleRect = rect(toggle)
          if (!toggleRect || toggleRect.width < 24 || toggleRect.height < 24)
            fail('#sidebar .taxonomy-toggle', 'hit area', `${Math.round(toggleRect?.width ?? 0)}x${Math.round(toggleRect?.height ?? 0)}px`, '>=24x24px')
          const headingRect = rect(toggle.closest('.taxonomy-heading'))
          if (toggleRect && headingRect && Math.abs((toggleRect.top + toggleRect.height / 2) - (headingRect.top + headingRect.height / 2)) > 2)
            fail('#sidebar .taxonomy-toggle', 'vertical center', 'off-center', 'within 2px of root row center')
          const list = document.getElementById(toggle.getAttribute('aria-controls'))
          if (!list || list.hidden)
            fail('#sidebar .taxonomy-toggle', 'controlled list', list ? 'hidden' : 'missing', 'visible by default')
        }
        for (const link of document.querySelectorAll('#sidebar .taxonomy-link')) {
          const linkRect = rect(link)
          if (!link.textContent.trim())
            fail('#sidebar .taxonomy-link', 'text', 'empty', 'non-empty')
          if (!link.querySelector('.taxonomy-icon'))
            fail('#sidebar .taxonomy-link .taxonomy-icon', 'presence', 'missing', 'required')
          if (link.scrollWidth > link.clientWidth + 1)
            fail('#sidebar .taxonomy-link', 'scrollWidth-clientWidth', `${link.scrollWidth - link.clientWidth}px`, '<=1px')
          if (linkRect && linkRect.right > innerWidth + 2)
            fail('#sidebar .taxonomy-link', 'right edge', `${linkRect.right}px`, '<=viewport')
        }
        assertReadableTaxonomyLabels()
        const active = first('#sidebar .nav-item.active a.nav-link')
        if (active) {
          const activeRect = rect(active)
          const activeStyle = style(active)
          const height = activeRect?.height
          const indicator = cssNumber(activeStyle?.borderLeftWidth)
          if (height == null || height < 36 || height > 48)
            fail('#sidebar .nav-item.active a.nav-link', 'height', `${height ?? 'missing'}px`, '36px-48px')
          if (indicator != null && indicator > 3)
            fail('#sidebar .nav-item.active a.nav-link', 'border-left-width', `${indicator}px`, '<=3px')
        }
      }

      const topbar = first('#topbar-wrapper')
      if (topbar) {
        const height = rect(topbar)?.height
        if (height == null || height < 48 || height > 60)
          fail('#topbar-wrapper', 'height', `${height ?? 'missing'}px`, '48px-60px')
      }

      if (route === 'home') {
        if (!hasText('#featured-title') || first('#featured-title')?.textContent.trim() !== '주요 카테고리')
          fail('#featured-title', 'text', first('#featured-title')?.textContent.trim() || 'empty', '주요 카테고리')
        if (document.body.textContent.includes('작업 흐름'))
          fail('body', 'text', '작업 흐름', 'absent')
        const featuredLinks = [...document.querySelectorAll('.home-featured li a')]
        if (featuredLinks.length < 3)
          fail('.home-featured a', 'count', featuredLinks.length, '>=3')
        for (const link of featuredLinks.slice(0, 3)) {
          if (!link.textContent.includes('›'))
            fail('.home-featured a', 'category path', link.textContent.trim(), 'two-level path with ›')
          if (!link.querySelector('i'))
            fail('.home-featured a i', 'presence', 'missing', 'required')
        }
        assertTextFits('.home-featured a span')

        if (!count('#post-list .post-preview'))
          fail('#post-list .post-preview', 'count', '0', '>0')
        if (!hasText('#post-list .post-title'))
          fail('#post-list .post-title', 'text', 'empty', 'non-empty')
        if (count('#panel-wrapper'))
          fail('#panel-wrapper', 'count', count('#panel-wrapper'), '0 on home')

        const list = first('#post-list')
        if (isDesktop && list) {
          const width = rect(list)?.width
          if (width == null || width < 760 || width > 880)
            fail('#post-list', 'width', `${width ?? 'missing'}px`, '760px-880px')
        }

        for (const row of document.querySelectorAll('#post-list .post-preview')) {
          const rowStyle = style(row)
          const rowRadius = cssNumber(rowStyle.borderTopLeftRadius)
          if (rowRadius != null && rowRadius > 4)
            fail('#post-list .post-preview', 'border-radius', `${rowRadius}px`, '<=4px')
          if (rowStyle.boxShadow !== 'none')
            fail('#post-list .post-preview', 'box-shadow', rowStyle.boxShadow, 'none')
          const excerpt = row.querySelector('.post-excerpt')
          if (excerpt && !excerpt.textContent.trim())
            fail('#post-list .post-excerpt', 'text', 'empty', 'absent or non-empty')
        }
      }

      if (route === 'posts-catalog' || route === 'category-ros') {
        const rows = [...document.querySelectorAll('[data-post-row]')].filter(visible)
        if (!rows.length) fail('[data-post-row]', 'count', '0', '>0')
        for (const row of rows) {
          const title = row.querySelector('[data-post-title]')
          const date = row.querySelector('[data-post-date]')
          const categories = row.querySelector('[data-post-categories]')
          const excerpt = row.querySelector('.post-row-excerpt')
          if (!title?.textContent.trim()) fail('[data-post-title]', 'text', 'empty', 'non-empty')
          if (!date?.textContent.trim()) fail('[data-post-date]', 'text', 'empty', 'non-empty')
          if (!excerpt?.textContent.trim()) fail('.post-row-excerpt', 'text', excerpt ? 'empty' : 'missing', 'non-empty body preview')
          if (categories) {
            const levels = categories.textContent.split('/').filter((part) => part.trim()).length
            if (levels > 2) fail('[data-post-categories]', 'levels', levels, '<=2')
          }
          const rowBox = rect(row.querySelector('a'))
          const metaBox = rect(row.querySelector('.post-row-meta'))
          if (isDesktop && rowBox && metaBox && metaBox.right > rowBox.right + 2)
            fail('.post-row-meta', 'right edge', `${metaBox.right}px`, `<=${rowBox.right}px`)
        }
        assertTextFits('[data-post-title]')
        assertTextFits('[data-post-categories]')
      }

      if (route === 'posts-catalog') {
        const catalog = first('[data-posts-catalog]')
        const items = [...document.querySelectorAll('[data-catalog-item]')]
        if (!catalog) fail('[data-posts-catalog]', 'presence', 'missing', 'required')
        if (!items.length) fail('[data-catalog-item]', 'count', '0', '>0')
        if (count('.posts-catalog .highlight'))
          fail('.posts-catalog .highlight', 'count', count('.posts-catalog .highlight'), '0 code-block wrappers')
        if (!hasText('.catalog-title'))
          fail('.catalog-title', 'text', 'empty', 'non-empty')

        const visibleItems = items.filter((item) => !item.hidden).length
        if (visibleItems !== Math.min(15, items.length))
          fail('[data-catalog-item]:not([hidden])', 'count', visibleItems, `first page ${Math.min(15, items.length)}`)

        const pager = first('[data-posts-pager]')
        if (items.length > 15) {
          const next = first('[data-page-next]')
          const prev = first('[data-page-prev]')
          const status = first('[data-page-status]')
          if (!pager) fail('[data-posts-pager]', 'presence', 'missing', 'required when item count > 15')
          if (!next || next.disabled) fail('[data-page-next]', 'enabled', next ? String(next.disabled) : 'missing', 'enabled')
          if (!prev || !prev.disabled) fail('[data-page-prev]', 'disabled', prev ? String(prev.disabled) : 'missing', 'disabled on first page')
          if (!status?.textContent.trim().startsWith('1 / '))
            fail('[data-page-status]', 'text', status?.textContent.trim() || 'empty', 'starts with 1 /')
          next?.click()
          if (status && !status.textContent.trim().startsWith('2 / '))
            fail('[data-page-status]', 'text after next', status.textContent.trim(), 'starts with 2 /')
          if (prev?.disabled)
            fail('[data-page-prev]', 'disabled after next', String(prev.disabled), 'false')
          const secondPageVisible = items.filter((item) => !item.hidden).length
          if (!secondPageVisible || secondPageVisible > 15)
            fail('[data-catalog-item]:not([hidden])', 'count after next', secondPageVisible, '1-15')
        }
      }

      if (route === 'categories') {
        if (!count('#main-wrapper .categories'))
          fail('.categories', 'count', '0', '>0')
        for (const card of document.querySelectorAll('#main-wrapper .categories')) {
          const cardStyle = style(card)
          const radius = cssNumber(cardStyle.borderTopLeftRadius)
          if (radius != null && radius > 8)
            fail('.categories', 'border-radius', `${radius}px`, '<=8px')
          if (cardStyle.boxShadow !== 'none')
            fail('.categories', 'box-shadow', cardStyle.boxShadow, 'none')
          const header = card.querySelector('.card-header')
          const headerRect = rect(header)
          if (headerRect && headerRect.height > 72)
            fail('.categories .card-header', 'height', `${headerRect.height}px`, '<=72px')
          const countNode = card.querySelector('.card-header .text-muted.small')
          if (countNode && style(countNode).display === 'block')
            fail('.categories .card-header .text-muted.small', 'display', 'block', 'inline or inline-block')
        }
      }

      if (route === 'category-ros') {
        if (!hasText('#page-category h1'))
          fail('#page-category h1', 'text', 'empty', 'non-empty')
        if (count('#page-category .dash'))
          fail('#page-category .dash', 'count', count('#page-category .dash'), '0')
      }

      if (route === 'archives') {
        if (!count('#archives .year')) fail('#archives .year', 'count', '0', '>0')
        if (!count('#archives li a')) fail('#archives li a', 'count', '0', '>0')
        const archives = first('#archives')
        const archivesStyle = style(archives)
        if (archives) {
          const radius = cssNumber(archivesStyle.borderTopLeftRadius)
          if (radius != null && radius > 4)
            fail('#archives', 'border-radius', `${radius}px`, '<=4px')
          if (archivesStyle.boxShadow !== 'none')
            fail('#archives', 'box-shadow', archivesStyle.boxShadow, 'none')
        }
      }

      if (route === 'tags') {
        if (!count('#tags .tag')) fail('#tags .tag', 'count', '0', '>0')
        for (const tag of document.querySelectorAll('#tags .tag')) {
          const tagStyle = style(tag)
          const radius = cssNumber(tagStyle.borderTopLeftRadius)
          if (radius != null && radius > 8)
            fail('#tags .tag', 'border-radius', `${radius}px`, '<=8px')
          if (tagStyle.boxShadow !== 'none')
            fail('#tags .tag', 'box-shadow', tagStyle.boxShadow, 'none')
        }
      }

      if (route === 'search-open') {
        const wrapper = first('#search-result-wrapper')
        const search = first('#search')
        const input = first('#search-input')
        if (!wrapper) fail('#search-result-wrapper', 'presence', 'missing', 'required')
        if (!input) fail('#search-input', 'presence', 'missing', 'required')
        if (wrapper && wrapper.classList.contains('d-none'))
          fail('#search-result-wrapper', 'opened state', 'hidden', 'visible after query')
        if (!count('#search-results article'))
          fail('#search-results article', 'count', '0', '>0 after query')
        if (search) {
          const searchStyle = style(search)
          const radius = cssNumber(searchStyle.borderTopLeftRadius)
          if (radius != null && radius > 16)
            fail('#search', 'border-radius', `${radius}px`, '<=16px')
          if (searchStyle.boxShadow !== 'none')
            fail('#search', 'box-shadow', searchStyle.boxShadow, 'none')
        }
      }

      if (route === 'post') {
        const article = first('article.post-article')
        if (!article) {
          fail('article.post-article', 'presence', 'missing', 'required')
        } else {
          if (!hasText('article.post-article > header h1'))
            fail('article.post-article > header h1', 'text', 'empty', 'non-empty')
          if (!hasText('article.post-article > .content'))
            fail('article.post-article > .content', 'text', 'empty', 'non-empty')
          if (count('.post-tail-section') !== 2)
            fail('.post-tail-section', 'count', count('.post-tail-section'), '2')
          if (!count('.post-tail-section--sequence .post-tail-list li'))
            fail('.post-tail-section--sequence .post-tail-list li', 'count', '0', '>0')
          if (!count('.post-tail-section--latest .post-tail-list li'))
            fail('.post-tail-section--latest .post-tail-list li', 'count', '0', '>0')

          const header = first('article.post-article > header')
          const headerStyle = style(header)
          const radius = cssNumber(headerStyle?.borderTopLeftRadius)
          if (radius != null && radius > 8)
            fail('article.post-article > header', 'border-radius', `${radius}px`, '<=8px')
          if (headerStyle?.boxShadow !== 'none')
            fail('article.post-article > header', 'box-shadow', headerStyle?.boxShadow, 'none')

          const content = first('article.post-article > .content')
          if (isDesktop && content) {
            const width = rect(content)?.width
            if (width == null || width < 760 || width > 940)
              fail('article.post-article > .content', 'width', `${width ?? 'missing'}px`, '760px-940px')
          }

          for (const selector of ['.highlight', '.table-wrapper']) {
            for (const element of document.querySelectorAll(selector)) {
              const elementStyle = style(element)
              if (!['auto', 'scroll'].includes(elementStyle.overflowX))
                fail(selector, 'overflow-x', elementStyle.overflowX, 'auto or scroll')
              if (elementStyle.maxWidth === 'none')
                fail(selector, 'max-width', elementStyle.maxWidth, '<=100%')
              if (elementStyle.boxShadow !== 'none')
                fail(selector, 'box-shadow', elementStyle.boxShadow, 'none')
            }
          }

          const widthButtons = [...document.querySelectorAll('.post-width-control button[data-post-width]')]
          const labels = widthButtons.map((button) => button.textContent.trim())
          const expectedLabels = ['좁게', '기본', '넓게']
          if (labels.join('|') !== expectedLabels.join('|'))
            fail('[data-post-width]', 'labels', labels.join('|'), expectedLabels.join('|'))
          const expectedNames = ['760', '900', '1100']
          widthButtons.forEach((button, index) => {
            const name = button.getAttribute('aria-label') || ''
            if (!name.includes(expectedNames[index]))
              fail('[data-post-width]', 'aria-label', name || 'missing', `contains ${expectedNames[index]}`)
          })
          const selectedDefault = first('.post-width-control button[data-post-width="900"]')
          if (selectedDefault?.getAttribute('aria-pressed') !== 'true')
            fail('[data-post-width="900"]', 'aria-pressed', selectedDefault?.getAttribute('aria-pressed'), 'true by default')

          for (const frame of document.querySelectorAll('iframe[src*="youtube"], iframe[src*="youtu.be"]')) {
            const media = frame.closest('[data-media], .media-embed, .embed, figure') || frame.parentElement
            const fallback = media?.querySelector('[data-media-fallback], .media-fallback, .embed-fallback, [role="status"]')
            if (!fallback || !style(fallback)?.display || style(fallback).display === 'none' || !fallback.textContent.trim())
              fail('iframe[src*="youtube"]', 'fallback', 'missing or hidden', 'visible media fallback')
          }

          for (const button of document.querySelectorAll('.code-header button')) {
            button.focus()
            const buttonStyle = style(button)
            if (!button.matches(':focus-visible'))
              fail('.code-header button', 'focus-visible', 'false', 'true')
            if (buttonStyle.outlineStyle === 'none' && buttonStyle.boxShadow === 'none')
              fail('.code-header button', 'focus indicator', 'none', 'visible outline or shadow')
          }
        }
      }

      return {
        route,
        viewport: viewportName,
        viewportPixels: viewportLabel,
        url: location.href,
        metrics: {
          document: { scrollWidth: root.scrollWidth, clientWidth: root.clientWidth },
          body: body ? { scrollWidth: body.scrollWidth, clientWidth: body.clientWidth } : null,
          main: main ? { scrollWidth: main.scrollWidth, clientWidth: main.clientWidth } : null,
        },
        failures,
      }
    },
    { route: routeName, viewportName: viewport, screenshotPath: screenshot },
  )
}

async function checkPostWidthPersistence(page, routeName, viewportName, screenshot) {
  if (routeName !== 'post' || viewportName !== 'desktop') return []
  const failures = []
  const fail = (selector, property, observed, budget) =>
    failures.push({
      route: routeName,
      viewport: `${page.viewportSize()?.width}x${page.viewportSize()?.height}`,
      selector,
      property,
      observed: String(observed),
      budget,
      screenshot,
    })
  const readState = () =>
    page.evaluate(() => ({
      stored: localStorage.getItem('knowgyu.postWidth'),
      applied: document.querySelector('[data-post-width-root]')?.style.getPropertyValue('--post-content-width'),
      pressed: document.querySelector('.post-width-control button[aria-pressed="true"]')?.dataset.postWidth,
    }))

  for (const width of ['760', '900', '1100']) {
    const button = page.locator(`.post-width-control button[data-post-width="${width}"]`)
    if (!(await button.count())) {
      fail(`[data-post-width="${width}"]`, 'presence', 'missing', 'required')
      continue
    }
    await button.click()
    let state = await readState()
    if (state.stored !== width) fail('localStorage knowgyu.postWidth', 'value after click', state.stored, width)
    if (state.applied !== `${width}px`) fail('[data-post-width-root]', '--post-content-width after click', state.applied || 'missing', `${width}px`)
    if (state.pressed !== width) fail('[data-post-width]', 'aria-pressed after click', state.pressed || 'missing', width)

    await page.reload({ waitUntil: 'networkidle' })
    state = await readState()
    if (state.stored !== width) fail('localStorage knowgyu.postWidth', 'value after reload', state.stored, width)
    if (state.applied !== `${width}px`) fail('[data-post-width-root]', '--post-content-width after reload', state.applied || 'missing', `${width}px`)
    if (state.pressed !== width) fail('[data-post-width]', 'aria-pressed after reload', state.pressed || 'missing', width)
  }

  return failures
}

async function checkSidebarCollapse(page, routeName, viewportName, screenshot) {
  if (viewportName !== 'desktop') return []
  const failures = []
  const fail = (selector, property, observed, budget) =>
    failures.push({
      route: routeName,
      viewport: `${page.viewportSize()?.width}x${page.viewportSize()?.height}`,
      selector,
      property,
      observed: String(observed),
      budget,
      screenshot,
    })

  const button = page.locator('#sidebar .sidebar-collapse-toggle')
  if (!(await button.count())) {
    fail('#sidebar .sidebar-collapse-toggle', 'presence', 'missing', 'required')
    return failures
  }

  const before = await page.locator('#main-wrapper').boundingBox()
  await button.click()
  await button.evaluate((element) => element.blur())
  await page.mouse.move(500, 500)
  const stateScreenshot = (state) => screenshot.replace(/\.png$/, `-sidebar-${state}.png`)
  await page.waitForFunction(() => document.querySelector('#sidebar')?.getBoundingClientRect().width <= 80, null, { timeout: 1000 }).catch(() => {})
  const collapsed = await page.evaluate(() => ({
    pressed: document.querySelector('#sidebar .sidebar-collapse-toggle')?.getAttribute('aria-pressed'),
    stored: localStorage.getItem('knowgyu:sidebar-collapsed'),
    width: document.querySelector('#sidebar')?.getBoundingClientRect().width,
  }))
  await page.screenshot({ path: stateScreenshot('collapsed') })
  const after = await page.locator('#main-wrapper').boundingBox()
  if (collapsed.pressed !== 'true') fail('#sidebar .sidebar-collapse-toggle', 'aria-pressed', collapsed.pressed, 'true')
  if (collapsed.stored !== 'true') fail('localStorage knowgyu:sidebar-collapsed', 'value', collapsed.stored, 'true')
  if (collapsed.width == null || collapsed.width > 80) fail('#sidebar', 'collapsed width', `${collapsed.width ?? 'missing'}px`, '<=80px')
  if (before && after && Math.abs(before.x - after.x) > 1) fail('#main-wrapper', 'x after collapse', `${after.x}px`, `${before.x}px +/-1`)
  await page.reload({ waitUntil: 'networkidle' })
  await page.mouse.move(500, 500)
  await page.waitForFunction(() => document.querySelector('#sidebar')?.getBoundingClientRect().width <= 80, null, { timeout: 1000 }).catch(() => {})
  const restored = await page.evaluate(() => ({
    pressed: document.querySelector('#sidebar .sidebar-collapse-toggle')?.getAttribute('aria-pressed'),
    stored: localStorage.getItem('knowgyu:sidebar-collapsed'),
    width: document.querySelector('#sidebar')?.getBoundingClientRect().width,
  }))
  if (restored.pressed !== 'true') fail('#sidebar .sidebar-collapse-toggle', 'aria-pressed after reload', restored.pressed, 'true')
  if (restored.stored !== 'true') fail('localStorage knowgyu:sidebar-collapsed', 'value after reload', restored.stored, 'true')
  if (restored.width == null || restored.width > 80) fail('#sidebar', 'collapsed width after reload', `${restored.width ?? 'missing'}px`, '<=80px')

  await page.locator('#sidebar').hover()
  await page.waitForFunction(() => document.querySelector('#sidebar')?.getBoundingClientRect().width >= 256, null, { timeout: 1000 }).catch(() => {})
  const hoverWidth = await page.locator('#sidebar').evaluate((element) => element.getBoundingClientRect().width)
  await page.screenshot({ path: stateScreenshot('hover') })
  if (hoverWidth < 256 || hoverWidth > 272) fail('#sidebar:hover', 'width', `${hoverWidth}px`, '256px-272px')
  await button.focus()
  await page.waitForFunction(() => document.querySelector('#sidebar')?.getBoundingClientRect().width >= 256, null, { timeout: 1000 }).catch(() => {})
  const focusWidth = await page.locator('#sidebar').evaluate((element) => element.getBoundingClientRect().width)
  await page.screenshot({ path: stateScreenshot('focus') })
  if (focusWidth < 256 || focusWidth > 272) fail('#sidebar:focus-within', 'width', `${focusWidth}px`, '256px-272px')

  return failures
}

async function checkSidebarScrollPersistence(page, routeName, viewportName, screenshot) {
  if (routeName !== 'home' || viewportName !== 'desktop') return []
  const failures = []
  const fail = (selector, property, observed, budget) =>
    failures.push({ route: routeName, viewport: `${page.viewportSize()?.width}x${page.viewportSize()?.height}`, selector, property, observed: String(observed), budget, screenshot })
  const target = page.locator('#sidebar .taxonomy-tree a[href*="/categories/ros/"]').first()
  if (!(await target.count())) {
    fail('#sidebar .taxonomy-tree a[href*="/categories/ros/"]', 'presence', 'missing', 'required')
    return failures
  }
  await page.locator('#sidebar .sidebar-nav').evaluate((element) => {
    element.scrollTop = Math.min(160, Math.max(40, element.scrollHeight - element.clientHeight))
    element.dispatchEvent(new Event('scroll'))
  })
  await Promise.all([page.waitForNavigation({ waitUntil: 'networkidle' }), target.click()])
  const restored = await page.locator('#sidebar .sidebar-nav').evaluate((element) => ({
    scrollTop: element.scrollTop,
    stored: localStorage.getItem('knowgyu:sidebar-scroll'),
  }))
  if (restored.scrollTop < 32) fail('#sidebar .sidebar-nav', 'scrollTop after category navigation', restored.scrollTop, '>=32px')
  if (Number(restored.stored) < 32) fail('localStorage knowgyu:sidebar-scroll', 'stored scroll position', restored.stored, '>=32px')
  return failures
}

async function resolvePostPath(page, baseURL, fallback) {
  const response = await page.goto(new URL('/', baseURL).href, { waitUntil: 'domcontentloaded' })
  if (!response?.ok()) return fallback
  const link = page
    .locator('#post-list a.post-preview')
    .filter({ hasText: /하이퍼파라미터/ })
    .first()
  if (await link.count()) return (await link.getAttribute('href')) || fallback
  return fallback
}

async function main() {
  fs.mkdirSync(outputDir, { recursive: true })
  const browser = await chromium.launch({ headless: true })
  const baseURL = process.env.BASE_URL || 'https://knowgyu.github.io'
  let failed = 0

  for (const route of selected('VISUAL_QA_ROUTES', ROUTES)) {
    for (const mode of selected('VISUAL_QA_MODES', MODES)) {
      for (const viewport of selected('VISUAL_QA_VIEWPORTS', VIEWPORTS)) {
        const label = `${route.name} ${mode} ${viewport.name}`
        const context = await browser.newContext({
          viewport: { width: viewport.width, height: viewport.height },
          colorScheme: mode,
        })
        const page = await context.newPage()
        let routePath = route.path
        try {
          if (route.name === 'post' && process.env.POST_PATH) routePath = process.env.POST_PATH
          if (route.name === 'post' && !process.env.POST_PATH) routePath = await resolvePostPath(page, baseURL, route.path)

          let navigationError
          const response = await page
            .goto(new URL(routePath, baseURL).href, { waitUntil: 'networkidle' })
            .catch((error) => {
              navigationError = error
              return null
            })
          const image = screenshotPath(route.name, mode, viewport.name)
          if (!response?.ok()) {
            await page.screenshot({ path: image, fullPage: true })
            const failure = {
              route: route.name,
              viewport: `${viewport.width}x${viewport.height}`,
              selector: route.path,
              property: 'navigation',
              observed: response?.status() ?? navigationError?.message ?? 'unreachable',
              budget: 'HTTP 2xx',
              screenshot: image,
            }
            fs.writeFileSync(
              evidencePath(route.name, mode, viewport.name),
              `${JSON.stringify({ route: route.name, viewport: viewport.name, url: routePath, failures: [failure] }, null, 2)}\n`,
            )
            throw new Error(formatFailure(failure))
          }
          if (route.name === 'search-open') {
            await page.locator('#search-trigger').click().catch(() => {})
            await page.locator('#search-input').fill('ROS')
            await page.waitForFunction(() => {
              const wrapper = document.querySelector('#search-result-wrapper')
              const results = document.querySelector('#search-results')
              return wrapper && !wrapper.classList.contains('d-none') && results && results.textContent.trim().length > 0
            }, null, { timeout: 5000 })
          }
          await page.screenshot({ path: image, fullPage: true })
          const evidence = await collectEvidence(page, route.name, viewport.name, image)
          if (route.name === 'home' && mode === 'light')
            evidence.failures.push(...await checkSidebarScrollPersistence(page, route.name, viewport.name, image))
          if (route.name === 'home' && mode === 'light')
            evidence.failures.push(...await checkSidebarCollapse(page, route.name, viewport.name, image))
          if (route.name === 'post' && mode === 'light')
            evidence.failures.push(...await checkPostWidthPersistence(page, route.name, viewport.name, image))
          fs.writeFileSync(evidencePath(route.name, mode, viewport.name), `${JSON.stringify(evidence, null, 2)}\n`)
          if (evidence.failures.length) throw new Error(evidence.failures.map(formatFailure).join('\n'))
          console.log(`PASS ${label}`)
        } catch (error) {
          failed += 1
          console.error(`FAIL ${label}`)
          console.error(error instanceof Error ? error.message : error)
        } finally {
          await context.close()
        }
      }
    }
  }

  await browser.close()
  if (failed) process.exitCode = 1
}

if (require.main === module) main().catch((error) => {
  console.error(error)
  process.exitCode = 1
})

module.exports = { collectEvidence, resolvePostPath }
