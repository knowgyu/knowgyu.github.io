const fs = require('node:fs')
const path = require('node:path')
const { chromium } = require('playwright')

const ROUTES = [
  { name: 'home', path: '/' },
  { name: 'categories', path: '/categories/' },
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
        if (width == null || width < 202 || width > 214)
          fail('#sidebar', 'width', `${width ?? 'missing'}px`, '202px-214px')
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
            if (width == null || width < 720 || width > 860)
              fail('article.post-article > .content', 'width', `${width ?? 'missing'}px`, '720px-860px')
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
