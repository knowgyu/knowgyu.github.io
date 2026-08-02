# Design guide

`DESIGN.md` is the canonical source of truth for the site's visual contract.
Use it before changing layout, colors, navigation, or interaction.

## File map

- `_sass/themes/_light.scss`, `_sass/themes/_dark.scss`: semantic color roles.
- `_sass/abstracts/_variables.scss`: shared dimensions and type variables.
- `_sass/base/`: document defaults and reading typography.
- `_sass/layout/_sidebar.scss`: desktop rail and mobile drawer.
- `_sass/layout/_topbar.scss`: breadcrumb, search, and mobile controls.
- `_sass/layout/_panel.scss`: optional post TOC panel.
- `_sass/pages/`: page-specific editorial treatment.
- `_layouts/default.html`: shell ownership and responsive columns.
- `_includes/sidebar.html`: navigation markup sourced from `_tabs`.

Keep route/content behavior intact. Prefer existing semantic tokens and the
smallest reversible change; update `DESIGN.md` when the contract changes.
