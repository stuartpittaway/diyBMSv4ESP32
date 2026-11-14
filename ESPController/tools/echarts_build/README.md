# Custom ECharts build

Produces `web_src/echarts.min.js`, a cut-down build of [Apache ECharts](https://echarts.apache.org)
containing only the chart types and components the web UI uses. This saves ESP32 flash space
compared with the stock builds.

This is **not** part of the PlatformIO build. Run it by hand when upgrading ECharts or changing
the component list, then commit the regenerated `web_src/echarts.min.js`. Normal firmware builds
need no Node.js and no network access.

## What is included, and why

See the table at the top of [`entry.js`](entry.js). That file is the single source of truth for
which ECharts items are bundled.

## Usage

Requires Node.js 18 or later.

```sh
cd ESPController/tools/echarts_build
npm ci          # installs the exact versions pinned in package-lock.json
npm run build   # writes ../../web_src/echarts.min.js
```

Then rebuild the firmware and check the graphs (including the 3D view) in a browser.

## Upgrading ECharts

1. Change the `echarts` version in `package.json`.
2. Run `npm install` (updates `package-lock.json`), then `npm run build`.
3. Test the web UI, then commit `package.json`, `package-lock.json` and `web_src/echarts.min.js`.

## Adding a chart type or component

If `pagecode.js` starts using something new (for example a `title` or a `pie` series), add the
import and the `echarts.use([...])` entry in `entry.js`, add a row to the table explaining what
it is used for, and rebuild. Options for components that are not included are silently ignored
by ECharts, so a missing component shows up as a missing feature rather than an error.

The 3D view (`bar3D`, `grid3D`, ...) comes from `echarts-gl`, which `default.htm` loads from the
jsDelivr CDN. Keep its version compatible with the ECharts major version used here.
