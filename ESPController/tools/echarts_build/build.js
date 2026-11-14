#!/usr/bin/env node
/*
  Builds the custom ECharts bundle used by the web UI.

  Bundles entry.js with esbuild (tree-shaken to the components listed there),
  minifies with terser, and writes ../../web_src/echarts.min.js.

  Run manually when upgrading ECharts or changing the component list - this is
  NOT part of the PlatformIO build. See README.md.
*/

const fs = require('fs');
const path = require('path');
const zlib = require('zlib');
const esbuild = require('esbuild');
const { minify } = require('terser');

const ENTRY = path.join(__dirname, 'entry.js');
const OUTPUT = path.resolve(__dirname, '..', '..', 'web_src', 'echarts.min.js');

function packageVersion(name) {
    const pkg = path.join(__dirname, 'node_modules', name, 'package.json');
    return JSON.parse(fs.readFileSync(pkg, 'utf8')).version;
}

// List of items registered via echarts.use([...]) in entry.js, for the file header
function includedItems() {
    const src = fs.readFileSync(ENTRY, 'utf8');
    const match = src.match(/echarts\.use\(\[([^\]]+)\]\)/);
    if (!match) {
        throw new Error('Could not find echarts.use([...]) in entry.js');
    }
    return match[1].split(',').map(s => s.trim()).filter(Boolean);
}

async function main() {
    const version = packageVersion('echarts');
    const items = includedItems();

    const bundled = await esbuild.build({
        entryPoints: [ENTRY],
        bundle: true,
        format: 'iife',
        globalName: 'echarts',
        target: 'es2017',
        legalComments: 'none',
        define: { 'process.env.NODE_ENV': '"production"' },
        write: false,
        logLevel: 'warning'
    });

    const minified = await minify(bundled.outputFiles[0].text, {
        ecma: 2017,
        compress: { passes: 2, pure_getters: true },
        mangle: true,
        format: { comments: false }
    });

    const header = `/*! Apache ECharts ${version} | Apache License 2.0 | https://echarts.apache.org\n` +
        ` * Custom build for DIYBMS (tools/echarts_build): ${items.join(', ')} */\n`;
    const output = header + minified.code + '\n';

    fs.writeFileSync(OUTPUT, output);

    const gzipped = zlib.gzipSync(output, { level: 9 }).length;
    console.log(`ECharts ${version} -> ${path.relative(process.cwd(), OUTPUT)}`);
    console.log(`Included: ${items.join(', ')}`);
    console.log(`Size: ${output.length} bytes, ${gzipped} bytes gzipped (approx. flash usage)`);
}

main().catch(err => {
    console.error(err);
    process.exit(1);
});
