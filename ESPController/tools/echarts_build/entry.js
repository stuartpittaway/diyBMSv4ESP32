/*
  Custom ECharts build for the DIYBMS web UI (output: web_src/echarts.min.js)

  Only the parts of ECharts that pagecode.js actually uses are included, to save
  ESP32 flash space. If pagecode.js starts using a new chart type or option
  component, add it below and re-run the build (see README.md).

  Item                          Used for (pagecode.js)
  ----------------------------  ------------------------------------------------------------
  LineChart                     Cell voltage lines on the main graph (g1) and the line chart (g3)
  BarChart                      Bypass / temperature / other bars on the main graph (g1)
  GridSimpleComponent           x/y axes and the two stacked grids (gridIndex 0/1) on the main graph
  TooltipComponent              Hover tooltips, including axisPointer type 'cross' (pulls in AxisPointer)
  MarkLineComponent             The dotted "avg" line and the dashed per-bank divider lines
  VisualMapContinuousComponent  Colour mapping of the bars in the 3D view (g2)
  CanvasRenderer                Drawing the charts (ECharts default renderer)

  Deliberately NOT included:
  - PieChart, CandlestickChart  No series of these types exist
  - TitleComponent              No chart sets a title
  - MarkPointComponent          Not used
  - LegendComponent             The only legend is { show: false } and nothing toggles series

  Not part of this build: bar3D / grid3D / xAxis3D / yAxis3D / zAxis3D come from
  echarts-gl, which default.htm loads separately from the jsDelivr CDN.
*/

import * as echarts from 'echarts/core';
import { LineChart, BarChart } from 'echarts/charts';
import {
    GridSimpleComponent,
    TooltipComponent,
    MarkLineComponent,
    VisualMapContinuousComponent
} from 'echarts/components';
import { CanvasRenderer } from 'echarts/renderers';

echarts.use([
    LineChart,
    BarChart,
    GridSimpleComponent,
    TooltipComponent,
    MarkLineComponent,
    VisualMapContinuousComponent,
    CanvasRenderer
]);

// Expose the full core API (init, use, graphic, util, ...) as the global "echarts".
// echarts-gl relies on this global to register its 3D chart types.
export * from 'echarts/core';
