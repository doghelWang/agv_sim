// 在仓库根目录执行: npx tailwindcss@3 -c web/css/tailwind.config.js -i web/css/src.css -o web/css/app.css --minify
// (app.css 是离线编译产物，改了网页里的样式类之后要重新生成)
module.exports = {
  content: ['web/**/*.{html,js}', 'model_editor.html'],
  // 代码里拼出来的颜色类 (如 bg-${color}-50) 扫描不到，固定保留
  safelist: ['select-all', 'h-[60vh]', { pattern: /^(bg|text|border)-(blue|purple|emerald|amber|rose|slate|cyan|pink)-(50|100|200|600|700)$/ }],
  theme: { extend: {} }, plugins: [] };
