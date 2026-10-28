// Builds src/html/styles.css: only the Tailwind / daisyUI classes the web
// pages use, so the pages look right without internet (e.g. in hotspot mode).
// Regenerate with scripts/build_css.sh after adding new classes to a page.
module.exports = {
    content: ["../../src/html/*.html"],
    // dark: classes follow the design chosen on the page (data-theme), or
    // the device when the page is on automatic
    darkMode: ["variant", [
        "@media (prefers-color-scheme: dark) { &:not([data-theme=light], [data-theme=light] *) }",
        "&:is([data-theme=dark], [data-theme=dark] *)",
    ]],
    plugins: [require("daisyui")],
    daisyui: {
        themes: ["light", "dark"],
        darkTheme: "dark",
        logs: false,
    },
};
