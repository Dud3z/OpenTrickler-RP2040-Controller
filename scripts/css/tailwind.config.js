// Builds src/html/styles.css: only the Tailwind / daisyUI classes the web
// pages use, so the pages look right without internet (e.g. in hotspot mode).
// Regenerate with scripts/build_css.sh after adding new classes to a page.
module.exports = {
    content: ["../../src/html/*.html"],
    plugins: [require("daisyui")],
    daisyui: {
        themes: ["light", "dark"],
        darkTheme: "dark",
        logs: false,
    },
};
