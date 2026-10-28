/*
 * Browser test of the web pages against tests/ai_web_mock.py (no hardware):
 * every settings page, recipes, statistics and CSV, the options of
 * feature_config, unsaved settings, error log, theme, events and the AI page.
 *
 * Usage
 *
 *     make -C tests/sim trickle_export
 *     python3 tests/ai_web_mock.py 8099 &
 *     node tests/web/ui_test.js http://localhost:8099
 *
 * Needs Playwright with Chromium (npm install playwright; npx playwright install chromium).
 */
const { chromium } = require('playwright');

const BASE = process.argv[2] || 'http://localhost:8099';
let failures = 0;

function check(name, ok, detail) {
    console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}${detail !== undefined ? ' (' + detail + ')' : ''}`);
    if (!ok) failures += 1;
}

(async () => {
    const browser = await chromium.launch();
    const page = await browser.newPage({ viewport: { width: 390, height: 844 }, acceptDownloads: true });
    const errors = [];
    page.on('pageerror', e => errors.push(e.message));
    page.on('dialog', d => d.accept());

    // Trickler page ------------------------------------------------------------
    await page.goto(BASE + '/');
    await page.waitForTimeout(1500);
    const summary = await page.textContent('#sessionSummary');
    check('session statistics', /charges/.test(summary), summary.slice(0, 60));
    check('statistics per load', await page.locator('#sessionGroups tr').count() >= 1);
    check('recipe list on the trickler page', await page.isVisible('#recipeSelect'));
    await page.selectOption('#recipeSelect', '0');
    await page.waitForTimeout(500);
    check('recipe fills the charge weight', await page.inputValue('#chargeWeightInput') === '42.000');

    const [download] = await Promise.all([page.waitForEvent('download'), page.click('text=Export CSV')]);
    const csv = require('fs').readFileSync(await download.path(), 'utf8');
    const lines = csv.trim().split('\n');
    check('CSV export', lines.length > 2 && lines[0].includes('"Target (gr)"') && lines[1].includes(';'), lines.length + ' lines');

    check('update badge (device found a newer build)', await page.isVisible('#settingsUpdateBadge'));

    // Pan removed event -> dialog
    await page.evaluate(b => fetch(b + '/__event?e=8&s=4'), BASE);
    await page.waitForTimeout(1500);
    check('pan removed dialog', await page.evaluate(() => document.getElementById('panRemovedDialog').open));
    await page.keyboard.press('Escape');

    // Settings -------------------------------------------------------------------
    await page.click('#settingsNavButton');
    await page.waitForTimeout(800);
    const pages = ['settings-charge-mode', 'settings-recipes', 'settings-profile', 'settings-ai-trickle', 'settings-scale',
                   'settings-motors', 'settings-servo-gate', 'settings-display', 'settings-wireless',
                   'settings-system-control', 'settings-firmware'];
    for (const id of pages) {
        await page.evaluate(id => onSettingsLinkClicked(id), id);
        await page.waitForTimeout(400);
        check(`page ${id}`, await page.isVisible('#' + id));
    }

    await page.evaluate(() => onSettingsLinkClicked('settings-charge-mode'));
    await page.waitForTimeout(600);
    check('safety options loaded', await page.inputValue('#chargeSafetyForm [name=f0]') === 'true' &&
                                   await page.inputValue('#chargeSafetyForm [name=f2]') === '0.5');
    check('fine reverse setting on Charging', await page.inputValue('#chargeModeConfigForm [name=c17]') === 'auto');

    // Apply without saving -> unsaved banner, Save now -> gone
    await page.selectOption('#chargeSafetyForm [name=f1]', 'false');
    await page.click('#settings-charge-mode .page-apply-btn');
    await page.click('#dialogApplySettingsButton');
    await page.waitForTimeout(1200);
    await page.keyboard.press('Escape');
    check('unsaved banner after Apply', await page.isVisible('#unsavedBanner'), await page.textContent('#unsavedText'));
    await page.click('#unsavedBanner button');
    await page.waitForTimeout(800);
    check('banner gone after Save now', !(await page.isVisible('#unsavedBanner')));

    // Controller switch hides the PID fields
    await page.selectOption('#chargeControllerForm [name=a0]', 'true');
    await page.click('#settings-charge-mode .page-apply-btn');
    await page.click('#dialogSaveToEepromButton');
    await page.waitForTimeout(1200);
    await page.keyboard.press('Escape');
    check('PID thresholds hidden with AI Trickle', !(await page.isVisible('#chargeModeConfigForm [name=c5]')));

    await page.evaluate(() => onSettingsLinkClicked('settings-motors'));
    await page.waitForTimeout(500);
    check('standstill current option', await page.inputValue('#motorFeatureForm [name=f4]') === '30');

    await page.evaluate(() => onSettingsLinkClicked('settings-display'));
    await page.waitForTimeout(500);
    check('dimming options', await page.inputValue('#displayFeatureForm [name=f6]') === '300');

    await page.evaluate(() => onSettingsLinkClicked('settings-system-control'));
    await page.waitForTimeout(700);
    check('error log entries', await page.locator('#errorLogList .alert').count() === 2,
          await page.textContent('#errorLogSummary'));

    // Theme toggle
    await page.click('button[aria-label="Light or dark design"]');
    const theme1 = await page.evaluate(() => document.documentElement.getAttribute('data-theme'));
    await page.click('button[aria-label="Light or dark design"]');
    const theme2 = await page.evaluate(() => document.documentElement.getAttribute('data-theme'));
    check('theme toggle', theme1 === 'light' && theme2 === 'dark', `${theme1} -> ${theme2}`);
    await page.click('button[aria-label="Light or dark design"]');

    // Old link of a merged page
    await page.goto(BASE + '/#settings-fine-motor');
    await page.waitForTimeout(800);
    check('old link opens the merged page', await page.isVisible('#settings-motors'));

    // Icons
    const icon = await page.evaluate(async b => (await fetch(b + '/apple-touch-icon.png')).status, BASE);
    check('home screen icon served', icon === 200);

    // AI page ----------------------------------------------------------------------
    await page.goto(BASE + '/ai');
    await page.waitForTimeout(2500);
    const session = await page.evaluate(async () => {
        const s = await sessionStatistics();
        return s ? Object.keys(s.per_load).length + ':' + s.charges.length : null;
    });
    check('session evaluation data for the assistant', session !== null, session);

    check('no script errors', errors.length === 0, errors.join(' | '));
    await browser.close();
    console.log(failures ? `${failures} check(s) failed` : 'All web checks passed');
    process.exit(failures ? 1 : 0);
})();
