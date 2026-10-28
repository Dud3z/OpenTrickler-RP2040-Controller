"""
Mock of the controller's REST interface for developing the AI Trickle page
(src/html/ai_trickle.html) without hardware. Data comes from the powder
simulator (tests/sim/trickle_export).

Usage

    make -C tests/sim trickle_export
    python3 tests/ai_web_mock.py [port]

Then open http://localhost:8080/ai
"""

import hashlib
import json
import os
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse


SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
HTML_PATH = os.path.join(SCRIPT_DIR, '..', 'src', 'html', 'ai_trickle.html')
PORTAL_PATH = os.path.join(SCRIPT_DIR, '..', 'src', 'html', 'web_portal.html')
EXPORT_TOOL = os.path.join(SCRIPT_DIR, 'sim', 'trickle_export')


def load_export():
    output = subprocess.run([EXPORT_TOOL], capture_output=True, check=True, text=True).stdout
    return json.loads(output)


EXPORT = load_export()

STATE = {
    'config': {'a0': True, 'a1': 50, 'a2': True, 'a3': 8, 'a4': True, 'a5': 0.02, 'tol': 0.02, 'sp': 70,
               'as': [0.04, 0.6, 9.8, 70, 9.6], 'lim': [0.1, 5.0, 0.08, 2.0], 'pmx': [8.0, 2.0], 'acc': [62.5, 90.9],
               'est': 24.0, 'tg': 42.0,
               'pf': 0, 'pn': EXPORT['model']['pn'], 'rd': True, 'a7': 20, 'a8': 'auto', 'kw': 0.021, 'tg1': 0.011,
               'rv': [0.02, 0.012, 6, 6, 3, 0.02]},
    'model': EXPORT['model'],
    'history': EXPORT['history'],
    'trace': EXPORT['trace'],
    'learn': {'la': False, 'ld': 0, 'lt': 8},
    'series': {'sa': False, 'lad': False, 'sd': 0, 'n': 0, 's0': 0.0, 'step': 0.0, 'rep': 1, 'seq0': 0, 'tg': 42.0, 'err': ''},
    'diag': {'st': 0, 'el': 0.0, 'dur': 0, 'start': 0.0},
    'fw': {'board': 'pico2_w', 'ver': 'v2.0-5-g3428e5e', 'vcs': '3428e5e', 'build': 2, 'sup': True, 'part': 0, 'ab': True,
           'max': 2080768, 'st': 'idle', 'err': '', 'rx': 0, 'size': 0, 'sha': '', 'conf': False},
    'reboot_until': 0.0,
    'models': {},
    'requests': [],
    'selected': 0,
    'taps': 0,
}

NAMES = [EXPORT['model']['pn'], 'AR2209,gr', '8208XBR,gr', 'Benchmark2,gr', 'Profile4', 'Profile5', 'Profile6', 'Profile7']

PROFILE = {'pf': 0, 'p0': 0, 'p1': 0, 'p2': EXPORT['model']['pn'], 'p3': 0.05, 'p4': 0.0, 'p5': 0.3, 'p6': 0.1,
           'p7': 8.0, 'p8': 0.8, 'p9': 0.0, 'p10': 15.0, 'p11': 0.08, 'p12': 2.0}
CHARGE_MODE = {'c1': '#00ff00', 'c2': '#ffff00', 'c3': '#ff0000', 'c4': '#0000ff', 'c5': 5.0, 'c6': 0.03,
               'c7': 0.02, 'c8': 0.02, 'c9': 0, 'c10': False, 'c11': 1000, 'c12': 2.0, 'c13': 0.0, 'c14': True,
               'c15': 0.25, 'c16': 1.0, 'c17': 'auto'}

RECIPES = {'r': [['.308 Match', '175 SMK, CCI BR2', 42.0, 0]], 'a': -1}

FEATURES = {'f0': True, 'f1': True, 'f2': 0.5, 'f3': True, 'f4': 30, 'f5': True, 'f6': 300, 'f7': 10,
            'f8': True, 'f9': True, 'f10': True}
ERROR_LOG = {'bc': 42, 'lr': 'watchdog', 'up': 3725, 'wd': True, 'heap': 81920,
             'e': [[41, 'watchdog', 0, 5400, ''], [17, 'assert', 312, 80, 'trickle_model.c']]}
UNSAVED = set()
UNSAVED_PAGES = {'/rest/charge_mode_config': 'Charging', '/rest/feature_config': 'Safety & options', '/rest/ai_config': 'AI Trickle',
                 '/rest/coarse_motor_config': 'Motors', '/rest/fine_motor_config': 'Motors'}
# Charge mode events for the next state polls (tests set them through /__event)
EVENTS = []

SERVO = {'c0': False, 'c1': 0.05, 'c2': 0.1, 'c3': 0.05, 'c4': 0.1, 'c5': 1.0, 'c6': 1.0}

MODEL_KEYS = ['kc', 'kf', 'lat', 'tl', 'tsd', 'tau', 'cv', 'bias', 'lsd']


def as_bool(value):
    return value == 'true'


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def send_json(self, data):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        STATE['requests'].append(self.path)

        # Unsaved settings, like the firmware (settings_state.c)
        if url.path in UNSAVED_PAGES:
            if as_bool(q.get('ee', 'false')):
                UNSAVED.discard(UNSAVED_PAGES[url.path])
            elif any(k not in ('pf', 'ro', 'ee') for k in q):
                UNSAVED.add(UNSAVED_PAGES[url.path])

        if url.path in ('/favicon.ico', '/apple-touch-icon.png'):
            name = 'favicon.ico' if url.path == '/favicon.ico' else 'apple-touch-icon.png'
            with open(os.path.join(SCRIPT_DIR, '..', 'src', 'html', 'icons', name), 'rb') as fp:
                body = fp.read()
            self.send_response(200)
            self.send_header('Content-Type', 'image/x-icon' if name.endswith('.ico') else 'image/png')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if url.path == '/styles.css':
            with open(os.path.join(SCRIPT_DIR, '..', 'src', 'html', 'styles.css'), 'rb') as fp:
                body = fp.read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/css')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if url.path in ('/ai', '/'):
            with open(HTML_PATH if url.path == '/ai' else PORTAL_PATH, 'rb') as fp:
                body = fp.read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif url.path == '/rest/ai_config':
            c = STATE['config']
            if 'a0' in q:
                c['a0'] = as_bool(q['a0'])
            if 'a1' in q:
                c['a1'] = int(q['a1'])
            if 'a4' in q:
                c['a4'] = as_bool(q['a4'])
            if 'a5' in q:
                c['a5'] = c['tol'] = float(q['a5'])
            if 'a6' in q:
                c['sp'] = int(q['a6'])
                c['as'] = [0.0, 0.0, 0.0, c['sp'], 0.0]
            if not c['a4']:
                c['sp'] = c['a1']
            if 'a2' in q:
                c['a2'] = as_bool(q['a2'])
            if 'a3' in q:
                c['a3'] = int(q['a3'])
            if 'a7' in q:
                c['a7'] = int(q['a7'])
            self.send_json(c)
        elif url.path == '/rest/ai_model':
            pf = int(q.get('pf', 0))
            if pf != 0:
                m = STATE['models'].setdefault(pf, {**STATE['model'], 'pf': pf, 'pn': f'Profile{pf}', 'n': 0, 'fl': 0})
            else:
                m = STATE['model']
            for key in ('fm', 'fsd', 'mt'):
                if key in q:
                    m[key] = float(q[key])
            for key in ('n', 'fl'):
                if key in q:
                    m[key] = int(q[key])
            for key in MODEL_KEYS:
                if key in q:
                    m[key] = float(q[key])
            if as_bool(q.get('rst', 'false')):
                m.update({'n': 0, 'fl': 0})
            self.send_json(m)
        elif url.path == '/rest/ai_status':
            last = STATE['history'][-1]
            t = time.time() % 30
            weight = min(42.0, t * 3.0)
            self.send_json({'cs': 2, 'ar': t < 14, 'ph': 1 if t < 14 else 4, 'pn': 'Fine' if t < 14 else 'Done',
                            'pr': weight + 0.4, 'w': weight, 'tg': 42.0, 'la': STATE['learn']['la'],
                            'ld': STATE['learn']['ld'], 'lt': STATE['learn']['lt'], 'lc': False,
                            'seq': last[0], 'le': round(last[3] - last[2], 3), 'lad': True, 'tu': last[6]})
        elif url.path == '/rest/ai_history':
            # Like the firmware: pages of 40, the oldest first, with tolerance, recipe and uptime
            if as_bool(q.get('ns', 'false')):
                STATE['session'] = STATE['history'][-1][0]
            since = int(q.get('since', 0))
            rows = [h for h in STATE['history'] if h[0] > since]
            page = [(h + [0.02, 0 if abs(h[2] - 42.0) < 0.001 else -1, 60 + h[0] * 30])[:12] for h in rows[:40]]
            self.send_json({'seq': STATE['history'][-1][0], 'ss': STATE.get('session', 0), 'up': 9999,
                            'more': len(rows) > 40, 'h': page})
        elif url.path == '/rest/scale_config':
            self.send_json({'s0': 0, 's1': 2, 's2': 0})
        elif url.path == '/rest/mini_12864_config':
            self.send_json({'b0': False, 'b1': 0})
        elif url.path == '/rest/neopixel_led_config':
            self.send_json({'bl': '#ffffff', 'l1': '#0f0f0f', 'l2': '#0f0f0f', 'l3': 2, 'l4': True, 'l5': 0})
        elif url.path == '/rest/wireless_config':
            self.send_json({'w0': 'HomeWiFi', 'w2': 3, 'w3': 30000, 'w4': True})
        elif url.path == '/rest/feature_config':
            for key, value in q.items():
                if key in FEATURES:
                    FEATURES[key] = as_bool(value) if isinstance(FEATURES[key], bool) else float(value)
            self.send_json(FEATURES)
        elif url.path == '/rest/error_log':
            if as_bool(q.get('clr', 'false')):
                ERROR_LOG['e'] = []
            self.send_json(ERROR_LOG)
        elif url.path == '/rest/unsaved':
            self.send_json({'p': sorted(UNSAVED)})
        elif url.path == '/rest/system_control':
            if as_bool(q.get('s4', 'false')):
                UNSAVED.clear()
            self.send_json({'s0': 'E661410403', 's1': 'v2.0', 's2': '3428e5e', 's3': 'Release', 's4': False, 's5': False, 's6': False})
        elif url.path == '/__event':
            EVENTS.extend(int(e) for e in q.get('e', '').split(',') if e)
            STATE['charge_state'] = int(q.get('s', STATE.get('charge_state', 3)))
            self.send_json({'ok': True})
        elif url.path == '/rest/recipes':
            r = RECIPES['r']
            ok = True
            if 'i' in q:
                i = int(q['i'])
                if i == len(r):
                    r.append([f'Recipe {i + 1}', '', 42.0, STATE['selected']])
                if 'n' in q:
                    r[i][0] = q['n']
                if 'no' in q:
                    r[i][1] = q['no']
                if 't' in q:
                    r[i][2] = float(q['t'])
                if 'p' in q:
                    r[i][3] = int(q['p'])
            if 'del' in q:
                r.pop(int(q['del']))
                RECIPES['a'] = -1
            if 'up' in q:
                u = int(q['up'])
                r[u - 1], r[u] = r[u], r[u - 1]
            if 'apply' in q:
                a = int(q['apply'])
                RECIPES['a'] = a
                STATE['selected'] = r[a][3]
                STATE['config']['pf'] = r[a][3]
                STATE['config']['pn'] = NAMES[r[a][3]]
            self.send_json({'a': RECIPES['a'], 'ok': ok, 'r': r})
        elif url.path == '/rest/ai_trace':
            self.send_json(STATE['trace'])
        elif url.path == '/rest/ai_learn':
            if as_bool(q.get('go', 'false')):
                STATE['learn'] = {'la': True, 'ld': 0, 'lt': int(q.get('n', 8))}
            if as_bool(q.get('stop', 'false')):
                STATE['learn']['la'] = False
            self.send_json({**STATE['learn'], 'tg': float(q.get('s0', 42.0))})
        elif url.path == '/rest/profile_config':
            pf = int(q.get('pf', STATE['selected']))
            if not as_bool(q.get('ro', 'false')):
                STATE['selected'] = pf
                STATE['config']['pf'] = pf
                STATE['config']['pn'] = NAMES[pf]
            for key in PROFILE:
                if key in q and key not in ('pf', 'p2'):
                    PROFILE[key] = float(q[key])
            self.send_json({**PROFILE, 'pf': pf, 'p2': NAMES[pf]})
        elif url.path == '/__state':
            self.send_json({'taps': STATE['taps'], 'selected': STATE['selected'], 'names': NAMES,
                            'learn': STATE['learn']})
        elif url.path == '/rest/profile_new':
            slot = int(q['slot']) if 'slot' in q else next((i for i, n in enumerate(NAMES) if n == f'Profile{i}'), -1)
            if slot < 0:
                self.send_json({'pf': -1, 'err': 'No free profile slot'})
            else:
                NAMES[slot] = q.get('n', '')
                STATE['selected'] = slot
                STATE['config']['pf'] = slot
                STATE['config']['pn'] = NAMES[slot]
                self.send_json({'pf': slot, 'err': ''})
        elif url.path in ('/rest/coarse_motor_config', '/rest/fine_motor_config'):
            self.send_json({'m0': 5000.0, 'm1': 200, 'm2': 500, 'm3': 256,
                            'm4': 20 if 'coarse' in url.path else 6, 'm5': 110, 'm6': 0.02, 'm7': 1.0,
                            'm8': False, 'm9': False})
        elif url.path == '/rest/servo_gate_config':
            if 'c0' in q:
                SERVO['c0'] = as_bool(q['c0'])
            self.send_json(SERVO)
        elif url.path == '/rest/charge_mode_state':
            if 'tap' in q:
                STATE['taps'] += int(q['tap'])
            event = EVENTS.pop(0) if EVENTS else 1
            self.send_json({'s0': 42.0, 's1': 12.3, 's2': STATE.get('charge_state', 3), 's3': event,
                            's4': NAMES[STATE['selected']], 's5': '9.80', 's6': True})
        elif url.path == '/rest/charge_mode_config':
            for key in CHARGE_MODE:
                if key in q:
                    try:
                        CHARGE_MODE[key] = float(q[key])
                    except ValueError:
                        CHARGE_MODE[key] = q[key]
            self.send_json(CHARGE_MODE)
        elif url.path == '/rest/ai_series':
            sr = STATE['series']
            if as_bool(q.get('go', 'false')):
                sr.update({'sa': True, 'lad': as_bool(q.get('lad', 'false')), 'sd': 0, 'n': int(q['n']),
                           's0': float(q['s0']), 'step': float(q.get('step', 0)), 'rep': int(q.get('rep', 1)),
                           'seq0': STATE['history'][-1][0] + 1, 'tg': float(q['s0'])})
                # Simulate the first three charges of the series
                for i in range(3):
                    last = STATE['history'][-1]
                    target = sr['s0'] + sr['step'] * (i // sr['rep'])
                    error = [0.0, 0.02, -0.02][i]
                    STATE['history'].append([last[0] + 1, 0, target, target + error, target - 0.05, 20.0 + i, 1, 1 | 32, 0])
                sr['sd'] = 3
                sr['tg'] = sr['s0'] + sr['step'] * (3 // sr['rep'])
            if as_bool(q.get('stop', 'false')):
                sr['sa'] = False
            self.send_json(sr)
        elif url.path == '/rest/scale_diag':
            d = STATE['diag']
            if as_bool(q.get('go', 'false')):
                d.update({'st': 1, 'dur': int(q.get('s', 20)), 'start': time.time()})
            out = {'st': d['st'], 'el': 0.0, 'dur': d['dur'], 'err': ''}
            if d['st'] == 1:
                el = (time.time() - d['start']) * 5  # 5x faster than real time
                out['el'] = min(el, d['dur'])
                if el >= d['dur']:
                    d['st'] = 2
            out['st'] = d['st']
            if d['st'] >= 2:
                out.update({'n': 200, 'ch': 40, 'hz': 9.8, 'mean': 0.002, 'sd': 0.0162, 'drift': 0.031,
                            'min': -0.04, 'max': 0.04, 'jump': 0.04, 'res': 0.02})
            self.send_json(out)
        elif url.path == '/rest/profile_summary':
            self.send_json({'s0': {str(i): NAMES[i] for i in range(8)}, 's1': STATE['selected'],
                            's2': [0 if n == f'Profile{i}' else 1 for i, n in enumerate(NAMES)]})
        elif url.path == '/rest/fw_info':
            fw = STATE['fw']
            if fw['st'] == 'rebooting':
                if time.time() < STATE['reboot_until']:
                    self.send_response(503)
                    self.end_headers()
                    return
                fw.update({'st': 'idle', 'build': fw['build'] + 1, 'part': 1 - fw['part'], 'conf': True})
            self.send_json(fw)
        elif url.path == '/rest/fw_apply':
            fw = STATE['fw']
            if as_bool(q.get('go', 'false')) and fw['st'] == 'verified':
                fw['st'] = 'rebooting'
                STATE['reboot_until'] = time.time() + 6
            self.send_json(fw)
        elif url.path == '/mock/requests':
            self.send_json(STATE['requests'])
        else:
            self.send_response(404)
            self.end_headers()


def do_POST(self):
    url = urlparse(self.path)
    q = {k: v[0] for k, v in parse_qs(url.query).items()}
    STATE['requests'].append('POST ' + self.path)
    length = int(self.headers.get('Content-Length', 0))
    body = self.rfile.read(length)
    fw = STATE['fw']
    if url.path == '/upload/firmware':
        fw['rx'] = len(body)
        fw['size'] = int(q.get('size', 0))
        fw['sha'] = hashlib.sha256(body).hexdigest()
        if fw['size'] != len(body):
            fw.update({'st': 'error', 'err': 'Size mismatch'})
        elif fw['sha'] != q.get('sha'):
            fw.update({'st': 'error', 'err': 'Checksum mismatch'})
        else:
            fw.update({'st': 'verified', 'err': ''})
    self.send_json(fw)


Handler.do_POST = do_POST


if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    print(f'Serving on http://localhost:{port}/ai')
    ThreadingHTTPServer(('', port), Handler).serve_forever()
