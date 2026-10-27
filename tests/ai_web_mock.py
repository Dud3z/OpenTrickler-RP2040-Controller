"""
Mock of the controller's REST interface for developing the AI Trickle page
(src/html/ai_trickle.html) without hardware. Data comes from the powder
simulator (tests/sim/trickle_export).

Usage

    make -C tests/sim trickle_export
    python3 tests/ai_web_mock.py [port]

Then open http://localhost:8080/ai
"""

import json
import os
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse


SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
HTML_PATH = os.path.join(SCRIPT_DIR, '..', 'src', 'html', 'ai_trickle.html')
EXPORT_TOOL = os.path.join(SCRIPT_DIR, 'sim', 'trickle_export')


def load_export():
    output = subprocess.run([EXPORT_TOOL], capture_output=True, check=True, text=True).stdout
    return json.loads(output)


EXPORT = load_export()

STATE = {
    'config': {'a0': True, 'a1': 0, 'a2': True, 'a3': 8, 'tol': 0.02, 'est': 24.0, 'tg': 42.0,
               'pf': 0, 'pn': EXPORT['model']['pn'], 'rd': True},
    'model': EXPORT['model'],
    'history': EXPORT['history'],
    'trace': EXPORT['trace'],
    'learn': {'la': False, 'ld': 0, 'lt': 8},
    'requests': [],
}

PROFILE = {'pf': 0, 'p0': 0, 'p1': 0, 'p2': EXPORT['model']['pn'], 'p3': 0.05, 'p4': 0.0, 'p5': 0.3, 'p6': 0.1,
           'p7': 8.0, 'p8': 0.8, 'p9': 0.0, 'p10': 15.0, 'p11': 0.08, 'p12': 2.0}
CHARGE_MODE = {'c1': '#00ff00', 'c2': '#ffff00', 'c3': '#ff0000', 'c4': '#0000ff', 'c5': 5.0, 'c6': 0.03,
               'c7': 0.02, 'c8': 0.02, 'c9': 0, 'c10': False, 'c11': 1000, 'c12': 2.0, 'c13': 0.0, 'c14': True,
               'c15': 0.25, 'c16': 1.0}

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

        if url.path in ('/ai', '/'):
            with open(HTML_PATH, 'rb') as fp:
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
                c['tol'] = 0.02 * (5 ** (c['a1'] / 100))
            if 'a2' in q:
                c['a2'] = as_bool(q['a2'])
            if 'a3' in q:
                c['a3'] = int(q['a3'])
            self.send_json(c)
        elif url.path == '/rest/ai_model':
            m = STATE['model']
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
            since = int(q.get('since', 0))
            self.send_json({'seq': STATE['history'][-1][0], 'h': [h for h in STATE['history'] if h[0] > since]})
        elif url.path == '/rest/ai_trace':
            self.send_json(STATE['trace'])
        elif url.path == '/rest/ai_learn':
            if as_bool(q.get('go', 'false')):
                STATE['learn'] = {'la': True, 'ld': 0, 'lt': int(q.get('n', 8))}
            if as_bool(q.get('stop', 'false')):
                STATE['learn']['la'] = False
            self.send_json({**STATE['learn'], 'tg': float(q.get('s0', 42.0))})
        elif url.path == '/rest/profile_config':
            for key in PROFILE:
                if key in q and key != 'pf':
                    PROFILE[key] = float(q[key])
            self.send_json(PROFILE)
        elif url.path == '/rest/charge_mode_config':
            for key in CHARGE_MODE:
                if key in q:
                    CHARGE_MODE[key] = float(q[key])
            self.send_json(CHARGE_MODE)
        elif url.path == '/mock/requests':
            self.send_json(STATE['requests'])
        else:
            self.send_response(404)
            self.end_headers()


if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    print(f'Serving on http://localhost:{port}/ai')
    ThreadingHTTPServer(('', port), Handler).serve_forever()
