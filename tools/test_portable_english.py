"""Check localized syntax, original documents and real browser rendering without hardware."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import urllib.request
from html.parser import HTMLParser
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--original', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    spec = importlib.util.spec_from_file_location('portable', root / 'portable.py')
    portable = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(portable)
    env = portable.initialize()
    os.environ.clear(); os.environ.update(env)
    han = re.compile(r'[\u3400-\u9fff]')
    node = root / 'tools/node/node.exe'
    scripts = []
    for path in (root / 'project/upper/web').glob('*.js'):
        subprocess.run([str(node), '--check', str(path)], env=env, check=True, capture_output=True)
        scripts.append(path.name)
    class InlineScripts(HTMLParser):
        def __init__(self):
            super().__init__(); self.active = False; self.scripts = []
        def handle_starttag(self, tag, attrs):
            if tag == 'script': self.active = True
        def handle_endtag(self, tag):
            if tag == 'script': self.active = False
        def handle_data(self, value):
            if self.active and value.strip(): self.scripts.append(value)
    for path in (root / 'project/main/control.html', root / 'project/upper/web/index.html', root / 'project/upper/web/dynamics.html'):
        parsed = InlineScripts(); parsed.feed(path.read_text(encoding='utf-8'))
        for script in parsed.scripts:
            subprocess.run([str(node), '--check', '-'], input=script, env=env, text=True,
                           encoding='utf-8', check=True, capture_output=True)
    documents = 0
    with zipfile.ZipFile(args.original) as archive:
        for item in archive.infolist():
            relative = item.filename.split('/', 1)[1]
            if relative.endswith(('.md', '.txt')) and relative not in ('START_HERE_EN.txt',):
                assert (root / relative).read_bytes() == archive.read(item), relative
                documents += 1
    launch = json.loads((root / '.vscode/launch.json').read_text(encoding='utf-8'))
    assert len(launch['configurations']) == 5
    assert all(not han.search(item['name']) for item in launch['configurations'])
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0)); port = listener.getsockname()[1]
    env['ROBOFISH_HOST_PORT'] = str(port)
    address = 'http://127.0.0.1:' + str(port)
    python = root / 'runtime/python.exe'
    def run(action):
        result = subprocess.run([str(python), '-I', '-B', str(root / 'portable.py'), action, '--no-browser'],
            cwd=root, env=env, capture_output=True, text=True, encoding='utf-8', timeout=50)
        assert result.returncode == 0, result.stdout + result.stderr
        assert not han.search(result.stdout), result.stdout
    report = {'documents_unchanged': documents, 'javascript_files_checked': scripts,
              'launch_names': [item['name'] for item in launch['configurations']], 'pages': {},
              'physical_camera_or_serial_used': False}
    try:
        run('start')
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(address + '/api/health', timeout=5) as response:
            assert Path(json.load(response)['instance']).resolve() == root / 'project/upper'
        from playwright.sync_api import sync_playwright
        edge = Path(env['PROGRAMFILES(X86)']) / 'Microsoft/Edge/Application/msedge.exe'
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(executable_path=str(edge), headless=True, args=['--disable-gpu'])
            try:
                context = browser.new_context(viewport={'width': 1440, 'height': 1000}, locale='en-US')
                def route(request):
                    url = request.request.url
                    if not url.startswith(address): return request.abort()
                    if '/api/device/discover' in url:
                        return request.fulfill(json={'candidates': [], 'usb': [], 'robotfish': None})
                    if '/api/vision/cameras' in url:
                        return request.fulfill(json={'cameras': [], 'error': None, 'scanning': False})
                    if '/api/development/' in url:
                        return request.fulfill(status=503, json={'error': 'No development device in this UI check'})
                    return request.continue_()
                context.route('**/*', route)
                page = context.new_page()
                errors = []; page.on('pageerror', lambda error: errors.append(str(error)))
                page.goto(address, wait_until='domcontentloaded')
                for name in ('overview', 'vision', 'data', 'network', 'debug'):
                    selector = '[data-page=' + name + '],[data-go=' + name + ']'
                    if name == 'network': selector += ',[data-page=settings],[data-go=settings]'
                    page.locator(selector).first.click()
                    page.wait_for_timeout(400)
                    text = page.locator('body').inner_text()
                    assert not han.search(text), (name, han.findall(text))
                    assert page.locator('html').get_attribute('lang') in ('en', 'en-US')
                    report['pages'][name] = {'english': True, 'title': page.title()}
                    if name == 'overview':
                        page.screenshot(path=str(root / 'state/english-ui.png'), full_page=True)
                assert not errors, errors
                report['browser_errors'] = errors
            finally:
                browser.close()
    finally:
        run('stop')
    (root / 'state/english-validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
