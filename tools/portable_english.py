"""Create an English UI variant while preserving documents and compiled dependencies."""
import argparse
import ast
import copy
import hashlib
import html
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import os
import shutil
import struct
import subprocess
import zipfile

HAN = re.compile(r'[\u3400-\u9fff]')
TOOLS = Path(__file__).resolve().parent
BASE = TOOLS.parents[2] / 'RobotFish-Dev'
TS = BASE / 'editor/e646868227/resources/app/extensions/node_modules/typescript/lib/typescript.js'


def byte_position(text, line, column, utf8=False):
    lines = text.splitlines(keepends=True)
    return sum(len(value.encode('utf-8')) for value in lines[:line - 1]) + (
        column if utf8 else len(lines[line - 1][:column].encode('utf-8')))


def python_strings(text):
    tree = ast.parse(text)
    result = []
    documents = {id(node.body[0].value) for node in ast.walk(tree)
                 if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                 and node.body and isinstance(node.body[0], ast.Expr)
                 and isinstance(node.body[0].value, ast.Constant) and isinstance(node.body[0].value.value, str)}
    def visit(node):
        if id(node) in documents:
            return
        if isinstance(node, ast.JoinedStr):
            values = [child.value for child in ast.walk(node)
                      if isinstance(child, ast.Constant) and isinstance(child.value, str) and HAN.search(child.value)]
            if values:
                result.append({'start': byte_position(text, node.lineno, node.col_offset, True),
                    'end': byte_position(text, node.end_lineno, node.end_col_offset, True),
                    'values': values, 'kind': 'PythonFString'})
            return
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and HAN.search(node.value):
            if not re.match(r'^[A-Za-z]:[/\\]', node.value) and not re.fullmatch(r'[^\r\n]*\d{4}-\d{2}-\d{2} \d{6}\.mp4', node.value):
                result.append({'start': byte_position(text, node.lineno, node.col_offset, True),
                    'end': byte_position(text, node.end_lineno, node.end_col_offset, True),
                    'value': node.value, 'kind': 'PythonString'})
        for child in ast.iter_child_nodes(node):
            visit(child)
    visit(tree)
    return result


class PageStrings(HTMLParser):
    def __init__(self, text):
        super().__init__(convert_charrefs=False)
        self.text = text
        self.items = []
        self.scripts = []
        self.inside = None
        self.feed(text)

    def handle_starttag(self, tag, attrs):
        self.inside = tag if tag in ('script', 'style') else self.inside
        raw = self.get_starttag_text()
        base = byte_position(self.text, *self.getpos())
        for match in re.finditer(r'([\w:-]+)\s*=\s*([\'\"])(.*?)\2', raw, re.S):
            value = html.unescape(match[3])
            if HAN.search(value) and not match[1].lower().startswith('on'):
                self.items.append({'start': base + len(raw[:match.start(3)].encode('utf-8')),
                    'end': base + len(raw[:match.end(3)].encode('utf-8')), 'value': value, 'kind': 'HTMLAttribute'})

    def handle_startendtag(self, tag, attrs):
        self.handle_starttag(tag, attrs)

    def handle_endtag(self, tag):
        if self.inside == tag:
            self.inside = None

    def handle_data(self, data):
        if not HAN.search(data):
            return
        base = byte_position(self.text, *self.getpos())
        if self.inside == 'script':
            self.scripts.append((base, data))
        elif self.inside != 'style':
            trimmed = data.strip()
            left = data.index(trimmed)
            self.items.append({'start': base + len(data[:left].encode('utf-8')),
                'end': base + len(data[:left + len(trimmed)].encode('utf-8')),
                'value': trimmed, 'kind': 'HTMLText'})


def collect(bundle):
    records = {}
    scripts = []
    with zipfile.ZipFile(bundle) as archive:
        for info in archive.infolist():
            name = info.filename
            relative = name.split('/', 1)[1]
            if relative.startswith('project/upper/tools/'):
                continue  # Existing report-generation tools and their document prose remain unchanged.
            if not (relative in ('portable.py', 'verify_delivery.py', '.vscode/launch.json', 'RobotFish.code-workspace')
                    or relative.startswith(('project/upper/', 'project/vision/', 'project/tests/')) or relative == 'project/main/control.html'):
                continue
            suffix = Path(name).suffix
            if suffix not in ('.py', '.js', '.html', '.json', '.code-workspace', '.css'):
                continue
            text = archive.read(info).decode('utf-8-sig')
            if not HAN.search(text):
                continue
            record = {'text': text, 'items': []}
            records[name] = record
            if suffix == '.py':
                record['items'] = python_strings(text)
            elif suffix == '.js':
                scripts.append({'path': name, 'text': text, 'base': 0})
            elif suffix == '.html':
                page = PageStrings(text)
                record['items'] = page.items
                scripts += [{'path': name + '#' + str(base), 'text': script, 'base': base, 'parent': name}
                            for base, script in page.scripts]
            elif suffix in ('.json', '.code-workspace'):
                for match in re.finditer(r'"(?:[^"\\]|\\.)*"', text):
                    value = json.loads(match[0])
                    if HAN.search(value):
                        record['items'].append({'start': len(text[:match.start()].encode('utf-8')),
                            'end': len(text[:match.end()].encode('utf-8')), 'value': value, 'kind': 'JSONString'})
    parsed = subprocess.run([str(BASE / 'tools/node/node.exe'), str(TOOLS / 'english_js_strings.cjs'), str(TS)],
        input=json.dumps(scripts), capture_output=True, text=True, encoding='utf-8', check=True)
    for source, result in zip(scripts, json.loads(parsed.stdout)):
        for item in result['strings']:
            item['start'] += source['base']
            item['end'] += source['base']
            records[source.get('parent', source['path'])]['items'].append(item)
    return records


def translated_files(records, translations):
    result = {}
    for name, record in records.items():
        original = record['text'].encode('utf-8')
        edits = []
        for item in record['items']:
            kind = item['kind']
            if kind == 'PythonFString':
                expression = ast.parse('(' + original[item['start']:item['end']].decode('utf-8') + ')', mode='eval')
                class Replace(ast.NodeTransformer):
                    def visit_Constant(self, node):
                        if isinstance(node.value, str) and HAN.search(node.value):
                            node.value = translations[node.value]
                        return node
                replacement = ast.unparse(Replace().visit(expression).body)
            else:
                value = translations[item['value']]
                assert not HAN.search(value), item['value']
                if kind == 'PythonString':
                    replacement = repr(value)
                elif kind in ('HTMLText', 'HTMLAttribute'):
                    replacement = html.escape(value, quote=kind == 'HTMLAttribute')
                elif kind.startswith('Template'):
                    escaped = value.replace('\\', '\\\\').replace('`', '\\`').replace('${', '\\${')
                    left = '`' if kind == 'TemplateHead' else '}'
                    right = '`' if kind == 'TemplateTail' else '${'
                    replacement = left + escaped + right
                elif kind == 'RegularExpressionLiteral':
                    replacement = value
                else:
                    replacement = json.dumps(value, ensure_ascii=False)
            edits.append((item['start'], item['end'], replacement.encode('utf-8')))
        end = len(original)
        for start, stop, replacement in sorted(edits, reverse=True):
            assert stop <= end, (name, start, stop, end)
            original = original[:start] + replacement + original[stop:]
            end = start
        text = original.decode('utf-8')
        if name.endswith(('.html', '.js')):
            text = text.replace('zh-CN', 'en-US').replace('lang="zh"', 'lang="en"')
        if name.endswith('/portable.py'):
            text = text.replace("env.get('ROBOFISH_HOST_PORT', '8767')", "env.get('ROBOFISH_HOST_PORT', '8768')")
            text = text.replace("'--new-window'", "'--locale=en', '--new-window'")
        if name.endswith('.py') and '/tests/' in name:
            # Chinese assertions had no case distinction; preserve that property in English.
            patterns = {translations[node.args[1].value] for node in ast.walk(ast.parse(record['text']))
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'assertRaisesRegex' and len(node.args) > 1
                and isinstance(node.args[1], ast.Constant) and isinstance(node.args[1].value, str)
                and HAN.search(node.args[1].value)}
            regex_edits = []
            for node in ast.walk(ast.parse(text)):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == 'assertRaisesRegex' and len(node.args) > 1:
                    pattern = node.args[1]
                    if isinstance(pattern, ast.Constant) and pattern.value in patterns:
                        regex_edits.append((byte_position(text, pattern.lineno, pattern.col_offset, True),
                            byte_position(text, pattern.end_lineno, pattern.end_col_offset, True),
                            repr('(?i)' + pattern.value).encode('utf-8')))
            data = text.encode('utf-8')
            for start, stop, replacement in sorted(regex_edits, reverse=True):
                data = data[:start] + replacement + data[stop:]
            text = data.decode('utf-8')
        if name.endswith('.py'):
            compile(text, name, 'exec')
            # All executable structure remains unchanged; only string constants are localized.
            if not name.endswith('/portable.py'):
                class IgnoreStrings(ast.NodeTransformer):
                    def visit_Constant(self, node):
                        if isinstance(node.value, str): node.value = ''
                        return node
                before = ast.dump(IgnoreStrings().visit(ast.parse(record['text'])), include_attributes=False)
                after = ast.dump(IgnoreStrings().visit(ast.parse(text)), include_attributes=False)
                assert before == after, name
        if text != record['text']:
            result[name] = text.encode('utf-8')
    css_name = 'RobotFish-Dev/project/upper/style.css'
    if css_name in records:
        result[css_name] = records[css_name]['text'].encode('utf-8') + (
            '\n/* English labels may wrap in the narrow sidebar. */\n'
            'html[lang^="en"] .sidebar-bottom .button {white-space:normal;'
            'height:auto;min-height:36px;line-height:1.35;text-align:center;}\n').encode()
    return result


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def write_zip(bundle, output, replacements):
    """Reuse compressed payloads, rewriting names and localized files only."""
    assert not output.exists(), output
    pending = output.with_suffix('.zip.partial')
    assert not pending.exists(), pending
    expected = {}
    with zipfile.ZipFile(bundle) as source, bundle.open('rb') as raw_source, \
            zipfile.ZipFile(pending, 'w', allowZip64=True) as target:
        for entry in source.infolist():
            name = entry.filename.replace('RobotFish-Dev/', 'RobotFish-Dev-English/', 1)
            if entry.filename in replacements:
                data = replacements[entry.filename]
                target.writestr(name, data, compress_type=zipfile.ZIP_DEFLATED, compresslevel=1)
                expected[name] = (zipfile.crc32(data), len(data))
                continue
            raw_source.seek(entry.header_offset)
            header = raw_source.read(30)
            fields = struct.unpack('<4s5H3I2H', header)
            assert fields[0] == b'PK\x03\x04'
            raw_source.seek(fields[-2] + fields[-1], 1)
            info = copy.copy(entry)
            info.filename = name
            info.header_offset = target.fp.tell()
            info.flag_bits &= ~8
            target.fp.write(info.FileHeader())
            remaining = entry.compress_size
            while remaining:
                data = raw_source.read(min(8 * 1024**2, remaining))
                assert data
                target.fp.write(data)
                remaining -= len(data)
            target.filelist.append(info)
            target.NameToInfo[name] = info
            target.start_dir = target.fp.tell()
            expected[name] = (entry.CRC, entry.file_size)
        for original_name, data in replacements.items():
            name = original_name.replace('RobotFish-Dev/', 'RobotFish-Dev-English/', 1)
            if name not in expected:
                target.writestr(name, data, compress_type=zipfile.ZIP_DEFLATED, compresslevel=1)
                expected[name] = (zipfile.crc32(data), len(data))
        target._didModify = True
    print('Checking all archive entries...', flush=True)
    with zipfile.ZipFile(pending) as archive:
        assert len(archive.infolist()) == len(expected)
        assert all((item.CRC, item.file_size) == expected[item.filename] for item in archive.infolist())
        assert archive.testzip() is None
    pending.rename(output)
    print(json.dumps({'zip': str(output), 'bytes': output.stat().st_size, 'files': len(expected)}), flush=True)


def materialize(bundle, folder, replacements):
    if folder.exists():
        for original_name, data in replacements.items():
            target = folder / original_name.split('/', 1)[1]
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        print('English localized files refreshed: ' + str(folder), flush=True)
        return
    folder.mkdir()
    with zipfile.ZipFile(bundle) as archive:
        for entry in archive.infolist():
            relative = Path(entry.filename.split('/', 1)[1])
            target = folder / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if entry.filename in replacements:
                target.write_bytes(replacements[entry.filename])
                continue
            original = BASE / relative
            shared = relative.parts[0] in ('runtime', 'tools', 'datasets', 'samples', 'firmware', 'fonts') or (
                relative.parts[:3] == ('idf-env', 'Lib', 'site-packages')) or (
                relative.parts[0] == 'editor' and 'data' not in relative.parts)
            if shared and original.is_file() and original.stat().st_size == entry.file_size:
                os.link(original, target)
            else:
                target.write_bytes(archive.read(entry))
        for original_name, data in replacements.items():
            target = folder / original_name.split('/', 1)[1]
            if not target.exists():
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
    print('English review folder ready: ' + str(folder), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--catalog', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--review-folder', type=Path)
    args = parser.parse_args()
    records = collect(args.bundle)
    values = sorted({value for record in records.values() for item in record['items']
                     for value in item.get('values', [item.get('value')]) if value is not None})
    args.catalog.write_bytes((json.dumps({'values': values, 'records': records}, ensure_ascii=False, indent=2) + '\n').encode())
    print(json.dumps({'unique_strings': len(values), 'characters': sum(map(len, values)), 'files': len(records)}))
    if args.output or args.review_folder:
        translations = json.loads((TOOLS / 'english_strings.json').read_text(encoding='utf-8'))
        assert set(values) <= set(translations), 'Untranslated interface strings remain'
        replacements = translated_files(records, translations)
        replacements['RobotFish-Dev/START_HERE_EN.txt'] = (TOOLS / 'portable_english_usage.txt').read_bytes()
        with zipfile.ZipFile(args.bundle) as archive:
            manifest = json.loads(archive.read('RobotFish-Dev/manifest.json'))
            versions = json.loads(archive.read('RobotFish-Dev/environment-versions.json'))
            versions['base_release_tag'] = versions.pop('release_tag')
            versions.update(language='en', default_host_port=8768)
            replacements['RobotFish-Dev/environment-versions.json'] = (json.dumps(versions, indent=2) + '\n').encode()
            for name, data in replacements.items():
                relative = name.removeprefix('RobotFish-Dev/')
                if relative.startswith('project/'):
                    manifest['source_files'][relative.removeprefix('project/')] = sha256(data)
                elif relative in manifest['launcher_files']:
                    manifest['launcher_files'][relative] = sha256(data)
            if 'release' in manifest: manifest['base_release'] = manifest.pop('release')
            manifest.update(language='en', default_host_port=8768,
                localization={'translated_strings': len(values), 'existing_documents_preserved': True,
                              'guide': 'START_HERE_EN.txt', 'firmware_binaries_preserved': True,
                              'firmware_translation_requires_rebuild': True})
            if args.review_folder:
                for name in ('english-validation.json', 'english-regression.json'):
                    report = args.review_folder / 'state' / name
                    if report.is_file(): manifest['localization'][name.removesuffix('.json')] = json.loads(report.read_text(encoding='utf-8'))
            manifest['launcher_files']['START_HERE_EN.txt'] = sha256(replacements['RobotFish-Dev/START_HERE_EN.txt'])
            sizes = {info.filename: info.file_size for info in archive.infolist()}
            manifest['folder_files'] = len(set(sizes) | set(replacements))
            for _ in range(3):
                replacements['RobotFish-Dev/manifest.json'] = (json.dumps(manifest, indent=2) + '\n').encode()
                manifest['folder_bytes'] = sum(sizes.values()) + sum(len(data) - sizes.get(name, 0) for name, data in replacements.items())
            replacements['RobotFish-Dev/manifest.json'] = (json.dumps(manifest, indent=2) + '\n').encode()
        if args.review_folder:
            materialize(args.bundle, args.review_folder, replacements)
        if args.output:
            write_zip(args.bundle, args.output, replacements)


if __name__ == '__main__':
    main()
