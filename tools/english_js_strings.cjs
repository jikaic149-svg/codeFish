'use strict';
const fs = require('node:fs');
const ts = require(process.argv[2]);
const input = JSON.parse(fs.readFileSync(0, 'utf8'));
const result = [];
for (const item of input) {
  const file = ts.createSourceFile(item.path, item.text, ts.ScriptTarget.Latest, true, ts.ScriptKind.JS);
  const strings = [];
  function visit(node) {
    const kinds = new Set([ts.SyntaxKind.StringLiteral, ts.SyntaxKind.NoSubstitutionTemplateLiteral,
      ts.SyntaxKind.TemplateHead, ts.SyntaxKind.TemplateMiddle, ts.SyntaxKind.TemplateTail]);
    if (item.path.includes('/tests/')) kinds.add(ts.SyntaxKind.RegularExpressionLiteral);
    if (kinds.has(node.kind) && /[\u3400-\u9fff]/.test(node.text || '')) {
      strings.push({start: Buffer.byteLength(item.text.slice(0, node.getStart(file)), 'utf8'),
        end: Buffer.byteLength(item.text.slice(0, node.end), 'utf8'), value: node.text,
        kind: node.kind === ts.SyntaxKind.RegularExpressionLiteral ? 'RegularExpressionLiteral' :
          node.kind === ts.SyntaxKind.TemplateHead ? 'TemplateHead' :
          node.kind === ts.SyntaxKind.TemplateMiddle ? 'TemplateMiddle' :
          node.kind === ts.SyntaxKind.TemplateTail ? 'TemplateTail' : 'StringLiteral'});
    }
    ts.forEachChild(node, visit);
  }
  visit(file);
  result.push({path: item.path, strings});
}
process.stdout.write(JSON.stringify(result));
