'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../web/dynamics.js'),'utf8');
const fn=source.slice(source.indexOf('function stepTailBias('),source.indexOf('function release('));
const fields={bias:{value:'0'},surge:{value:'8'},tail:{value:'0'},diff:{value:'0'}},sent=[];
const context={held:true,mode:3,recipe:'static',driveRequest:null,motion:'left',generation:7,staticBiasLevel:0,
  $:id=>fields[id],updatePreview(){},notice(){},driveOnce(epoch){sent.push({epoch,...Object.fromEntries(Object.entries(fields).map(([key,input])=>[key,Number(input.value)]))});}};
vm.createContext(context);vm.runInContext(fn,context);
context.stepTailBias(60);context.stepTailBias(0);
assert.deepEqual(sent,[{epoch:7,bias:-60,surge:8,tail:0,diff:0},{epoch:7,bias:0,surge:8,tail:0,diff:0}]);
context.motion='right';context.stepTailBias(60);assert.equal(sent.at(-1).bias,60);
context.driveRequest={};context.stepTailBias(0);assert.equal(sent.length,3);assert.equal(fields.bias.value,'60');
context.driveRequest=null;context.held=false;context.stepTailBias(0);assert.equal(sent.length,3);
context.held=true;context.motion='straight';context.stepTailBias(60);assert.equal(sent.length,3);
context.recipe='swim';context.motion='left';fields.tail.value='8';
context.stepTailBias(60);context.stepTailBias(0);
assert.deepEqual(sent.slice(-2),[{epoch:7,bias:-60,surge:8,tail:8,diff:0},{epoch:7,bias:0,surge:8,tail:8,diff:0}]);
console.log('Both tail-test recipes preserve their propulsion during bias steps and require a manual hold.');
