'use strict';
// Exercise the actual page handlers with slow HTTP and image reloads; no device calls.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../vision.js'),'utf8');
const handlers=source.slice(source.indexOf('function visionImageBounds('),source.indexOf("$('vision-start').onclick="));
function fixture(task='MARKER_ROI'){
  const overlay={style:{},setAttribute(){}},listeners={},sent=[],pending=[];
  const parent={append(){},getBoundingClientRect:()=>({left:10,top:20})};
  const image={naturalWidth:1000,naturalHeight:500,parentElement:parent,
    dataset:{pointerReady:'true',selectionId:'selection',pointerSpace:'image',processId:'1'},
    getBoundingClientRect:()=>({left:10,top:20,width:1000,height:700}),setPointerCapture(){}};
  const context={pointerDown:false,lastPointerAt:0,visionState:{process_id:1,selection_id:'selection',result:{interaction:task}},
    $:()=>image,document:{createElement:()=>overlay},window:{addEventListener:(type,fn)=>listeners[type]=fn},
    location:{href:'http://localhost/'},URL,robotText(){},pollVision(){},
    service:async(endpoint,body)=>{assert.equal(endpoint,'vision/action');sent.push(body.event);await new Promise((resolve,reject)=>pending.push({resolve,reject}));}};
  vm.createContext(context);vm.runInContext(handlers,context);
  const event=(x,y,id=1)=>({clientX:10+x*1000,clientY:120+y*500,pointerId:id,button:0,target:image,preventDefault(){}});
  const tick=()=>new Promise(resolve=>setImmediate(resolve));
  const drain=async()=>{while(pending.length){pending.shift().resolve();await tick();}};
  return {context,image,overlay,listeners,sent,pending,event,drain,tick};
}
async function main(){
  {
    const f=fixture(),{image,event}=f;
    image.onpointerdown(event(.2,.3));
    image.onpointermove(event(.7,.8));
    assert.equal(f.overlay.hidden,false);
    assert.ok(Math.abs(parseFloat(f.overlay.style.width)-500)<1e-6);
    assert.equal(parseFloat(f.overlay.style.height),250);
    assert.equal(f.sent.length,1,'ROI movement must not wait for/send HTTP');
    image.dataset.pointerReady='false';
    // A newly loading image cannot change the coordinate system of an active drag.
    image.getBoundingClientRect=()=>({left:0,top:0,width:500,height:250});
    image.onpointerup(event(.8,.9));image.onlostpointercapture(event(.8,.9));
    await f.drain();
    assert.deepEqual(f.sent.map(item=>item.kind),['down','up']);
    assert.equal(f.sent[1].x,.8);assert.equal(f.sent[1].y,.9);
    assert.equal(f.context.pointerDown,false);
  }
  {
    const f=fixture(''),{image,event}=f;
    image.onpointerdown(event(.1,.1));
    for(let i=1;i<=20;i++)f.context.sendVisionPointer(event(i/25,.5),'move');
    image.onpointerup(event(.9,.6));await f.drain();
    assert.deepEqual(f.sent.map(item=>item.kind),['down','move','up']);
    assert.equal(f.sent[1].x,.8);
    assert.equal(f.sent[2].x,.9);
  }
  for(const method of ['onpointercancel','onlostpointercapture','blur']){
    const f=fixture(),{image,event}=f;image.onpointerdown(event(.2,.3));
    if(method==='blur')f.listeners.blur();else image[method](event(.4,.5));
    await f.drain();
    assert.deepEqual(f.sent.map(item=>item.kind),['down','right']);
    assert.equal(f.context.pointerDown,false);assert.equal(f.overlay.hidden,true);
  }
  for(const changed of ['selection_id','process_id']){
    const f=fixture(),{image,event}=f;
    image.onpointerdown(event(.2,.3));image.onpointerup(event(.5,.6));
    f.context.visionState[changed]=changed==='process_id'?2:'new';
    await f.drain();assert.deepEqual(f.sent.map(item=>item.kind),['down']);
  }
  {
    const f=fixture(),{image,event}=f;
    image.onpointerdown(event(.2,.3));image.onpointerup(event(.5,.6));
    f.pending.shift().reject(Error('offline'));await f.tick();
    assert.deepEqual(f.sent.map(item=>item.kind),['down']);
    assert.equal(f.overlay.hidden,true);assert.equal(f.context.pointerDown,false);
  }
  {
    const f=fixture('HEAD_DIRECTION'),{image,event}=f;
    image.onpointerdown(event(.5,.5));
    assert.equal(f.overlay.style.borderRadius,'50%');assert.equal(f.overlay.style.width,'12px');
    image.onpointerup(event(.5,.5));await f.drain();
    image.dataset.processId='0';image.onpointerdown(event(.2,.3));
    assert.equal(f.context.pointerDown,false,'old process image must not accept a new gesture');
    assert.equal(f.context.pointInImage({clientX:0,clientY:0},image),null);
    const clipped=f.context.pointInImage({clientX:-20,clientY:900},image,true);
    assert.equal(clipped.x,0);assert.equal(clipped.y,1);
  }
  console.log('9 pointer regressions passed: instant ROI/head feedback, reload-safe release, bounded moves, cancellation, stale sessions and errors.');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
