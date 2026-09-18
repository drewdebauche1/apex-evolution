import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import * as engine from './engine.mjs';

// Run real editor handlers with a recording canvas and in-memory browser storage.
function app(){
 const strokes=[];let path=[];
 const context=new Proxy({
  beginPath(){path=[];},moveTo(x,y){path.push([x,y]);},lineTo(x,y){path.push([x,y]);},
  stroke(){strokes.push({color:this.strokeStyle,points:path.slice()});}
 },{get:(o,k)=>k in o?o[k]:()=>{}});
 const elements=new Map();
 const element=()=>({set id(id){elements.set(id,this);},value:'1',width:1060,height:540,checked:false,
  classList:{toggle(){},add(){},remove(){}},append(){},prepend(){},insertBefore(){},
  addEventListener(){},blur(){},getContext:()=>context});
 const get=id=>{if(!elements.has(id)){const e=element();e.parentElement=element();elements.set(id,e);}return elements.get(id);};
 const custom=[[100,400],[500,400],[500,100]];
 const storage=new Map([['apex-evolution-rounded-reset','1'],['apex-evolution-v1',JSON.stringify({version:2,activeTrack:1,tracks:{1:{custom,cornerRadius:52},2:{custom,cornerRadius:0}}})]]);
 const sandbox=vm.createContext({...engine,document:{getElementById:get,createElement:element,querySelector:get,querySelectorAll:()=>[],addEventListener(){},body:element()},
  window:{addEventListener(){}},localStorage:{getItem:k=>storage.get(k),setItem:(k,v)=>storage.set(k,v),removeItem:k=>storage.delete(k)},
  requestAnimationFrame(){},setTimeout(){},clearTimeout(){},confirm:()=>true,performance:{now:()=>0}});
 vm.runInContext(fs.readFileSync(new URL('./app.mjs',import.meta.url),'utf8').replace(/^import[^\n]+\n/,''),sandbox);
 return {get,storage,run:code=>vm.runInContext(code,sandbox),road:()=>strokes.filter(s=>s.color==='#263540').at(-1).points};
}

test('canvas road, saved radius, editor preview and cancellation share consistent geometry',()=>{
 const ui=app();
 assert.equal(JSON.stringify(ui.road()),ui.run('JSON.stringify(track.points)'));
 ui.get('edit').onclick();
 ui.get('cornerRadius').value='120';ui.get('cornerRadius').oninput();
 const preview=JSON.stringify(ui.road());
 assert.notEqual(preview,ui.run('JSON.stringify(track.points)'));
 ui.get('cancelEdit').onclick();
 assert.equal(JSON.stringify(ui.road()),ui.run('JSON.stringify(track.points)'));
 assert.equal(ui.run('cornerRadius'),52);
 ui.get('edit').onclick();
 ui.get('cornerRadius').value='120';ui.get('cornerRadius').oninput();
 ui.get('apply').onclick();
 assert.equal(ui.run('cornerRadius'),120);
 assert.equal(JSON.stringify(ui.road()),preview);
 assert.equal(JSON.parse(ui.storage.get('apex-evolution-v1')).tracks[1].cornerRadius,120);
 ui.get('trackSelect').value='2';ui.get('trackSelect').onchange();
 assert.equal(ui.run('cornerRadius'),0);
 assert.equal(JSON.stringify(ui.road()),ui.run('JSON.stringify(custom)'));
});

test('a generation evaluates each network across three perturbed rollouts',()=>{
 const ui=app(),finish=()=>ui.run('agents.forEach((a,i)=>{a.alive=false;a.finished=true;a.time=10+i/100;a.progress=track.length;});finishGeneration()');
 finish();assert.equal(ui.run('evaluationRound'),2);assert.equal(ui.run('generation'),1);
 finish();assert.equal(ui.run('evaluationRound'),3);assert.equal(ui.run('generation'),1);
 finish();assert.equal(ui.run('evaluationRound'),1);assert.equal(ui.run('generation'),2);
 assert.equal(ui.run('history.at(-1).count'),50);
});

test('training speed is locked after a generation starts',()=>{
 const ui=app(),speed=ui.get('carSpeed');
 speed.value='300';speed.oninput();assert.equal(ui.run('generationSpeed'),300);
 ui.run('running=true;simTime=1');speed.value='25';speed.oninput();
 assert.equal(ui.run('carSpeed'),25);
 assert.equal(ui.run('generationSpeed'),300);
});
