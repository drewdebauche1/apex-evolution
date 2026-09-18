import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {DT,MAX_STEER,STEER_RATE,LATERAL_GRIP,TRACK_LIMIT,LEGACY_NN_WEIGHTS,NN_INPUTS,NN_HIDDEN,NN_WEIGHTS,STRAIGHT,DEFAULT_TRACK,makeTrack,spawn,step,aiControls,neuralInputs,roadSensors,runNetwork,baselineNetwork,randomGenes,trialFitness,breed,nearest,at} from './engine.mjs';

const genes=baselineNetwork();
const rightAngle=[[100,400],[500,400],[500,100],[900,100]];
const hairpin=[[100,400],[800,400],[600,280],[100,280]];
// Approximately the user's screenshot: two very tight, overlapping switchbacks.
const switchbacks=[[100,270],[960,270],[300,170],[580,100],[860,100]];

test('the driving policy is a 15-input, 8-hidden, 2-output neural network',()=>{
  const weights=baselineNetwork(),track=makeTrack(STRAIGHT),car=spawn(track,weights);
  assert.equal(NN_INPUTS,15);
  assert.equal(NN_HIDDEN,8);
  assert.equal(weights.length,NN_WEIGHTS);
  assert.equal(neuralInputs(car,track).length,NN_INPUTS);
  assert.equal(runNetwork(weights,neuralInputs(car,track)).length,2);
  assert.throws(()=>runNetwork(weights,[0]),/shape/);
});

test('neural output biases directly control steering and throttle',()=>{
  const weights=new Array(NN_WEIGHTS).fill(0),inputs=new Array(NN_INPUTS).fill(0);
  const outputStart=NN_HIDDEN*(NN_INPUTS+1),outputStride=NN_HIDDEN+1;
  weights[outputStart+NN_HIDDEN]=1;
  weights[outputStart+outputStride+NN_HIDDEN]=-1;
  const [turn,throttle]=runNetwork(weights,inputs);
  assert.equal(turn,Math.tanh(1));
  assert.equal(throttle,Math.tanh(-1));
});

test('legacy driving parameters migrate to finite neural weights',()=>{
  for(const legacy of [[70,2,1,1,0],new Array(LEGACY_NN_WEIGHTS).fill(.25)]){
    const car=spawn(makeTrack(STRAIGHT),legacy);
    assert.equal(car.genes.length,NN_WEIGHTS);
    assert.ok(car.genes.every(Number.isFinite));
  }
});

test('road sensors report open space ahead and nearby side boundaries',()=>{
  const sensors=roadSensors(makeTrack(STRAIGHT),300,270,0);
  assert.equal(sensors.length,5);
  assert.equal(sensors[2],1);
  assert.ok(sensors[0]<.5&&sensors[4]<.5);
});

test('fitness rewards clearance and smooth control after race outcome',()=>{
  const track=makeTrack(STRAIGHT),safe={finished:true,time:10,clearanceTotal:80,clearanceSamples:100,steeringChange:1,ticks:100},risky={...safe,clearanceTotal:10,steeringChange:8};
  assert.ok(trialFitness(safe,track)>trialFitness(risky,track));
});

test('rounded road collision checks follow the visible curve instead of the sharp corner',()=>{
  const points=[[100,400],[500,400],[500,100]],track=makeTrack(points,120),sharp=makeTrack(points);
  for(const [x,y,inside] of [[470,370,true],[515,415,false]]){
    assert.equal(nearest(track,x,y).distance<=TRACK_LIMIT,inside);
    assert.equal(nearest(sharp,x,y).distance<=TRACK_LIMIT,!inside);
    const car=spawn(track,genes);car.x=x;car.y=y;
    step(car,track,{turn:0,throttle:0});
    assert.equal(car.alive,inside);
  }
  assert.deepEqual(track.points[0],points[0]);
  assert.deepEqual(track.points.at(-1),points.at(-1));
  assert.equal(at(track,0).heading,0);
  assert.equal(at(track,track.length).heading,-Math.PI/2);
});

test('AI and manual runs are both eliminated after crossing the track boundary',()=>{
  const track=makeTrack(STRAIGHT),manual=spawn(track,genes),ai=spawn(track,genes);
  for(const car of [manual,ai]){car.x=300;car.y=270+TRACK_LIMIT+1;car.speed=0;}
  step(manual,track,{turn:0,throttle:0});
  step(ai,track);
  assert.equal(manual.alive,false);
  assert.equal(ai.alive,false);
  assert.equal(manual.stopReason,'off-track');
  assert.equal(ai.stopReason,'off-track');
});

test('rounded centerline stays within 0.1 px of the quadratic shown in the editor',()=>{
  for(const radius of [0,1,52,120]){
    const points=[[100,400],[500,400],[500,100]],track=makeTrack(points,radius);
    for(let i=0;i<=100;i++){
      const t=i/100,u=1-t;
      const x=u*u*(500-radius)+2*u*t*500+t*t*500;
      const y=u*u*400+2*u*t*400+t*t*(400-radius);
      assert.ok(nearest(track,x,y).distance<=.101);
    }
    if(radius===0)assert.deepEqual(track.points,points);
    assert.deepEqual(makeTrack(STRAIGHT,radius).points,STRAIGHT);
  }
});

test('AI completes rounded courses using the same road geometry as manual driving',()=>{
  for(const points of [DEFAULT_TRACK,rightAngle,[[150,190],[480,100],[760,330]]]){
    const track=makeTrack(points,52),car=spawn(track,genes);
    for(let tick=0;tick<15000&&car.alive;tick++)step(car,track,undefined,125);
    assert.ok(car.finished,`Rounded course ${JSON.stringify(points)}: ${car.stopReason}`);
  }
});

test('full braking stops from cruising and maximum speed quickly',()=>{
  for(const speed of [125,300]){
    const track=makeTrack(STRAIGHT),car=spawn(track,genes);
    car.speed=speed;
    while(car.speed>0&&car.alive)step(car,track,{turn:0,throttle:-1},speed);
    assert.equal(car.speed,0);
    assert.ok(car.alive);
    assert.ok(car.time<(speed===125?.4:.9));
    assert.ok(car.x-100<(speed===125?24:133));
  }
});

test('steering cannot rotate a stopped car and reverses gradually',()=>{
  const track=makeTrack(STRAIGHT),car=spawn(track,genes);
  for(let i=0;i<60;i++)step(car,track,{turn:1,throttle:0});
  assert.equal(car.heading,0);
  assert.equal(car.x,100);
  assert.equal(car.steer,MAX_STEER);
  step(car,track,{turn:-1,throttle:0});
  assert.ok(Math.abs(car.steer-(MAX_STEER-STEER_RATE*DT))<1e-10);
});

test('high-speed turning stays within the lateral grip limit',()=>{
  const track=makeTrack(STRAIGHT),car=spawn(track,genes);
  car.speed=300;
  car.steer=MAX_STEER;
  step(car,track,{turn:1,throttle:1},300);
  assert.ok(Math.abs(car.heading)/DT<=LATERAL_GRIP/car.speed+1e-10);
});

test('the baseline neural policy brakes before a sharp corner',()=>{
  const track=makeTrack(rightAngle),car=spawn(track,genes);
  car.progress=220;
  car.x=320;
  car.speed=300;
  assert.ok(aiControls(car,track,300).throttle<0);
});

test('a centered AI completes straight, curved, right-angle and hairpin courses',()=>{
  for(const points of [STRAIGHT,DEFAULT_TRACK,rightAngle,hairpin,switchbacks,switchbacks.map(([x,y])=>[1060-x,y])]){
    for(const speed of [25,125,300]){
      const track=makeTrack(points),car=spawn(track,genes);
      for(let i=0;i<15000&&car.alive;i++)step(car,track,undefined,speed);
      assert.ok(car.finished,`Course ${JSON.stringify(points)} at ${speed}: ${car.stopReason}`);
      assert.equal(car.progress,track.length);
    }
  }
});

test('a target behind the car commands a tight turn instead of unwinding steering',()=>{
  const track=makeTrack(STRAIGHT),car=spawn(track,genes);
  car.x=200;
  car.y=275;
  assert.ok(aiControls(car,track).turn<0);
});

test('most seeded drivers complete the screenshot-style switchbacks',()=>{
  let seed=42;
  const rng=()=>((seed=(1664525*seed+1013904223)>>>0)/4294967296);
  const track=makeTrack(switchbacks);
  let finishers=0;
  for(let i=0;i<100;i++){
    const car=spawn(track,randomGenes(rng,track));
    for(let tick=0;tick<15000&&car.alive;tick++)step(car,track);
    if(car.finished)finishers++;
  }
  assert.ok(finishers>=75,`${finishers}/100 finished`);
});

test('a nearby return lane does not freeze progress on the current straight',()=>{
  const track=makeTrack(switchbacks),car=spawn(track,genes);
  car.x=700;
  car.y=250;
  car.progress=600;
  car.speed=125;
  assert.ok(nearest(track,car.x,car.y).s>car.progress+70);
  step(car,track,{turn:0,throttle:0});
  assert.ok(car.alive);
  assert.ok(car.progress>601&&car.progress<603);
});

test('the actual Track 7 saved population takes legal cuts and beats its old champion',()=>{
  const data=JSON.parse(fs.readFileSync(new URL('./track7-fixture.json',import.meta.url)));
  const track=makeTrack(data.custom);
  let finishers=0,best=Infinity;
  for(const genes of data.genes){
    const car=spawn(track,genes);
    assert.equal(car.guide.arcs.length,2);
    for(const p of car.guide.points)assert.ok(nearest(track,...p).distance<=25.01);
    for(let i=0;i<15000&&car.alive;i++){
      step(car,track,undefined,data.carSpeed);
      assert.ok(nearest(track,car.x,car.y).distance<=29.001);
    }
    if(car.finished){finishers++;best=Math.min(best,car.time);}
  }
  assert.equal(finishers,50);
  assert.ok(best<11.4,`Expected at least 20% improvement on 14.23 seconds; got ${best}`);
});

test('evolving drivers retain successful finishers with the new controls',()=>{
  let seed=42;
  const rng=()=>((seed=(1664525*seed+1013904223)>>>0)/4294967296);
  const track=makeTrack(DEFAULT_TRACK);
  let population=Array.from({length:50},(_,i)=>spawn(track,randomGenes(rng,track),i));
  for(let generation=0;generation<5;generation++){
    for(const car of population)for(let i=0;i<15000&&car.alive;i++)step(car,track,undefined,300);
    assert.ok(population.some(car=>car.finished));
    population=breed(population,track,rng);
  }
});
