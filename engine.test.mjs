import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {DT,MAX_STEER,STEER_RATE,LATERAL_GRIP,STRAIGHT,DEFAULT_TRACK,makeTrack,spawn,step,aiControls,randomGenes,breed,nearest} from './engine.mjs';

const genes=[70,2,1,1,0];
const rightAngle=[[100,400],[500,400],[500,100],[900,100]];
const hairpin=[[100,400],[800,400],[600,280],[100,280]];
// Approximately the user's screenshot: two very tight, overlapping switchbacks.
const switchbacks=[[100,270],[960,270],[300,170],[580,100],[860,100]];

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

test('AI brakes before a sharp corner even with the least cautious brake gene',()=>{
  const track=makeTrack(rightAngle),car=spawn(track,[70,2,1,0,0]);
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
  assert.equal(aiControls(car,track).turn,-1);
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
