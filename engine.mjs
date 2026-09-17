export const ENGINE_VERSION='1.3';
export const DT=1/60, POP=50, WIDTH=70, MAX_SPEED=170, ACCELERATION=260, BRAKING=340;
export const SECTOR_COUNT=3;
export const WHEELBASE=20, MAX_STEER=1.0, STEER_RATE=6.5, LATERAL_GRIP=300;
export const TRACK_LIMIT=WIDTH/2-6;
export const USE_ROAD_POLYGON=false;
export const DEFAULT_TRACK=[[100,440],[240,440],[360,390],[385,255],[300,145],[420,90],[610,115],[720,230],[680,360],[800,450],[925,410],[950,230]];
export const STRAIGHT=[[100,270],[960,270]];
export const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export const angle=v=>Math.atan2(Math.sin(v),Math.cos(v));
// Sample the rounded centerline once; rendering and road checks use these exact points.
export function roundedTrackPoints(points,cornerRadius=0){
 const source=points.filter((p,i)=>!i||Math.hypot(p[0]-points[i-1][0],p[1]-points[i-1][1])>1e-9);
 if(source.length<3||cornerRadius<=0)return source.map(p=>[...p]);
 const result=[[...source[0]]];
 for(let i=1;i<source.length-1;i++){
  const a=source[i-1],b=source[i],c=source[i+1];
  const ab=Math.hypot(b[0]-a[0],b[1]-a[1]),bc=Math.hypot(c[0]-b[0],c[1]-b[1]);
  const trim=Math.min(cornerRadius,ab*.45,bc*.45);
  const entry=b.map((v,j)=>v+(a[j]-v)*trim/ab),exit=b.map((v,j)=>v+(c[j]-v)*trim/bc);
  result.push(entry);
  // Quadratic chord error is at most 0.1 px, including tight reversing bends.
  const curvature=Math.hypot(entry[0]-2*b[0]+exit[0],entry[1]-2*b[1]+exit[1]);
  const steps=Math.max(2,Math.ceil(Math.sqrt(curvature/.4)));
  for(let j=1;j<=steps;j++){
   const t=j/steps,u=1-t;
   result.push([u*u*entry[0]+2*u*t*b[0]+t*t*exit[0],u*u*entry[1]+2*u*t*b[1]+t*t*exit[1]]);
  }
 }
 result.push([...source.at(-1)]);
 return result;
}
export function makeTrack(points,cornerRadius=0){points=roundedTrackPoints(points,cornerRadius);let length=0;const segments=[];for(let i=1;i<points.length;i++){const a=points[i-1],b=points[i],len=Math.hypot(b[0]-a[0],b[1]-a[1]);if(len<1e-9)continue;segments.push({a,b,len,start:length});length+=len;}const corners=[],lineStations=[{s:0},{s:length}];

for(let i=1;i<segments.length;i++){const prev=segments[i-1],next=segments[i],corner=next.start;const bend=angle(Math.atan2(next.b[1]-next.a[1],next.b[0]-next.a[0])-Math.atan2(prev.b[1]-prev.a[1],prev.b[0]-prev.a[0])),direction=Math.sign(bend);if(Math.abs(bend)>.05)corners.push({s:corner,bend:Math.abs(bend),direction});lineStations.push({s:corner-prev.len*.35,direction,phase:-1},{s:corner,direction,phase:1},{s:corner+next.len*.35,direction,phase:-1});}
lineStations.sort((a,b)=>a.s-b.s);
const trackObj = {points,segments,length,lineStations,corners};
try{ if(USE_ROAD_POLYGON) makeRoad(trackObj, WIDTH); }catch(e){}
return trackObj;}
export function guideFromPoints(points,track){if(!points||!points.length)return null;return makeTrack(points.map(p=>[p[0],p[1]]));}
// Build road polygon (outer + inner offsets) and rounded inside corners.
export function makeRoad(track,width=WIDTH){
	const pts = track.points;
	const n = pts.length;
	if(n<2) return null;
	// segment normals
	const segNormals = [];
	for(let i=0;i<track.segments.length;i++){const s=track.segments[i];const dx=s.b[0]-s.a[0],dy=s.b[1]-s.a[1];const L=Math.hypot(dx,dy)||1;segNormals.push([-dy/L,dx/L]);}
	// vertex normals = average of adjacent segment normals
	const vNormals = [];
	for(let i=0;i<n;i++){
		let n1 = i-1<0?segNormals[0]:segNormals[i-1];
		let n2 = i-1>=track.segments.length?segNormals.at(-1):segNormals[Math.min(i,segNormals.length-1)];
		const nx = (n1[0]+n2[0]), ny=(n1[1]+n2[1]); const L=Math.hypot(nx,ny)||1; vNormals.push([nx/L,ny/L]);
	}
	const half = width/2;
	const outer = pts.map((p,i)=>[p[0]+vNormals[i][0]*half,p[1]+vNormals[i][1]*half]);
	const inner = pts.map((p,i)=>[p[0]-vNormals[i][0]*half,p[1]-vNormals[i][1]*half]);
	// build polygon: outer along track, then inner reversed
	const polygon = [];
	for(const p of outer) polygon.push(p);
	for(let i=inner.length-1;i>=0;i--) polygon.push(inner[i]);
	// simple rounding of inner corners: replace sharp inner vertices with small arc
	const roundedInner = [];
	for(let i=0;i<inner.length;i++){
		const prev = inner[(i-1+n)%n], cur = inner[i], next = inner[(i+1)%n];
		const vx1 = cur[0]-prev[0], vy1 = cur[1]-prev[1]; const a1=Math.atan2(vy1,vx1);
		const vx2 = next[0]-cur[0], vy2 = next[1]-cur[1]; const a2=Math.atan2(vy2,vx2);
		const da = ((a2 - a1 + Math.PI)%(2*Math.PI)) - Math.PI;
		const bend = Math.abs(da);
		if(bend>0.2 && bend<Math.PI){
			const steps = Math.min(6, Math.max(3, Math.floor(bend/(Math.PI/8))));
			for(let s=0;s<=steps;s++){
				const t = s/steps; const ang = a1 + da*t; const r=half;
				roundedInner.push([cur[0]+Math.cos(ang)*r, cur[1]+Math.sin(ang)*r]);
			}
		}else{
			roundedInner.push(cur);
		}
	}
	// rebuilt polygon with outer + rounded inner
	const poly2 = [...outer, ...roundedInner.slice().reverse()];
	track.road = {outer,inner,polygon:poly2,width};
	return track.road;
}

// point-in-polygon (ray-casting)
export function pointInPolygon(x,y,poly){let inside=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){const xi=poly[i][0], yi=poly[i][1], xj=poly[j][0], yj=poly[j][1];const intersect = ((yi>y)!=(yj>y)) && (x < (xj-xi)*(y-yi)/(yj-yi+1e-12)+xi); if(intersect) inside = !inside;}return inside;}

export function distanceToPolygonBoundary(x,y,poly){let best=Infinity;for(let i=0;i<poly.length;i++){const a=poly[i],b=poly[(i+1)%poly.length];const dx=b[0]-a[0],dy=b[1]-a[1];const t=clamp(((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy),0,1);const px=a[0]+t*dx, py=a[1]+t*dy;best=Math.min(best,Math.hypot(x-px,y-py));}return pointInPolygon(x,y,poly)?best:-best;}
export function at(track,s){s=clamp(s,0,track.length);const seg=track.segments.find(q=>q.start+q.len>=s)||track.segments.at(-1);const t=(s-seg.start)/seg.len;return{x:seg.a[0]+(seg.b[0]-seg.a[0])*t,y:seg.a[1]+(seg.b[1]-seg.a[1])*t,heading:Math.atan2(seg.b[1]-seg.a[1],seg.b[0]-seg.a[0])};}
export function nearest(track,x,y,maxProgress=Infinity){let best={distance:Infinity,s:0};for(const q of track.segments){const dx=q.b[0]-q.a[0],dy=q.b[1]-q.a[1],t=clamp(((x-q.a[0])*dx+(y-q.a[1])*dy)/(q.len*q.len),0,1);if(q.start+t*q.len>maxProgress)continue;const d=Math.hypot(x-q.a[0]-t*dx,y-q.a[1]-t*dy);if(d<best.distance)best={distance:d,s:q.start+t*q.len};}return best;}
export function randomGenes(rng=Math.random,track){const base=[20+rng()*100,.8+rng()*4,.2+rng()*.8,rng()*1.8,rng()*30-15];if(!track)return base;const centered=rng()<.5,guided=rng()<.5;return base.concat(track.lineStations.slice(1,-1).map(st=>centered?0:guided?clamp(st.direction*st.phase*(8+rng()*14)+(rng()-.5)*10,-24,24):(rng()-.5)*48));}
export function normalizeGenes(track,genes){const count=5+track.lineStations.length-2;return Array.from({length:count},(_,i)=>Number.isFinite(genes[i])?genes[i]:0);}
// Smoothly interpolate independently evolved entry, apex, and exit offsets.
export function racingOffset(track,genes,s){const stations=track.lineStations;s=clamp(s,0,track.length);let i=1;while(i<stations.length-1&&stations[i].s<s)i++;const left=stations[i-1],right=stations[i];const a=i===1?0:genes[5+i-2]||0,b=i===stations.length-1?0:genes[5+i-1]||0;const u=clamp((s-left.s)/Math.max(1,right.s-left.s),0,1),v=u*u*(3-2*u);return a+(b-a)*v;}
export function drivingTarget(track,genes,s){const p=at(track,s),before=at(track,s-12),after=at(track,s+12),heading=Math.atan2(after.y-before.y,after.x-before.x);let total=clamp((genes[4]||0)+racingOffset(track,genes,s),-24,24);
// Near a reversing bend, reserve space for the car's minimum turning radius.
// Blend into a modest outside offset on entry/exit instead of arbitrary evolved extremes.
for(const corner of track.corners){if(corner.bend<=2.5)continue;const blend=clamp((150-Math.abs(s-corner.s))/80,0,1);total=total*(1-blend)-corner.direction*16*blend;}
return{x:p.x-Math.sin(heading)*total,y:p.y+Math.cos(heading)*total};}
function roadClear(track,a,b,margin=4){
	const distance=Math.hypot(b[0]-a[0],b[1]-a[1]),steps=Math.max(1,Math.ceil(distance/4));
	for(let i=0;i<=steps;i++){
		const u=i/steps,x=a[0]+(b[0]-a[0])*u,y=a[1]+(b[1]-a[1])*u;
		if(track.road && track.road.polygon){
			const d = distanceToPolygonBoundary(x,y,track.road.polygon);
			if(d<margin) return false; // too close or outside
		}else{
			if(nearest(track,x,y).distance>TRACK_LIMIT-margin) return false;
		}
	}
	return true;
}
function shortcuts(points,track){const out=[];let i=0;while(i<points.length){out.push(points[i]);let best=i+1;for(let j=points.length-1;j>i+1;j--)if(roadClear(track,points[i],points[j])){best=j;break;}i=best;}return out;}
function rounded(points,track){let route=points.map(p=>[...p]);for(let pass=0;pass<2;pass++){const next=[route[0]];for(let i=1;i<route.length-1;i++){const a=route[i-1],b=route[i],c=route[i+1];const ab=Math.hypot(b[0]-a[0],b[1]-a[1]),bc=Math.hypot(c[0]-b[0],c[1]-b[1]),trim=Math.min(32,ab*.35,bc*.35);if(trim<5){next.push(b);continue;}const p=[b[0]+(a[0]-b[0])*trim/ab,b[1]+(a[1]-b[1])*trim/ab],q=[b[0]+(c[0]-b[0])*trim/bc,b[1]+(c[1]-b[1])*trim/bc];if(roadClear(track,p,q)){next.push(p,q);}else next.push(b);}next.push(route.at(-1));route=next;}return route;}
// Build the shortest legal route through the road surface, independent of the center line.
export function racingGuide(track){
if(track.fastGuide)return track.fastGuide;
const start=track.points[0],finish=track.points.at(-1),step=8,pad=TRACK_LIMIT+step;
const xs=track.points.map(p=>p[0]),ys=track.points.map(p=>p[1]);
const minX=Math.floor((Math.min(...xs)-pad)/step)*step,minY=Math.floor((Math.min(...ys)-pad)/step)*step;
const cols=Math.ceil((Math.max(...xs)+pad-minX)/step)+1,rows=Math.ceil((Math.max(...ys)+pad-minY)/step)+1,total=cols*rows;
const index=(x,y)=>y*cols+x,xy=i=>[i%cols,Math.floor(i/cols)],point=(x,y)=>[minX+x*step,minY+y*step];
const walkable=new Uint8Array(total);
for(let y=0;y<rows;y++)for(let x=0;x<cols;x++){
	const pt = point(x,y);
	if(track.road && track.road.polygon){
		walkable[index(x,y)] = pointInPolygon(pt[0],pt[1],track.road.polygon)?1:0;
	}else{
		walkable[index(x,y)] = nearest(track,...pt).distance<=TRACK_LIMIT-5?1:0;
	}
}
const closest=p=>{let best=0,bd=Infinity;for(let i=0;i<total;i++){if(!walkable[i])continue;const [x,y]=xy(i),q=point(x,y),d=Math.hypot(q[0]-p[0],q[1]-p[1]);if(d<bd){bd=d;best=i;}}return best;};
const startI=closest(start),finishI=closest(finish),open=[startI],g=new Float64Array(total),f=new Float64Array(total),came=new Int32Array(total),closed=new Uint8Array(total);
g.fill(Infinity);f.fill(Infinity);came.fill(-1);g[startI]=0;
const h=i=>{const [x,y]=xy(i),p=point(x,y);return Math.hypot(p[0]-finish[0],p[1]-finish[1]);};
f[startI]=h(startI);
while(open.length){
 let bi=0,bf=f[open[0]];for(let i=1;i<open.length;i++)if(f[open[i]]<bf){bf=f[open[i]];bi=i;}
 const current=open.splice(bi,1)[0];if(current===finishI)break;closed[current]=1;
 const [cx,cy]=xy(current),cp=point(cx,cy);
 for(let oy=-1;oy<=1;oy++)for(let ox=-1;ox<=1;ox++){if(!ox&&!oy)continue;const nx=cx+ox,ny=cy+oy;if(nx<0||ny<0||nx>=cols||ny>=rows)continue;const ni=index(nx,ny);if(!walkable[ni]||closed[ni])continue;const np=point(nx,ny);if(!roadClear(track,cp,np,5))continue;const tentative=g[current]+Math.hypot(np[0]-cp[0],np[1]-cp[1]);if(tentative>=g[ni])continue;came[ni]=current;g[ni]=tentative;f[ni]=tentative+h(ni);if(!open.includes(ni))open.push(ni);}
}
if(came[finishI]<0){track.fastGuide=null;return null;}
const cells=[];for(let i=finishI;i>=0;i=came[i]){cells.push(i);if(i===startI)break;}cells.reverse();
let points=[start,...cells.slice(1,-1).map(i=>point(...xy(i))),finish];
points=rounded(shortcuts(points,track),track).filter((p,i,a)=>!i||Math.hypot(p[0]-a[i-1][0],p[1]-a[i-1][1])>2);
const guide=makeTrack(points);const arcs=[];for(const corner of guide.corners){if(Math.abs(corner.bend)<.6)continue;const prev=guide.segments.filter(seg=>seg.start+seg.len<=corner.s).at(-1)||guide.segments[0];const next=guide.segments.filter(seg=>seg.start>=corner.s).at(0)||guide.segments.at(-1);const start=Math.max(0,corner.s-(prev?.len||0)*.35);const end=Math.min(guide.length,corner.s+(next?.len||0)*.35);const arc={...corner,start,end,radius:clamp(18/Math.tan(corner.bend/2),8,180)};const last=arcs.at(-1);if(last&&Math.abs(start-last.end)<60&&Math.sign(corner.direction)===Math.sign(last.direction)){last.end=end;last.radius=Math.max(last.radius,arc.radius);last.bend=Math.max(last.bend,arc.bend);continue;}arcs.push(arc);}guide.arcs=arcs;guide.source=points.map((_,i)=>track.length*i/(points.length-1));track.fastGuide=guide.length<track.length*.98?guide:null;return track.fastGuide;
}
function sourceProgress(guide,s){return guide.length?clamp(s/guide.length,0,1)*(guide.source?.at(-1)||guide.length):s;}
function guideTarget(track,guide,genes,s){
 const p=at(guide,s);
 return{x:p.x,y:p.y};
}
export function spawn(track,genes,id=0){const p=at(track,0);return{id,guide:racingGuide(track),guideProgress:0,genes:normalizeGenes(track,genes),x:p.x,y:p.y,heading:p.heading,speed:0,steer:0,time:0,progress:0,sectorTimes:[],nextSector:1,alive:true,finished:false,trace:[[p.x,p.y,p.heading]],ticks:0};}
export function finishCrossing(track,from,to,progress){const end=track.segments.at(-1),dx=(end.b[0]-end.a[0])/end.len,dy=(end.b[1]-end.a[1])/end.len;const before=(from.x-end.b[0])*dx+(from.y-end.b[1])*dy,after=(to.x-end.b[0])*dx+(to.y-end.b[1])*dy;if(before>0||after<0||after<=before||progress<end.start)return null;const fraction=-before/(after-before),x=from.x+(to.x-from.x)*fraction,y=from.y+(to.y-from.y)*fraction,lateral=-(x-end.b[0])*dy+(y-end.b[1])*dx;return Math.abs(lateral)<=TRACK_LIMIT?{fraction,x,y}:null;}
// Drive with a sane target speed on the actual track, and use the guide route only as a soft steering aid.
export function aiControls(a,t,speedLimit=MAX_SPEED){
const [look,gain,pace,brake]=a.genes;
const route=a.guide||t,progress=a.guide?a.guideProgress:a.progress;
const lookAhead=clamp(16+look*.18+a.speed*.12,16,70);
const pathTarget=at(route,progress+lookAhead);
let target={x:pathTarget.x,y:pathTarget.y};
for(const corner of route.corners){
const delta=corner.s-progress;
if(delta<10||delta>160)continue;
const weight=clamp((160-delta)/160,0,1);
	// Reduce lane bias at higher speeds to avoid aggressive apexing that causes overshoot.
	let laneBias=(8+Math.abs(corner.bend)*12)*weight;
	laneBias *= clamp(1 - a.speed/250, 0.45, 1);
	laneBias *= clamp(1.05-brake*.08,.85,1.05);
const normalX=-Math.sin(pathTarget.heading),normalY=Math.cos(pathTarget.heading);
target={x:pathTarget.x+normalX*corner.direction*laneBias,y:pathTarget.y+normalY*corner.direction*laneBias};
break;
}
if(a.guide)target=guideTarget(t,a.guide,a.genes,Math.min(a.guideProgress+lookAhead,a.guide.length));
const dx=target.x-a.x,dy=target.y-a.y,distance=Math.max(6,Math.hypot(dx,dy));
const error=angle(Math.atan2(dy,dx)-a.heading);
const speedBoost=1;
const steering=Math.abs(error)>Math.PI/2?Math.sign(error)*MAX_STEER:
Math.atan(2*WHEELBASE*Math.sin(error)/distance)*clamp((1.05+gain*.12)*speedBoost,1.0,2.2);
let desired=speedLimit*clamp(.25+pace*.75,.35,1);
const caution=clamp(1.18-brake*.22,.5,1.2);
for(const corner of route.corners){
	if(corner.s<progress)continue;
	const radius=corner.radius??clamp(18/Math.tan(Math.abs(corner.bend)/2),12,220);
	let cornerSpeed=Math.sqrt(LATERAL_GRIP*radius*caution)*(0.72+Math.max(0,1.4-Math.abs(corner.bend))*0.18);
	// Let evolution choose between a cautious apex and carrying momentum through a wider line.
	const momentumBias=clamp(1.12-brake*.14,.78,1.12);
	cornerSpeed*=momentumBias;
	// Sharp corners need a lower apex speed, but not an immediate half-speed penalty.
	if(Math.abs(corner.bend)>2.4) cornerSpeed*=.65;
	const distanceToCorner=Math.max(0,corner.s-progress);
	const brakingDistance=Math.max(0,(a.speed*a.speed-cornerSpeed*cornerSpeed)/(2*BRAKING));
	const brakeWindow=brakingDistance+Math.max(Math.abs(corner.bend)>2.4?42:28,a.speed*(Math.abs(corner.bend)>2.4?.5:.35));
	const urgency=clamp(1-distanceToCorner/Math.max(1,brakeWindow),0,1);
	// Hold top speed until braking is physically needed, then blend smoothly to apex speed.
	desired=Math.min(desired,speedLimit+(cornerSpeed-speedLimit)*urgency);
}
const headingPenalty=clamp(Math.abs(error)/1.2,0,1);
desired=Math.min(desired,speedLimit*(1-headingPenalty*.48));
// Test the normal target and one faster momentum target before committing.
// Low-brake agents get a larger opportunity to carry speed, but only when legal.
{
	const simTurn=clamp(steering/MAX_STEER,-1,1);
	const momentumFactor=clamp(1.08+(1-brake)*.04,.98,1.12);
	const predictSafe=targetSpeed=>{
		let simSpeed=a.speed,simX=a.x,simY=a.y,simHeading=a.heading,simSteer=a.steer||0;
		const simThrottleBase=clamp((targetSpeed-simSpeed)/55,-1,1);
		for(let i=0;i<12;i++){
			simSpeed=clamp(simSpeed+simThrottleBase*(simThrottleBase>0?ACCELERATION:BRAKING)*DT-2*DT,0,Math.max(speedLimit,1));
			const desiredSteerSim=simTurn*MAX_STEER;
			const steerRateSim=STEER_RATE*(1+clamp(simSpeed/170,0,1.25)*0.5);
			simSteer+=clamp(desiredSteerSim-simSteer,-steerRateSim*DT,steerRateSim*DT);
			const yawSim=simSpeed/WHEELBASE*Math.tan(simSteer),maxYawSim=LATERAL_GRIP/Math.max(simSpeed,1);
			simHeading=angle(simHeading+clamp(yawSim,-maxYawSim,maxYawSim)*DT);
			simX+=Math.cos(simHeading)*simSpeed*DT;simY+=Math.sin(simHeading)*simSpeed*DT;
			if(nearest(t,simX,simY).distance>TRACK_LIMIT)return false;
		}
		return true;
	};
	const baseSafe=predictSafe(desired);
	if(baseSafe&&predictSafe(Math.min(speedLimit,desired*momentumFactor)))desired=Math.min(speedLimit,desired*momentumFactor);
	else if(!baseSafe)desired*=.5;
}
const delta=desired-a.speed;
return{turn:clamp(steering/MAX_STEER,-1,1),throttle:clamp(delta/40,-1,1)};
}
export function step(a,t,manual,speedLimit=MAX_SPEED){if(!a.alive)return;speedLimit=Number.isFinite(speedLimit)?clamp(speedLimit,25,300):MAX_SPEED;a.speedMin=a.time===0?speedLimit:Math.min(a.speedMin??MAX_SPEED,speedLimit);a.speedMax=a.time===0?speedLimit:Math.max(a.speedMax??MAX_SPEED,speedLimit);
const {turn,throttle}=manual||aiControls(a,t,speedLimit);
const previous={x:a.x,y:a.y,heading:a.heading};
const inputThrottle=clamp(throttle,-1,1),coastDrag=inputThrottle===0?135:2;
a.speed=clamp(a.speed+inputThrottle*(inputThrottle>0?ACCELERATION:BRAKING)*DT-coastDrag*DT,0,speedLimit);
// Rate-limited road-wheel angle, bicycle geometry, and a grip limit: no rotation at rest.
const desiredSteer=clamp(turn,-1,1)*MAX_STEER;
a// faster steering rate at speed (keeps baseline at low speed)
const steerRate=STEER_RATE;
a.steer=(a.steer||0)+clamp(desiredSteer-(a.steer||0),-steerRate*DT,steerRate*DT);
const yaw=a.speed/WHEELBASE*Math.tan(a.steer),maxYaw=LATERAL_GRIP/Math.max(a.speed,1);
a.heading=angle(a.heading+clamp(yaw,-maxYaw,maxYaw)*DT);
a.x+=Math.cos(a.heading)*a.speed*DT;a.y+=Math.sin(a.heading)*a.speed*DT;a.time+=DT;a.ticks++;
if(t.openArea){
	if(a.x<35)a.x=1025;else if(a.x>1025)a.x=35;
	if(a.y<35)a.y=505;else if(a.y>505)a.y=35;
}
let n=t.openArea?{distance:0,s:0}:nearest(t,a.x,a.y);// Nearby return lanes are valid road, but are not necessarily the next route segment.
if(manual&&!t.openArea&&n.distance>TRACK_LIMIT){
	a.x=previous.x;a.y=previous.y;a.heading=previous.heading;a.speed=0;a.alive=false;a.stopReason='off-track';
	return;
}
// Small numerical drift can push the car fractionally past the TRACK_LIMIT.
// Nudge it back onto the road when it's only slightly over the limit.
// Increase tolerance slightly to recover marginal overshoots observed in probes.
if(t.road && t.road.polygon){
	const d = distanceToPolygonBoundary(a.x,a.y,t.road.polygon);
	if(d<0 && d>=-3.0){
		const p=at(t,n.s);
		const vx=a.x-p.x,vy=a.y-p.y,vd=Math.hypot(vx,vy)||1;const targetDist=TRACK_LIMIT-0.01;
		a.x=p.x+vx/vd*targetDist; a.y=p.y+vy/vd*targetDist; n=nearest(t,a.x,a.y);
	}
}else{
	if(n.distance>TRACK_LIMIT && n.distance<=TRACK_LIMIT+3.0){
		const p=at(t,n.s);
		const vx=a.x-p.x,vy=a.y-p.y,vd=Math.hypot(vx,vy)||1;
		const targetDist=TRACK_LIMIT-0.01;
		a.x=p.x+vx/vd*targetDist;
		a.y=p.y+vy/vd*targetDist;
		// refresh nearest after the nudge
		n=nearest(t,a.x,a.y);
	}
}
// Search for the nearest reachable projection instead of rejecting the global winner.
if(t.openArea){
	a.progress=0;
}else if(a.guide){
	const guideRoute=nearest(a.guide,a.x,a.y,a.guideProgress+WIDTH+a.speed*DT);
	a.guideProgress=Math.max(a.guideProgress,guideRoute.s);
	const mapped=sourceProgress(a.guide,a.guideProgress);
	const trackRoute=nearest(t,a.x,a.y,a.progress+WIDTH+a.speed*DT);
	a.progress=Math.max(a.progress,mapped,trackRoute.s);
}else{
	const route=nearest(t,a.x,a.y,a.progress+WIDTH+a.speed*DT);
	a.progress=Math.max(a.progress,route.s);
}
while(!t.openArea&&a.nextSector<SECTOR_COUNT&&a.progress>=t.length*a.nextSector/SECTOR_COUNT){a.sectorTimes.push(a.time);a.nextSector++;}
const crossing=t.openArea?null:finishCrossing(t,previous,a,a.progress);if(crossing){a.x=crossing.x;a.y=crossing.y;a.heading=previous.heading+angle(a.heading-previous.heading)*crossing.fraction;a.time-=DT*(1-crossing.fraction);a.progress=t.length;while(a.nextSector<=SECTOR_COUNT){a.sectorTimes.push(a.time);a.nextSector++;}a.finished=true;a.alive=false;a.stopReason='finished';a.trace.push([a.x,a.y,a.heading]);return;}
if(a.ticks%3===0)a.trace.push([a.x,a.y,a.heading]);if(!t.openArea&&n.distance>TRACK_LIMIT){a.alive=false;a.stopReason='off-track';}else if(!t.openArea&&a.time>t.length/20+15){a.alive=false;a.stopReason='timeout';}}

export const fitness=(a,t)=>a.finished?2+t.length/(MAX_SPEED*Math.max(a.time,.01)):a.progress/t.length;
export function breed(agents,t,rng=Math.random){const ranked=[...agents].sort((a,b)=>fitness(b,t)-fitness(a,t));const bounds=[[12,150],[.3,7],[.2,1],[0,2.5],[-22,22]],scales=[14,.55,.12,.25,4];return Array.from({length:POP},(_,i)=>{if(i<3)return spawn(t,ranked[i].genes,i);if(i>=45)return spawn(t,randomGenes(rng,t),i);const p=ranked[Math.floor(rng()*12)].genes,q=ranked[Math.floor(rng()*12)].genes;const chosenCorner=Math.floor(rng()*Math.max(1,(p.length-5)/3)),tuneControls=rng()<.35;
const genes=p.map((v,k)=>{if(k<5)return tuneControls?clamp(v+(rng()-.5)*2*scales[k],...bounds[k]):v;const corner=Math.floor((k-5)/3);if(corner!==chosenCorner)return v;const inherited=rng()<.15?q[k]:v;return clamp(inherited+(rng()-.5)*(rng()<.5?5:14),-24,24);});return spawn(t,genes,i);});}
