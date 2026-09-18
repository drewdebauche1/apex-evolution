export const ENGINE_VERSION='1.4';
export const DT=1/60, POP=50, WIDTH=70, MAX_SPEED=170, ACCELERATION=260, BRAKING=340;
export const SECTOR_COUNT=3;
export const WHEELBASE=20, MAX_STEER=1.0, STEER_RATE=6.5, LATERAL_GRIP=300;
export const NN_INPUTS=10, NN_HIDDEN=8, NN_OUTPUTS=2;
export const NN_WEIGHTS=NN_HIDDEN*(NN_INPUTS+1)+NN_OUTPUTS*(NN_HIDDEN+1);
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
// Chromosomes are the weights and biases of a 10 -> 8 -> 2 tanh network.
// This seed was calibrated in the deterministic physics simulation so evolution
// starts with basic driving ability instead of waiting to discover forward motion.
const BASELINE_NETWORK=[2.069775,-.198216,.055962,.087583,-.095877,.073432,-.114099,-.155642,-.12455,-.043738,-.050089,.123888,2.224874,.023793,-.075149,.232311,.008745,.058528,-.009685,-.167753,-.06635,-.093043,-.116369,-.022197,1.971408,.154552,-.065139,-.170434,.034073,.100768,.121801,.029611,-.071064,.054048,.024982,-.043503,1.533519,.206817,-.137329,.200267,-.009421,.230484,-.094134,-.182553,-.076892,-.049555,.073302,.066048,2.029333,-.158923,-.113553,.040568,.177715,-.099256,-.955496,.046985,-.138052,-.213207,.039411,-.062235,-.044038,.030557,.090839,1.966856,-.031079,-1.16765,.021194,.024693,.160223,.076474,-.069384,1.633191,-.1255,.046214,.151981,.040755,.196619,.078839,-.054103,.07727,-.053317,.080602,-.024293,.029106,.135026,.021742,2.102491,.17412,1.407219,.59148,.080059,-.423349,.169065,.013025,-.230041,.022523,.161604,.098566,.016472,-.005956,-.10389,-2.27572,2.081612,.065006,-.480911,.185327];
export function baselineNetwork(){return BASELINE_NETWORK.slice();}
export function randomGenes(rng=Math.random){const base=baselineNetwork();return base.map(v=>v+(rng()+rng()+rng()+rng()-2)*.18);}
export function normalizeGenes(_track,genes){
 if(Array.isArray(genes)&&genes.length===NN_WEIGHTS&&genes.every(Number.isFinite))return genes.slice();
 // Version 1.3 populations used short hand-tuned parameter arrays. Migrating
 // them to the neural baseline preserves tracks while starting valid networks.
 return baselineNetwork();
}
export function runNetwork(weights,inputs){
 if(weights.length!==NN_WEIGHTS||inputs.length!==NN_INPUTS)throw Error('Invalid neural-network shape.');
 const hidden=[],hiddenStride=NN_INPUTS+1,outputStart=NN_HIDDEN*hiddenStride,outputStride=NN_HIDDEN+1;
 for(let unit=0;unit<NN_HIDDEN;unit++){
  let sum=weights[unit*hiddenStride+NN_INPUTS];
  for(let input=0;input<NN_INPUTS;input++)sum+=weights[unit*hiddenStride+input]*inputs[input];
  hidden.push(Math.tanh(sum));
 }
 return Array.from({length:NN_OUTPUTS},(_,unit)=>{
  const start=outputStart+unit*outputStride;let sum=weights[start+NN_HIDDEN];
  for(let input=0;input<NN_HIDDEN;input++)sum+=weights[start+input]*hidden[input];
  return Math.tanh(sum);
 });
}
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
export function spawn(track,genes,id=0){const p=at(track,0);return{id,guide:racingGuide(track),guideProgress:0,genes:normalizeGenes(track,genes),x:p.x,y:p.y,heading:p.heading,speed:0,steer:0,time:0,progress:0,sectorTimes:[],nextSector:1,alive:true,finished:false,trace:[[p.x,p.y,p.heading]],ticks:0};}
export function finishCrossing(track,from,to,progress){const end=track.segments.at(-1),dx=(end.b[0]-end.a[0])/end.len,dy=(end.b[1]-end.a[1])/end.len;const before=(from.x-end.b[0])*dx+(from.y-end.b[1])*dy,after=(to.x-end.b[0])*dx+(to.y-end.b[1])*dy;if(before>0||after<0||after<=before||progress<end.start)return null;const fraction=-before/(after-before),x=from.x+(to.x-from.x)*fraction,y=from.y+(to.y-from.y)*fraction,lateral=-(x-end.b[0])*dy+(y-end.b[1])*dx;return Math.abs(lateral)<=TRACK_LIMIT?{fraction,x,y}:null;}
export function neuralInputs(a,t,speedLimit=MAX_SPEED){
 const route=a.guide||t,progress=a.guide?a.guideProgress:a.progress;
 const bearingError=distance=>{const target=at(route,Math.min(route.length,progress+distance));return angle(Math.atan2(target.y-a.y,target.x-a.x)-a.heading)/Math.PI;};
 const reference=at(route,progress),lateral=(-(a.x-reference.x)*Math.sin(reference.heading)+(a.y-reference.y)*Math.cos(reference.heading))/TRACK_LIMIT;
 const nextCorner=route.corners.find(corner=>corner.s>=progress);
 let signedBend=0,proximity=0,targetSpeedRatio=1;
 if(nextCorner){
  const distance=Math.max(0,nextCorner.s-progress),radius=nextCorner.radius??clamp(18/Math.tan(Math.abs(nextCorner.bend)/2),12,220);
  let cornerSpeed=Math.sqrt(LATERAL_GRIP*radius)*(0.72+Math.max(0,1.4-Math.abs(nextCorner.bend))*.18);
  if(Math.abs(nextCorner.bend)>2.4)cornerSpeed*=.65;
  const rawTarget=clamp(cornerSpeed/speedLimit,.15,1),brakingDistance=Math.max(0,(a.speed*a.speed-cornerSpeed*cornerSpeed)/(2*BRAKING));
  proximity=clamp(1-distance/Math.max(45,brakingDistance+70),0,1);
  targetSpeedRatio=1+(rawTarget-1)*proximity;signedBend=nextCorner.direction*nextCorner.bend/Math.PI;
 }
 const near=bearingError(clamp(16+a.speed*.045,16,30));
 return[near,bearingError(clamp(34+a.speed*.1,34,64)),bearingError(clamp(70+a.speed*.2,70,125)),clamp(lateral,-1.5,1.5),clamp(a.speed/speedLimit,0,1.5),clamp((a.steer||0)/MAX_STEER,-1,1),signedBend,proximity,targetSpeedRatio,Math.abs(near)];
}
// Every autonomous steering and throttle command comes from the neural policy.
export function aiControls(a,t,speedLimit=MAX_SPEED){
 const [turn,throttle]=runNetwork(a.genes,neuralInputs(a,t,speedLimit));
 return{turn,throttle};
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
export function breed(agents,t,rng=Math.random){const ranked=[...agents].sort((a,b)=>fitness(b,t)-fitness(a,t));return Array.from({length:POP},(_,i)=>{if(i<3)return spawn(t,ranked[i].genes,i);if(i>=45)return spawn(t,randomGenes(rng),i);const p=ranked[Math.floor(rng()*12)].genes,q=ranked[Math.floor(rng()*12)].genes;
const genes=p.map((value,k)=>{let next=rng()<.35?q[k]:value;if(rng()<.12)next+=(rng()+rng()+rng()+rng()-2)*.3;return clamp(next,-6,6);});return spawn(t,genes,i);});}
