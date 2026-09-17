import fs from 'fs';
const inPath = process.argv[2] || 'tools/synthetic_waypoints.json';
const outPath = process.argv[3] || 'tools/guide_from_image.js';
const data = JSON.parse(fs.readFileSync(inPath,'utf8'));
const pts = data.points.map(p=>[Math.round(p[0]*100)/100, Math.round(p[1]*100)/100]);
const content = `// Auto-generated guide points from image
export default ${JSON.stringify(pts, null, 2)};
`;
fs.writeFileSync(outPath, content);
console.log('Wrote', outPath);