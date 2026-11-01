const assert=require('node:assert/strict');
const R=require('./math.js');
const rows=[{time:0,xy:[0,0]},{time:.033,xy:[1,0]},{time:.066,xy:null},{time:.10,xy:[4,0]},{time:.4,xy:[8,0]}];
assert.equal(R.nearest(rows,.04),rows[1]);
assert.equal(R.nearest(rows,.07).xy,null,'A dropout must remain a dropout');
assert.equal(R.nearest(rows,-1),rows[0]);assert.equal(R.nearest(rows,10),rows[4]);
assert.equal(R.lowerBound(rows,.05),2);
const path=R.path(rows,'xy',-1,1);
assert.equal((path.match(/M/g)||[]).length,3,'Do not bridge absent observations or large time gaps');
assert.equal((path.match(/L/g)||[]).length,1);
assert.ok(Math.abs(R.angle(179,-179,.5)-180)<1e-9);
const robot=[{time:0,fused:[0,0],otos:[0,0],fused_heading:179,otos_heading:0,speed:0,command:0,phase:'free'},
             {time:.1,fused:[100,50],otos:[90,40],fused_heading:-179,otos_heading:0,speed:1,command:1,phase:'free'}];
assert.deepEqual(R.robotAt(robot,.05).fused,[50,25]);
assert.deepEqual(R.robotAt([{...robot[0]},{...robot[1],fused:[500,0]}],.05).fused,[0,0],'Do not interpolate a large initialization jump');
assert.deepEqual(R.robotAt([{...robot[0]},{...robot[1],time:1}],.5).fused,[0,0],'Do not interpolate long missing log intervals');
console.log('Replay math tests passed: nearest frame, dropout gaps, interpolation, angle wrapping and initialization jumps.');
