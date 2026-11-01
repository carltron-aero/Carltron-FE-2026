(function(root) {
  'use strict';
  function lowerBound(rows, time) {
    let lo=0, hi=rows.length;
    while(lo<hi) {const mid=(lo+hi)>>1;if(rows[mid].time<time)lo=mid+1;else hi=mid;}
    return lo;
  }
  function nearest(rows, time) {
    const i=lowerBound(rows,time);
    if(i===0)return rows[0];if(i===rows.length)return rows[i-1];
    return time-rows[i-1].time<=rows[i].time-time?rows[i-1]:rows[i];
  }
  function angle(a,b,f) {return a==null||b==null?null:a+(((b-a+540)%360)-180)*f;}
  function robotAt(rows,time) {
    const i=lowerBound(rows,time);
    if(i===0)return {...rows[0]};if(i===rows.length)return {...rows[i-1]};
    const a=rows[i-1],b=rows[i],dt=b.time-a.time;
    const f=dt<=0||dt>.15?0:(time-a.time)/dt;
    const p={...a,time};
    for(const key of ['fused','otos']) {
      const aa=a[key],bb=b[key];
      const fraction=aa&&bb&&Math.hypot(bb[0]-aa[0],bb[1]-aa[1])<=150?f:0;
      p[key]=!aa?null:!bb?[...aa]:aa.map((x,j)=>x+(bb[j]-x)*fraction);
      p[key+'_heading']=angle(a[key+'_heading'],b[key+'_heading'],fraction);
    }
    for(const key of ['speed','command'])p[key]=a[key]==null||b[key]==null?a[key]:a[key]+(b[key]-a[key])*f;
    return p;
  }
  function path(rows,key,start,end,maxGap=.15) {
    let result='',previous=null;
    for(const row of rows) {
      if(row.time<start)continue;if(row.time>end)break;
      const point=row[key];
      if(!point||!point.every(Number.isFinite)) {previous=null;continue;}
      const connected=previous&&row.time-previous.time<=maxGap&&Math.hypot(point[0]-previous.point[0],point[1]-previous.point[1])<=150;
      result+=(connected?'L':'M')+point[0].toFixed(2)+','+(3000-point[1]).toFixed(2);
      previous={time:row.time,point};
    }
    return result;
  }
  const api={lowerBound,nearest,angle,robotAt,path};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.ReplayMath=api;
})(typeof window!=='undefined'?window:globalThis);
