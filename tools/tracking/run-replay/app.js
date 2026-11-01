'use strict';
(() => {
  const el=id=>document.getElementById(id), R=window.ReplayMath;
  const colors={fused:'#55d5a4',otos:'#f5b34c',cam1:'#60b8ff',cam2:'#d29cff'};
  const names={fused:'Fused log',otos:'OTOS-only log',cam1:'Camera 1',cam2:'Camera 2'};
  const keys=Object.keys(names), ns='http://www.w3.org/2000/svg';
  const cache=new Map();let data, artwork, cameras, desiredTime=0, shownTime=0, playing=false, lastTick=null, lastRequest=0, renderToken=0;
  function svg(tag,attrs={},text) {const n=document.createElementNS(ns,tag);for(const [k,v] of Object.entries(attrs))n.setAttribute(k,v);if(text!=null)n.textContent=text;return n;}
  function fixed(v,decimals=1) {return v==null||!Number.isFinite(v)?'—':v.toFixed(decimals);}
  function signed(v) {return (v>=0?'+':'')+fixed(v);}
  function buildField() {
    const group=el('field-base');
    group.append(svg('rect',{x:0,y:0,width:3000,height:3000,fill:'#182534',stroke:'#88a2bc','stroke-width':16}));
    for(let v=0;v<=3000;v+=500) {
      group.append(svg('line',{x1:v,y1:0,x2:v,y2:3000,stroke:'#2b3b4a','stroke-width':3}),svg('line',{x1:0,y1:v,x2:3000,y2:v,stroke:'#2b3b4a','stroke-width':3}));
      if(v%1000===0)group.append(svg('text',{x:v,y:3125,'text-anchor':'middle'},String(v)),svg('text',{x:-55,y:3000-v+25,'text-anchor':'end'},String(v)));
    }
    group.append(svg('rect',{x:1000,y:1000,width:1000,height:1000,fill:'#293541',stroke:'#88a2bc','stroke-width':12}));
    const defs=svg('defs'),clip=svg('clipPath',{id:'field-artwork-clip'});
    clip.append(svg('rect',{x:0,y:0,width:3000,height:3000}));defs.append(clip);group.append(defs);
    const art=svg('g',{id:'printed-lines','clip-path':'url(#field-artwork-clip)',opacity:.7});
    for(const feature of artwork.features) {
      if(feature.kind==='blue-corner-line'||feature.kind==='orange-corner-line')continue;
      art.append(svg('path',{'data-field-feature':feature.kind,d:feature.d,fill:feature.fill,stroke:feature.stroke,'stroke-width':feature.stroke_width,'fill-rule':feature.fill_rule}));
    }
    group.append(art,svg('text',{x:1500,y:1500,'text-anchor':'middle'},'Inner square'),svg('text',{x:2970,y:3220,'text-anchor':'end'},'X (mm) →'),svg('text',{x:-135,y:-65},'Y (mm) ↑'));
    for(const key of keys) {
      const context=svg('path',{id:'context-'+key,fill:'none',stroke:colors[key],'stroke-width':10,opacity:.16});
      const trail=svg('path',{id:'trail-'+key,fill:'none',stroke:colors[key],'stroke-width':13,'stroke-linecap':'round','stroke-linejoin':'round'});
      if(key==='otos') {context.setAttribute('stroke-dasharray','30 18');trail.setAttribute('stroke-dasharray','30 18');}
      el('context-paths').append(context);el('trail-paths').append(trail);
      const row=document.createElement('tr');row.id='row-'+key;
      for(let i=0;i<4;i++) {const cell=document.createElement('td');if(i===0) {cell.textContent=names[key];cell.className=key;}row.append(cell);}
      el('coordinates').append(row);
      const rows=key.startsWith('cam')?(cameras[Number(key.slice(-1))].observations||cameras[Number(key.slice(-1))].frames):data.robot;
      context.setAttribute('d',R.path(rows,key.startsWith('cam')?'xy':key,-Infinity,Infinity));
    }
  }
  function currentSymbol(key,xy,heading) {
    const g=svg('g',{transform:`translate(${xy[0]},${3000-xy[1]})`});
    if(key==='cam2')g.append(svg('path',{d:'M0,-30L30,0L0,30L-30,0Z',fill:colors[key],stroke:'#10151d','stroke-width':6}));
    else g.append(svg('circle',{r:key.startsWith('cam')?28:24,fill:colors[key],stroke:'#10151d','stroke-width':6}));
    if(heading!=null) {
      const theta=heading*Math.PI/180,x=90*Math.cos(theta),y=-90*Math.sin(theta);
      g.append(svg('line',{x1:0,y1:0,x2:x,y2:y,stroke:colors[key],'stroke-width':10}));
      const left=[-y*.18,x*.18];g.append(svg('path',{d:`M${x*.75+left[0]},${y*.75+left[1]}L${x},${y}L${x*.75-left[0]},${y*.75-left[1]}`,fill:'none',stroke:colors[key],'stroke-width':8}));
    }
    return g;
  }
  function drawMap(time,selected) {
    const log=R.robotAt(data.robot,time), pos={fused:log.fused,otos:log.otos,cam1:selected[1].xy,cam2:selected[2].xy};
    const heads={fused:log.fused_heading,otos:log.otos_heading,cam1:selected[1].heading,cam2:selected[2].heading};
    const since=el('trail').value==='all'?-Infinity:time-Number(el('trail').value);
    el('current-positions').replaceChildren();
    el('printed-lines').style.display=el('show-artwork').checked?'':'none';
    for(const key of keys) {
      const visible=el('show-'+key).checked, cam=key.startsWith('cam'), c=cam?Number(key.slice(-1)):null;
      el('context-'+key).style.display=visible&&el('show-context').checked?'':'none';
      el('trail-'+key).style.display=visible?'':'none';
      el('trail-'+key).setAttribute('d',R.path(cam?(cameras[c].observations||cameras[c].frames):data.robot,cam?'xy':key,since,time));
      if(visible&&pos[key]) {
        if(cam&&selected[c].marker&&el('show-marker-centres').checked) {
          const marker=selected[c].marker,mx=marker[0],my=3000-marker[1];
          const centre=svg('g',{id:'marker-centre-'+c,'data-world-x':mx,'data-world-y':marker[1]});
          centre.append(svg('title',{},`Camera ${c} marker centre: X ${fixed(mx)}, Y ${fixed(marker[1])} mm`));
          centre.append(svg('line',{x1:pos[key][0],y1:3000-pos[key][1],x2:mx,y2:my,stroke:colors[key],'stroke-width':6,'stroke-dasharray':'12 10',opacity:.7}));
          const cross=`M${mx-23},${my}H${mx+23}M${mx},${my-23}V${my+23}`;
          centre.append(svg('path',{d:cross,fill:'none',stroke:'#10151d','stroke-width':17,'stroke-linecap':'round'}),svg('path',{d:cross,fill:'none',stroke:colors[key],'stroke-width':9,'stroke-linecap':'round'}));
          el('current-positions').append(centre);
        }
        el('current-positions').append(currentSymbol(key,pos[key],heads[key]));
      }
      const cells=el('row-'+key).children;cells[1].textContent=pos[key]?fixed(pos[key][0]):'—';cells[2].textContent=pos[key]?fixed(pos[key][1]):'—';
      const reference=cam?R.robotAt(data.robot,selected[c].time).fused:log.fused;
      cells[3].textContent=key==='fused'?'reference':pos[key]&&reference?signed(pos[key][0]-reference[0])+' / '+signed(pos[key][1]-reference[1]):'not detected';
      el('row-'+key).style.opacity=visible?'1':'.45';
    }
    el('time-readout').textContent=`${fixed(time,3)} / ${fixed(data.duration,3)} s`;
    el('phase').textContent=log.phase.replaceAll('_',' ');
    el('robot-speed').textContent=`OTOS ${fixed(log.speed,2)} m/s`;
    return log;
  }
  async function imageFor(camera,frame) {
    const key=camera+':'+frame.index;
    if(!cache.has(key)) {
      const promise=fetch(`/api/frame?camera=${camera}&index=${frame.index}`).then(r=>{if(!r.ok)throw new Error(`Camera ${camera}: saved frame could not be loaded`);return r.blob();}).then(blob=>createImageBitmap(blob));
      cache.set(key,promise);promise.catch(()=>cache.delete(key));
      while(cache.size>80) {const old=cache.keys().next().value;const value=cache.get(old);cache.delete(old);value.then(image=>image.close()).catch(()=>{});}
    }
    return cache.get(key);
  }
  function drawCamera(camera,frame,image,time) {
    const canvas=el('image-'+camera),ctx=canvas.getContext('2d');
    ctx.clearRect(0,0,canvas.width,canvas.height);ctx.drawImage(image,0,0,canvas.width,canvas.height);
    if(frame.quad&&frame.xy) {
      ctx.beginPath();frame.quad.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));ctx.closePath();ctx.strokeStyle=colors['cam'+camera];ctx.lineWidth=3;ctx.stroke();
      // Projective centre = intersection of marker diagonals, not corner average.
      const q=frame.quad,a=q[0],b=q[2],c=q[1],d=q[3],ab=[b[0]-a[0],b[1]-a[1]],cd=[d[0]-c[0],d[1]-c[1]],den=ab[0]*cd[1]-ab[1]*cd[0];
      if(Math.abs(den)>1e-8) {const f=((c[0]-a[0])*cd[1]-(c[1]-a[1])*cd[0])/den;ctx.beginPath();ctx.arc(a[0]+f*ab[0],a[1]+f*ab[1],4,0,Math.PI*2);ctx.fillStyle=ctx.strokeStyle;ctx.fill();}
    }
    el('frame-time-'+camera).textContent=`t ${fixed(frame.time,3)} s · Δ ${signed((frame.time-time)*1000)} ms`;
    el('frame-index-'+camera).textContent=`Saved frame ${frame.index}`;
    const status=el('detection-'+camera);status.textContent=frame.xy?'ID 9 detected · outline = black marker':'ID 9 not detected / outside view';status.className=frame.xy?'cam'+camera:'muted';
  }
  async function render(time) {
    if(!data)return;
    desiredTime=Math.max(0,Math.min(data.duration,time));el('scrubber').value=desiredTime;
    const requested=desiredTime,token=++renderToken,selected={1:R.nearest(cameras[1].frames,requested),2:R.nearest(cameras[2].frames,requested)};
    try {
      const images=await Promise.all([imageFor(1,selected[1]),imageFor(2,selected[2])]);
      if(token!==renderToken)return;
      drawCamera(1,selected[1],images[0],requested);drawCamera(2,selected[2],images[1],requested);drawMap(requested,selected);shownTime=requested;
      el('sync-status').textContent=`Robot t ${fixed(data.start_robot_t+requested,3)} s · LED alignment approximate`;
    } catch(error) {
      if(token!==renderToken)return;pause();el('sync-status').textContent=error.message;
    }
  }
  function pause() {playing=false;lastTick=null;el('play').textContent='Play';}
  function playToggle() {if(playing)pause();else {if(desiredTime>=data.duration-.001)desiredTime=0;playing=true;lastTick=null;el('play').textContent='Pause';}}
  function step(direction) {
    pause();const rows=cameras[1].frames,current=R.nearest(rows,desiredTime),i=rows.indexOf(current);
    render(rows[Math.max(0,Math.min(rows.length-1,i+direction))].time);
  }
  function tick(now) {
    if(playing) {
      if(lastTick==null)lastTick=now;
      desiredTime=Math.min(data.duration,desiredTime+(now-lastTick)*.001*Number(el('playback').value));lastTick=now;
      if(now-lastRequest>=32||desiredTime>=data.duration) {lastRequest=now;render(desiredTime);}
      if(desiredTime>=data.duration)pause();
    }
    requestAnimationFrame(tick);
  }
  el('play').onclick=playToggle;el('previous').onclick=()=>step(-1);el('next').onclick=()=>step(1);
  el('scrubber').addEventListener('input',()=>{pause();render(Number(el('scrubber').value));});
  for(const id of [...keys.map(key=>'show-'+key),'show-context','trail','show-artwork','show-marker-centres'])el(id).addEventListener('change',()=>render(shownTime));
  document.addEventListener('keydown',event=>{
    if(!data||event.target.matches('select,input:not([type=range]),textarea'))return;
    if(event.code==='Space') {event.preventDefault();playToggle();}
    if(event.code==='ArrowLeft'||event.code==='ArrowRight') {event.preventDefault();step(event.code==='ArrowLeft'?-1:1);}
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
  async function boot() {
    try {
      const [response,artResponse]=await Promise.all([fetch('/api/replay'),fetch('/field-artwork.json')]);
      if(!response.ok||!artResponse.ok)throw new Error('Replay or original field artwork could not be loaded');
      [data,artwork]=await Promise.all([response.json(),artResponse.json()]);cameras=Object.fromEntries(data.cameras.map(c=>[c.id,c]));
      el('scrubber').max=data.duration;el('range-label').textContent=`0–${fixed(data.duration,2)} seconds · nearest recorded frames`;
      if(data.example_kind)el('range-label').textContent='Example: sparse saved images, complete analyzed position traces. Frame Δ shows the image time offset.';
      for(const c of data.cameras)el('identity-'+c.id).textContent=c.identity?c.identity.slice(-8):'';
      for(const text of [data.reference,data.field_frame,data.timing_note,data.mapping_note,data.interpolation_note,'Map circles/diamonds show the rear axle. Matching coloured crosses show the recorded marker centres, 100.72 mm behind. No centre is invented during a detection gap. The video outline and centre dot show the printed marker. ΔX / ΔY is camera or OTOS minus fused, in millimetres; camera differences use that frame’s estimated time.',`Field line source: ${artwork.source}. ${artwork.qualification}`]) {const p=document.createElement('p');p.textContent=text;el('provenance').append(p);}
      el('viewer').hidden=false;el('load-status').hidden=true;buildField();await render(0);requestAnimationFrame(tick);
    } catch(error) {el('load-status').textContent=error.message;el('load-status').setAttribute('role','alert');}
  }
  boot();
})();
