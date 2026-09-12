import * as THREE from './vendor/three.module.js';
import {OrbitControls} from './vendor/OrbitControls.js';
import {GLTFLoader} from './vendor/GLTFLoader.js';
import {parts,bom,ABB_SOURCE} from './data.js';

const $=s=>document.querySelector(s);
let selected=null,tab='explore',meshList=[],root=null,labelsOn=false,isolated=false,explosion=0;
let renderer,scene,camera,controls,ready=false;
const groups=new Map(),labels=new Map(),reduced=matchMedia('(prefers-reduced-motion: reduce)').matches;
const home={position:new THREE.Vector3(1.08,.83,1.28),target:new THREE.Vector3(.0,.32,.08)};
const formatsign=v=>v>0?'+'+v:String(v).replace('-','−');
const sourceHTML=`<a href="${ABB_SOURCE}" target="_blank" rel="noopener">Ficha ABB · IRB 120 · Rev. J ↗</a>`;
function overview(){
 $('#detail').innerHTML=`<div class="detail-kicker"><span>FICHA GENERAL</span><span class="detail-index">06</span></div><h2>Pequeño formato.<br>Seis ejes.</h2><div class="detail-sub">ABB IRB 120–3/0.6</div><span class="badge verified">DATOS DE CATÁLOGO · ABB</span><p class="detail-body">Manipulador compacto para tareas de manipulación y ensamblaje. Explora sus conjuntos para entender cómo se conectan la estructura, las articulaciones y la herramienta.</p><div class="stats"><div class="stat"><b>580 <span>mm</span></b><small>Alcance</small></div><div class="stat"><b>3 <span>kg</span></b><small>Carga nominal¹</small></div><div class="stat"><b>25 <span>kg</span></b><small>Masa del robot</small></div><div class="stat"><b>0,01 <span>mm</span></b><small>Repetibilidad</small></div></div><h3>En este proyecto</h3><p class="detail-body">Clasificación y paletizado de piezas mediante visión artificial y una pinza neumática.</p><div class="detail-note">Vista educativa de geometría. Selecciona el brazo directamente o utiliza la lista de conjuntos.<br><br>¹ Herramienta y pieza deben evaluarse con el diagrama de carga. La repetibilidad no es la exactitud absoluta.</div><div class="detail-source">${sourceHTML}<br>Modelo visual de terceros · sin validación cinemática.</div>`;
}
function renderList(){
 const entries=tab==='axes'?parts.filter(p=>p.range):parts;
 $('.parts-panel h1').innerHTML=tab==='axes'?'Seis ejes.<br>Seis movimientos.':'Anatomía<br>de un robot.';
 $('.parts-panel .intro').textContent=tab==='axes'?'Consulta el movimiento y recorrido de cada articulación.':'Selecciona un conjunto y descubre su función.';
 $('.list-heading').innerHTML=`<span>${tab==='axes'?'ARTICULACIONES':'ESTRUCTURA'}</span><span>${tab==='axes'?'06':'09'}</span>`;
 $('#part-list').innerHTML=entries.map(p=>`<button class="part ${selected?.id===p.id?'selected':''}" data-part="${p.id}" aria-pressed="${selected?.id===p.id}"><span class="part-num">${p.num}</span><span>${tab==='axes'?p.motion:p.name}</span>${selected?.id===p.id?'<span class="part-arrow">›</span>':''}</button>`).join('');
 $('#part-list').querySelectorAll('button').forEach(b=>b.onclick=()=>selectPart(b.dataset.part));
 $('#explorer').classList.toggle('axis-mode',tab==='axes');
}
function detailPart(p){
 const spec=p.range?`<div class="axis-range"><div class="range-label">RECORRIDO ARTICULAR · ${p.num}</div><div class="range-value">${formatsign(p.range[0])}° <span style="color:#82919e">a</span> ${formatsign(p.range[1])}°</div><div class="axis-bar"></div><div class="axis-bar-labels"><span>Límite inferior</span><span>Límite superior</span></div></div><dl class="spec-list"><div><dt>Tipo de articulación</dt><dd>Rotativa</dd></div><div><dt>Velocidad de catálogo</dt><dd>${p.speed}°/s</dd></div><div><dt>Movimiento</dt><dd>${p.motion}</dd></div></dl>`:`<dl class="spec-list">${p.specs.map(([k,v])=>`<div><dt>${k}</dt><dd>${v}</dd></div>`).join('')}</dl>`;
 $('#detail').innerHTML=`<div class="detail-kicker"><span>CONJUNTO SELECCIONADO</span><span class="detail-index">${p.num}</span></div><h2>${p.name}</h2><div class="detail-sub">${p.subtitle}</div><span class="badge ${p.model?'model':'verified'}">${p.model?'MODELO VISUAL / PROYECTO':'DATOS DE CATÁLOGO · ABB'}</span><p class="detail-body">${p.body}</p>${spec}<div class="detail-note">${p.note}</div><button class="action-link" id="back-overview">← Volver a la ficha del robot</button><div class="detail-source">${p.model?'Fuente: inspección del modelo y guía PFI. Sin especificaciones de fabricante para este elemento.':sourceHTML+'<br>Identificación visual del conjunto en el GLTF.'}</div>`;
 $('#back-overview').onclick=clearSelection;
}
function selectPart(id){
 const p=parts.find(p=>p.id===id);if(!p)return;selected=p;detailPart(p);renderList();$('#isolate').disabled=!ready;updateMaterials();
 labels.forEach((el,key)=>el.classList.toggle('active',key===id));
}
function clearSelection(){selected=null;isolated=false;$('#isolate').disabled=true;$('#isolate').setAttribute('aria-pressed','false');overview();renderList();updateMaterials();labels.forEach(el=>el.classList.remove('active'));}
function updateMaterials(){
 for(const mesh of meshList){const active=selected&&mesh.userData.partId===selected.id;mesh.visible=!isolated||active;
  mesh.material.emissive.set(active?0xf16338:0x000000);mesh.material.emissiveIntensity=active?.25:0;
  mesh.material.color.copy(mesh.userData.originalColor);if(active)mesh.material.color.lerp(new THREE.Color('#f1774e'),.4);
 }
}
function switchTab(next){tab=next;document.querySelectorAll('[data-tab]').forEach(b=>{b.classList.toggle('active',b.dataset.tab===next);b.setAttribute('aria-pressed',String(b.dataset.tab===next));});$('#cell-view').hidden=next!=='cell';$('#explorer').hidden=next==='cell';if(next==='cell'){if(controls)controls.autoRotate=false;$('#rotate').setAttribute('aria-pressed','false');return;}renderList();if(next==='axes'&&!selected?.range)selectPart('j1');requestAnimationFrame(resize);}
document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>switchTab(b.dataset.tab));
$('#bom').innerHTML=bom.map(r=>'<tr>'+r.map(c=>`<td>${c}</td>`).join('')+'</tr>').join('');
$('#bom').insertAdjacentHTML('beforeend','<tr><td colspan="4" style="font-weight:400;font-size:12px;color:#76838f">* Una electroválvula como propuesta inicial; confirmar tipo y cantidad al diseñar el circuito neumático.</td></tr>');
$('#sources').onclick=()=>$('#sources-dialog').showModal();$('#close-sources').onclick=()=>$('#sources-dialog').close();
$('#sources-dialog').addEventListener('click',e=>{if(e.target===$('#sources-dialog')){const r=e.target.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)e.target.close();}});
$('#fullscreen').onclick=async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else await document.documentElement.requestFullscreen();}catch{$('#fullscreen').title='Utiliza F11 para presentar a pantalla completa';}};
$('#reset').onclick=()=>{if(!ready)return;camera.position.copy(home.position);controls.target.copy(home.target);controls.autoRotate=false;$('#rotate').setAttribute('aria-pressed','false');$('#explode').value=0;setExplosion(0);clearSelection();controls.update();};
$('#rotate').onclick=()=>{if(!ready)return;controls.autoRotate=!controls.autoRotate;$('#rotate').setAttribute('aria-pressed',String(controls.autoRotate));};
$('#label-toggle').onclick=()=>{labelsOn=!labelsOn;$('#label-toggle').setAttribute('aria-pressed',String(labelsOn));};
$('#isolate').onclick=()=>{if(!selected||!ready)return;isolated=!isolated;$('#isolate').setAttribute('aria-pressed',String(isolated));updateMaterials();};
$('#explode').oninput=e=>setExplosion(Number(e.target.value));
function setExplosion(value){explosion=value/100;$('#explode-value').textContent=value+'%';for(const p of parts){const group=groups.get(p.id);if(group)group.position.set(...p.offset).multiplyScalar(explosion);}if(root)root.updateMatrixWorld(true);}
function resize(){if(!renderer||$('#explorer').hidden)return;const r=$('#canvas-wrap').getBoundingClientRect();if(!r.width||!r.height)return;renderer.setSize(r.width,r.height,false);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();}
overview();renderList();
function showError(message){$('#load-state').innerHTML=`<strong>No se pudo mostrar el modelo 3D</strong><span>${message}</span><button class="action-link" id="retry">Reintentar</button>`;$('#load-state').hidden=false;$('#retry').onclick=()=>location.reload();$('#model-status').textContent='Visor no disponible';for(const id of ['rotate','isolate','explode','label-toggle'])$('#'+id).disabled=true;}
async function init(){
 try{
  renderer=new THREE.WebGLRenderer({canvas:$('#scene'),antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setClearColor(0x000000,0);renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.2;renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
  scene=new THREE.Scene();camera=new THREE.PerspectiveCamera(38,1,.01,30);camera.position.copy(home.position);controls=new OrbitControls(camera,renderer.domElement);controls.target.copy(home.target);controls.enableDamping=!reduced;controls.dampingFactor=.07;controls.minDistance=.45;controls.maxDistance=3.5;controls.maxPolarAngle=Math.PI*.49;controls.autoRotateSpeed=.65;controls.enablePan=true;
  scene.add(new THREE.HemisphereLight(0xe8f1ff,0x83909c,2.3));const key=new THREE.DirectionalLight(0xfff5e8,3.3);key.position.set(1.5,2.8,2);key.castShadow=true;key.shadow.mapSize.set(2048,2048);Object.assign(key.shadow.camera,{left:-1.3,right:1.3,top:1.3,bottom:-1.3,near:.1,far:7});key.shadow.bias=-.00015;key.shadow.normalBias=.015;scene.add(key);const fill=new THREE.DirectionalLight(0xcddfff,2);fill.position.set(-2,1,-1);scene.add(fill);
  const ground=new THREE.Mesh(new THREE.PlaneGeometry(8,8),new THREE.ShadowMaterial({opacity:.12}));ground.rotation.x=-Math.PI/2;ground.position.y=-.005;ground.receiveShadow=true;scene.add(ground);
  const grid=new THREE.GridHelper(2.4,24,0xb9c8d3,0xcbd5dd);grid.position.y=-.004;grid.material.transparent=true;grid.material.opacity=.22;scene.add(grid);
  const gltf=await new GLTFLoader().loadAsync('./assets/scene.gltf');root=new THREE.Group();scene.add(root);scene.add(gltf.scene);gltf.scene.updateMatrixWorld(true);
  const byNode=new Map();parts.forEach(p=>p.nodes.forEach(n=>byNode.set(n,p)));
  gltf.scene.traverse(obj=>{if(!obj.isMesh)return;const index=gltf.parser.associations.get(obj)?.nodes??Number(obj.name.replace('Object_',''));const p=byNode.get(index);if(!p)throw new Error('Unmapped mesh '+obj.name);obj.userData.partId=p.id;obj.userData.nodeIndex=index;meshList.push(obj);});
  for(const p of parts){const group=new THREE.Group();group.name=p.id;root.add(group);groups.set(p.id,group);for(const mesh of meshList.filter(m=>m.userData.partId===p.id)){group.attach(mesh);mesh.material=mesh.material.clone();mesh.userData.originalColor=mesh.material.color.clone();mesh.castShadow=true;mesh.receiveShadow=true;}
   const box=new THREE.Box3().setFromObject(group);group.userData.center=box.getCenter(new THREE.Vector3());const el=document.createElement('button');el.className='model-label';el.textContent=p.short;el.setAttribute('aria-label','Seleccionar '+p.name);el.hidden=true;el.onclick=()=>selectPart(p.id);$('#labels').append(el);labels.set(p.id,el);
  }
  scene.remove(gltf.scene);root.updateMatrixWorld(true);ready=true;$('#load-state').hidden=true;$('#model-status').textContent='Modelo 3D · 29 mallas';$('#isolate').disabled=!selected;updateMaterials();setExplosion(Number($('#explode').value));resize();controls.update();
  const raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();let down=null;
  function pick(e){const r=renderer.domElement.getBoundingClientRect();pointer.set((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1);raycaster.setFromCamera(pointer,camera);return raycaster.intersectObjects(meshList.filter(m=>m.visible),false)[0];}
  renderer.domElement.addEventListener('pointerdown',e=>{down={x:e.clientX,y:e.clientY};$('#hover-label').hidden=true;});
  renderer.domElement.addEventListener('pointerup',e=>{if(down&&Math.hypot(e.clientX-down.x,e.clientY-down.y)<6){const hit=pick(e);if(hit)selectPart(hit.object.userData.partId);}down=null;});
  renderer.domElement.addEventListener('pointercancel',()=>down=null);
  renderer.domElement.addEventListener('pointermove',e=>{if(down||e.pointerType==='touch')return;const hit=pick(e);renderer.domElement.style.cursor=hit?'pointer':'grab';const tip=$('#hover-label');tip.hidden=!hit;if(hit){const p=parts.find(p=>p.id===hit.object.userData.partId),r=$('#canvas-wrap').getBoundingClientRect();tip.textContent=p.name;tip.style.left=Math.min(e.clientX-r.left+15,r.width-180)+'px';tip.style.top=(e.clientY-r.top+16)+'px';}});
  renderer.domElement.addEventListener('pointerleave',()=>{$('#hover-label').hidden=true;down=null;});
  renderer.domElement.addEventListener('webglcontextlost',e=>{e.preventDefault();ready=false;showError('Se perdió el contexto gráfico. Reintenta para recuperar la vista.');});
  const temp=new THREE.Vector3();const size=new THREE.Vector2();
  renderer.setAnimationLoop(()=>{if(!ready||document.hidden||$('#explorer').hidden)return;controls.update();renderer.render(scene,camera);renderer.getSize(size);for(const p of parts){const el=labels.get(p.id),group=groups.get(p.id);if(!labelsOn||(isolated&&selected?.id!==p.id)){el.hidden=true;continue;}temp.copy(group.userData.center).add(group.position).project(camera);el.hidden=temp.z< -1||temp.z>1||Math.abs(temp.x)>1||Math.abs(temp.y)>1;el.style.left=((temp.x+1)*size.x/2)+'px';el.style.top=((-temp.y+1)*size.y/2)+'px';}});
 }catch(error){console.error(error);showError('Comprueba que WebGL esté habilitado y vuelve a intentarlo. Las fichas técnicas siguen disponibles.');}
}
new ResizeObserver(resize).observe($('#canvas-wrap'));window.addEventListener('resize',resize);init();
