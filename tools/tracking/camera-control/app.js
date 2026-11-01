'use strict';
let session, state, busy = false, jogActive = false;
let previewToken, previewPath;
let absoluteDevicePath, connectionRevision, polling = false, appStopped = false;
let presets = [];
const $ = id => document.getElementById(id);
const pretty = {WhiteBalance:'White balance', BacklightCompensation:'Backlight compensation', PowerLineFrequency:'Anti-flicker frequency'};
const keyOf = c => `${c.Group}-${c.Id}`;
function status(message, error = false) { $('status').textContent = message; $('status').title = message; $('status').classList.toggle('error', error); }
async function api(path, body) {
  const bound = ['set', 'jog', 'stop', 'extended', 'absolute', 'preview-start', 'apply-preset', 'save-preset', 'snapshot'];
  if (body && bound.includes(body.action)) body = {device_path:state?.Device?.Path, ...body};
  const response = await fetch(path, body ? {method:'POST', headers:{'Content-Type':'application/json', 'X-Carltron-Key':session.key}, body:JSON.stringify(body)} : {});
  const result = await response.json();
  if (response.status === 403 && body) {
    // Refresh the key for the next explicit attempt, but never replay a camera command.
    session = await api('/api/session');
    acceptConnection({...session, state:null});
    throw new Error('App connection renewed. Please retry the adjustment; no command was repeated.');
  }
  if (!response.ok) throw new Error(result.error || `Request failed (${response.status})`);
  return result;
}
function displayValue(c, value = c.Value) {
  if (c.Name === 'Exposure') return `1/${2 ** -value} s · ${value}${c.Mode === 'auto' ? ' · AUTO (nominal)' : ' · manual'}`;
  if (c.Name === 'WhiteBalance') return `${value} K${c.Mode === 'auto' ? ' · auto' : ''}`;
  if (c.Name === 'PowerLineFrequency') return value === 1 ? '50 Hz' : value === 2 ? '60 Hz' : 'Disabled';
  return `${value}${c.Mode === 'auto' ? ' · auto' : ''}`;
}
function findControl(group, id) { return state?.Controls.find(c => c.Group === group && c.Id === id); }
function acceptConnection(data, forceRender = false) {
  session ||= {};
  const changed = (data.key && session.key && data.key !== session.key)
    || connectionRevision !== data.connection_revision || state?.Device?.Path !== data.state?.Device?.Path;
  if (data.key) session.key = data.key;
  if (data.devices) {
    session.devices = data.devices;
    session.selected = data.selected;
    const signature = JSON.stringify([data.devices, data.selected]);
    if ($('device').dataset.inventory !== signature) {
      $('device').replaceChildren();
      if (data.selected == null) $('device').append(new Option(data.devices.length ? 'Selected camera disconnected — choose a camera' : 'No PTZ Pro 2 detected', ''));
      data.devices.forEach(d => $('device').append(new Option(`${d.Name} · ${cameraLabel(d)}`, d.Index)));
      $('device').value = data.selected ?? '';
      $('device').disabled = !data.devices.length;
      $('device').dataset.inventory = signature;
    }
  }
  connectionRevision = data.connection_revision;
  session.connection_error = data.connection_error;
  state = data.state;
  if (previewToken && (previewPath !== state?.Device?.Path || data.preview?.token !== previewToken || data.preview?.error)) stopPreview();
  if (changed) {
    absoluteDevicePath = undefined;
    $('absolute-enable').checked = false;
    $('keys').checked = Boolean(state);
    if (presets.length) populatePresets(undefined, true);
  }
  if (changed || forceRender || !state) render();
  else {
    $('connection').textContent = `${state.Device.Name} · connected`; $('dot').className = 'ready';
    state.Controls.filter(c => c.Supported).forEach(updateRow);
    const priority = state.Extended.find(c => c.Name === 'AutoExposurePriority');
    const toggle = $('extended-controls').querySelector('input');
    if (priority && toggle) toggle.checked = priority.Value !== 0;
    updateAbsolute();
  }
  if (changed && state) status('Camera connected. Controls reread; no settings or movements restored.');
  if (previewToken && data.preview) $('preview-info').textContent = `${cameraLabel(state.Device)} · linked preview/control · ${data.preview.width} × ${data.preview.height} · ${data.preview.fps} preview fps`;
}
function cameraLabel(device) {
  const identity = device.Path.split('#')[2]?.split('&')[1] || 'USB camera';
  return `Camera ${device.Index + 1} · ${identity}`;
}
function connectionFailed(error) {
  acceptConnection({state:null, connection_error:error.message});
  $('connection').textContent = 'Connection lost · retrying';
}
async function refresh(forceRender = true) {
  try { acceptConnection(await api('/api/state'), forceRender); }
  catch (error) { connectionFailed(error); throw error; }
}
async function rediscover() {
  if (busy || polling || appStopped) return;
  busy = true; $('rediscover').disabled = true;
  status('Rediscovering cameras and controls…');
  try {
    session = await api('/api/session');
    acceptConnection((await api('/api/action', {action:'rediscover'})).result, true);
    if (state) status('Cameras rediscovered. No settings changed or movement restored.');
  } catch (error) { connectionFailed(error); }
  finally { busy = false; $('rediscover').disabled = false; updateAbsolute(); }
}
async function setControl(group, id, value, auto) {
  if (!state || !findControl(group, id)?.Supported) return;
  if (busy) { status('Another adjustment is in progress; try again shortly.'); return; }
  busy = true;
  try {
    const result = (await api('/api/action', {action:'set', group, id, value:Number(value), auto:Boolean(auto)})).result;
    const index = state.Controls.findIndex(c => c.Group === group && c.Id === id);
    state.Controls[index] = result;
    updateRow(result);
    status(`${pretty[result.Name] || result.Name}: ${displayValue(result)} — driver read-back confirmed.`);
  } catch (error) { await refresh().catch(() => {}); status(error.message, true); }
  finally { busy = false; }
}
function stepControl(group, id, delta) {
  const c = findControl(group, id);
  if (!c) return;
  const value = Math.min(c.Max, Math.max(c.Min, c.Value + delta * c.Step));
  return setControl(group, id, value, false);
}
function updateRow(c) {
  const row = $(keyOf(c));
  if (!row) return;
  row.querySelector('.value-label').textContent = displayValue(c);
  row.querySelector('input[type=range]').value = c.Value;
  row.querySelector('input[type=number]').value = c.Value;
  const automatic = row.querySelector('input[type=checkbox]');
  if (automatic) automatic.checked = c.Mode === 'auto';
  row.querySelectorAll('.manual-control').forEach(el => el.disabled = c.Mode === 'auto');
}
function createRow(c) {
  const row = document.createElement('section');
  row.className = 'control'; row.id = keyOf(c);
  row.innerHTML = `<div class="control-heading"><h3></h3><span class="value-label"></span></div><input class="manual-control" type="range"><div class="control-inputs"><button class="manual-control" data-step="-10" title="Ten supported steps">−10</button><button class="manual-control" data-step="-1" title="One supported step">−</button><input class="manual-control" type="number"><button class="manual-control" data-step="1" title="One supported step">+</button><button class="manual-control" data-step="10" title="Ten supported steps">+10</button><button class="manual-control" data-apply>Apply</button></div><div class="range-note"></div>`;
  row.querySelector('h3').textContent = pretty[c.Name] || c.Name;
  const slider = row.querySelector('input[type=range]'), number = row.querySelector('input[type=number]');
  for (const input of [slider, number]) { input.min = c.Min; input.max = c.Max; input.step = c.Step; input.setAttribute('aria-label', `${c.Name} ${input.type === 'range' ? 'slider' : 'value'}`); }
  slider.oninput = () => { number.value = slider.value; row.querySelector('.value-label').textContent = displayValue(c, Number(slider.value)); };
  slider.onchange = () => setControl(c.Group, c.Id, slider.value, false);
  number.onkeydown = event => { if (event.key === 'Enter') { event.preventDefault(); setControl(c.Group, c.Id, number.value, false); } };
  row.querySelector('[data-apply]').onclick = () => setControl(c.Group, c.Id, number.value, false);
  row.querySelectorAll('[data-step]').forEach(button => button.onclick = () => stepControl(c.Group, c.Id, Number(button.dataset.step)));
  row.querySelector('.range-note').textContent = `Range ${c.Min} … ${c.Max} · minimum step ${c.Step} · driver default ${c.Default}`;
  if (c.Auto) {
    const label = document.createElement('label');
    label.innerHTML = '<input type="checkbox"> Auto';
    label.querySelector('input').onchange = event => setControl(c.Group, c.Id, findControl(c.Group, c.Id).Value, event.target.checked);
    row.querySelector('.control-inputs').append(label);
  }
  return row;
}
function copyAbsolutePosition() {
  const c = state?.Absolute;
  if (!c?.Readable) return;
  for (const axis of ['pan', 'tilt']) {
    const value = c[axis === 'pan' ? 'Pan' : 'Tilt'] / 3600;
    $(`absolute-${axis}`).value = value;
    $(`absolute-${axis}-slider`).value = value;
  }
}
function updateAbsolute() {
  const c = state?.Absolute;
  const changed = absoluteDevicePath !== state?.Device?.Path;
  if (changed) { absoluteDevicePath = state?.Device?.Path; $('absolute-enable').checked = false; }
  $('absolute-enable').disabled = !c?.Writable;
  $('absolute-copy').disabled = !c?.Readable || busy;
  $('absolute-readout').textContent = c?.Readable
    ? `Reported pan ${(c.Pan / 3600).toFixed(2)}° · tilt ${(c.Tilt / 3600).toFixed(2)}° (logical, not measured)`
    : c?.Error || 'Absolute interface unavailable.';
  $('absolute-range').textContent = c?.Writable
    ? `Pan ${c.PanMin / 3600} … ${c.PanMax / 3600}°, step ${c.PanStep / 3600}° · tilt ${c.TiltMin / 3600} … ${c.TiltMax / 3600}°, step ${c.TiltStep / 3600}°`
    : c?.Error || 'No verified writable range; movement disabled.';
  const enabled = Boolean(c?.Writable && $('absolute-enable').checked && !busy);
  for (const axis of ['pan', 'tilt']) {
    const name = axis === 'pan' ? 'Pan' : 'Tilt';
    for (const id of [`absolute-${axis}`, `absolute-${axis}-slider`]) {
      const input = $(id); input.disabled = !enabled;
      if (c?.Writable) { input.min = c[`${name}Min`] / 3600; input.max = c[`${name}Max`] / 3600; input.step = c[`${name}Step`] / 3600; }
    }
  }
  $('absolute-apply').disabled = !enabled;
  if (changed) copyAbsolutePosition();
  updatePreset();
}
function selectedPreset() { return presets.find(preset => preset.id === $('preset').value); }
function populatePresets(selectedId, matchCamera = false) {
  $('preset').replaceChildren();
  if (!presets.length) $('preset').append(new Option('No saved views', ''));
  else presets.forEach(preset => $('preset').append(new Option(preset.name +
    (state && preset.state?.Device?.Path !== state.Device.Path ? ' · other camera' : ''), preset.id)));
  const preferred = !matchCamera && presets.find(preset => preset.id === selectedId)
    || presets.find(preset => preset.state?.Device?.Path === state?.Device?.Path);
  if (preferred) $('preset').value = preferred.id;
}
function updatePreset() {
  const saved = selectedPreset(), lens = saved?.state?.Controls || [], pose = saved?.state?.Absolute;
  const zoom = lens.find(c => c.Group === 'camera' && c.Id === 3);
  const focus = lens.find(c => c.Group === 'camera' && c.Id === 6);
  $('preset-save').disabled = !state || busy || appStopped;
  $('preset').disabled = !presets.length || busy || appStopped;
  $('preset-restore').disabled = !state || !saved || !pose?.Readable || !state.Absolute?.Writable || busy || appStopped;
  $('preset-info').textContent = saved
    ? `Zoom ${zoom?.Value ?? '?'} · focus ${focus?.Value ?? '?'}${focus?.Mode === 'auto' ? ' (auto)' : ' (manual)'} · ${pose?.Readable ? `pan ${(pose.Pan / 3600).toFixed(2)}° / tilt ${(pose.Tilt / 3600).toFixed(2)}°` : 'position unavailable'} · saved ${new Date(saved.saved_at).toLocaleString()}`
    : 'No saved views yet. Save the current setup without changing the camera.';
  if (saved && state && saved.state.Device.Path !== state.Device.Path) $('preset-info').textContent += ' · WARNING: saved for a different camera identifier';
}
async function loadPresets(selectedId = $('preset').value) {
  const data = await api('/api/presets');
  presets = data.presets;
  populatePresets(selectedId);
  updatePreset();
  if (data.errors?.length) status(`Some saved views could not be read: ${data.errors.join('; ')}`, true);
}
async function saveView() {
  if (busy || !state || appStopped) return;
  const name = prompt('Name this view. Saving reads the camera and does not move or change it.', 'Full field view');
  if (name === null) return;
  busy = true; updateAbsolute();
  try {
    const result = (await api('/api/action', {action:'save-preset', name})).result;
    await loadPresets(result.preset.id);
    status(`Saved “${result.preset.name}”. Camera settings and position were not changed.`);
  } catch (error) { status(error.message, true); }
  finally { busy = false; updateAbsolute(); }
}
async function restoreView() {
  const saved = selectedPreset();
  if (busy || !state || !saved || appStopped) return;
  const devicePath = state.Device.Path, differentDevice = saved.state.Device.Path !== devicePath;
  if (!confirm(`Restore “${saved.name}” to ${state.Device.Name} · ${cameraLabel(state.Device)}?\n\nThis can MOVE the camera and changes zoom, focus, exposure and image settings. Wait for settling and recheck the field reference markers.${differentDevice ? '\n\nWarning: this view was saved with a different camera/USB identifier. Confirm that this is the intended camera.' : ''}`)) return;
  busy = true; updateAbsolute();
  try {
    const result = (await api('/api/action', {action:'apply-preset', id:saved.id, confirmed:true,
      device_path:devicePath, allow_different_device:differentDevice})).result;
    await refresh();
    status(`Restored “${saved.name}” · ${result.applied_count} commands. ${result.note}`);
  } catch (error) { await refresh().catch(() => {}); status(error.message, true); }
  finally { busy = false; updateAbsolute(); }
}
function renderVendor() {
  const host = $('vendor-info'); host.replaceChildren();
  for (const c of state?.Vendor || []) {
    const row = document.createElement('div'); row.className = 'vendor-row';
    const name = document.createElement('span'); name.textContent = c.Name;
    const value = document.createElement('code'); value.textContent = c.Readable ? c.DataHex : 'Unavailable';
    value.title = c.Readable ? `Raw bytes · selector ${c.Selector} · driver write flag ${c.AdvertisesWrite ? 'yes' : 'no'} · app read-only` : c.Error || 'Not readable';
    row.append(name, value); host.append(row);
  }
}
async function applyAbsolute() {
  if (busy || !$('absolute-enable').checked || !state?.Absolute?.Writable) return;
  try {
    const c = state.Absolute, target = {};
    for (const axis of ['pan', 'tilt']) {
      const input = $(`absolute-${axis}`), name = axis === 'pan' ? 'Pan' : 'Tilt';
      const arcseconds = Number(input.value) * 3600;
      if (!input.value.trim() || !input.checkValidity() || !Number.isSafeInteger(arcseconds)
          || arcseconds < c[`${name}Min`] || arcseconds > c[`${name}Max`]
          || (arcseconds - c[`${name}Min`]) % c[`${name}Step`] !== 0)
        throw new Error(`${name} target must match the displayed range and step.`);
      target[axis] = arcseconds;
    }
    busy = true; updateAbsolute();
    state.Absolute = (await api('/api/action', {action:'absolute', ...target, confirmed:true})).result;
    status('Absolute movement command accepted. Read-back is logical only; wait for settling and recheck field references.');
  } catch (error) { status(error.message, true); await refresh().catch(() => {}); }
  finally { busy = false; updateAbsolute(); }
}
function render() {
  for (const id of ['snapshot', 'export', 'import', 'keys', 'duration']) $(id).disabled = !state;
  document.querySelectorAll('[data-exposure], [data-duration], [data-pan]').forEach(button => button.disabled = !state);
  if (!state) {
    $('controls').replaceChildren(); $('extended-controls').replaceChildren(); $('vendor-info').replaceChildren();
    $('diagnostics').textContent = session?.connection_error || 'Waiting for camera controls.';
    $('absolute-enable').checked = false; $('keys').checked = false; updateAbsolute();
    $('connection').textContent = 'Camera unavailable · retrying'; $('dot').className = 'error';
    status(`${session?.connection_error || 'Camera unavailable.'} Reconnect it, or use Rediscover cameras.`, true);
    return;
  }
  $('connection').textContent = `${state.Device.Name} · connected`; $('dot').className = 'ready';
  const host = $('controls'); host.replaceChildren();
  state.Controls.filter(c => c.Supported && c.Manual).forEach(c => { host.append(createRow(c)); updateRow(c); });
  const extHost = $('extended-controls'); extHost.replaceChildren();
  const priority = state.Extended.find(c => c.Name === 'AutoExposurePriority');
  if (priority && priority.Writable && priority.Value !== null) {
    const section = document.createElement('section'); section.className = 'control';
    section.innerHTML = '<label><input type="checkbox"> Low-light compensation / variable frame rate</label><p class="muted">Allows auto-exposure to lower frame rate in dim light. Usually disable for timing-sensitive tracking. Relevant when exposure is automatic.</p>';
    const checkbox = section.querySelector('input'); checkbox.checked = priority.Value !== 0;
    checkbox.onchange = async () => { try { await api('/api/action', {action:'extended', id:19, value:checkbox.checked ? 1 : 0}); await refresh(); status('Low-light compensation updated.'); } catch (error) { status(error.message, true); await refresh(); } };
    extHost.append(section);
  }
  document.querySelectorAll('[data-pan]').forEach(button => button.disabled = (Number(button.dataset.pan) !== 0 && !state.PanJog) || (Number(button.dataset.tilt) !== 0 && !state.TiltJog));
  document.querySelectorAll('[data-exposure]').forEach(button => button.disabled = !findControl('camera', 4)?.Supported);
  updateAbsolute(); renderVendor();
  $('diagnostics').textContent = JSON.stringify({camera:state.Device.Name, controls:state.Controls, extended:state.Extended, absolute:state.Absolute, vendor:state.Vendor, notes:['Absolute values are logical driver feedback, not proof of mechanical arrival or accuracy.', 'No movement or preset is applied automatically.', 'Exposure conversions are nominal; verify image response.', 'Unknown vendor settings are read-only.', 'Hardware preset storage and animation sequencing are not implemented.', `Session output: ${session.output}`]}, null, 2);
}
async function jog(pan, tilt, coarse = false) {
  if (busy || !state || (pan && !state.PanJog) || (tilt && !state.TiltJog)) return;
  busy = true; jogActive = true;
  const duration = Math.min(400, Number($('duration').value) * (coarse ? 3 : 1));
  try { await api('/api/action', {action:'jog', pan, tilt, duration, speed:1}); status(`Jog complete · ${duration} ms pulse · motors stopped. Calibration needs rechecking.`); }
  catch (error) { await refresh().catch(() => {}); status(error.message, true); }
  finally { busy = false; jogActive = false; }
}
async function stop() { if (!state || appStopped) return; try { await api('/api/action', {action:'stop'}); status('Pan/tilt stop sent.'); } catch (error) { status(error.message, true); await refresh().catch(() => {}); } }
async function stopPreview() {
  const token = previewToken;
  previewToken = previewPath = undefined;
  $('video').removeAttribute('src'); $('video').hidden = true;
  $('preview-placeholder').hidden = false; $('preview-info').textContent = '';
  if (token && session?.key) {
    try { await api('/api/action', {action:'preview-stop', token}); }
    catch (error) { status(error.message, true); }
  }
}
async function startPreview() {
  if (busy || !state || appStopped) return;
  busy = true; updateAbsolute();
  try {
    await stopPreview();
    const path = state.Device.Path;
    status(`Starting linked preview for ${cameraLabel(state.Device)}…`);
    const capture = (await api('/api/action', {action:'preview-start', device_path:path})).result;
    if (capture.device_path !== path) throw new Error('Preview camera identity does not match the controls.');
    previewToken = capture.token; previewPath = capture.device_path;
    await refresh();
    if (previewToken !== capture.token || state?.Device?.Path !== path) throw new Error('Camera selection changed while starting preview; no alternate camera was used.');
    $('video').src = `/api/preview?token=${encodeURIComponent(capture.token)}`;
    $('video').hidden = false; $('preview-placeholder').hidden = true;
    status(`${cameraLabel(state.Device)}: preview and all adjustments are linked to this physical camera.`);
  } catch (error) {
    await stopPreview(); status(`Preview: ${error.message}. Stop other camera captures if the camera is busy.`, true);
  }
  finally { busy = false; updateAbsolute(); }
}
async function exportSettings() {
  await refresh();
  if (!state) throw new Error('Connect a camera before exporting settings.');
  const blob = new Blob([JSON.stringify({format:'carltron-camera-settings-v1', captured_at:new Date().toISOString(), state}, null, 2)], {type:'application/json'});
  const link = document.createElement('a'); link.href = URL.createObjectURL(blob); link.download = 'Carltron-camera-settings.json'; link.click(); setTimeout(() => URL.revokeObjectURL(link.href), 1000);
}
async function importSettings(event) {
  try {
    const file = event.target.files[0]; if (!file) return;
    const saved = JSON.parse(await file.text());
    const source = saved.state?.Controls;
    if (!Array.isArray(source)) throw new Error('This file is not a camera settings snapshot.');
    if (!confirm('Apply saved image, exposure, focus and zoom settings? This may invalidate calibration. Pan/tilt position is NOT restored.')) return;
    if (busy) throw new Error('Wait for the current adjustment to finish.');
    busy = true;
    await api('/api/action', {action:'snapshot', label:'before-restore'});
    let count = 0;
    for (const c of source) {
      const current = findControl(c.Group, c.Id);
      if (!current || !current.Supported || !current.Manual || c.Value === null || (c.Group === 'camera' && [0, 1, 2].includes(c.Id))) continue;
      await api('/api/action', {action:'set', group:c.Group, id:c.Id, value:c.Value, auto:c.Mode === 'auto'}); count++;
    }
    const priority = saved.state?.Extended?.find(c => c.Name === 'AutoExposurePriority');
    if (priority && priority.Value !== null && priority.Writable) await api('/api/action', {action:'extended', id:19, value:priority.Value});
    await refresh(); status(`Restored ${count} controls. Pan/tilt pose was not changed; recheck calibration.`);
  } catch (error) { status(error.message, true); await refresh().catch(() => {}); }
  finally { busy = false; event.target.value = ''; }
}
async function initialize() {
  try {
    session = await api('/api/session');
    acceptConnection(session);
    await refresh();
    await loadPresets();
    if (state) status('Ready. No settings changed. Saved views are never restored automatically.');
  } catch (error) { connectionFailed(error); }
}
$('device').onchange = async () => {
  if (busy || polling || !$('device').value) return;
  const index = Number($('device').value), path = session.devices.find(d => d.Index === index)?.Path;
  const restartPreview = Boolean(previewToken);
  busy = true;
  try { await stopPreview(); await api('/api/action', {action:'select', index, path}); await refresh(); if (state) status('Camera selected; settings left unchanged.'); }
  catch (error) { status(error.message, true); await refresh().catch(() => {}); }
  finally { busy = false; updateAbsolute(); }
  if (restartPreview && state?.Device.Path === path) await startPreview();
};
$('rediscover').onclick = rediscover;
$('refresh').onclick = () => refresh().then(() => loadPresets()).then(() => { if (state) status('Settings read; no changes.'); }).catch(error => status(error.message, true));
$('preset').onchange = updatePreset;
$('preset-save').onclick = saveView;
$('preset-restore').onclick = restoreView;
$('snapshot').onclick = async () => { try { const result = (await api('/api/action', {action:'snapshot', label:'manual'})).result; status(`Snapshot saved: ${result.path}`); } catch (error) { status(error.message, true); } };
$('export').onclick = () => exportSettings().catch(error => status(error.message, true));
$('import').onchange = importSettings;
$('preview-start').onclick = startPreview; $('preview-stop').onclick = stopPreview;
$('video').onerror = () => { if (previewToken) { stopPreview(); status('Preview ended. Click Start preview to reconnect the selected camera.', true); } };
$('crosshair').onchange = () => document.querySelector('.preview').classList.toggle('crosshair', $('crosshair').checked);
$('absolute-enable').onchange = updateAbsolute;
$('absolute-copy').onclick = copyAbsolutePosition;
$('absolute-apply').onclick = applyAbsolute;
for (const axis of ['pan', 'tilt']) {
  $(`absolute-${axis}-slider`).oninput = () => { $(`absolute-${axis}`).value = $(`absolute-${axis}-slider`).value; };
  $(`absolute-${axis}`).oninput = () => { $(`absolute-${axis}-slider`).value = $(`absolute-${axis}`).value; };
}
$('duration').oninput = () => $('duration-label').textContent = `${$('duration').value} ms`;
document.querySelectorAll('[data-duration]').forEach(button => button.onclick = () => { $('duration').value = button.dataset.duration; $('duration').oninput(); });
document.querySelectorAll('[data-pan]').forEach(button => button.onclick = event => jog(Number(button.dataset.pan), Number(button.dataset.tilt), event.shiftKey));
document.querySelectorAll('[data-exposure]').forEach(button => button.onclick = () => setControl('camera', 4, Number(button.dataset.exposure), false));
$('stop').onclick = stop;
$('shutdown').onclick = async () => {
  if (!confirm('Stop the local camera-control server? The camera image settings are left as they are.')) return;
  stopPreview();
  try { await api('/api/action', {action:'shutdown'}); appStopped = true; acceptConnection({state:null}); $('connection').textContent = 'App stopped'; $('dot').className = ''; $('rediscover').disabled = true; status('App stopped. Relaunch using Start-Camera-Control.cmd.'); }
  catch (error) { status(error.message, true); }
};
window.addEventListener('keydown', event => {
  if (event.key === 'Escape') { event.preventDefault(); stop(); return; }
  if (!$('keys').checked || event.target.matches('input,select,textarea,[contenteditable="true"]') || event.ctrlKey || event.altKey || event.metaKey) return;
  const moves = {ArrowLeft:[-1,0], ArrowRight:[1,0], ArrowUp:[0,1], ArrowDown:[0,-1]};
  if (moves[event.key]) { event.preventDefault(); jog(...moves[event.key], event.shiftKey); }
  else if (['+','=', '-', 'PageUp', 'PageDown'].includes(event.key)) { event.preventDefault(); stepControl('camera', 3, ['-', 'PageDown'].includes(event.key) ? -1 : 1); }
});
window.addEventListener('blur', () => { if (jogActive) stop(); });
window.addEventListener('beforeunload', () => {
  if (previewToken && session?.key) fetch('/api/action', {method:'POST', keepalive:true,
    headers:{'Content-Type':'application/json', 'X-Carltron-Key':session.key},
    body:JSON.stringify({action:'preview-stop', token:previewToken})});
});
initialize();
setInterval(async () => {
  if (busy || polling || document.hidden || appStopped) return;
  // Even while editing, detect disconnects; do not overwrite healthy draft controls.
  const editing = document.activeElement?.matches('input,select,textarea');
  polling = true;
  try {
    if (!session) session = await api('/api/session');
    const fresh = await api('/api/state');
    if (!editing || !state || !fresh.state || fresh.connection_revision !== connectionRevision
        || fresh.state.Device.Path !== state.Device.Path || fresh.key !== session.key) acceptConnection(fresh);
  } catch (error) { connectionFailed(error); }
  finally { polling = false; }
}, 2500);
