// Builds settings forms from the schema sent by vexora_sim. No parameter is hand-coded here:
// labels, units, ranges, help and simple/advanced grouping all come from the schema.

const numberFormat = new Intl.NumberFormat('th-TH', { maximumFractionDigits: 3 });

function formatValue(param, v) {
  const text = numberFormat.format(v);
  return param.unit ? `${text} ${param.unit}` : text;
}

// Very wide ranges (e.g. 100 .. 2,000,000 particles) get a logarithmic slider so both ends are usable.
function isLog(param) {
  return param.min > 0 && param.max / param.min >= 1000;
}

function toSlider(param, v) {
  if (!isLog(param)) return v;
  return (Math.log(v) - Math.log(param.min)) / (Math.log(param.max) - Math.log(param.min)) * 1000;
}

function fromSlider(param, s) {
  let v = isLog(param) ? Math.exp(Math.log(param.min) + (s / 1000) * (Math.log(param.max) - Math.log(param.min))) : Number(s);
  if (param.type === 'int') v = Math.round(v);
  else if (isLog(param)) v = Number(v.toPrecision(3));
  else v = Math.round(v / param.step) * param.step;
  return Math.min(param.max, Math.max(param.min, Number(v.toFixed(6))));
}

function el(tag, attrs = {}, children = []) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'text') node.textContent = v;
    else if (k === 'class') node.className = v;
    else node.setAttribute(k, v);
  }
  for (const c of children) node.append(c);
  return node;
}

let fieldId = 0;

/**
 * @param {HTMLFormElement} container
 * @param {Array} params schema parameters
 * @param {Object} values current values (missing keys use defaults)
 * @param {(key: string, value: any) => void} onCommit called when the user finishes changing a value
 */
export function createForm(container, params, values, onCommit) {
  container.replaceChildren();
  const fields = [];
  const current = {};

  for (const p of params) {
    current[p.key] = values?.[p.key] ?? p.default;
    const id = `f${++fieldId}`;
    const field = el('div', { class: 'field' });
    field.dataset.advanced = String(p.advanced);
    const head = el('div', { class: 'field-head' });
    const label = el('label', { for: id, text: p.label });
    if (p.advanced) label.append(el('span', { class: 'adv', text: 'ขั้นสูง' }));
    head.append(label);
    field.append(head);

    if (p.type === 'number' || p.type === 'int') {
      const valueText = el('span', { class: 'value', text: formatValue(p, current[p.key]) });
      head.append(valueText);
      const slider = el('input', { type: 'range', min: isLog(p) ? 0 : p.min, max: isLog(p) ? 1000 : p.max,
                                   step: isLog(p) ? 1 : p.step, 'aria-label': p.label });
      const box = el('input', { type: 'number', id, min: p.min, max: p.max, step: p.type === 'int' ? 1 : p.step });
      slider.value = toSlider(p, current[p.key]);
      box.value = current[p.key];
      const show = (v) => {
        valueText.textContent = formatValue(p, v);
        box.value = v;
        box.classList.remove('invalid');
      };
      slider.addEventListener('input', () => show(fromSlider(p, slider.value)));
      slider.addEventListener('change', () => {
        current[p.key] = fromSlider(p, slider.value);
        onCommit(p.key, current[p.key]);
      });
      box.addEventListener('change', () => {
        const v = Number(box.value);
        const ok = box.value !== '' && Number.isFinite(v) && v >= p.min && v <= p.max && (p.type !== 'int' || Number.isInteger(v));
        box.classList.toggle('invalid', !ok);
        box.title = ok ? '' : `ต้องอยู่ระหว่าง ${numberFormat.format(p.min)} ถึง ${numberFormat.format(p.max)}`;
        if (!ok) return;
        slider.value = toSlider(p, v);
        valueText.textContent = formatValue(p, v);
        current[p.key] = v;
        onCommit(p.key, v);
      });
      field.append(el('div', { class: 'field-row' }, [slider, box]));
    } else if (p.type === 'bool') {
      const box = el('input', { type: 'checkbox', id });
      box.checked = Boolean(current[p.key]);
      box.addEventListener('change', () => {
        current[p.key] = box.checked;
        onCommit(p.key, box.checked);
      });
      head.replaceChildren(el('label', { class: 'switch', for: id }, [box, p.label]));
      if (p.advanced) head.append(el('span', { class: 'adv', text: 'ขั้นสูง' }));
    } else if (p.type === 'enum') {
      const select = el('select', { id });
      for (const o of p.options) {
        const opt = el('option', { value: o.value, text: o.label });
        if (o.value === current[p.key]) opt.selected = true;
        select.append(opt);
      }
      select.addEventListener('change', () => {
        current[p.key] = select.value;
        onCommit(p.key, select.value);
      });
      field.append(select);
    }
    if (p.help) field.append(el('div', { class: 'help', text: p.help }));
    container.append(field);
    fields.push(field);
  }

  return {
    setAdvanced(show) {
      for (const f of fields) f.hidden = f.dataset.advanced === 'true' && !show;
    },
    values: () => ({ ...current }),
  };
}
