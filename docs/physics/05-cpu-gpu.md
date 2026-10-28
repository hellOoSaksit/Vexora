# 05 — CPU / GPU Compute Support

> สถานะ ณ 2026-10-28 · ✅ = มีโค้ดและ test ผ่าน · 🟡 = ยังไม่มี · ⚠️ = ทำแล้วแต่ยังไม่ได้วัดบนฮาร์ดแวร์จริง

## 1. ระบบไหนใช้ GPU ได้

| ระบบ | CPU | GPU | หมายเหตุ |
|---|---|---|---|
| Rigid body, collision, constraints | ✅ Jolt (multi-thread) | ⛔ ไม่รองรับ | Jolt v5.3.0 ไม่มี GPU simulation — UI ต้องแสดงว่า "CPU เท่านั้น" ไม่ใช่ซ่อนตัวเลือก |
| Particles (VFX) | ✅ Vexora (multi-thread) | ✅ Vulkan compute | backend เดียวกัน kernel เดียวกัน เลือกได้ Auto / CPU / GPU |
| Cloth / Hair / Soft body | 🟡 | 🟡 | Phase D — จะใช้ระบบ backend เดียวกันนี้ |

## 2. ทำไมเลือก Vulkan compute

- **ทำงานได้กับ GPU ทุกค่าย** (NVIDIA, AMD, Intel) บน Windows และ Linux; macOS ผ่าน MoltenVK (โค้ดรองรับ portability enumeration แล้ว **แต่ยังไม่ได้ทดสอบบน Mac**)
- ไม่ผูกกับ CUDA (ต่างจากฟีเจอร์ GPU ของ PhysX)
- Physics process ของ Electron เป็น native C++ จึงเรียก Vulkan ได้ตรง ไม่ต้องผ่าน WebGPU ของ renderer
- **Vulkan loader ถูกโหลดตอน runtime** (volk) — เครื่องที่ไม่มี driver ยังเปิดโปรแกรมได้ และใช้ CPU แทนพร้อมบอกเหตุผล (ทดสอบแล้วด้วยการซ่อน driver)
- Build option: `-DVEXORA_PHYSICS_GPU=OFF` ตัดส่วน GPU ออกทั้งหมด (ทดสอบแล้ว: 46/46 ผ่าน)

## 3. การทำงานของ Particle backend

```
CPU (ทุก backend):  อายุ/การตาย/การเกิด (seeded RNG, นับเป็นจำนวน step แบบ integer)
                    → เขียนเฉพาะ slot ที่เปลี่ยนลง staging buffer
GPU backend:        copy เฉพาะช่วง slot ที่เปลี่ยน → dispatch compute (256 threads/group) → รอ fence
CPU backend:        loop เดียวกัน แบ่งช่วงให้ worker threads (ผลไม่ขึ้นกับจำนวน thread)
```

- **ชุด particle ที่มีชีวิตตรงกันทุกประการ** ระหว่าง CPU และ GPU เพราะ lifecycle ตัดสินบน CPU
- **ตำแหน่งตรงกันในระดับ float tolerance แต่ไม่ตรงกันระดับบิต** (compiler/FMA ต่างกัน) — ห้ามใช้ cache ที่ bake บน CPU แทน GPU แบบอ้างว่าเหมือนกันทุกบิต
- ข้อมูลอยู่ใน GPU memory (DEVICE_LOCAL) การส่งข้อมูลต่อ step เป็นสัดส่วนกับจำนวนที่เกิด/ตาย ไม่ใช่จำนวนทั้งหมด
- ข้อจำกัดปัจจุบัน: submit แบบรอผล (CPU รอ GPU ทุก step), `readback()` คัดลอกทั้ง buffer — การแสดงผลจริงควรวาดจาก GPU buffer โดยตรง (Phase E ต่อ)

## 4. กฎการเลือก backend

| คำขอ | ผล |
|---|---|
| `CPU` | CPU เสมอ |
| `GPU` | GPU ที่ดีที่สุด (discrete > integrated > virtual) หรือ **error `DeviceUnavailable` พร้อมเหตุผล — ไม่ fallback แบบเงียบ** |
| `Auto` | GPU เฉพาะเมื่อเป็น GPU จริง **และ** จำนวน particle ≥ `autoGpuMinParticles`; นอกนั้น CPU พร้อม `selectionReason()` สำหรับแสดงใน UI |
| Software Vulkan (llvmpipe/lavapipe) | ไม่ถูกเลือกโดย Auto (ช้ากว่า CPU path ของเรา) ใช้ได้เมื่อ `allowSoftwareGpu = true` (สำหรับ test/CI) |

⚠️ **ค่า `autoGpuMinParticles = 50,000` เป็นค่าตั้งต้นที่ยังไม่ได้วัดบน GPU จริง** ต้อง calibrate ด้วย `vexora_particle_bench` บนเครื่อง NVIDIA / AMD / Intel อย่างน้อยอย่างละเครื่องก่อน release

## 5. ผลการวัด (เครื่อง container 4 vCPU, ไม่มี GPU จริง)

`./build/engine/physics/bench/vexora_particle_bench 300 10000 100000 1000000`

| Backend | Device | Particles | avg ms/step | p95 ms | readback ms |
|---|---|---|---|---|---|
| CPU 1 thread | CPU | 10,000 | 0.17 | 0.22 | 0.28 |
| CPU all threads | CPU | 10,000 | 0.22 | 0.43 | 0.61 |
| GPU | llvmpipe (software) | 10,000 | 0.23 | 0.31 | 0.39 |
| CPU 1 thread | CPU | 100,000 | 1.46 | 1.99 | 3.33 |
| CPU all threads | CPU | 100,000 | 0.84 | 1.11 | 2.94 |
| GPU | llvmpipe (software) | 100,000 | 1.35 | 1.94 | 3.52 |
| CPU 1 thread | CPU | 1,000,000 | 14.80 | 21.20 | 35.05 |
| CPU all threads | CPU | 1,000,000 | 6.37 | 8.93 | 31.75 |
| GPU | llvmpipe (software) | 1,000,000 | 11.22 | 20.13 | 35.70 |

**ตีความ:** แถว GPU ในตารางนี้รันบน llvmpipe ซึ่งจำลอง GPU ด้วย CPU **จึงไม่ได้บอกความเร็วของ GPU จริงเลย** มีไว้พิสูจน์ว่าโค้ด GPU ทำงานถูกต้องเท่านั้น ส่วน CPU: 10,000 particle ไม่คุ้มที่จะแบ่ง thread (ต่ำกว่าเกณฑ์ 16,384 ต่อ thread จึงรัน thread เดียว — ค่าที่ต่างกันเป็น noise) ที่ 1 ล้าน particle multi-thread เร็วกว่า 2.3 เท่า

## 6. Test ที่เกี่ยวข้อง (`scene09_particles.cpp`)

| Test | ผล |
|---|---|
| Enumerate devices (CPU มาก่อนเสมอ) | ✅ พบ CPU + llvmpipe |
| CPU vs GPU parity 50,000 particles, 3 s, ชนพื้น + ทรงกลม | ✅ slot ที่มีชีวิตไม่ตรงกัน 0, max \|Δp\| = 1.9e-6 m, max \|Δv\| = 5.3e-6 m/s (บน llvmpipe) |
| ไม่ทะลุพื้น | ✅ 0.000 m (plane ถูก resolve ซ้ำเป็นลำดับสุดท้าย) |
| ไม่ทะลุทรงกลม | ✅ ≤ 3.1 mm (particle ที่ติดระหว่างพื้นกับทรงกลม) |
| Bounce e = 0.5 จาก 2 m | ✅ 0.466 m (คาด 0.5, ±10%) |
| อายุ/pool เต็ม แม่นยำแบบ integer | ✅ |
| ผลเหมือนกันทุกบิต: รันซ้ำ, 1 vs 4 threads, reset | ✅ (CPU) |
| ขอ GPU ที่ไม่มี → error, Auto → CPU พร้อมเหตุผล, Auto ไม่เลือก software GPU | ✅ |
| ไม่มี Vulkan driver / build แบบ GPU=OFF | ✅ fallback CPU |
| SPIR-V ที่ commit ไว้ตรงกับ `shaders/particles.comp` | ✅ ctest `shader_spirv_up_to_date` |
| ASan + UBSan (รวม Vulkan path บน llvmpipe) | ✅ ไม่พบปัญหา |

**ยังไม่ได้ทดสอบ:** GPU จริงทุกค่าย, Windows (D3D-less Vulkan driver ของผู้ผลิต), macOS/MoltenVK, GPU หลายตัวในเครื่องเดียว, device lost ระหว่างทำงาน

## 7. การแสดงใน UI (Electron, แผน)

- หน้า Settings → "การประมวลผล": รายการอุปกรณ์จาก `enumerateComputeDevices()` (ชื่อ, ชนิด, หน่วยความจำ)
- ต่อระบบ: dropdown อัตโนมัติ / CPU / GPU — ระบบที่ทำบน GPU ไม่ได้ (rigid body) แสดงเป็น "CPU (GPU ไม่รองรับสำหรับระบบนี้)"
- แสดง `selectionReason()` ใต้ตัวเลือก เช่น "อัตโนมัติ: ใช้ CPU เพราะจำนวนอนุภาคน้อยกว่า 50,000"
- เมื่อเลือก GPU แต่ใช้ไม่ได้: แสดงข้อความจาก error ตรง ๆ พร้อมปุ่ม "ใช้ CPU แทน"
