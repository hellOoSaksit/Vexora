# Vexora Physics & Simulation

เอกสารของระบบ Physics & Simulation ซึ่งเป็น Core System ของ Vexora

| เอกสาร | เนื้อหา |
|---|---|
| [01-engine-evaluation.md](01-engine-evaluation.md) | การเลือก Physics Engine (Jolt / PhysX / Bullet), license, สิ่งที่ตรวจแล้วเทียบกับที่ยังไม่ได้ตรวจ |
| [02-architecture.md](02-architecture.md) | ขอบเขตระบบ, Physics World vs Rendering World, transform ownership, ลำดับการคำนวณ, cache, export |
| [03-test-plan.md](03-test-plan.md) | Test Scenes 1–13: Expected Results, Acceptance Criteria, ค่าที่วัดได้จริง |
| [04-roadmap.md](04-roadmap.md) | Phase A–F พร้อม Dependencies, Risks, Acceptance, DoD |
| [05-cpu-gpu.md](05-cpu-gpu.md) | CPU/GPU: ระบบไหนใช้ GPU ได้, Vulkan backend, กฎเลือก Auto/CPU/GPU, ผลวัด |

## Capability Matrix — สถานะจริง (2026-10-28)

แหล่งที่คำนวณ: **Jolt** = Third-party engine · **Vexora** = โค้ดของเราเอง · **Preview** = การประมาณเพื่อ preview · **Bake** = ข้อมูลที่บันทึกไว้เล่นซ้ำ · **Export** = ส่งให้ Game Engine คำนวณต่อ

| ระบบ | ผู้คำนวณ | สถานะ |
|---|---|---|
| Rigid bodies (static/kinematic/dynamic), shapes, mass/COM, forces, sleeping | Jolt | ✅ ทดสอบแล้ว |
| Collision layers/masks, triggers, contact events | Jolt (filter + การเรียง event ทำโดย Vexora) | ✅ ทดสอบแล้ว |
| CCD (linear cast) | Jolt | ✅ ทดสอบแล้ว |
| Fixed / Point / Hinge / Slider / Distance-spring + limits + motors | Jolt | ✅ ทดสอบแล้ว |
| Breakable constraints | Vexora (จาก impulse ที่ Jolt รายงาน) | ✅ ทดสอบแล้ว |
| Fixed timestep, interpolation, quality profiles | Vexora | ✅ ทดสอบแล้ว |
| Snapshot/restore, determinism (binary เดียวกัน) | Jolt + Vexora | ✅ ทดสอบแล้ว |
| Determinism ข้ามแพลตฟอร์ม | — | ⛔ ไม่รับประกัน (ยังไม่ได้ตรวจ) |
| Debug draw data, solver warnings, step stats | Vexora | ✅ ทดสอบแล้ว (ยังไม่มี UI) |
| Ragdoll, animation/physics blending | Jolt (ragdoll) + Vexora (blend) | 🟡 ออกแบบแล้ว ยังไม่มีโค้ด |
| Spring secondary motion | Vexora (Preview/การประมาณ) | 🟡 ออกแบบแล้ว ยังไม่มีโค้ด |
| Cloth / Hair / Soft body | ยังไม่ได้เลือก solver | 🟡 ยังไม่มีโค้ด |
| Particles: emitter, lifetime, gravity, wind/drag, ชน plane + sphere | Vexora — CPU (multi-thread) หรือ GPU (Vulkan compute) | ✅ ทดสอบแล้ว (GPU ทดสอบบน software Vulkan เท่านั้น ยังไม่ได้วัดบน GPU จริง) |
| Particles: trails, sub-emitters, force fields, ชนกับ mesh/ตัวละคร | Vexora | 🟡 ยังไม่มีโค้ด |
| GPU สำหรับ rigid body | — | ⛔ Jolt ไม่รองรับ (CPU เท่านั้น) |
| Simulation cache / bake ลงไฟล์ | Vexora (Bake) | 🟡 มีเพียงรากฐาน (snapshot) |
| Export ไป Game Engine | Export | 🟡 ยังไม่มีโค้ด |
| แอป Electron: ตั้งค่าจาก schema, viewport 3D, เล่น/หยุด/ทีละขั้น, แผง CPU/GPU, คำเตือนภาษาไทย | Vexora (`app/`, `engine/simserver/`) | ✅ ใช้งานได้ (ทดสอบบน Linux; Windows build ผ่าน CI) — ยังไม่มี Joint editor, Cache inspector, Profiler |

**ระบบ Physics ยังไม่สมบูรณ์** — Phase A (prototype + core rigid body) ใช้งานได้และมี test รองรับ ส่วนอื่นยังเป็นแผน
