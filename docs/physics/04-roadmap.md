# 04 — Implementation Roadmap (Phases A–F)

แต่ละ Phase ต้องมี Dependencies, Technical Risks, Acceptance Criteria และ Definition of Done ก่อนเริ่ม ลำดับนี้ปรับได้ตาม MVP

**Definition of Done ร่วมของทุก Phase:** test scenes ของ Phase นั้นรันแบบ headless ใน CI และผ่าน · ค่าที่วัดได้ถูกบันทึกใน `03-test-plan.md` · ไม่มี type ของ Engine หลุดออกนอก module · เอกสาร capability matrix (README) อัปเดตตามจริง

---

## Phase A — Physics Prototype ✅ เสร็จ (2026-10-28)

| | |
|---|---|
| Dependencies | ไม่มี (รีโปว่าง) |
| Risks ที่ตรวจแล้ว | License (MIT ✅), build ใน CMake (✅), determinism (✅ binary เดียวกัน/ข้าม thread count), ความเสถียรของ stack/joint (✅ หลังปรับ profile), performance (✅ วัดแล้ว) |
| Acceptance | Scene 1, 2, 3, 10 (ส่วนรากฐาน), 13 ผ่าน และ transform sync ถูกต้อง |
| ส่งมอบ | `engine/physics` (PhysicsWorld บน Jolt), 38 test cases, benchmark, เอกสาร 01–04 |

สิ่งที่ Phase A **ไม่ได้** ส่งมอบ: UI, Ragdoll, secondary motion, cloth, hair, soft body, particles, cache/bake ลงไฟล์, export

## Phase B — Core Rigid Body (ทำให้ครบระดับ production)

| | |
|---|---|
| Dependencies | Phase A; ต้องมี Scene graph/asset system ของ Vexora (ยังไม่มีในรีโป) เพื่อ bind `BodyHandle` กับ node |
| งาน | Shape cache + collision shape baking (hash), Convex decomposition (ประเมิน V-HACD vs CoACD — license + คุณภาพ), constraint ชนิด SwingTwist/SixDOF/Cone, joint friction, raycast/shape-cast query API, ปรับ multi-thread scaling, build AVX2 แยก, CI matrix Windows/macOS/Linux + ASan/UBSan, ทดลองโหมด cross-platform determinism และเทียบ hash ข้าม OS |
| Risks | Multi-thread scaling ต่ำ (วัดได้ 1.3–2.1× ที่ 4 cores); convex decomposition ช้า/คุณภาพไม่คงที่; ความแม่นยำ float ไกลจาก origin |
| Acceptance | Scene 1–3 ผ่านบนทั้ง 3 OS; เกณฑ์ performance ของ Scene 13; การ decomposition ของ mesh ทดสอบ 5 ชิ้นได้ hull ≤ N ชิ้นและ volume error ≤ 10%; ไม่มี leak ใน ASan |
| DoD | ข้างต้น + Physics Properties / Rigid Body Inspector / Collision Visualization ใช้ API ที่มีอยู่โดยไม่ถือ state |

## Phase C — Character Physics — ส่วนแรกเสร็จ: Ragdoll, blending, secondary motion (ยังไม่มี Character controller และการเชื่อมกับระบบ Animation จริง)

| | |
|---|---|
| Dependencies | Skeleton + Animation system ของ Vexora (ยังไม่มี) ต้องนิยาม Pose (local/model space) และ bind pose ก่อน; Phase B constraints (SwingTwist) |
| งาน | Ragdoll builder จาก skeleton (ใช้ `JPH::Ragdoll`), โหมด Animated/Simulated/Powered ต่อ bone, blend weights, animation↔physics transitions, secondary-motion solver ของ Vexora (bone chains) |
| Risks | Jitter/instability ของ ragdoll (สูง); การแปลง pose ระหว่าง space; motor tuning; ความต่อเนื่องของความเร็วตอนสลับโหมด |
| Acceptance | Scene 4, 5, 6, 8 (bone chain) ตามเกณฑ์ใน test plan |
| ทางเลือกสำรอง | หาก ragdoll บน Jolt ไม่ผ่าน Scene 4 หลัง tuning → ประเมิน PhysX articulations |

## Phase R — Rig & Animation (เพิ่มตามที่ตกลง) — R1–R4 เสร็จ: ดู [docs/rig/README.md](../rig/README.md)
| | |
|---|---|
| ทำแล้ว | core/rig แยกโมดูล, glTF import, skinning, clip playback, auto ragdoll, ฉากตัวละครในแอป |
| ถัดไป | R5 เครื่องมือ Rig ใน UI, IK, FBX (ประเมิน ufbx), animation blending หลายท่า |

## Phase D — Advanced Simulation (Cloth, Hair, Soft Body)

| | |
|---|---|
| Dependencies | Phase C (skeleton/skinning สำหรับ pinning), mesh deformation pipeline |
| งาน | Spike เปรียบเทียบ: Jolt soft body (มี skinned constraints, ไม่มี soft-vs-soft) vs XPBD ของ Vexora vs Bullet soft body — ตัดสินจาก Scene 7 + performance; hair guide strands; mechanical constraints ขั้นสูง |
| Risks | สูงมาก: self-collision, ความเร็ว, GPU vs CPU, คุณภาพผ้าที่ resolution ต่ำ |
| Acceptance | Scene 7, 8 |

## Phase E — VFX Physics (Particles) — เริ่มแล้ว: CPU + Vulkan GPU backend, plane/sphere collision (ดู 05-cpu-gpu.md)

| | |
|---|---|
| Dependencies | Renderer/GPU backend ของ Vexora (สำหรับ GPU path); collision query API (Phase B) |
| งาน | CPU particle system (emitter, lifetime, forces, wind/force fields, sub-emitters, collision แบบ ray/SDF/primitive), จากนั้น GPU compute path; แยก simulation ออกจาก rendering |
| Risks | GPU readback latency; determinism ของ GPU; การเลือก collision แบบเต็มรูปแบบ vs ประมาณ |
| Acceptance | Scene 9 รวมผลวัด CPU vs GPU ที่ 1k/10k/100k ก่อนเลือกค่าเริ่มต้น |

## Phase F — Production Pipeline

| | |
|---|---|
| Dependencies | Asset versioning ของ Vexora, Phase A snapshot/determinism (มีแล้ว) |
| งาน | Simulation cache (format, chunking, manifest, crash recovery), dependency tracking + invalidation, bake controls/timeline/cache inspector, exporter (glTF physics extension / FBX / engine-specific) + Export Validation Report, regression test suite ที่เทียบ hash ของ cache |
| Risks | Cache key ไม่ครบ → เล่น cache ผิดเวอร์ชันแบบเงียบ; ความแตกต่างระหว่าง engine ปลายทาง |
| Acceptance | Scene 10 (ส่วน cache), 11, 12 |
