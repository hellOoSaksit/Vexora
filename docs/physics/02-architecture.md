# 02 — Physics & Simulation Architecture

> สัญลักษณ์: ✅ = มีโค้ดและ Test ผ่านแล้ว · 🟡 = ออกแบบแล้ว ยังไม่มีโค้ด · ⛔ = ตั้งใจไม่ทำ/ไม่รองรับ

## 1. ขอบเขตของระบบและทิศทางการพึ่งพา

```
 UI (Inspector, Timeline, Viewport)        ← อ่าน state / ส่งคำสั่ง ไม่ถือ Simulation State
        │ commands / read-only queries
        ▼
 Scene & Authoring Data (ข้อมูลต้นฉบับของผู้ใช้: BodyDesc, ShapeDesc, PhysicsMaterial, ConstraintDesc)
        │ build / rebuild
        ▼
 Simulation Layer  ── owns ──▶  vexora::physics::PhysicsWorld   ✅
        │                              │ (pimpl, PRIVATE link)
        │                              ▼
        │                        Jolt Physics v5.3.0 (CPU)       ✅
        ├── Secondary-motion solver (Vexora)                       🟡 Phase C
        ├── Cloth / Hair solver (TBD)                              🟡 Phase D
        ├── Particle simulation (Vexora, CPU→GPU)                  🟡 Phase E
        └── Simulation Cache / Bake                                🟡 Phase F
```

กฎ:
1. **UI ไม่เป็นเจ้าของ Simulation State** — UI เก็บเพียง handle และเรียก API ของ `PhysicsWorld` ผ่าน Simulation Layer ✅ (library ไม่ขึ้นกับ UI ใด ๆ และ test ทั้งหมดรันแบบ headless)
2. **Header สาธารณะไม่มี type ของ Jolt** — `vexora_physics` link กับ Jolt แบบ `PRIVATE` ✅
3. **ข้อมูลต้นฉบับแยกจากผลจำลอง** — `BodyDesc` คือข้อมูลที่ผู้ใช้สร้าง; ตำแหน่ง/ความเร็วระหว่างจำลองอยู่ใน `PhysicsWorld` และ cache เท่านั้น ไม่เขียนกลับไปที่ authored data โดยอัตโนมัติ
4. **Render mesh ไม่ถูกใช้เป็น collision shape โดยอัตโนมัติ** — `ShapeDesc` เป็นข้อมูลที่ authoring แยกต่างหาก ✅

## 2. Physics World vs Rendering World

| | Physics World | Rendering World |
|---|---|---|
| ข้อมูล | Body, Shape, Constraint, contact cache | Mesh, Material, Light |
| เวลา | Fixed step (`QualitySettings::fixedDt`) | Frame rate ของจอ/UI |
| การเชื่อม | `BodyHandle` ↔ Scene node | อ่าน `getInterpolatedTransform(handle, alpha)` |

### 2.1 Transform Synchronization และเจ้าของ Transform (Single-writer rule) ✅

ทุก body มี **ผู้เขียน transform ได้เพียงหนึ่งเดียวต่อ step** กำหนดจาก `MotionType`:

| MotionType | ผู้เขียน | API | ผู้อ่าน |
|---|---|---|---|
| Static | ไม่มี (ย้ายได้ด้วย `teleport()` แบบ explicit) | `teleport` | Scene |
| Kinematic | Scene/Animation | `setKinematicTarget()` (Jolt คำนวณความเร็วจาก target ทำให้ชนวัตถุอื่นได้ถูกต้อง) | Physics |
| Dynamic | Physics | `addForce/addImpulse/...` | Scene (`getInterpolatedTransform`) |

API บังคับกฎนี้: เรียก `setKinematicTarget` กับ dynamic body หรือ `addForce` กับ kinematic body จะได้ `ErrorCode::InvalidArgument` (มี test) — ป้องกัน **Transform feedback loop** และการเขียนทับกันโดยไม่ควบคุม

### 2.2 ลำดับการคำนวณต่อเฟรม

```
frame(dt_frame):
  n = clock.advance(dt_frame)                       ✅ FixedStepClock (จำกัด maxStepsPerFrame, รายงาน droppedTime)
  repeat n:
     1. Animation evaluate → Animation Pose (local)        🟡 Phase C
     2. เขียน kinematic targets (bones/objects ที่ animation เป็นเจ้าของ)   ✅ API
     3. physics.step()                                      ✅
          - เก็บ previous transforms (สำหรับ interpolation)
          - Jolt Update(fixedDt, collisionSteps)
          - รวบรวม contact events (เรียงลำดับแบบ deterministic)
          - ตรวจ breakable constraints, ตรวจ NaN/ความเร็วผิดปกติ
     4. อ่าน Physics Pose ของ bone ที่ physics เป็นเจ้าของ               🟡 Phase C
     5. Blend(Animation Pose, Physics Pose, weights) → Final Pose     🟡 Phase C
  render: alpha = clock.interpolationAlpha(); ใช้ getInterpolatedTransform(h, alpha)   ✅
```

ผลลัพธ์ไม่ขึ้นกับ frame rate ของการแสดงผล เพราะ physics เดินด้วย fixed step เท่านั้น (test: 30/60/144 FPS → จำนวน step เท่ากัน) ✅

## 3. Character Physics — Animation Pose vs Physics Pose 🟡 (Phase C)

ออกแบบไว้ล่วงหน้าเพื่อไม่ให้ขัดกับสถาปัตยกรรมปัจจุบัน:

- **Animation Pose**: ผลจาก animation graph (local space ของ skeleton) — ไม่ถูก physics แก้ไข
- **Physics Pose**: transform ของ ragdoll bodies (world space) แปลงกลับเป็น local space ของ bone
- **Final Pose** = blend ต่อ bone ด้วย `physicsWeight[bone] ∈ [0,1]`
- โหมดต่อ bone:
  - `Animated` — body เป็น Kinematic, ใช้ `setKinematicTarget` ตาม Animation Pose (ชนกับฉากได้ ผลักวัตถุได้)
  - `Simulated` — body เป็น Dynamic อิสระ (ragdoll)
  - `Powered` — body เป็น Dynamic แต่ใช้ motor ขับไปหา Animation Pose (Jolt: `Ragdoll::DriveToPoseUsingMotors`, ยังไม่ทดสอบ)
- **Animation → Physics**: ตั้ง velocity ของ body จากความต่างของ pose สองเฟรมล่าสุด เพื่อไม่ให้ตัวละครหยุดนิ่งกะทันหันตอนเริ่ม ragdoll
- **Physics → Animation (ลุกขึ้นยืน)**: เลือก get-up animation จากทิศของ pelvis แล้ว blend weight จาก 1 → 0 ภายในช่วงเวลาที่กำหนด ระหว่างนั้น bone อยู่ในโหมด Powered
- **เมื่อสองระบบพยายามควบคุม bone เดียวกัน**: แต่ละ bone มีโหมดเดียวต่อ step (single-writer rule ข้อ 2.1) การผสมเกิดที่ขั้น Blend เท่านั้น ไม่ใช่การเขียน transform ทับกัน
- **Jitter**: ใช้ collision group ของ ragdoll (ปิดการชนระหว่าง bone ที่ติดกัน), joint limits แบบ SwingTwist, เพิ่ม solver iterations เฉพาะ ragdoll และ sleeping threshold

## 4. Secondary Motion (Spring Physics) 🟡 (Phase C)

ระบบแยกที่นำกลับมาใช้ซ้ำได้ ทำงานบน bone chain ใน pose space **ไม่ใช้ Jolt** (ต้นทุนต่ำ ควบคุมได้ และไม่ต้องสร้าง rigid body ต่อ bone)
- Verlet/XPBD per bone: stiffness (ดึงกลับสู่ animation pose), damping, inertia จากการเคลื่อนที่ของ root, gravity, angular limits, collision แบบ sphere/capsule
- ป้องกันพลังงานสะสม: integrate ด้วย fixed step เดียวกับ physics, clamp ความเร็ว, damping ≥ 0
- Reset: snap กลับไป animation pose; Loop: warm-up N รอบแล้ว bake
- **ต้องระบุใน UI ว่าเป็นการประมาณผล ไม่ใช่การจำลองแบบเต็มรูปแบบ**

## 5. Physics Material และ Collision ✅

- `PhysicsMaterial { friction, restitution, density }` แยกจาก render material ✅
- Presets: Metal, Wood, Stone, Rubber, Ice, Fabric, FleshLike ✅ — **เป็นค่าเริ่มต้นสำหรับปรับแต่ง ไม่ใช่ค่าทางกายภาพที่ถูกต้องทุกกรณี** (Fabric/FleshLike เป็นการประมาณสำหรับ rigid body เท่านั้น)
- การรวมค่าเมื่อชนกัน: friction = sqrt(a·b), restitution = max(a,b) (ค่าเริ่มต้นของ Jolt) ✅
- Collision layers 16 ชั้น + mask แบบสมมาตร ✅, Trigger volumes ✅, Contact events Begin/Persist/End ✅
- Shapes: Box, Sphere, Capsule, ConvexHull, TriangleMesh (Static/Kinematic เท่านั้น), Compound (static compound) ✅

### Dynamic concave mesh
Triangle mesh ไม่มีปริมาตร: การชนระหว่าง mesh กับ mesh เกิด contact ที่ไม่เสถียรและคำนวณ mass/inertia ไม่ได้ จึง **ปฏิเสธ TriangleMesh ที่เป็น Dynamic** พร้อมข้อความแนะนำ ✅ ทางเลือกคือ ConvexHull (วัตถุนูน) หรือ Compound ของชิ้นนูนหลายชิ้นจาก Convex Decomposition (เช่น V-HACD/CoACD — ยังไม่ได้ integrate, 🟡 Phase B) Convex hull ที่จุดอยู่บนระนาบเดียวกัน (ไม่มีปริมาตร) ก็ถูกปฏิเสธเช่นกัน ✅

### Collision Shape Lifecycle
`ShapeDesc` (authored) → สร้าง shape ของ Jolt ตอน `createBody` 🟡 Phase B: cache shape ที่สร้างแล้วตาม content hash เพื่อใช้ซ้ำระหว่าง body (Jolt อนุญาตให้ share shape ได้) และเป็นข้อมูล "Collision Shape Baking"

## 6. Simulation Settings, Quality Profiles ✅

ค่าจริงที่ใช้ (เลือกจาก parameter sweep ใน test plan):

| Profile | fixedDt | substeps | velocity it. | position it. | CCD | ใช้เมื่อ |
|---|---|---|---|---|---|---|
| InteractivePreview | 1/60 | 1 | 10 | 2 | เฉพาะ body ที่เปิด | แก้ฉาก/scrub แบบ live |
| HighQualityPreview | 1/120 | 2 | 10 | 2 | เฉพาะ body ที่เปิด | review playback |
| FinalBake | 1/240 | 2 | 16 | 4 | ทุก dynamic body | bake แบบ offline |

- **Game Runtime Export ไม่ใช่ simulation profile** ⛔ — Game Engine ปลายทางจำลองเองด้วยค่าของตัวเอง สิ่งที่ส่งออกคือ collider/constraint/material ที่แปลงแล้ว หรือ animation ที่ bake แล้ว (หัวข้อ 9)
- ผลลัพธ์ต่าง profile **ต่างกันจริง** → cache ต้องใช้ค่าของ profile เป็นส่วนหนึ่งของ key
- `WorldSettings::qualityOverride` สำหรับค่าแบบกำหนดเอง ✅

## 7. Determinism และข้อจำกัด ✅ (เท่าที่วัดได้)

| ระดับ | สถานะ |
|---|---|
| binary เดียวกัน, ลำดับ API เดียวกัน → ผลเหมือนกันระดับบิต | ✅ วัดแล้ว (Scene10) |
| จำนวน worker threads ต่างกัน (0 vs 4) → ผลเหมือนกัน | ✅ วัดแล้ว (Scene10) |
| snapshot → restore → replay เหมือนเดิม | ✅ วัดแล้ว (Scene10) |
| ข้าม compiler/OS/CPU | ⛔ **ไม่รับประกัน** — Jolt มีโหมด `CROSS_PLATFORM_DETERMINISTIC` (option `VEXORA_PHYSICS_CROSS_PLATFORM_DETERMINISM`) แต่ยังไม่มี CI matrix ตรวจสอบ |
| เปลี่ยน profile/fixedDt | ผลต่างกันโดยตั้งใจ |

เงื่อนไขที่โค้ดรักษาไว้: contact events ถูกเรียงลำดับหลัง step (callback มาจากหลาย thread), stateHash เดินตามลำดับ handle, ไม่ใช้ `std::hash`/unordered containers ในเส้นทางที่กระทบผลลัพธ์

## 8. Error Handling และการป้องกันปัญหา

| ปัญหา | การป้องกัน | สถานะ |
|---|---|---|
| Input ผิด (NaN, ขนาด ≤ 0, restitution > 1, layer เกิน) | ตรวจก่อนถึง solver คืน `Result/Status` พร้อมข้อความ | ✅ |
| Handle ที่ถูกลบแล้ว | Generational handle | ✅ |
| ลบ body ที่มี constraint | ลบ constraint ที่อ้างถึงก่อน | ✅ |
| NaN เกิดระหว่าง step | คืน pose ล่าสุดที่ถูกต้อง, หยุด body, ออก `SolverWarning::NonFiniteState` | ✅ (โค้ด) |
| Solver explosion | `SolverWarning::ExcessiveVelocity` เมื่อเกิน `explosionSpeedWarning` | ✅ |
| Engine buffer เต็ม | `SolverWarning::EngineCapacity`; temp allocator มี malloc fallback แทนการ abort | ✅ |
| Tunneling | Linear-cast CCD ต่อ body / บังคับใน FinalBake | ✅ |
| Jitter | fixed step + interpolation + sleeping | ✅ |
| Floating-point drift ไกลจาก origin | 🟡 ตัวเลือก double precision ของ Jolt ยังไม่ได้เปิด/ทดสอบ |
| Snapshot ไม่ตรงโครงสร้างโลก | `structureVersion` → `ErrorCode::StateMismatch` โดยไม่แตะ state | ✅ |
| Memory leak | RAII (pimpl, Jolt Ref) และ Jolt registration แบบ ref-count | ✅ โค้ด / 🟡 ยังไม่ได้รันกับ ASan ใน CI |

## 9. Simulation Cache, Baking และ Reproducibility 🟡 (Phase F)

รากฐานที่มีแล้ว: fixed step, snapshot/restore, stateHash, determinism ในเครื่องเดียวกัน ✅

การออกแบบ cache:
- **แยกไฟล์** ออกจากข้อมูลต้นฉบับ (`*.vxcache` ใน `.gitignore` แล้ว) ลบแล้วสร้างใหม่ได้เสมอ
- **Cache key** = hash ของ: authored data ของทุก body/shape/constraint/material ที่เกี่ยวข้อง, skeleton + mesh version, `QualitySettings`, gravity, external forces, random seed, initial state, frame range, cache format version, Jolt version + build flags
- **Invalidation**: dependency graph จาก cache ไปยัง asset; เปลี่ยน input ใด ๆ → hash ไม่ตรง → แสดง "stale" ใน Cache Inspector และไม่เล่น cache ผิดเวอร์ชันแบบเงียบ ๆ
- **เนื้อหา**: transform ต่อ frame ต่อ body (และ particle/vertex ภายหลัง) + snapshot เป็นระยะ (keyframe) เพื่อ resume การ bake กลางทางได้
- **กู้คืนหลังโปรแกรมปิดผิดปกติ**: เขียนแบบ chunk + manifest ที่ commit หลัง chunk สมบูรณ์ chunk ที่ไม่สมบูรณ์ถูกทิ้ง แล้ว resume จาก snapshot ล่าสุด
- **ลบ cache ที่ไม่ใช้**: GC ตาม cache key ที่ไม่มี asset อ้างถึง

## 10. Export และ Interoperability 🟡 (Phase F)

| ข้อมูล | วิธีส่งออก |
|---|---|
| Collision shapes แบบ primitive/convex | แปลงตรง (ระวังแกน capsule, convex radius, ขีดจำกัดจำนวน vertex ของ hull ใน engine ปลายทาง) |
| Triangle mesh collider | ส่งออกได้เฉพาะ static |
| Physics material | แปลงค่า + **วิธี combine ต่างกัน** ในแต่ละ engine (รายงานเป็น warning) |
| Constraints | map ตามชนิด; limits/motors/spring ไม่ตรงกัน 1:1 → รายงาน |
| Breakable constraints | เป็นตรรกะของ Vexora → ส่งออกเป็น metadata เท่านั้น |
| Ragdoll | ส่งออกเป็น bodies + joints + skeleton mapping |
| Cloth/hair/particle | bake เป็น animation/vertex cache หรือสร้างใหม่ใน engine ปลายทาง |
| Units & axes | Vexora: เมตร, Y-up, right-handed → แปลงใน export layer (เช่น Unreal: cm, Z-up, left-handed) |

**ห้ามสมมติว่าส่งออกแล้วได้ผลเหมือนเดิม** — exporter ต้องสร้าง Export Validation Report แสดงสิ่งที่สูญหาย/ประมาณค่า และ Scene12 ต้องทดสอบ round-trip

## 11. Physics UI และ Debugging

ส่วนที่ทำแล้วในระดับข้อมูล (renderer-agnostic) ✅:
- `collectDebugDraw()` → bounds (สีตามสถานะ static/active/sleeping/trigger), center of mass, velocity vectors, contact points + normals, constraint lines (สีแดงเมื่อหักแล้ว)
- `warnings()`, `brokenConstraints()`, `contactEvents()`, `lastStepStats()` (เวลา/step, จำนวน body ที่ active) — เป็นแหล่งข้อมูลของ Solver Warnings และ Performance Profiler

UI panels (Properties, Rigid Body Inspector, Collision Shape Editor, Joint Editor, Bake Controls, Cache Inspector, Profiler) 🟡 — ยังไม่มี UI framework ในรีโป จะสร้างบน API ข้างต้นโดยไม่ถือ simulation state

## 12. Threading

ทุก method ของ `PhysicsWorld` เรียกจาก thread เจ้าของโลกเท่านั้น `step()` ใช้ worker threads ภายในได้ (`WorldSettings::workerThreads`) ✅ หลายโลกทำงานพร้อมกันได้ (Jolt registration แบบ ref-count) ✅ — แต่ยังไม่ได้ทดสอบการ step หลายโลกจากหลาย thread พร้อมกัน 🟡
