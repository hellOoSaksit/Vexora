# 03 — Physics Test Scenes: Expected Results, Acceptance Criteria, Measured Results

**วิธีรัน**

```bash
cmake -S . -B build && cmake --build build -j
./build/engine/physics/tests/vexora_physics_tests      # Test scenes (บรรทัด [measure] คือค่าที่วัดได้)
./build/engine/physics/bench/vexora_physics_bench 4 500 2000 5000   # Scene 13
```

**สถานะ:** ✅ Implemented & passing · 🟡 Planned (ยังไม่มีโค้ด — ห้ามถือว่าระบบนั้นใช้งานได้)

ค่าที่วัดได้ด้านล่างมาจากการรันจริงเมื่อ 2026-10-28: Linux x86-64, GCC 13.3, Release, Jolt 5.3.0 (SSE4.2, ไม่เปิด AVX) ผล: **38 test cases / 6,426 assertions ผ่านทั้งหมด**

**หลักการตั้ง Threshold:** ค่าที่มีสูตรทางฟิสิกส์ใช้สูตรนั้นเป็นค่าคาดหวัง (free fall, e²h, tan θ, spring sag) ค่าที่ขึ้นกับ solver (joint stretch, penetration) ใช้ค่าที่วัดได้ + headroom เพื่อเป็น regression guard และระบุไว้ว่าเป็นเช่นนั้น ไม่ใช้เกณฑ์เดียวกันทุกระบบ

---

## Scene 1 — Rigid Body Falling and Collision ✅
ไฟล์: `engine/physics/tests/scene01_rigid_body_falling.cpp`

| กรณี | Expected | Acceptance | วัดได้ |
|---|---|---|---|
| Box/Sphere/Capsule/ConvexHull/Compound ตกจาก 3 m ลงพื้น box และพื้น triangle mesh | มี contact Begin, หยุดนิ่ง, sleep | ความสูงหยุดนิ่งอยู่ในช่วง ±3 cm จากค่าที่คาด (penetration slop ของ Jolt = 2 cm), ความเร็ว < 0.05 m/s, ต่ำสุดระหว่างกระแทก > rest − 10 cm, sleep ภายใน 5 s | ทุก shape จมอยู่ราว 2.0 cm (box/sphere 0.480 m เทียบกับ 0.5 m), จมสูงสุดชั่วขณะ 4.6 cm, sleep ทุกตัว |
| Free fall 1 s | y = y0 − ½gt² − ½g·t·dt (semi-implicit Euler) | คลาดเคลื่อน ≤ 0.1% | 95.0132 เทียบกับ 95.0133 |
| Restitution e = 0, 0.5, 0.8 ตกจาก 2 m | เด้งสูง e²·h | คลาดเคลื่อน ≤ 15% หรือ ≤ 5 cm | 0.000 / 0.518 (คาด 0.500) / 1.289 (คาด 1.280) |
| Friction บนพื้นเอียง 20° (tan = 0.365) | μ = 0.6 → อยู่กับที่ · μ = 0.2 → ไถลลง | เคลื่อน < 2 cm / > 1 m ใน 2 s | 0.0000 m / 2.97 m |
| Tunnelling: ทรงกลม 5 cm, 300 m/s, ผนังหนา 2 cm | discrete ทะลุ (ข้อจำกัดที่บันทึกไว้), CCD ไม่ทะลุ | CCD หยุดก่อน x = 5 | discrete x = 150 (ทะลุ), CCD x = 4.952 |
| Collision layers | layer ที่ปิด mask ตกทะลุพื้น | y < −5 | ✅ |
| Mass/COM | density 500 × 1 m³ = 500 kg, override 42 kg, COM ของ compound = 0.984 | ±0.1% | 500 / 42 / 0.9841 |
| Force/Torque/Impulse | J = 4 N·s, m = 2 kg → Δv = 2 m/s; F = 2 N เป็นเวลา 1 s → v = 1 m/s | ±0.1% | ✅ |

## Scene 2 — Stacked Objects and Stability ✅
ไฟล์: `scene02_stacking.cpp`

| กรณี | Acceptance | วัดได้ (drift / sink) |
|---|---|---|
| หอกล่อง 10 ชั้น, 10 s, ทุก profile | drift แนวนอน < 2 cm, จมสะสม < 5 cm, sleep ครบ, ไม่มี warning | Interactive 0.49 cm / 1.28 cm · HQ 0.17 / 0.07 cm · FinalBake 0.02 / 0.01 cm |
| พีระมิด 55 กล่อง | เคลื่อน < 5 cm, sleep ครบ | 0.89 cm, sleep 55/55 |
| Stack ที่ sleep ถูกชนแล้วตื่น และกลับมา sleep | ตื่นหลังโดนชน, sleep ครบภายใน 15 s | ✅ |

**ผลการ sweep พารามิเตอร์ (ข้อมูลที่ใช้เลือกค่า profile)** — tower drift / hinge-chain max gap:

| dt | substeps | vel/pos iter | tower drift | chain gap |
|---|---|---|---|---|
| 1/60 | 1 | 6/1 | **541 cm (ถล่ม)** | 9.5 cm |
| 1/60 | 1 | 8/2 | 1.7 cm | 6.4 cm |
| 1/60 | 1 | 10/2 (Jolt default) | 0.49 cm | 6.4 cm |
| 1/120 | 2 | 10/2 | 0.17 cm | 0.88 cm |
| 1/240 | 2 | 16/4 | 0.02 cm | 0.21 cm |

ข้อสรุป: velocity iterations < 8 ที่ 60 Hz ทำให้หอถล่ม (ค่า Interactive รุ่นแรกที่ 6/1 ถูกจับได้โดย test นี้และแก้แล้ว) ส่วน joint stretch ขึ้นกับ dt/substeps มากกว่าจำนวน iteration

## Scene 3 — Jointed Mechanical Objects ✅
ไฟล์: `scene03_joints.cpp` — constraint ทุกชนิดคำนวณโดย Jolt ยกเว้น *การหัก* (breakable) ที่ Vexora คำนวณจาก lambda หลัง step

| กรณี | Acceptance | วัดได้ |
|---|---|---|
| โซ่ hinge 10 ข้อ ปล่อยจากแนวนอน 10 s | ระยะห่างของข้อต่อสูงสุด: Interactive < 8 cm, HQ < 1.5 cm, FinalBake < 0.5 cm; **พลังงานรวมไม่เพิ่มขึ้นเกิน 1%** (dissipate ได้ แต่ห้ามเพิ่ม = ไม่ระเบิด) ไม่มี solver warning | gap 6.4 / 0.88 / 0.21 cm; energy max = ค่าเริ่มต้นพอดีทุก profile (ไม่เพิ่ม) |
| Hinge limit ±0.5 rad ใต้แรงโน้มถ่วง | overshoot ชั่วขณะ < 0.04 rad, หยุดที่ limit ±0.005 rad | overshoot 0.0295 rad, หยุดที่ −0.500 |
| Hinge velocity motor 3 rad/s | ±1% ภายใน 2 s | 3.0000 |
| Slider limit −1 m, Fixed joint, Distance spring 2 Hz (1 kg) | slider หยุดที่ −1 m; fixed เคลื่อน < 1 cm; spring ยุบ g/(2πf)² = 6.2 cm ±1% | −1.000; 0.000; 3.938 m (คาด 3.938) |
| Breakable fixed joint (breakForce 1,000 N) | 50 kg (490 N) ไม่หัก; 200 kg (1,962 N) หัก + มี event + ตกลง | ✅ ตามคาด |

## Scene 4 — Character Ragdoll 🟡 Phase C
- **Setup:** skeleton มนุษย์ 15–20 bone, capsule ต่อ bone, SwingTwist joints พร้อม limit ตาม anatomy, collision group ปิดการชนของ bone ที่ติดกัน
- **Expected:** ล้มลงพื้นและหยุดนิ่งตามธรรมชาติ
- **Acceptance:** ทุก joint อยู่ใน limit (+ tolerance ที่วัดได้), ไม่มี bone ทะลุพื้น > 2 cm, ไม่มีการสั่นค้าง (ความเร็วเชิงมุมทุก bone < 0.05 rad/s หลัง 5 s) และ sleep, ไม่มี NaN/explosion warning จากการปล่อย 100 ครั้งด้วย pose/ความเร็วเริ่มต้นแบบสุ่ม (seed คงที่)
- **วิธีทดสอบ:** headless test + บันทึกค่าต่อ bone

## Scene 5 — Animation-to-Physics Blending 🟡 Phase C
- **Setup:** ตัวละครเดินด้วย animation → ถูกชนด้วย impulse → ragdoll → get-up animation
- **Acceptance:** (1) ที่ weight = 0 final pose ตรงกับ animation pose (error < 1e-4 m) (2) ตอนสลับไป ragdoll ความเร็วของ bone ต่อเนื่อง (ต่างจากความเร็วของ animation ≤ 10%) (3) ระหว่าง blend กลับ ไม่มี bone ใดถูกเขียนโดยสองระบบใน step เดียว (assert ใน debug build) (4) ไม่มีการกระโดดของตำแหน่งเกิน threshold ต่อ frame

## Scene 6 — Spring-based Secondary Motion 🟡 Phase C
- **Setup:** หาง 8 bone ติดกับ root ที่ขยับซ้าย–ขวาแบบ sine แล้วหยุด
- **Acceptance:** หลัง root หยุด แอมพลิจูดลดลงแบบ monotonic (ตรวจ peak ต่อเนื่อง), พลังงานไม่เพิ่ม, กลับสู่ rest pose ภายในเวลาที่ damping กำหนด ±10%, reset แล้วผลซ้ำเดิมระดับบิต, ผลที่ 30/60/144 FPS เหมือนกัน (fixed step)

## Scene 7 — Cloth Collision 🟡 Phase D
- **Setup:** ผ้า 32×32 pin มุมบนสองมุม ตกลงบนทรงกลมและ capsule ของตัวละคร มีลม
- **Acceptance:** ไม่มี vertex ทะลุ collider เกิน cloth thickness, ความยาว edge ยืดไม่เกิน X% (ตั้งจาก stretch stiffness), pinned vertex ไม่เคลื่อน, self-collision (หาก solver รองรับ — Jolt ไม่รองรับ soft-vs-soft) ไม่ทะลุกันเอง, bake + playback ตรงกัน

## Scene 8 — Hair or Bone Chain Simulation 🟡 Phase C/D
- **Setup:** 100 guide strands × 16 จุด หรือ bone chain ของผมหางม้า, หัวหมุน 180° ใน 0.5 s
- **Acceptance:** ไม่ทะลุ collision capsule ของหัว, ความยาว segment คงที่ ±1%, ไม่มีการสั่นค้างหลังหยุด, เวลา/frame อยู่ในงบที่กำหนดของ Interactive Preview

## Scene 9 — Particle Collision 🟡 Phase E
- **Setup:** emitter 10k particles, gravity + wind field, ชนพื้นและ capsule
- **Acceptance:** ไม่มี particle อยู่ใต้พื้น (CPU: ทุก particle, GPU: ตรวจจาก readback), bounce ตาม restitution ±10%, lifetime ตรงตามกำหนด, CPU simulation ด้วย seed เดิมได้ผลเดิม; GPU ไม่รับประกัน determinism เว้นแต่วัดได้
- **Performance:** วัด CPU vs GPU ที่ 1k / 10k / 100k particles ก่อนเลือกค่าเริ่มต้น

## Scene 10 — Simulation Baking and Playback
**ส่วนที่ทำแล้ว ✅ (รากฐาน):** ไฟล์ `scene10_determinism_snapshot.cpp`

| กรณี | Acceptance | วัดได้ |
|---|---|---|
| 200 bodies แบบสุ่ม (seed คงที่) + โซ่ 8 ข้อ รัน 2 ครั้ง | stateHash เท่ากัน | `237a41788bf44e6e` ทั้งสองครั้ง |
| 0 vs 4 worker threads | stateHash เท่ากัน | เท่ากันทั้ง 3 ครั้ง |
| Snapshot ที่ step 60 → รันไป 120 step → restore → รันซ้ำ | hash เท่ากัน, stepIndex กลับเป็น 60 | ✅ snapshot 48.6 KB |
| Snapshot จากโลกที่มีจำนวน body ต่างกัน | ปฏิเสธด้วย StateMismatch และไม่แตะ state | ✅ |
| สถานะการหักของ constraint ใน snapshot | restore แล้วสถานะ enabled ถูกต้องทั้งก่อนและหลังหัก | ✅ |

**ส่วนที่ยังไม่ทำ 🟡 (Phase F):** bake ช่วง frame ลงไฟล์ cache → เปิดโปรแกรมใหม่ → playback ตรงกับ live simulation (transform error = 0 บน binary เดิม), scrub timeline ไปข้างหลังได้, resume การ bake จาก snapshot

## Scene 11 — Cache Invalidation 🟡 Phase F
- **Acceptance:** เปลี่ยน input ทีละอย่าง (mesh, collision shape, material, constraint, solver settings, gravity, seed, frame range, Jolt version) → cache ต้องเปลี่ยนเป็น stale **ทุกกรณี**; เปลี่ยนสิ่งที่ไม่เกี่ยว (ชื่อ object, render material) → cache ยังใช้ได้; ฆ่า process ระหว่าง bake → เปิดใหม่ไม่ใช้ chunk ที่เสียและ resume ได้

## Scene 12 — Export and Re-import Validation 🟡 Phase F
- **Acceptance:** ส่งออก colliders/joints/materials → import กลับ → ค่าตรงกันภายใน tolerance ของการแปลง unit/axis; ข้อมูลที่ไม่รองรับ (breakable, motor บางชนิด, combine mode) ปรากฏใน Export Validation Report ครบ ไม่มีการตัดทิ้งแบบเงียบ ๆ

## Scene 13 — Performance Stress Test ✅
ไฟล์: `engine/physics/bench/stress_bench.cpp` — วัตถุผสม (sphere/box/capsule) ตกลงในถังที่มีผนัง ทุกวัตถุกองชนกัน (กรณีที่หนัก) จำลอง 4 s

เครื่องวัด: container 4 vCPU Intel Xeon 2.1 GHz (VM ที่ใช้ร่วมกับงานอื่น ค่า max มีสัญญาณรบกวน) · `x-real` = (dt ของ profile) / (เวลาเฉลี่ยต่อ step) — มากกว่า 1 คือเร็วกว่า real time · ไม่มี warning ในทุก run

| Profile | Bodies | Workers | avg ms/step | p95 ms | x-real |
|---|---|---|---|---|---|
| Interactive (1/60) | 500 | 0 | 1.21 | 1.74 | 13.8 |
| Interactive | 500 | 3 | 0.93 | 1.27 | 17.9 |
| Interactive | 2000 | 0 | 11.19 | 16.80 | 1.49 |
| Interactive | 2000 | 3 | 7.14 | 11.19 | 2.33 |
| Interactive | 5000 | 0 | 47.62 | 89.49 | 0.35 |
| Interactive | 5000 | 3 | 22.54 | 39.44 | 0.74 |
| HighQuality (1/120, 2 sub) | 500 | 3 | 1.57 | 2.09 | 5.32 |
| HighQuality | 2000 | 3 | 12.69 | 19.94 | 0.66 |
| HighQuality | 5000 | 3 | 33.30 | 59.26 | 0.25 |
| FinalBake (1/240, 2 sub) | 500 | 3 | 1.62 | 2.04 | 2.57 |
| FinalBake | 2000 | 3 | 13.65 | 22.42 | 0.31 |
| FinalBake | 5000 | 3 | 39.88 | 73.87 | 0.10 |

**ข้อสังเกต / งานต่อ:**
- Interactive Preview จำลองฉากที่กองชนกันแน่นได้ราว 2,000 วัตถุแบบ real-time บนเครื่องนี้ ฉากที่มากกว่านั้นต้องใช้ Bake (FinalBake ไม่จำเป็นต้อง real-time)
- Multi-threading เร็วขึ้น 1.3–2.1 เท่าที่ 4 cores — scaling ยังต่ำกว่าที่ควร ต้อง profile ต่อ (Phase B)
- Build ปัจจุบันปิด AVX/AVX2 เพื่อความเข้ากันได้ ควรวัด build AVX2 แยกก่อนกำหนด target ขั้นต่ำ
- **Acceptance สำหรับ Phase B:** Interactive 500 bodies ≥ 10× real-time และ 2,000 bodies ≥ 1× real-time บนเครื่องอ้างอิงที่กำหนดในอนาคต (ตอนนี้ผ่านบนเครื่องวัดนี้) และไม่มี capacity warning

## Tests อื่น ✅
- `test_clock_and_math.cpp` — FixedStepClock (frame-rate independence 30/60/144 FPS, จำกัด steps/frame + droppedTime, input NaN), interpolation, profile ต่างกันจริง
- `test_validation.cpp` — ปฏิเสธ dynamic triangle mesh พร้อมคำแนะนำ, shape ขนาดผิด/NaN/convex hull ไม่มีปริมาตร, material ผิด, layer เกิน, generational handles, ลบ body แล้ว constraint หายตาม, constraint ที่ผิด
- `test_transform_sync_and_events.cpp` — kinematic ตาม target ได้แม่นยำ (error 0) และผลักวัตถุได้, single-writer rule, interpolation + teleport, trigger/contact events, debug draw, NaN rejection, excessive velocity warning
