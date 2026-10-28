# 01 — Physics Engine Evaluation (Phase A)

> สถานะเอกสาร: ผลการประเมินของ Phase A ณ 2026-10-28
> หลักการ: แยก **"ตรวจสอบแล้วด้วย Test ในรีโปนี้"** ออกจาก **"มีใน Source/เอกสารของ Engine แต่ยังไม่ได้ทดสอบ"** และ **"ข้อมูลจากแหล่งภายนอก ยังไม่ยืนยัน"** อย่างชัดเจน

## 1. ประเภท Simulation ที่ MVP ต้องใช้

| ระบบ | จำเป็นสำหรับ MVP? | เหตุผล |
|---|---|---|
| Rigid Body + Collision + Materials | **ใช่** | เป็นฐานของทุกระบบ (Ragdoll, Props, Debris, Collider ที่ส่งออกไป Game Engine) |
| Constraints / Joints | **ใช่** | Ragdoll และ Mechanical Props ต้องใช้ |
| Fixed timestep, Snapshot, Determinism | **ใช่** | Baking/Cache (Phase F) ต้องพึ่งการเล่นซ้ำได้ผลเหมือนเดิม |
| Ragdoll + Animation/Physics Blending | ใช่ (Phase C) | Use case หลักของ Character Physics |
| Spring Secondary Motion (bone chain) | ใช่ (Phase C) | ใช้บ่อยที่สุดในงานตัวละครเกม ต้นทุนต่ำ |
| Cloth | ภายหลัง (Phase D) | ความเสี่ยงสูง ต้องเลือก Solver แยก |
| Hair strands / Soft body | ภายหลัง (Phase D) | ไม่มีใน Rigid-body engine ส่วนใหญ่ |
| Particles (VFX) | ภายหลัง (Phase E) | แยกจาก Rigid-body engine โดยธรรมชาติ |

ข้อสรุป: ตัวเลือก Engine ใน Phase A ตัดสินจาก **Rigid Body + Constraints + Determinism + License + การ Embed ใน DCC tool** เป็นหลัก ระบบ Deformable/Particle ประเมินแยก (หัวข้อ 4)

## 2. ตัวเลือกที่พิจารณา

### 2.1 Jolt Physics v5.3.0 — **เลือกใช้**

- **License:** MIT (ตรวจจากไฟล์ `LICENSE` ใน source ที่ pin ไว้) ใช้เชิงพาณิชย์และ static link ได้ ต้องแนบ copyright notice
- **ภาษา/การ Build:** C++17, CMake, ไม่มี dependency ภายนอก Build เป็น static lib ได้ใน CMake ของเรา (ตรวจแล้ว: `cmake/Dependencies.cmake`)
- **Precision:** float เป็นค่าเริ่มต้น มีโหมด double precision (`DOUBLE_PRECISION`) สำหรับฉากขนาดใหญ่มาก

**ตรวจสอบแล้วด้วย Test ในรีโปนี้** (ดู `docs/physics/03-test-plan.md`):

| ความสามารถ | หลักฐาน |
|---|---|
| Static / Kinematic / Dynamic bodies | Scene01, test_transform_sync |
| Box, Sphere, Capsule, Convex Hull, Triangle Mesh (static), Static Compound | Scene01 |
| Mass จาก density, Mass override, Center of Mass ของ compound | Scene01 |
| Force / Torque / Impulse | Scene01 |
| Friction / Restitution (combine: friction = sqrt(a·b), restitution = max(a,b)) | Scene01 |
| Linear-cast CCD ป้องกัน tunnelling | Scene01 |
| Sleeping / Wake-up | Scene01, Scene02 |
| Stacking stability (tower 10 ชั้น, pyramid 55 กล่อง) | Scene02 |
| Fixed / Point / Hinge / Slider / Distance(spring) + limits + velocity motor | Scene03 |
| Breakable constraints (คำนวณโดย **Vexora** จาก lambda ของ Jolt) | Scene03 |
| Sensor (trigger) + contact begin/persist/end | test_transform_sync_and_events |
| Collision layers/masks (ทำผ่าน `ObjectLayerPairFilter` ของเราเอง) | Scene01 |
| SaveState/RestoreState + replay ได้ผลเหมือนเดิมระดับบิต | Scene10 |
| Determinism: binary เดียวกัน, รัน 2 ครั้ง และ 0 vs 4 worker threads ได้ hash เดียวกัน | Scene10 |
| Multi-threading (JobSystemThreadPool) | Scene10, bench |

**มีใน Source/เอกสารของ Jolt v5.3.0 แต่ยัง *ไม่ได้* ทดสอบใน Vexora** (ห้ามอ้างว่าใช้ได้จนกว่าจะมี Test):

- `Ragdoll` / `RagdollSettings` พร้อม `DriveToPoseUsingMotors`, `DriveToPoseUsingKinematics`, `SetPose/GetPose` และ `Skeleton`/`SkeletonPose` — ตรงกับ Phase C
- `SwingTwistConstraint`, `SixDOFConstraint`, `ConeConstraint` (สำคัญสำหรับข้อต่อ Ragdoll), `PathConstraint`, `GearConstraint`, `RackAndPinionConstraint`, `PulleyConstraint`
- `CharacterVirtual` / `Character` (Character controller)
- Soft bodies (`SoftBodySharedSettings`): edge/bend/volume constraints, skinned constraints สำหรับยึดผ้ากับ skeleton
  - **ข้อจำกัดตามเอกสารของ Jolt เอง (Architecture.md, "Soft Body Work In Progress"):** soft body ชนได้เฉพาะกับ rigid body (ยังไม่มี soft-vs-soft), constraint ใช้กับ soft body ไม่ได้, `AddImpulse/SetLinearVelocity` ไม่มีผลกับ soft body
- Vehicles
- โหมด `CROSS_PLATFORM_DETERMINISTIC` — เอกสาร Jolt ระบุว่าช้าลงประมาณ 8% และ CI ของ Jolt ตรวจหลายสถาปัตยกรรม **แต่ Vexora ยังไม่ได้ตรวจข้ามแพลตฟอร์ม** จึงปิดไว้เป็นค่าเริ่มต้น และ **ไม่รับประกัน** determinism ข้ามแพลตฟอร์ม

**ไม่มีใน Jolt v5.3.0:** Hair/strand solver, Particle/VFX system, GPU simulation (ตัวเลือก `JPH_ENABLE_VULKAN` ใน CMake ของ Jolt ใช้กับ renderer ของ Samples เท่านั้น ตรวจจาก `TestFramework.cmake`)

### 2.2 NVIDIA PhysX 5 — ไม่เลือกเป็นตัวหลักในตอนนี้

ข้อมูลจากแหล่งภายนอก (ยังไม่ได้ build/ทดสอบใน Vexora):
- CPU source เป็น BSD-3 รายงานในปี 2025 ระบุว่า source ของ GPU kernels เปิดเผยแล้วด้วย
- PhysX 5 รวมความสามารถของ Flex: FEM soft body, PBD สำหรับผ้า/ของเหลว/วัตถุพองลม **แต่ฟีเจอร์เหล่านี้เป็น GPU-only และต้องใช้ GPU NVIDIA ที่รองรับ CUDA**
- ข้อดี: rigid body ระดับ production และ articulation (reduced coordinates) เหมาะกับ ragdoll/กลไก ใช้ใน Omniverse/Isaac
- ข้อเสียสำหรับ Vexora: ฟีเจอร์ deformable ผูกกับ CUDA (ผู้ใช้ AMD/Apple ใช้ไม่ได้) และการ build ซับซ้อนกว่า

→ **ทางเลือกสำรอง** หาก Phase C พบว่า Ragdoll บน Jolt ไม่เสถียรพอ หรือหากต้องการ GPU deformables บนเครื่อง NVIDIA เป็นฟีเจอร์เสริม

### 2.3 Bullet Physics (bullet3) — ไม่เลือก

ข้อมูลจากแหล่งภายนอก (ยังไม่ยืนยัน): license zlib, มี rigid + soft body (cloth/rope) บน CPU, มี multibody (Featherstone) แต่รอบการออก release ไม่สม่ำเสมอ (พบ 3.26 RC ใน registry ปี 2026) แหล่งข้อมูลยังขัดแย้งกันเรื่องสถานะการดูแลโครงการ
→ soft body ของ Bullet ทำงานบน CPU ได้ทุกแพลตฟอร์ม ซึ่งเป็นข้อดีเทียบกับ PhysX แต่คุณภาพ/ความเร็วของ rigid body และ API ด้อยกว่า Jolt สำหรับการใช้งานแบบ multi-threaded จึงไม่ใช้เป็นตัวหลัก **อาจพิจารณาเฉพาะ soft body ใน Phase D** หลังทดสอบเทียบกับ soft body ของ Jolt

### 2.4 ตัวเลือกอื่นที่ควรประเมินใน Phase D/E (ยังไม่ได้ศึกษาในเชิงลึก)

- Solver แบบ XPBD ที่เขียนเอง สำหรับผ้า/ผม/bone chain — ควบคุมได้เต็มที่ ทำงานบน CPU และ GPU (compute shader) โดยไม่ผูกกับผู้ผลิต GPU
- Rapier (Rust, Apache-2.0) — ต้องผ่าน FFI ไม่คุ้มเมื่อแกนเป็น C++
- MuJoCo (Apache-2.0) — แม่นยำสำหรับ robotics/articulation แต่ไม่ได้ออกแบบมาสำหรับฉาก DCC/เกมที่มีวัตถุจำนวนมาก

## 3. ผลการวัดประสิทธิภาพ (Phase A)

ดูตารางผลการวัดจริงที่ `docs/physics/03-test-plan.md` §Scene 13 สรุป: บนเครื่อง container 4 vCPU (Xeon 2.1 GHz, build ที่ไม่เปิด AVX) Interactive Preview จำลอง 2,000 วัตถุที่กองชนกันได้เร็วกว่า real-time และ 5,000 วัตถุยังไม่ถึง real-time ตัวเลขนี้ใช้เปรียบเทียบเชิงสัมพัทธ์เท่านั้น ไม่ใช่ spec ของเครื่องผู้ใช้

## 4. แผนการเลือก Solver สำหรับระบบที่ Jolt ไม่ครอบคลุม

| ระบบ | ข้อเสนอเบื้องต้น | ต้องพิสูจน์ก่อนตัดสินใจ |
|---|---|---|
| Spring secondary motion | Solver ของ Vexora เอง (Verlet/XPBD บน bone chain) ทำงานใน Pose space ของ skeleton **เป็นการประมาณผล ไม่ใช่การจำลองเต็มรูปแบบ** | Scene06/08: ไม่สะสมพลังงาน, reset ได้, loop ได้ |
| Cloth | เทียบ soft body ของ Jolt (มี skinned constraints) กับ XPBD ของเราเอง | Scene07: ชนกับตัวละคร, pin, wind, self-collision (Jolt ไม่มี soft-vs-soft) |
| Hair | Guide strands บน XPBD ของเราเอง (Jolt ไม่มี) / bone-based ใช้ระบบ spring | Scene08 |
| Particles | ระบบของเราเอง: CPU ก่อน (อ่านค่ากลับได้, deterministic), GPU compute ภายหลังเมื่อวัดผลแล้วคุ้ม | Scene09 |

## 5. ข้อสรุป

ใช้ **Jolt Physics v5.3.0** เป็น backend เดียวสำหรับ Rigid Body, Collision, Constraints และ (ตามแผน) Ragdoll โดยซ่อนไว้หลัง `vexora::physics::PhysicsWorld` (pimpl) ที่ไม่ส่ง type ของ Jolt ออกมาใน public header

**ไม่มี** multi-engine abstraction และ **ไม่อ้าง** ว่ารองรับหลาย Engine การแยก interface มีไว้เพื่อ (1) กันไม่ให้ระบบอื่นผูกกับ type ของ Jolt (2) ให้ทดสอบแยกจาก UI ได้ และ (3) ลดต้นทุนหากต้องเปลี่ยน Engine ในอนาคต — ถ้าจะเพิ่ม backend ที่สองต้องผ่าน test scenes ชุดเดียวกันทั้งหมดก่อน

## แหล่งอ้างอิง

- Jolt Physics v5.3.0 source และ `Docs/Architecture.md` (ส่วน Deterministic Simulation, Soft Bodies) — https://github.com/jrouwe/JoltPhysics/tree/v5.3.0
- NVIDIA PhysX 5 release / open source GPU: https://developer.nvidia.com/blog/open-source-simulation-expands-with-nvidia-physx-5-release , https://www.gamingonlinux.com/2025/04/nvidia-open-sourced-physx-and-flow-gpu-code/page=1/ , https://gamefromscratch.com/physx-5-1-released-open-source/
- Bullet: https://en.wikipedia.org/wiki/Bullet_(software) , https://registry-preview.bazel.build/modules/bullet
