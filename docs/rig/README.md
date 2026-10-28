# Vexora Rig & Animation (Phase R)

> สถานะ ณ 2026-10-28 · ✅ = มีโค้ดและ test ผ่าน · 🟡 = ยังไม่มี

## โครงสร้างโมดูล
```
engine/core   คณิตศาสตร์ (Vec3/Quat/Transform) + Result/Status — ทุกโมดูลใช้ร่วมกัน
engine/rig    Skeleton, Pose, SkinnedMesh, AnimationClip, glTF import   ← ไม่ขึ้นกับฟิสิกส์
engine/physics  ใช้ rig (Ragdoll, SecondaryMotion) ผ่าน Skeleton/ModelPose
```

## สิ่งที่ทำแล้ว
| ระบบ | สถานะ |
|---|---|
| Skeleton (หลาย root ได้, parents-first), LocalPose ↔ ModelPose, blend | ✅ |
| Linear blend skinning บน CPU (4 influences) | ✅ |
| Animation clip + sampling (loop / clamp) | ✅ |
| Import glTF 2.0 (.glb/.gltf) ด้วย cgltf v1.15 (MIT) | ✅ |
| Ragdoll อัตโนมัติจากโครงกระดูกใดก็ได้ (`makeAutoRagdoll`) | ✅ |
| ฉาก "ตัวละคร (glTF)" ในแอป: เลือกไฟล์, เล่นท่า, ผลักเป็น ragdoll, ลุกขึ้น, แสดงกระดูก/capsule | ✅ |
| FBX import | 🟡 (FBX SDK มีเงื่อนไข license — ประเมิน ufbx (MIT) ก่อน) |
| เครื่องมือสร้าง/แก้ Rig ใน UI, ทาน้ำหนัก skin, IK | 🟡 R5 |
| Animation graph / blend tree / timeline keyframe | 🟡 |
| Morph targets (blend shapes) | 🟡 (ถูกข้ามพร้อมแจ้งเตือน) |

## กฎการแปลงไฟล์ (ผู้ใช้เห็นทุกข้อที่ไม่ตรงกับไฟล์)
- หน่วยของ Vexora: เมตร, +Y ขึ้น, มือขวา (เหมือน glTF)
- Node ที่อยู่เหนือโครงกระดูก (armature, การหมุนแกน, scale 0.01) ถูกรวมเข้าไปใน root bone
- Transform ของ Vexora ไม่มี scale: scale แบบสม่ำเสมอถูก bake เข้าไปในตำแหน่ง ส่วน scale ไม่สม่ำเสมอบน joint → แจ้งเตือน
- Animation ถูก bake ที่ **เวลา keyframe จริงของไฟล์** (LINEAR ตรงทุก key), STEP/CUBICSPLINE เพิ่มจุด sample
- ถ้าท่า rest ไม่ตรงกับ bind pose → แจ้งเตือนว่า skinning เป็นค่าประมาณ
- ขนาดผิดปกติ (สูง < 0.3 m หรือ > 3 m) → "อัตโนมัติ" ปรับเป็น 1.7 m และแจ้งในหน้าจอ (เลือก "ตามไฟล์" หรือ "กำหนดความสูงเอง" ได้)

## ผลทดสอบ (`engine/rig/tests/test_gltf_import.cpp`)
เทียบกับ **reference อิสระ** ที่คำนวณด้วยฟังก์ชันของ cgltf โดยตรง (ตั้ง TRS ของ node ตาม keyframe แล้ว skin vertex ต้นฉบับด้วย world × inverseBind)

| ไฟล์ | กระดูก | ท่า | rest skinning error | animated error ที่ keyframe (สัดส่วนต่อความสูง) |
|---|---|---|---|---|
| RiggedSimple | 2 | 1 | 5.5e-8 | ≤ 1.4e-7 |
| RiggedFigure | 19 | 1 | 1.8e-7 | ≤ 6.5e-7 |
| Fox | 24 | 3 (Survey/Walk/Run) | 1.6e-7 | ≤ 5.2e-7 |

- ไฟล์ถูกตัดท้าย 6 แบบ → ถูกปฏิเสธทั้งหมด; ไฟล์ที่ byte เสีย → ไม่ crash (ASan/UBSan สะอาด)
- **Bug ที่ test จับได้และแก้แล้ว:** รุ่นแรก bake animation ที่ 60 fps ทำให้ key ที่ไม่ตรงจังหวะ (เช่น 1/24 s) คลาด 1.2e-3 — เปลี่ยนเป็น bake ที่เวลา keyframe จริง
- Ragdoll อัตโนมัติ: RiggedFigure 19 bones → 11 parts, Fox 24 → 17 parts; ล้มแล้วนิ่งใน 6 s, ไม่จมพื้นเกิน slop, ไม่มี warning (`scene04_ragdoll.cpp`)

## ไฟล์ตัวอย่าง
`assets/samples/` — RiggedSimple, RiggedFigure (CC-BY 4.0, Cesium), Fox (CC0 + CC-BY 4.0) จาก Khronos glTF Sample Assets ดู `assets/samples/README.md` สำหรับเครดิต
