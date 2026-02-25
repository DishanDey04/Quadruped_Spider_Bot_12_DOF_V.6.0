# Q_BEE V5 🐝

**Short description**

Q_BEE V5 is a cleanly reorganized version of the previous refactor. It implements cartesian gait motions and precomputes servo targets/steps so the ISR remains short and deterministic.

---

## 🔧 Features

- Precomputed servo angle targets and per-tick steps (`servo_target`, `servo_step`, `servo_now`) for smooth interpolation.
- Short timer ISR (`servo_service`) that only increments angles and writes to hardware (20 ms tick via FlexiTimer2).
- Motion primitives separated into `Q_BEE_V5_moves.h` (walk, turn, wave, shake, dance, head up/down).
- Safe math: clamps for acos inputs, EPSILON to avoid divide-by-zero.

---

## 🗂 Key files

- `Q_BEE_V5.ino` — main sketch (initialization, kinematics, ISR, set_site logic)
- `Q_BEE_V5_moves.h` + `Q_BEE_V5_moves.cpp` — motion primitives and choreography functions

---

## 🔧 Build / Upload

1. Install Arduino IDE (or use PlatformIO).
2. Install libraries: `Servo` (builtin) and `FlexiTimer2` (available in Library Manager).
3. Open `Q_BEE_V5.ino` in the Arduino IDE.
4. Select the correct board and serial port and upload.
5. Power the servos from an appropriate external supply before running.

---

If you want, I can also update the top-level `README.md` to point to this new folder and add a brief migration note.
