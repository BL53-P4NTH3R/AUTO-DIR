# Auto-DIR: Autonomous Drainage Inspection Rover

A modular, waterproofed differential-drive inspection rover engineered to navigate, map, and detect blockages inside storm drains and culverts. Designed around a stacked octagonal chassis fabricated on a **Bambu Lab X1 Carbon** 3D printer.

## System Architecture

* **Mechanical & Enclosure:** Regular octagonal two-deck chassis (`210 mm` flat-to-flat) with interlocking perimeter gasket grooves (`2.0 mm`), under-slung front motor cradles, two `65 mm` front drive wheels, and one rear-center swivel caster.
* **Power Plant:** `3S2P` `18650` Li-ion battery pack (`68 x 65 x 45 mm`, `11.1V` nominal / `12.6V` full) managed by a `12.6V 40A` BMS, `3S 4A` charging module, dual isolation switches (mobile base and top frame), `1N5408` protection diodes, `100uF` decoupling capacitor, and a `9A` DC-DC buck converter stepping down to `5V` for compute and logic rails.
* **Low-Level Control:** **Arduino Nano** mounted on a half-size breadboard inside the mobile base driving dual **JGB37-520** DC gearmotors via an **LM298N** (or **BTS7960**) H-bridge motor driver.
* **High-Level Autonomy & Perception:** **Raspberry Pi 4 Model B** housed in the top frame running ROS 2, fused with a top-mounted `360°` **Camsense X1 / LD06 LiDAR** for SLAM and obstacle avoidance, plus a front-mounted **RPi Camera v1** for visual obstruction inspection.