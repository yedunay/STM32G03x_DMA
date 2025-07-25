# 📡 BQ25672 Charge Management State Machine

## What It Does

This module sets up a clean, step-by-step state machine for the **BQ25672** power management chip. It controls how the battery gets charged, following the stages outlined in TI’s datasheet. The whole idea is to make sure each step in the charging process happens in the right order, safely and efficiently.

The state machine is designed to work smoothly with USB BC1.2 detection, JEITA thermal rules, and built-in fault handling. It gives you full control over the charging process through I²C, making the system smarter and more adaptable.


## 🔧 Why It Matters

Charging today’s lithium batteries isn’t just about plugging in power—it has to follow strict safety and performance rules. The BQ25672 chip handles a lot on its own (like trickle charge, fast charge, and status flags such as CHG\_STAT, VBUS\_STAT, FAULT\_STAT), but managing it manually gets messy fast. That’s where the **state machine** comes in.

It helps:

* Start and stop charging safely
* Handle adapter (VBUS) and OTG modes smartly
* Recover from errors without rebooting everything
* Work seamlessly with the main controller code
* Grow easily if you want to add custom logic later

In short, this setup lets your system respond to charging events as they happen, without relying on constant polling or messy one-off logic.


## 🔁 How It Works (State Flow)

```text
[IDLE]
   │
   ├── Adapter Plugged In ─► [VBUS_DETECT]
   │                            │
   │                            ├── BC1.2 Check Done ─► [CHARGE_CONFIG]
   │                                                      │
   │                                                      ├── Charging Enabled ─► [TRICKLE]
   │                                                                           │
   │                                                                           ├── Battery Voltage OK ─► [PRECHARGE]
   │                                                                                                 │
   │                                                                                                 ├── Voltage High Enough ─► [FAST CHARGE]
   │                                                                                                                          │
   │                                                                                                                          ├── Near Full ─► [TOP-OFF]
   │                                                                                                                                         │
   │                                                                                                                                         └── Fully Charged ─► [CHARGE_DONE]
   │
   └── Error Detected ─► [FAULT] ── Reset Attempt ──► [IDLE or INIT]
```

