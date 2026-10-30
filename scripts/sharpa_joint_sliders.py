#!/usr/bin/env python3
"""
Sharpa hand joint sliders — run on the workstation.

Shows the live joint state published by the DDS bridge on Thor and lets you drive
the joints with sliders. Each joint has a slider (the target you command) and a bar
(the state the hand reports). The window starts in monitor-only mode and sends
nothing until you tick "Enable control".

Topics (see sharpa_dds_bridge.cpp):
  Subscribe rt/sharpa/<side>/state  HandState_  22 joints, degrees
  Publish   rt/sharpa/<side>/cmd    HandCmd_    22 joints, radians

Usage:
    conda activate tv
    python scripts/sharpa_joint_sliders.py [--side left|right|both]
                                           [--interface eno2] [--domain 0] [--check]

--interface defaults to the interface that holds a 192.168.123.x address (the robot's
DDS network, which the bridge service publishes on), else one with a 192.168.124.x address. --check prints the state rate for each hand
without opening a window and sends nothing.

Slider ranges are the test ranges from the Sharpa SDK sample (sharpa_wave_example.py),
not the hardware limits. Closing the window stops commands; the hand keeps its last
target, it does not go back to zero.
"""

import argparse
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402

from teleop.robot_control.robot_hand_sharpa_dds import NUM_JOINTS, SharpaHandDDSClient  # noqa: E402

# Robot DDS network first (what the bridge service pins to), then the hand network.
DDS_NET_PREFIXES = ("192.168.123.", "192.168.124.")
SEND_HZ = 30
UI_MS = 50

# (name, min deg, max deg) — Sharpa SDK sample ranges, in joint order.
JOINTS = [
    ("Thumb CMC flex/ext", 0, 50),
    ("Thumb CMC abd/add", 0, 10),
    ("Thumb MCP flex/ext", 0, 30),
    ("Thumb MCP abd/add", 0, 10),
    ("Thumb DIP flex/ext", 0, 40),
    ("Index MCP flex/ext", 0, 20),
    ("Index MCP abd/add", -20, 20),
    ("Index PIP flex/ext", 0, 20),
    ("Index DIP flex/ext", 0, 20),
    ("Middle MCP flex/ext", 0, 20),
    ("Middle MCP abd/add", -20, 20),
    ("Middle PIP flex/ext", 0, 20),
    ("Middle DIP flex/ext", 0, 20),
    ("Ring MCP flex/ext", 0, 20),
    ("Ring MCP abd/add", -20, 20),
    ("Ring PIP flex/ext", 0, 20),
    ("Ring DIP flex/ext", 0, 20),
    ("Pinky CMC flex/ext", 0, 10),
    ("Pinky MCP flex/ext", 0, 20),
    ("Pinky MCP abd/add", -20, 20),
    ("Pinky PIP flex/ext", 0, 20),
    ("Pinky DIP flex/ext", 0, 20),
]
assert len(JOINTS) == NUM_JOINTS


def detect_interface(prefixes=DDS_NET_PREFIXES):
    """Name of the first interface holding an IPv4 address that starts with one of prefixes."""
    out = subprocess.run(["ip", "-o", "-4", "addr"], capture_output=True, text=True).stdout
    for prefix in prefixes:
        for line in out.splitlines():
            parts = line.split()
            if len(parts) > 3 and parts[3].startswith(prefix):
                return parts[1]
    return None


class CountingClient(SharpaHandDDSClient):
    """SharpaHandDDSClient that counts received state messages."""

    def __init__(self, side):
        self.msg_count = 0
        super().__init__(side)

    def _on_state(self, msg):
        super()._on_state(msg)
        self.msg_count += 1


def run_check(clients, seconds=4):
    """Print the state rate and first joints for each hand, then exit."""
    prev = {s: 0 for s in clients}
    for t in range(1, seconds + 1):
        time.sleep(1)
        rates = {s: c.msg_count - prev[s] for s, c in clients.items()}
        prev = {s: c.msg_count for s, c in clients.items()}
        print(f"t={t}s  " + "  ".join(f"{s}: {r} msg/s" for s, r in rates.items()))
    ok = True
    for s, c in clients.items():
        _, deg = c.get_joint_position_degree()
        print(f"{s}: {c.msg_count} msgs, first joints (deg): " + " ".join(f"{v:.2f}" for v in deg[:6]))
        ok &= c.msg_count > 0
    return 0 if ok else 2


def run_gui(clients):
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("Sharpa hand joint sliders")

    top = ttk.Frame(root, padding=8)
    top.pack(fill="x")
    control = tk.BooleanVar(value=False)
    status = {}
    panels = {}

    def sync_all():
        for p in panels.values():
            p.sync_to_state()

    def on_enable():
        if control.get():
            sync_all()  # start from where the hands are, so nothing jumps

    ttk.Checkbutton(top, text="Enable control (sends commands)", variable=control, command=on_enable).pack(side="left")
    ttk.Button(top, text="Sync sliders to state", command=sync_all).pack(side="left", padx=8)
    ttk.Button(top, text="All zero", command=lambda: [p.zero() for p in panels.values()]).pack(side="left")
    ctrl_lbl = tk.Label(top, text="CONTROL OFF - sliders do nothing", fg="white", bg="#b00020", padx=6)
    ctrl_lbl.pack(side="left", padx=8)
    for side in clients:
        lbl = tk.Label(top, text=f"{side}: no data", fg="#b00020", width=18, anchor="e")
        lbl.pack(side="right")
        status[side] = lbl

    class Panel(ttk.Frame):
        def __init__(self, master, client):
            super().__init__(master, padding=6)
            self.client = client
            self.target = []
            self.state_bars = []
            self.state_lbls = []
            self.target_lbls = []
            ttk.Label(self, text="Joint", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=0, sticky="w")
            ttk.Label(self, text="Target (slide)", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=1)
            ttk.Label(self, text="State (bar)", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=3)
            for i, (name, lo, hi) in enumerate(JOINTS):
                r = i + 1
                var = tk.DoubleVar(value=0.0)
                self.target.append(var)
                ttk.Label(self, text=f"{i:2d}  {name}", width=24, anchor="w").grid(row=r, column=0, sticky="w")
                ttk.Scale(self, from_=lo, to=hi, variable=var, orient="horizontal", length=260).grid(
                    row=r, column=1, padx=6
                )
                tl = ttk.Label(self, text="0.0°", width=7, anchor="e")
                tl.grid(row=r, column=2)
                var.trace_add("write", lambda *_, v=var, lbl=tl: lbl.config(text=f"{v.get():.1f}°"))
                bar = ttk.Progressbar(self, maximum=hi - lo, length=170)
                bar.grid(row=r, column=3, padx=6)
                sl = ttk.Label(self, text="  0.0°", width=8, anchor="e")
                sl.grid(row=r, column=4)
                self.state_bars.append(bar)
                self.state_lbls.append(sl)

        def refresh_state(self):
            _, deg = self.client.get_joint_position_degree()
            for i, (_, lo, hi) in enumerate(JOINTS):
                v = deg[i] if i < len(deg) else 0.0
                self.state_bars[i]["value"] = min(max(v - lo, 0.0), hi - lo)
                self.state_lbls[i].config(text=f"{v:.1f}°")

        def sync_to_state(self):
            _, deg = self.client.get_joint_position_degree()
            for i, (_, lo, hi) in enumerate(JOINTS):
                v = deg[i] if i < len(deg) else 0.0
                self.target[i].set(min(max(v, lo), hi))

        def zero(self):
            for i, (_, lo, hi) in enumerate(JOINTS):
                self.target[i].set(min(max(0.0, lo), hi))

        def send(self):
            self.client.set_joint_position([math.radians(v.get()) for v in self.target])

    nb = ttk.Notebook(root)
    nb.pack(fill="both", expand=True, padx=8, pady=(0, 8))
    for side, client in clients.items():
        panels[side] = Panel(nb, client)
        nb.add(panels[side], text=f" {side} hand ")

    last = {"t": time.time(), "counts": {s: 0 for s in clients}, "sent": 0.0}

    def tick():
        now = time.time()
        for p in panels.values():
            p.refresh_state()
        if now - last["t"] >= 1.0:
            for s, c in clients.items():
                rate = (c.msg_count - last["counts"][s]) / (now - last["t"])
                last["counts"][s] = c.msg_count
                status[s].config(
                    text=f"{s}: {rate:.0f} Hz" if rate > 0 else f"{s}: no data", fg="#1b7f3b" if rate > 0 else "#b00020"
                )
            last["t"] = now
        if control.get():
            ctrl_lbl.config(text=f"CONTROL ON - sending {SEND_HZ} Hz", bg="#1b7f3b")
        else:
            ctrl_lbl.config(text="CONTROL OFF - sliders do nothing", bg="#b00020")
        if control.get() and now - last["sent"] >= 1.0 / SEND_HZ:
            for p in panels.values():
                p.send()
            last["sent"] = now
        root.after(UI_MS, tick)

    root.after(UI_MS, tick)
    root.mainloop()


def main():
    ap = argparse.ArgumentParser(description="Slider GUI for the Sharpa hands via the DDS bridge.")
    ap.add_argument("--side", choices=["left", "right", "both"], default="both")
    ap.add_argument(
        "--interface",
        default=None,
        help="DDS network interface (default: the one with a 192.168.123.x address, else 192.168.124.x)",
    )
    ap.add_argument("--domain", type=int, default=0, help="DDS domain id (must match the bridge; default 0)")
    ap.add_argument(
        "--check", action="store_true", help="print state rates for a few seconds and exit; no window, no commands"
    )
    args = ap.parse_args()

    iface = args.interface or detect_interface()
    if iface:
        print(f"DDS interface: {iface}")
    else:
        print("WARNING: no interface on 192.168.123.x or 192.168.124.x; using the DDS default", file=sys.stderr)
    ChannelFactoryInitialize(args.domain, iface)

    sides = ["left", "right"] if args.side == "both" else [args.side]
    clients = {s: CountingClient(s) for s in sides}

    if args.check:
        sys.exit(run_check(clients))
    run_gui(clients)


if __name__ == "__main__":
    main()
