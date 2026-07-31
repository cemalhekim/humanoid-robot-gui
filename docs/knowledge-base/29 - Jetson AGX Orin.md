---
tags: [jetson, agx-orin, network, hardware, edge-gpu]
summary: Jetson AGX Orin Developer Kit — static 192.168.123.100, SSH user jetson, JetPack 5.1.2, added 2026-07-31.
---

# 29 - Jetson AGX Orin

New edge-GPU box added to the project on **2026-07-31**. First set up headless
over a direct ethernet cable to the Mac; discovered via IPv6 link-local
multicast (`ping6 ff02::1%en0`), then given a static IP on the robot subnet.

## Hardware / software

| Item | Value |
| --- | --- |
| Model | NVIDIA Jetson AGX Orin Developer Kit (64 GB RAM variant) |
| Hostname | `jetson-agx-orin` |
| OS | Ubuntu 20.04 (L4T R35.4.1 = **JetPack 5.1.2**), kernel `5.10.120-tegra` |
| CPU | 8 cores (aarch64) |
| Storage | 57 GB eMMC root (`/dev/mmcblk0p1`), ~49 GB free |
| eth0 MAC | `3c:6d:66:4e:9d:09` (NVIDIA OUI) |

## Network

| Interface | Address | Notes |
| --- | --- | --- |
| `eth0` | **`192.168.123.100/24` static** | NM profile "Wired connection 1", `ipv4.method manual`, no gateway, autoconnect on |
| `l4tbr0` (USB device mode) | `192.168.55.1/24` | NVIDIA default USB-ethernet bridge |
| `wlan0` | `10.2.100.178/24` (DHCP, lease-based — may change) | Connected to `Vaderfone@CSIG` lab Wi-Fi since 2026-07-31, autoconnect on, internet works |

The static address lives on the **robot subnet** (`192.168.123.0/24`) so the
Jetson works both on a direct Mac↔Jetson cable and later on the robot switch.
`.100` chosen clear of the robot PC (`.164`), BagPack (`.165`) and the Mac
(`.222`).

## Access

- `ssh jetson` from the Mac (alias in `~/.ssh/config` → `jetson@192.168.123.100`,
  key `id_ed25519_robot`, password auth also enabled).
- Over Wi-Fi: `ssh jetson@10.2.100.178` (same key) — works from the lab Wi-Fi
  without the direct cable.
- Password is in the local credentials store (never in this repo) — see the
  memory note `credentials.md` on the Mac.
- USB device mode alternative: plug USB-C, then `ssh jetson@192.168.55.1`.

## Gotchas learned during bring-up

- The plain `ip -4` view shows nothing on `eth0` until NM finishes; fresh
  JetPack waits forever on DHCP ("connecting (getting IP configuration)") on a
  direct cable — hence the static profile.
- The board's 10G NIC negotiates `10Gbase-T` with the Mac; a "Jetson Nano"
  assumption (1G max) is wrong — this is an AGX Orin.
- Ethernet link drops entirely while the box reboots/powers off, so a dead
  `en0` link on the Mac usually means the Jetson is off, not a cable fault.
