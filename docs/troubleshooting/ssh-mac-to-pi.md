# Troubleshooting: SSH from Mac to Pi

**Status:** UNRESOLVED as of 2026-03-13
**Symptom:** Mac cannot SSH into Pi at 192.168.68.60. Fails with `No route to host` or times out.

---

## Environment

| Device | Interface | IP | Subnet | MAC |
|---|---|---|---|---|
| MacBook Pro 2021 | en0 (Wi-Fi) | 192.168.68.55 | /22 (255.255.252.0) | — |
| Raspberry Pi 5 | wlan0 | 192.168.68.60 | /22 (255.255.252.0) | 2c:cf:67:dc:5b:d8 |
| Deco router/gateway | — | 192.168.68.1 | — | — |

- Network: TP-Link Deco mesh Wi-Fi
- Pi OS: Ubuntu 24.04 Desktop, ROS 2 Jazzy
- Mac OS: macOS (Darwin 25.3.0)
- SSH user on Pi: `nate`
- Pi hostname: `nate-raspi5` (mDNS: `nate-raspi5.local`)
- DHCP reservation set in Deco app: MAC `2c:cf:67:dc:5b:d8` → 192.168.68.60

---

## Confirmed Working

- Pi has internet access: `ping 8.8.8.8` from Pi → 0% packet loss
- Pi gateway reachable: Pi → 192.168.68.1 ✓
- **Pi → Mac ping: 0% packet loss** (asymmetric — Pi can reach Mac, Mac cannot reach Pi)
- Mac → router (192.168.68.1): works after VPN disconnect
- Pi firewall: `sudo ufw status` → inactive
- Pi wlan0: UP, `192.168.68.60/22`, eth0: DOWN
- Mac ARP entry for Pi: `2c:cf:67:dc:5b:d8` on en0 (correct MAC, valid, not expired)
- Mac route to Pi: via en0, flags UP — route looks correct

---

## Timeline of Attempts

### Session ~2026-03-12 (prior — Desktop, hit context limit)

**Symptom:** SSH from Mac to Pi failing.

**Diagnosis:** Stale ARP cache on Mac.
- `arp -a` showed Pi entry with `expire: -101` (expired)

**Fix applied:** `sudo arp -d 192.168.68.60`
- SSH worked temporarily after this

**Also done:** Set DHCP reservation in Deco app (MAC → IP) to prevent IP drift.

**Outcome:** Temporary fix only. Problem returned.

---

### Session 2026-03-12 (current)

SSH failing again despite DHCP reservation.

**Step 1 — nmap scan from Mac**
```bash
brew install nmap   # nmap not installed
nmap -sn 192.168.68.55/24
```
Result: Only `192.168.68.55` found (the Mac itself). Pi not visible.

**Step 2 — Connected monitor/keyboard directly to Pi**

```bash
ip addr
```
Output: `wlan0: UP, 192.168.68.60/22, brd 192.168.71.255`. eth0: DOWN.

```bash
sudo ufw status
```
Output: inactive

```bash
ip route show default
```
Output: default via 192.168.68.1

```bash
ping -c 4 8.8.8.8
```
Output: 0% packet loss — Pi has full internet.

**Step 3 — Directional ping test**
- Pi → Mac (`ping 192.168.68.55`): **0% packet loss** ✓
- Mac → Pi (`ping 192.168.68.60`): **100% packet loss, "No route to host"** ✗

This asymmetry is the key finding. Pi can reach Mac; Mac cannot reach Pi.

**Step 4 — Checked Mac routing**

```bash
arp -a | grep 68.60
```
Output: `? (192.168.68.60) at 2c:cf:67:dc:5b:d8 on en0 ifscope [ethernet]`
→ ARP entry valid, correct MAC, not expired.

```bash
route get 192.168.68.60
```
Output:
```
route to: 192.168.68.60
destination: 192.168.68.60
interface: en0
flags: UP,HOST,DONE,LLINFO,WASCLONED,IFSCOPE,IFREF
expire: 919
```
→ Route exists, looks correct.

```bash
ipconfig getpacket en0 | grep subnet
```
Output: `subnet_mask (ip): 255.255.252.0`
→ Mac on /22 — same subnet as Pi. No mismatch.

**Step 5 — Discovered VPN was active on Mac**

Deco Advanced Settings checked for client isolation toggle — not found.

Noticed Mac couldn't reach router either. Asked if VPN was active — it was.

After VPN disconnect:
- Mac → router (`ping 192.168.68.1`): **works** ✓
- Mac → Pi (`ping 192.168.68.60`): **still fails** ✗

**Step 6 — Checked Mac network interfaces**

```bash
ifconfig | grep -E "^en[0-9]|inet |status"
```

Key findings:
- en0 is the only active interface (status: active)
- en0 has TWO inet addresses:
  - `inet 192.168.68.55 netmask 0xfffffc00` — LAN address (correct)
  - `inet 10.5.0.2 --> 10.5.0.2 netmask 0xffff0000` — **residual VPN tunnel address** (not cleared after disconnect)
- All other interfaces (en1–en6): status inactive

**Step 7 — SSH attempts (all failed)**
```
ssh rpi                  → No route to host (nate-raspi5.local)
ssh nate@192.168.68.60   → No route to host
ssh nate@192.168.68.55   → Connection refused (that's the Mac itself)
```

Session paused here.

---

### Session 2026-03-13

**Context:** SSH still failing. Resumed troubleshooting.

**Step 1 — Deco app review**

Two Pi-like devices visible in Deco:
- "nate-RasPi5" at .55 — active upload/download traffic
- "Raspi5" at .60 — matches DHCP reservation

Initially suspected Pi had drifted to .55. Tested:
```
ssh nate-raspi5.local  → Undefined error: 0
ssh nate@192.168.68.55 → Connection refused (port 22)
ssh nate@192.168.68.60 → No route to host
```
"Connection refused" at .55 means something is reachable there but SSH not running. Turned out .55 is the Mac itself. "nate-RasPi5" at .55 in Deco is a **stale cached entry** from a previous lease. Deleted from Deco app.

**Step 2 — Confirmed Pi state (monitor/keyboard)**

```bash
sudo systemctl enable --now ssh   # SSH was already running (started 1 day prior)
ip addr show wlan0
```
Output: `inet 192.168.68.60/22`, MAC `2c:cf:67:dc:5b:d8` — confirmed correct. SSH active and listening on 0.0.0.0:22.

DHCP reservation was previously disabled during troubleshooting — **re-enabled** for MAC `2c:cf:67:dc:5b:d8` → .60.

**Step 3 — Confirmed VPN residual cleared on Mac**

```bash
ifconfig en0 | grep inet
```
Output: only `inet 192.168.68.55` — no `10.5.0.2` VPN tunnel address. Mac is clean.

**Step 4 — ARP flush + ping (still fails)**

```bash
sudo arp -d 192.168.68.60 && ping -c 2 192.168.68.60; arp -a | grep 68.60
```
Result: 100% packet loss, "No route to host". No ARP entry repopulated after delete.

**Step 5 — Deco isolation settings checked**

- No "Client Isolation" or "AP Isolation" toggle found
- "Device Isolation" feature exists but Pi was NOT listed as an isolated device

**Step 6 — tcpdump on Pi while pinging from Mac (KEY FINDING)**

On Pi:
```bash
sudo tcpdump -i wlan0 arp
```
Pinged .60 from Mac twice. Result:
- Pi sees **heavy ARP traffic from .55** (Mac scanning subnet — likely background nmap)
- Pi sees its own ARP replies to various requests
- **NO ARP request for 192.168.68.60 from 192.168.68.55 appeared at all**

This confirms: the Mac's kernel never sends an ARP broadcast for .60 onto the wire. The packet is rejected before it reaches the network layer.

**Step 7 — Attempted to clear stale host route on Mac**

```bash
sudo route delete -host 192.168.68.60
```
Output: `delete host 192.168.68.60: not in table` — no stale host route existed.

Ping still fails immediately with "No route to host" even after ARP + route cleared.

---

## Confirmed Working (updated)

- Pi SSH: active, enabled, listening on 0.0.0.0:22
- Pi IP: confirmed .60 with correct MAC via direct observation
- Pi → Mac: works
- Mac → router (.1): works
- No VPN residual on Mac
- No stale host route on Mac
- Pi not in Deco Device Isolation list
- Pi UFW: inactive

---

## Key Finding

The Mac's kernel drops packets to .60 *before* generating an ARP request. The Pi never sees an ARP broadcast from the Mac for .60, even immediately after `arp -d` and `route delete`. This is a Mac-side kernel/routing issue.

---

## Hypotheses (updated, most → least likely)

1. **macOS packet filter (pf) rule blocking .60** — VPN software may have installed pf rules that were not cleaned up on disconnect. pf can block outbound traffic silently, causing immediate EHOSTUNREACH before ARP.

2. **Deco proxy ARP intercepting and swallowing requests** — Deco mesh may be doing proxy ARP for .60 and failing silently (no response forwarded to Pi). Mac sends ARP, Deco intercepts, never forwards, Mac gets no reply and falls back to EHOSTUNREACH. Would explain why Pi sees no ARP requests for .60 while other hosts' ARPs do reach it.

3. **macOS routing table anomaly** — Some other routing entry or flag is causing .60 specifically to be unreachable, even though the /22 network route via en0 looks correct.

---

## Next Steps (in recommended order)

1. **Check Mac pf rules:**
   ```bash
   sudo pfctl -sr 2>/dev/null | grep -E "60|block"
   netstat -rn | grep "68\."
   route get 192.168.68.60
   ```

2. **Ethernet test** — plug Pi into Deco node via ethernet cable. If SSH works over eth0 → Deco WiFi/proxy ARP is the problem.

3. **Check if Deco is doing proxy ARP** — from Mac, after `arp -d 192.168.68.60`, run `arp -a | grep 68.60` immediately after a ping attempt. If an entry appears with the **Deco's MAC** (not Pi's MAC `2c:cf:67:dc:5b:d8`) → proxy ARP confirmed.

4. **Reboot Mac** — fully resets pf, routing table, and any other kernel state.

---

## What We Know Is NOT the Problem

- Pi firewall (UFW inactive)
- Pi IP drift (DHCP reservation re-enabled, Pi confirmed at .60)
- Subnet mismatch (both on /22)
- Pi internet connectivity (works fine)
- Pi SSH service (confirmed running)
- Mac VPN residual (cleared)
- Stale host route on Mac (not in table)
- Deco Device Isolation (Pi not listed)
- Pi ARP responsiveness (Pi does respond to ARP from other hosts)
