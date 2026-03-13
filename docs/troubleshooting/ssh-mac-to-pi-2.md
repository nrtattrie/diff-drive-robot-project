# SSH: Mac → Pi Troubleshooting (2026-03-13)

## Symptom
Mac unable to SSH into Pi. Error: `port 22: No route to host`.

## Environment
- Pi: Raspberry Pi 5, Ubuntu, IP `192.168.68.60/22` (WiFi via wlan0)
- Mac: `192.168.68.55/22` (same subnet, same Deco mesh pod)
- Router: Spectrum SBG8300 in passthrough mode → two TP-Link Deco pods in WiFi Router mode

## What We Checked

### Pi side (all fine)
- `sshd` active, listening on port 22 (`0.0.0.0:22` and `[::]:22`)
- `~/.ssh/authorized_keys` exists, permissions correct (700/600)
- `ufw` was active but had no rules — ran `sudo ufw allow ssh`, confirmed it did nothing useful since ufw status showed **inactive**
- `iptables` empty — no firewall rules at all
- Pi → Mac ping: **100% success**

### Network side (ruled out)
- Both devices on same `/22` subnet — routing not the issue
- Both on same Deco mesh pod — inter-node isolation not the issue
- Deco client isolation: confirmed disabled
- Mac ARP table had Pi's MAC address — Layer 2 working

### Root cause: NordVPN network extension on Mac
- Mac → Pi ping: 100% packet loss, `sendto: No route to host`
- `ifconfig | grep utun` revealed **9 active utun interfaces** — NordVPN's system extension was running even though NordVPN was "disconnected"
- Routing table showed `192.168.68/22` with a **reject route (`!`)** — NordVPN's kernel extension was intercepting all traffic

## Fix Applied
1. Fully quit NordVPN from menu bar (not just disconnect)
2. Restarted Mac

## Permanent Solution: Tailscale
Raw LAN SSH is fragile — subject to IP changes, VPN interference, mesh quirks. Replaced with Tailscale.

### What Tailscale does
- Creates a private WireGuard-based overlay network ("tailnet") across all your devices
- Each device gets a stable `100.x.x.x` IP that never changes regardless of network
- Works through NAT, firewalls, VPNs automatically
- Free tier: up to 100 devices

### Setup on Pi
```bash
curl -fsSL https://tailscale.com/install.sh | sudo sh
sudo tailscale up   # opens auth URL — log in to authorize
tailscale ip        # get Pi's permanent 100.x.x.x address
```

### Setup on Mac
- Download Tailscale from tailscale.com, log into same account
- Add to `~/.ssh/config`:
```
Host pi
    HostName 100.x.x.x
    User nate
    IdentityFile ~/.ssh/id_ed25519
```
- Run `ssh-copy-id pi` for passwordless auth
- Install VS Code "Remote - SSH" extension → connect to `pi` for full remote dev experience

## Result
SSH via `ssh pi` works permanently regardless of NordVPN state, IP changes, or mesh topology.
