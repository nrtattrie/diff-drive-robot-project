# Troubleshooting Guide

## Connectivity Diagnostics

### Quick Check: "Is it me or the Pi?"

Run the automated diagnostic script from your MacBook:

```bash
bash scripts/check_connectivity.sh
```

Or with a custom IP (default is `192.168.0.99`):

```bash
bash scripts/check_connectivity.sh 192.168.0.100
```

The script tests connectivity layer by layer and provides a diagnosis at the end.

### Manual Step-by-Step Diagnosis

Work through these checks in order. The first failure tells you where the problem is.

#### 1. Is your WiFi working?

```bash
ping -c 3 8.8.8.8
```

- **Works:** Your WiFi and internet are fine. Problem is between you and the Pi.
- **Fails:** Your WiFi is down. Reconnect to your network.

#### 2. Can you reach the router?

```bash
# macOS
route -n get default | grep gateway
# Then ping the gateway IP
ping -c 3 <gateway-ip>
```

- **Works:** Local network is fine. Problem is the Pi.
- **Fails:** Router issue or you're on the wrong network.

#### 3. Can you reach the Pi?

```bash
ping -c 3 192.168.0.99
```

- **Works:** Pi is on the network. Problem is at the SSH/application layer.
- **Fails:** Pi is off, on a different network, or its IP changed. Check your router's DHCP lease table for the Pi's current IP.

#### 4. Is SSH running on the Pi?

```bash
nc -z -w 5 192.168.0.99 22 && echo "SSH open" || echo "SSH closed"
```

- **Works:** SSH daemon is running. Problem is authentication.
- **Fails:** SSH service is down. Use the USB-TTL serial cable to access the Pi directly and restart it:
  ```bash
  sudo systemctl restart ssh
  ```

#### 5. Can you log in?

```bash
ssh pi
# or
ssh <username>@192.168.0.99
```

- **Works:** You're connected.
- **Fails with "Permission denied":** Wrong password or SSH key issue.
- **Hangs then times out:** Firewall or network issue (go back to step 3).

### Common Scenarios

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Nothing works (no internet, no Pi) | WiFi disconnected | Reconnect to home WiFi |
| Internet works, Pi doesn't ping | Pi is off or IP changed | Power cycle Pi; check router for new IP |
| Pi pings but SSH fails | sshd stopped | Serial cable → `sudo systemctl restart ssh` |
| SSH connects but VS Code Remote fails | VS Code extension issue | Restart VS Code; check "Local Network" permission on macOS |
| Everything worked yesterday | Pi lost WiFi after reboot | Serial cable → check `nmcli device status` |

### Fallback: USB-TTL Serial Console

When SSH is completely unavailable, use the Adafruit USB to TTL Serial Cable (#954):

1. Connect the cable to the Pi's GPIO header:
   - **White (RX)** → Pi GPIO 14 (TXD, pin 8)
   - **Green (TX)** → Pi GPIO 15 (RXD, pin 10)
   - **Black (GND)** → Pi GND (pin 6)
   - **Do NOT connect the red (5V) wire** if the Pi is powered separately

2. On your MacBook:
   ```bash
   ls /dev/tty.usbserial*
   screen /dev/tty.usbserial-XXXX 115200
   ```

3. Press Enter to get a login prompt. Log in with your Pi credentials.

4. Diagnose the network:
   ```bash
   nmcli device status          # Check WiFi connection state
   nmcli device wifi list       # Scan available networks
   ip addr show wlan0           # Check IP assignment
   sudo systemctl restart ssh   # Restart SSH if needed
   ```

### Preventing Future Issues

- **Set a static IP** for the Pi in your router's DHCP settings (reserve `192.168.0.99`)
- **Set up SSH key authentication** to avoid password issues:
  ```bash
  ssh-keygen -t ed25519
  ssh-copy-id <username>@192.168.0.99
  ```
- **Enable mDNS** so you can reach the Pi by hostname instead of IP:
  ```bash
  # On the Pi
  sudo apt install avahi-daemon
  # Then connect from Mac with:
  ssh <username>@<hostname>.local
  ```
