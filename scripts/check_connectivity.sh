#!/usr/bin/env bash
# check_connectivity.sh - Diagnose connectivity issues with the Raspberry Pi 5
#
# Run this from your development machine (MacBook) to determine whether
# the problem is your WiFi, your local network, or the Pi itself.
#
# Usage: bash scripts/check_connectivity.sh [PI_IP]
#   PI_IP defaults to 192.168.0.99

set -euo pipefail

PI_IP="${1:-192.168.0.99}"
SSH_ALIAS="pi"
PASS="\033[0;32mPASS\033[0m"
FAIL="\033[0;31mFAIL\033[0m"
WARN="\033[0;33mWARN\033[0m"
SKIP="\033[0;36mSKIP\033[0m"

results=()

log_result() {
    local status="$1"
    local message="$2"
    results+=("$status|$message")
    case "$status" in
        PASS) echo -e "  [$PASS] $message" ;;
        FAIL) echo -e "  [$FAIL] $message" ;;
        WARN) echo -e "  [$WARN] $message" ;;
        SKIP) echo -e "  [$SKIP] $message" ;;
    esac
}

echo "============================================"
echo " Diff-Drive Robot Connectivity Diagnostics"
echo " Target: $PI_IP"
echo " Date:   $(date)"
echo "============================================"
echo ""

# --- Step 1: Check local network interface ---
echo "1. Local Network Interface"
if command -v ip &>/dev/null; then
    if ip route get 1.1.1.1 &>/dev/null; then
        IFACE=$(ip route get 1.1.1.1 2>/dev/null | head -1 | sed 's/.*dev \([^ ]*\).*/\1/')
        LOCAL_IP=$(ip route get 1.1.1.1 2>/dev/null | head -1 | sed 's/.*src \([^ ]*\).*/\1/')
        log_result "PASS" "Network interface '$IFACE' is up (local IP: $LOCAL_IP)"
    else
        log_result "FAIL" "No default route — not connected to any network"
    fi
elif command -v ifconfig &>/dev/null; then
    if ifconfig | grep -q "inet "; then
        log_result "PASS" "Network interface is up"
    else
        log_result "FAIL" "No active network interface found"
    fi
else
    log_result "SKIP" "Cannot determine interface (no ip or ifconfig)"
fi
echo ""

# --- Step 2: Check internet connectivity (is WiFi working at all?) ---
echo "2. Internet Connectivity (WiFi check)"
if ping -c 2 -W 3 8.8.8.8 &>/dev/null; then
    log_result "PASS" "Can reach internet (ping 8.8.8.8)"
else
    log_result "FAIL" "Cannot reach internet — WiFi may be down or disconnected"
fi

if ping -c 2 -W 3 google.com &>/dev/null; then
    log_result "PASS" "DNS resolution works (ping google.com)"
else
    log_result "WARN" "DNS resolution failed — internet reachable but DNS may be misconfigured"
fi
echo ""

# --- Step 3: Check gateway reachability ---
echo "3. Local Gateway"
if command -v ip &>/dev/null; then
    GW=$(ip route | grep default | head -1 | awk '{print $3}')
elif command -v route &>/dev/null; then
    GW=$(route -n get default 2>/dev/null | grep gateway | awk '{print $2}')
else
    GW=""
fi

if [ -n "$GW" ]; then
    if ping -c 2 -W 3 "$GW" &>/dev/null; then
        log_result "PASS" "Gateway $GW is reachable"
    else
        log_result "FAIL" "Gateway $GW is unreachable — local network issue"
    fi
else
    log_result "SKIP" "Could not determine gateway address"
fi
echo ""

# --- Step 4: Ping the Pi ---
echo "4. Raspberry Pi Reachability"
if ping -c 3 -W 3 "$PI_IP" &>/dev/null; then
    RTT=$(ping -c 3 -W 3 "$PI_IP" 2>/dev/null | tail -1 | sed 's/.*= //' | cut -d'/' -f2)
    log_result "PASS" "Pi responds to ping at $PI_IP (avg RTT: ${RTT}ms)"
else
    log_result "FAIL" "Pi not responding to ping at $PI_IP"
    echo "         Possible causes:"
    echo "           - Pi is powered off"
    echo "           - Pi WiFi is disconnected"
    echo "           - Pi IP has changed (check router DHCP leases)"
    echo "           - Pi firewall blocking ICMP"
fi
echo ""

# --- Step 5: Check SSH port ---
echo "5. SSH Service"
if command -v nc &>/dev/null; then
    NC_CMD="nc"
elif command -v ncat &>/dev/null; then
    NC_CMD="ncat"
else
    NC_CMD=""
fi

if [ -n "$NC_CMD" ]; then
    if $NC_CMD -z -w 5 "$PI_IP" 22 &>/dev/null; then
        log_result "PASS" "SSH port 22 is open on $PI_IP"
    else
        log_result "FAIL" "SSH port 22 is closed or filtered on $PI_IP"
        echo "         Pi may be up but sshd is not running"
    fi
else
    # Fallback: try ssh with a short timeout
    if timeout 5 bash -c "echo > /dev/tcp/$PI_IP/22" 2>/dev/null; then
        log_result "PASS" "SSH port 22 is open on $PI_IP"
    else
        log_result "WARN" "Could not verify SSH port (no nc/ncat available)"
    fi
fi
echo ""

# --- Step 6: Test SSH connection ---
echo "6. SSH Login"
if ssh -o ConnectTimeout=5 -o BatchMode=yes "$PI_IP" "echo ok" &>/dev/null 2>&1; then
    log_result "PASS" "SSH login successful (key-based auth)"
elif ssh -o ConnectTimeout=5 -o BatchMode=yes "$SSH_ALIAS" "echo ok" &>/dev/null 2>&1; then
    log_result "PASS" "SSH login successful via alias '$SSH_ALIAS'"
else
    log_result "WARN" "SSH key-based login failed (may need password, or key not configured)"
    echo "         Try manually: ssh $SSH_ALIAS"
fi
echo ""

# --- Step 7: Check if Pi's ROS 2 is responsive (if SSH works) ---
echo "7. ROS 2 Environment (requires SSH access)"
ROS_CHECK=$(ssh -o ConnectTimeout=5 -o BatchMode=yes "$PI_IP" \
    "source /opt/ros/jazzy/setup.bash 2>/dev/null && ros2 doctor --report 2>/dev/null | head -5" 2>/dev/null) || true

if [ -n "$ROS_CHECK" ]; then
    log_result "PASS" "ROS 2 Jazzy responding on Pi"
else
    log_result "SKIP" "Could not verify ROS 2 (SSH access required)"
fi
echo ""

# --- Summary ---
echo "============================================"
echo " SUMMARY"
echo "============================================"

fail_count=0
warn_count=0
pass_count=0
for r in "${results[@]}"; do
    case "${r%%|*}" in
        FAIL) ((fail_count++)) ;;
        WARN) ((warn_count++)) ;;
        PASS) ((pass_count++)) ;;
    esac
done

echo "  Passed: $pass_count  |  Warnings: $warn_count  |  Failed: $fail_count"
echo ""

if [ "$fail_count" -eq 0 ] && [ "$warn_count" -eq 0 ]; then
    echo "  All checks passed. Connectivity looks good."
elif [ "$fail_count" -eq 0 ]; then
    echo "  Minor issues detected — review warnings above."
else
    # Provide targeted diagnosis
    has_internet=true
    has_pi_ping=true
    has_ssh=true
    for r in "${results[@]}"; do
        case "$r" in
            *"Cannot reach internet"*) has_internet=false ;;
            *"Pi not responding"*) has_pi_ping=false ;;
            *"SSH port 22 is closed"*) has_ssh=false ;;
        esac
    done

    if [ "$has_internet" = false ]; then
        echo "  DIAGNOSIS: Your WiFi/internet connection is down."
        echo "  ACTION:    Check your WiFi connection and router."
    elif [ "$has_pi_ping" = false ]; then
        echo "  DIAGNOSIS: WiFi works, but the Pi is unreachable."
        echo "  ACTION:    Check if the Pi is powered on and connected"
        echo "             to the same network. Try the USB-TTL serial"
        echo "             cable for direct console access."
    elif [ "$has_ssh" = false ]; then
        echo "  DIAGNOSIS: Pi is on the network but SSH is down."
        echo "  ACTION:    Connect via serial cable and run:"
        echo "               sudo systemctl restart ssh"
    fi
fi
echo ""
