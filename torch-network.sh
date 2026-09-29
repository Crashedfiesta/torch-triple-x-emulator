#!/usr/bin/env bash
#
# torch-network.sh
#
# Sets up the host-side TAP interface, IP forwarding and NAT needed by
# the Torch Triple X emulator.
#
# Network layout:
#   Torch la0 : 192.168.2.2/24
#   Host tap0 : 192.168.2.1/24
#   Torch default gateway: 192.168.2.1
#
# Run this script on the Linux host before starting Triplex.
# Triplex itself should then be launched with:
#
#   ./triplex ... --tap tap0
#
# The script automatically detects the host's normal outbound interface.

set -euo pipefail

TAP_IF="tap0"
TAP_ADDR="192.168.2.1/24"
TORCH_NET="192.168.2.0/24"
OWNER="${SUDO_USER:-$USER}"

if [[ $EUID -ne 0 ]]; then
    echo "This script needs root privileges for TAP/NAT setup."
    echo "Re-running with sudo..."
    exec sudo "$0" "$@"
fi

# Find the normal outbound interface from the default route.
WAN_IF="$(ip route show default | awk '/default/ {print $5; exit}')"

if [[ -z "${WAN_IF:-}" ]]; then
    echo "ERROR: Could not determine the outbound network interface."
    exit 1
fi

echo "[Torch network] Outbound interface: $WAN_IF"
echo "[Torch network] TAP owner: $OWNER"

# Ensure the TUN/TAP kernel module is available.
modprobe tun

# Create tap0 if it does not already exist.
if ! ip link show "$TAP_IF" >/dev/null 2>&1; then
    echo "[Torch network] Creating $TAP_IF..."
    ip tuntap add dev "$TAP_IF" mode tap user "$OWNER"
else
    echo "[Torch network] $TAP_IF already exists."
fi

# Ensure tap0 has the correct host-side address.
if ! ip -4 addr show dev "$TAP_IF" | grep -qF "192.168.2.1/24"; then
    echo "[Torch network] Configuring $TAP_IF as $TAP_ADDR..."
    ip addr flush dev "$TAP_IF" scope global
    ip addr add "$TAP_ADDR" dev "$TAP_IF"
fi

ip link set "$TAP_IF" up

# Enable IPv4 forwarding.
echo "[Torch network] Enabling IPv4 forwarding..."
sysctl -w net.ipv4.ip_forward=1 >/dev/null

# Add NAT rule only if it is not already present.
if ! iptables -t nat -C POSTROUTING -s "$TORCH_NET" -o "$WAN_IF" -j MASQUERADE 2>/dev/null; then
    echo "[Torch network] Adding NAT rule..."
    iptables -t nat -A POSTROUTING -s "$TORCH_NET" -o "$WAN_IF" -j MASQUERADE
fi

# Allow forwarding from the Torch TAP to the outside world.
if ! iptables -C FORWARD -i "$TAP_IF" -o "$WAN_IF" -j ACCEPT 2>/dev/null; then
    echo "[Torch network] Allowing outbound forwarding..."
    iptables -A FORWARD -i "$TAP_IF" -o "$WAN_IF" -j ACCEPT
fi

# Allow return traffic back to the Torch.
if ! iptables -C FORWARD -i "$WAN_IF" -o "$TAP_IF" \
        -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT 2>/dev/null; then
    echo "[Torch network] Allowing return traffic..."
    iptables -A FORWARD -i "$WAN_IF" -o "$TAP_IF" \
        -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
fi

echo
echo "Torch networking is ready."
echo
echo "Host TAP:            192.168.2.1"
echo "Torch la0:           192.168.2.2"
echo "Torch default route: 192.168.2.1"
echo "Triplex option:      --tap tap0"
echo
echo "Current tap0 status:"
ip -brief addr show "$TAP_IF"
