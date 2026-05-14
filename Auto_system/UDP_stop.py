import socket

# ============================================
# ROBOT NETWORK CONFIG
# ============================================

ROBOT_IP = "192.168.1.123"   # 修改成 Arduino 输出的 IP
ROBOT_PORT = 4210            # 修改成你的 UDP port

# ============================================
# UDP STOP COMMAND
# ============================================

STOP_COMMAND = "Stop"

# ============================================
# CREATE UDP SOCKET
# ============================================

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

# ============================================
# SEND STOP SIGNAL
# ============================================

sock.sendto(
    STOP_COMMAND.encode(),
    (ROBOT_IP, ROBOT_PORT)
)

print("STOP SIGNAL SENT")