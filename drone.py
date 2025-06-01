import socket
import threading
import time
import pygame
import subprocess
import shlex





# ─── Protocol constants ──────────────────────────────────────────────────────
HEADER         = 102  # CONTROL_VALUES_BYTE0
TAIL           = 153  # CONTROL_VALUES_BYTE7
PREFIX_CMD     = 3  # “continuous‐control” packet
PREFIX_SIMPLE  = 0x01  # “simple” packet (keep‐alive)
NEUTRAL        = 128
INTERVAL       = 0.05  # 50 ms

# network ports
LOCAL_PORT     = 35071  # source port for commands
DRONE_PORT     = 7099   # destination port on drone

# expected “alive” payload from drone
ALIVE_PAYLOAD  = bytes([0x48, 0x01, 0x00, 0x00, 0x00])

class DroneController:

    def __init__(self, ip: str, local_port=LOCAL_PORT, drone_port=DRONE_PORT, interval=INTERVAL):
        # Prepare UDP socket bound to LOCAL_PORT
        self.addr = (ip, drone_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', local_port))
        self.sock.settimeout(interval)

        self.interval=interval

        # joystick state (floats in [-1,1])
        self.x = 0.0  # right (+) / left (–)
        self.y = 0.0  # forward (+) / backward (–)

        # derived byte values (1…255)
        self.roll = self.pitch = self.THROTTLE = self.YAW = NEUTRAL


        self.accel = 0
        self.turn = 128


        self.is_fast_fly       = False
        self.is_fast_drop      = False
        self.is_emergency_stop = False
        self.is_circle_turn_end= False
        self.is_no_head_mode   = False
        self.is_fast_return    = False
        self.is_gyro_corr      = False

        # liveness
        self.alive = False

        # start background threads
        threading.Thread(target=self._sender_loop,   daemon=True).start()
        threading.Thread(target=self._receiver_loop, daemon=True).start()


    # Build the raw 8-byte payload
    def build_message(self) -> bytes:

        # modes (octet 5)
        modes = (
            self.is_fast_fly,
            self.is_fast_drop,
            self.is_emergency_stop,
            self.is_circle_turn_end,
            self.is_no_head_mode,
            self.is_fast_return,
            self.is_gyro_corr
        )
        flags1 = sum(1 << i for i, f in enumerate(modes) if f)

        pitch = 128
        roll = 128

        crc = (((int(pitch) ^ int(roll)) ^ int(self.accel)) ^ int(self.turn)) ^ (flags1 & 255)

        print("Myval: ")
        print(bytes([PREFIX_CMD, HEADER, pitch, roll, self.accel, self.turn, flags1, crc, TAIL]))

        return bytes([PREFIX_CMD, HEADER, pitch, roll, self.accel, self.turn, flags1, crc, TAIL])


    def build_keep_alive(self) -> bytes:
        return bytes([PREFIX_SIMPLE, PREFIX_SIMPLE])

    # Sender thread: send every INTERVAL
    def _sender_loop(self):
        while True:
            # update roll/pitch from joystick floats
            self.roll  = self._float_to_byte(self.x)
            self.pitch = self._float_to_byte(self.y)

            # send control or keep-alive
            pkt = self.build_message()
            print(pkt)
            self.sock.sendto(pkt, self.addr)
            time.sleep(self.interval)

    # Receiver thread: listen for ALIVE_PAYLOAD
    def _receiver_loop(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                self.alive = (data == ALIVE_PAYLOAD)
            except socket.timeout:
                self.alive = False

    # Action ponctuelles
    def takeoff(self):
        """Arm + décollage."""
        self.is_unlock = True
        self.sock.sendto(self.build_message(), self.addr)
        self.is_unlock = False

    def land(self):
        """Atterrissage / arrêt d’urgence."""
        self.is_emergency_stop = True
        self.sock.sendto(self.build_message(), self.addr)
        self.is_emergency_stop = False

    # Joystick helper
    def _float_to_byte(self, v: float) -> int:
        if not -1.0 <= v <= 1.0:
            raise ValueError("Value must be between -1 and 1")
        return int(v * 127) + 128

    def set_direction(self, x: float, y: float):
        """
        x: right (+1) / left (–1)
        y: forward (+1) / backward (–1)
        """
        self.x = x
        self.y = y




def drone_connect():
    """
    Connect to drone wifi network
    """
    cmd = "sudo nmcli con up 'drone'"
    args = shlex.split(cmd)

    try:
        subprocess.run(args, check=True)
    except subprocess.CalledProcessError as exc:
        print(f"Erreur connect to drone: {exc.returncode}")


def rtsp_ffplay(rtsp_url):
    """
    Show RTSP video stream
    """

    cmd = f"ffplay -fflags nobuffer -loglevel quiet -nostats -flags low_delay -fs -max_delay 0 -probesize 32 -analyzeduration 0 -framedrop -strict experimental \"{rtsp_url}\""
    args = shlex.split(cmd)

    try:
        subprocess.run(args, check=True)
    except subprocess.CalledProcessError as exc:
        print(f"FFplay error (code {exc.returncode}): Could not read video stream.")




if __name__ == "__main__":

    drone_connect()

    url = "rtsp://192.168.1.1:7070/webcam/track0"
    video_thread = threading.Thread(target=rtsp_ffplay, args=(url,))
    video_thread.start()

    pygame.init()

    pygame.joystick.init()


    controller = DroneController('192.168.1.1', 7099)

    time.sleep(2)

    controller.accel = 0
    time.sleep(1)
    controller.accel = 250
    time.sleep(10)
    
    
#    if pygame.joystick.get_count() == 0:
#        print("No gamepas detected")
#    else:
#        # Initialiser le premier joystick
#        joystick = pygame.joystick.Joystick(0)
#        joystick.init()
#    
#        print(f"Gamepad détecté : {joystick.get_name()}")
#    
#        # Boucle principale
#        running = True
#        while running:
#            for event in pygame.event.get():
#                if event.type == pygame.QUIT:
#                    running = False
#    
#                # Vérifier les événements de bouton
#                elif event.type == pygame.JOYBUTTONDOWN:
#                    print(f"Bouton {event.button} pressé")
#    
#                # Vérifier les événements d'axe
#                elif event.type == pygame.JOYAXISMOTION:
#                    print(f"Axe {event.axis} valeur {event.value}")
#    
#                # Vérifier les événements de chapeau (D-pad)
#                elif event.type == pygame.JOYHATMOTION:
#                    print(f"Chapeau {event.hat} valeur {event.value}")
#    
#        # Quitter Pygame
#        pygame.quit()

