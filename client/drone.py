import socket
import threading
import time
import pygame
import subprocess
import shlex



HEADER         = 102
TAIL           = 153
PREFIX_CMD     = 3
PREFIX_SIMPLE  = 0x01
NEUTRAL        = 128
INTERVAL       = 0.05  # 50 ms

LOCAL_PORT     = 35071
DRONE_PORT     = 7099

ALIVE_PAYLOAD  = bytes([0x48, 0x01, 0x00, 0x00, 0x00])

class DroneController:

    def __init__(self, ip: str, local_port=LOCAL_PORT, drone_port=DRONE_PORT, interval=INTERVAL):
        # Prepare UDP socket bound to LOCAL_PORT
        self.addr = (ip, drone_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', local_port))
        self.sock.settimeout(interval)

        self.interval=interval

        self.running = True

        # derived byte values (1…255)
        self.roll = NEUTRAL
        self.pitch = NEUTRAL


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

        crc = (((int(self.pitch) ^ int(self.roll)) ^ int(self.accel)) ^ int(self.turn)) ^ (flags1 & 255)

        return bytes([PREFIX_CMD, HEADER, self.pitch, self.roll, self.accel, self.turn, flags1, crc, TAIL])


    def build_keep_alive(self) -> bytes:
        return bytes([PREFIX_SIMPLE, PREFIX_SIMPLE])

    # Sender thread: send every INTERVAL
    def _sender_loop(self):
        try:
            while True:
                # send control or keep-alive
                pkt = self.build_message()
                self.sock.sendto(pkt, self.addr)
                time.sleep(self.interval)
        except Exception as e:
            self.running = False

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


def convert_float(value):
    if value < -1.0:
        value = -1.0
    if value > 1.0:
        value = 1.0

    converted_value = int((value + 1.0) * 127.5 + 0.5)
    return converted_value


if __name__ == "__main__":

    drone_connect()

    url = "rtsp://192.168.1.1:7070/webcam/track0"
    #video_thread = threading.Thread(target=rtsp_ffplay, args=(url,))
    #video_thread.start()

    pygame.init()

    pygame.joystick.init()


    controller = DroneController('192.168.1.1', 7099)

    time.sleep(2)

    
    if pygame.joystick.get_count() == 0:
        print("No gamepas detected")
    else:
        # Initialiser le premier joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    
        print(f"Gamepad détecté : {joystick.get_name()}")

        val = 128
    
        while controller.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
    
                # Vérifier les événements de bouton
                elif event.type == pygame.JOYBUTTONDOWN:
                    print(f"Bouton {event.button} pressé")
    
                # Vérifier les événements d'axe
                elif event.type == pygame.JOYAXISMOTION:
                    if event.axis == 0:
                        controller.pitch = convert_float(event.value)
                    if event.axis == 1:
                        controller.roll = convert_float(-1.0 * event.value)
                    if event.axis == 3:
                        controller.accel = convert_float(-1.0 * event.value)

    
                # Vérifier les événements de chapeau (D-pad)
                elif event.type == pygame.JOYHATMOTION:
                    if event.value == (0, 1):
                        if controller.accel < 255:
                            controller.accel = controller.accel + 1
                    if event.value == (0, -1):
                        if controller.accel > 1:
                            controller.accel = controller.accel - 1

                    if event.value == (-1, 0):
                        if controller.roll < 255:
                            controller.roll = controller.roll + 1
                    if event.value == (1, 0):
                        if controller.roll > 1:
                            controller.roll = controller.roll - 1

    
        # Quitter Pygame
        pygame.quit()

