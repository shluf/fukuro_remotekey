#!/usr/bin/env python3
"""
Fukuro Remote Key Controller
Keyboard teleoperation for Fukuro omniwheel robot.

Controls:
  Movement (normal mode - rotation):
     u    i    o
     j    k    l
     m    ,    .

  Movement (holonomic mode - strafing, hold SHIFT):
     U    I    O
     J    K    L
     M    <    >

  Robot actions:
     d   : toggle dribbler ON/OFF
     f   : kick
     r   : toggle robot READY / STOP

  PWM control:
     t/g : increase/decrease dribbler PWM by 10
     y/h : increase/decrease kick power by 10

  Speed:
     q/z : increase/decrease max speeds by 10%
     w/x : increase/decrease linear speed by 10%
     e/c : increase/decrease angular speed by 10%

  SPACE       : stop movement immediately
  CTRL-C / ESC: quit
"""

import re
import sys
import select
import termios
import tty
import shutil
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from fukuro_interface.srv import DribblerControl, KickService, StopRobot

# ─────────────────────────────────────────────
# Key binding tables
# ─────────────────────────────────────────────

# Normal mode  →  (vx, vy, omega) direction multipliers
MOVE_BINDINGS = {
    'u': ( 1,  0,  1),
    'i': ( 1,  0,  0),
    'o': ( 1,  0, -1),
    'j': ( 0,  0,  1),
    'k': ( 0,  0,  0),
    'l': ( 0,  0, -1),
    'm': (-1,  0,  1),
    ',': (-1,  0,  0),
    '.': (-1,  0, -1),
}

# Holonomic / strafe mode (shift)  →  (vx, vy, omega)
HOLONOMIC_BINDINGS = {
    'U': ( 1,  1,  0),
    'I': ( 1,  0,  0),
    'O': ( 1, -1,  0),
    'J': ( 0,  1,  0),
    'K': ( 0,  0,  0),
    'L': ( 0, -1,  0),
    'M': (-1,  1,  0),
    '<': (-1,  0,  0),
    '>': (-1, -1,  0),
}

# Speed-adjustment keys  →  (linear_factor, angular_factor)
SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

# PWM step size for dribbler and kick power
PWM_STEP      = 10
PWM_MIN       = 0
PWM_MAX       = 255

# Strip ANSI escape codes to measure plain-text length
_ANSI_RE = re.compile(r'\033(?:\[[0-9;]*[mKJHABCDfnsu]|[78])')

# ─────────────────────────────────────────────
# Help banner
# ─────────────────────────────────────────────
BANNER = """
\033[1;36m╔══════════════════════════════════════════════════╗
║      Fukuro OmniWheel Remote Key Controller      ║
╚══════════════════════════════════════════════════╝\033[0m

\033[1mMoving around (normal mode):\033[0m
   \033[1;33mu\033[0m    \033[1;33mi\033[0m    \033[1;33mo\033[0m      ↖ forward ↗  / rotate left·right
   \033[1;33mj\033[0m    \033[1;33mk\033[0m    \033[1;33ml\033[0m      rotate ←  stop  rotate →
   \033[1;33mm\033[0m    \033[1;33m,\033[0m    \033[1;33m.\033[0m      ↙ backward ↘

\033[1mHolonomic strafe mode (hold SHIFT):\033[0m
   \033[1;33mU\033[0m    \033[1;33mI\033[0m    \033[1;33mO\033[0m      ↖ forward ↗  / strafe left·right
   \033[1;33mJ\033[0m    \033[1;33mK\033[0m    \033[1;33mL\033[0m      strafe ←  stop  strafe →
   \033[1;33mM\033[0m    \033[1;33m<\033[0m    \033[1;33m>\033[0m      ↙ backward ↘

\033[1mRobot actions:\033[0m
   \033[1;32md\033[0m   : toggle Dribbler ON/OFF
   \033[1;32mf\033[0m   : Kick!
   \033[1;32mr\033[0m   : toggle robot Ready / Stop

\033[1mPWM control:\033[0m
   \033[1;35mt\033[0m / \033[1;35mg\033[0m : increase / decrease dribbler PWM by 10  (0–255)
   \033[1;35my\033[0m / \033[1;35mh\033[0m : increase / decrease kick power by 10     (0–255)

\033[1mSpeed control:\033[0m
   \033[1;34mq\033[0m / \033[1;34mz\033[0m : increase / decrease max speeds by 10%%
   \033[1;34mw\033[0m / \033[1;34mx\033[0m : increase / decrease linear speed by 10%%
   \033[1;34me\033[0m / \033[1;34mc\033[0m : increase / decrease angular speed by 10%%

   \033[1;31mSPACE\033[0m : stop movement immediately
   \033[1;31mCTRL-C / ESC\033[0m : quit

\033[90m────────────────────────────────────────────────────\033[0m
"""


# ─────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────

class FukuroRemoteKey(Node):
    """Keyboard teleoperation node for the Fukuro omniwheel robot."""

    def __init__(self):
        super().__init__('fukuro_remotekey')

        # ── Parameters ───────────────────────────────
        self.declare_parameter('linear_speed', 0.5)    # m/s
        self.declare_parameter('angular_speed', 1.0)   # rad/s
        self.declare_parameter('dribbler_pwm', 200)    # PWM value for dribbler
        self.declare_parameter('kick_power', 200)        # kick power
        self.declare_parameter('servo_pos', 0.0)        # servo position for kick

        self.speed = self.get_parameter('linear_speed').value
        self.turn  = self.get_parameter('angular_speed').value
        self.dribbler_pwm = self.get_parameter('dribbler_pwm').value
        self.kick_power   = self.get_parameter('kick_power').value
        self.servo_pos    = self.get_parameter('servo_pos').value

        # ── State ─────────────────────────────────────
        self.vx              = 0.0
        self.vy              = 0.0
        self.omega           = 0.0
        self.dribbler_active    = False
        self.robot_ready        = True   # True = ready, False = stopped
        self.running            = True
        self._print_lock        = threading.RLock()  # mencegah log duplikat antar thread
        self._last_status_lines = 0

        # ── Publisher ─────────────────────────────────
        # Publish langsung ke topic OmnidirectionalController (use_stamped_vel: false)
        self.cmd_vel_pub = self.create_publisher(Twist, '/omnidirectional_controller/cmd_vel_unstamped', 10)

        # ── Service clients ───────────────────────────
        self.dribbler_client = self.create_client(
            DribblerControl, '/fukuro/controller/dribbler_control')
        self.kick_client = self.create_client(
            KickService, '/fukuro/controller/kick_service')
        self.stop_client = self.create_client(
            StopRobot, '/fukuro/controller/stop_service')

        # ── Timer: publish velocity at 10 Hz ──────────
        self.timer = self.create_timer(0.1, self._publish_cmd_vel)

        # ── Print banner ────────────
        print(BANNER)
        self._print_status()

        # ── Keyboard thread ────────────
        self._key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._key_thread.start()

    # ─── Velocity publisher ───────────────────────────────────────────────────

    def _publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x  = float(self.vx)
        msg.linear.y  = float(self.vy)
        msg.angular.z = float(self.omega)
        self.cmd_vel_pub.publish(msg)

    # ─── Service calls ────────────────────────────────────────────────────────

    def _call_dribbler(self, activate: bool):
        if not self.dribbler_client.service_is_ready():
            self.get_logger().warn('⚠  Dribbler service not ready – skipping')
            return
        req = DribblerControl.Request()
        req.dribbler_pwm = self.dribbler_pwm if activate else 0
        req.is_active    = True
        future = self.dribbler_client.call_async(req)
        future.add_done_callback(self._dribbler_cb)

    def _dribbler_cb(self, future):
        try:
            result = future.result()
            state  = '\033[1;32mON\033[0m' if self.dribbler_active else '\033[1;31mOFF\033[0m'
            with self._print_lock:
                self._erase_status()
                print(f'  Dribbler → {state}  (success={result.success})')
                self._print_status()
        except Exception as exc:
            self.get_logger().error(f'Dribbler service error: {exc}')

    def _call_kick(self):
        if not self.kick_client.service_is_ready():
            self.get_logger().warn('⚠  Kick service not ready – skipping')
            return
        req = KickService.Request()
        req.servo_pos  = float(self.servo_pos)
        req.kick_power = int(self.kick_power)
        req.is_kick    = True
        future = self.kick_client.call_async(req)
        future.add_done_callback(self._kick_cb)

    def _kick_cb(self, future):
        try:
            result = future.result()
            with self._print_lock:
                self._erase_status()
                print(f'  \033[1;33mKICK!\033[0m  power={self.kick_power}  done={result.kick_done}')
                self._print_status()
        except Exception as exc:
            self.get_logger().error(f'Kick service error: {exc}')

    def _call_stop_robot(self, is_stopped: bool):
        if not self.stop_client.service_is_ready():
            self.get_logger().warn('⚠  StopRobot service not ready – skipping')
            return
        req = StopRobot.Request()
        req.is_stop = is_stopped  # field sesuai StopRobot.srv
        future = self.stop_client.call_async(req)
        future.add_done_callback(self._stop_robot_cb)

    def _stop_robot_cb(self, future):
        try:
            result = future.result()
            state  = '\033[1;32mREADY\033[0m' if self.robot_ready else '\033[1;31mSTOPPED\033[0m'
            with self._print_lock:
                self._erase_status()
                print(f'  Robot → {state}  (success={result.success})')
                self._print_status()
        except Exception as exc:
            self.get_logger().error(f'StopRobot service error: {exc}')
    # ─── Keyboard loop ────────────────────────────────────────────────────────

    def _get_key(self, settings):
        """Read one key press with a 0.1-second timeout."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def _keyboard_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while self.running and rclpy.ok():
                key = self._get_key(settings)
                if key:
                    try:
                        self._handle_key(key)
                    except Exception as exc:
                        self.get_logger().error(f'Key handler error: {exc}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def _handle_key(self, key: str):
        # ── Movement (normal) ──────────────────────────
        if key in MOVE_BINDINGS:
            x, y, th = MOVE_BINDINGS[key]
            self.vx    = x  * self.speed
            self.vy    = y  * self.speed
            self.omega = th * self.turn
            self._print_status()

        # ── Movement (holonomic / strafe) ──────────────
        elif key in HOLONOMIC_BINDINGS:
            x, y, th = HOLONOMIC_BINDINGS[key]
            self.vx    = x  * self.speed
            self.vy    = y  * self.speed
            self.omega = th * self.turn
            self._print_status()

        # ── Speed adjustment ───────────────────────────
        elif key in SPEED_BINDINGS:
            lin_f, ang_f = SPEED_BINDINGS[key]
            self.speed = round(self.speed * lin_f, 4)
            self.turn  = round(self.turn  * ang_f, 4)
            self._print_status()

        # ── Dribbler toggle ────────────────────────────
        elif key == 'd':
            self.dribbler_active = not self.dribbler_active
            self._call_dribbler(self.dribbler_active)

        # ── Dribbler PWM adjust ────────────────────────
        elif key == 't':
            self.dribbler_pwm = min(PWM_MAX, self.dribbler_pwm + PWM_STEP)
            self._print_status()
        elif key == 'g':
            self.dribbler_pwm = max(PWM_MIN, self.dribbler_pwm - PWM_STEP)
            self._print_status()

        # ── Kick ──────────────────────────────────────
        elif key == 'f':
            self._call_kick()

        # ── Kick power adjust ─────────────────────────
        elif key == 'y':
            self.kick_power = min(PWM_MAX, self.kick_power + PWM_STEP)
            self._print_status()
        elif key == 'h':
            self.kick_power = max(PWM_MIN, self.kick_power - PWM_STEP)
            self._print_status()

        # ── Robot ready / stop toggle ──────────────────
        elif key == 'r':
            self.robot_ready = not self.robot_ready
            # is_stop = True  saat robot_ready = False, dan sebaliknya
            self._call_stop_robot(not self.robot_ready)

        # ── Emergency stop ─────────────────────────────
        elif key == ' ':
            self.vx = self.vy = self.omega = 0.0
            with self._print_lock:
                self._erase_status()
                print('  \033[1;31m⛔ EMERGENCY STOP\033[0m')
                self._print_status()

        # ── Quit ───────────────────────────────────────
        elif key in ('\x03', '\x1b'):  # CTRL-C or ESC
            with self._print_lock:
                self._erase_status()
                print('\033[1;90mBye!\033[0m')
            self.running = False
            self.vx = self.vy = self.omega = 0.0
            self._publish_cmd_vel()   # send zero before quitting
            rclpy.shutdown()

        # ── Any other key → stop movement ─────────────
        else:
            self.vx = self.vy = self.omega = 0.0
            self._print_status()

    # ─── Status line ──────────────────────────────────────────────────────────

    def _erase_status(self):
        if self._last_status_lines > 0:
            up = '\033[A' * (self._last_status_lines - 1)
            sys.stdout.write(up + '\r\033[J')
            sys.stdout.flush()
            self._last_status_lines = 0

    def _print_status(self):
        drib  = '\033[1;32mON\033[0m ' if self.dribbler_active else '\033[1;31mOFF\033[0m'
        ready = '\033[1;32mREADY  \033[0m' if self.robot_ready else '\033[1;31mSTOPPED\033[0m'
        status = (
            f'  spd \033[1m{self.speed:.2f}\033[0m m/s  '
            f'turn \033[1m{self.turn:.2f}\033[0m rad/s  │  '
            f'drbl: {drib} pwm=\033[1;35m{self.dribbler_pwm}\033[0m  │  '
            f'kick pwr=\033[1;35m{self.kick_power}\033[0m  │  '
            f'robot: {ready}  │  '
            f'vel: ({self.vx:+.2f}, {self.vy:+.2f}, {self.omega:+.2f})'
        )
        with self._print_lock:
            term_cols = shutil.get_terminal_size().columns
            plain_len = len(_ANSI_RE.sub('', status))
            new_lines = max(1, (plain_len + term_cols - 1) // term_cols)
            self._erase_status()
            sys.stdout.write(status)
            sys.stdout.flush()
            self._last_status_lines = new_lines


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FukuroRemoteKey()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.running = False
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
