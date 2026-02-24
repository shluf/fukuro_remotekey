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

  Speed:
     q/z : increase/decrease max speeds by 10%
     w/x : increase/decrease linear speed by 10%
     e/c : increase/decrease angular speed by 10%

  SPACE       : stop movement immediately
  CTRL-C / ESC: quit
"""

import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from fukuro_interface.srv import DribblerControl, KickService, SetReady, StopRobot

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
        self.dribbler_active = False
        self.robot_ready     = True   # True = ready, False = stopped
        self.running         = True

        # ── Publisher ─────────────────────────────────
        # Default topic is 'cmd_vel'; remap in launch / CLI as needed.
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ── Service clients ───────────────────────────
        self.dribbler_client = self.create_client(
            DribblerControl, '/fukuro/controller/dribbler_control')
        self.kick_client = self.create_client(
            KickService, '/fukuro/controller/kick_service')
        self.ready_client = self.create_client(
            StopRobot, '/fukuro/controller/stop_service')

        # ── Timer: publish velocity at 10 Hz ──────────
        self.timer = self.create_timer(0.1, self._publish_cmd_vel)

        # ── Keyboard thread ───────────────────────────
        self._key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._key_thread.start()

        print(BANNER)
        self._print_status()

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
            print(f'\n  Dribbler → {state}  (success={result.success})')
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
            print(f'\n  \033[1;33mKICK!\033[0m  power={self.kick_power}  done={result.kick_done}')
            self._print_status()
        except Exception as exc:
            self.get_logger().error(f'Kick service error: {exc}')

    def _call_set_ready(self, is_ready: bool):
        if not self.ready_client.service_is_ready():
            self.get_logger().warn('⚠  SetReady service not ready – skipping')
            return
        req = SetReady.Request()
        req.is_ready = is_ready
        future = self.ready_client.call_async(req)
        future.add_done_callback(self._set_ready_cb)

    def _set_ready_cb(self, future):
        try:
            result = future.result()
            state  = '\033[1;32mREADY\033[0m' if self.robot_ready else '\033[1;31mSTOPPED\033[0m'
            print(f'\n  Robot → {state}  (success={result.success})')
            self._print_status()
        except Exception as exc:
            self.get_logger().error(f'SetReady service error: {exc}')

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
                    self._handle_key(key)
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
            print()
            self._print_status()

        # ── Dribbler toggle ────────────────────────────
        elif key == 'd':
            self.dribbler_active = not self.dribbler_active
            self._call_dribbler(self.dribbler_active)

        # ── Kick ──────────────────────────────────────
        elif key == 'f':
            self._call_kick()

        # ── Robot ready / stop toggle ──────────────────
        elif key == 'r':
            self.robot_ready = not self.robot_ready
            self._call_set_ready(self.robot_ready)

        # ── Emergency stop ─────────────────────────────
        elif key == ' ':
            self.vx = self.vy = self.omega = 0.0
            print('\n  \033[1;31m⛔ EMERGENCY STOP\033[0m')
            self._print_status()

        # ── Quit ───────────────────────────────────────
        elif key in ('\x03', '\x1b'):  # CTRL-C or ESC
            print('\n\033[1;90mBye!\033[0m')
            self.running = False
            self.vx = self.vy = self.omega = 0.0
            self._publish_cmd_vel()   # send zero before quitting
            rclpy.shutdown()

        # ── Any other key → stop movement ─────────────
        else:
            self.vx = self.vy = self.omega = 0.0
            self._print_status()

    # ─── Status line ──────────────────────────────────────────────────────────

    def _print_status(self):
        drib  = '\033[1;32mON\033[0m ' if self.dribbler_active else '\033[1;31mOFF\033[0m'
        ready = '\033[1;32mREADY  \033[0m' if self.robot_ready else '\033[1;31mSTOPPED\033[0m'
        print(
            f'\r\033[K'
            f'  speed \033[1m{self.speed:.2f}\033[0m m/s  '
            f'turn \033[1m{self.turn:.2f}\033[0m rad/s  │  '
            f'dribbler: {drib}  │  '
            f'robot: {ready}  │  '
            f'vel: ({self.vx:+.2f}, {self.vy:+.2f}, {self.omega:+.2f})',
            end='', flush=True,
        )


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
