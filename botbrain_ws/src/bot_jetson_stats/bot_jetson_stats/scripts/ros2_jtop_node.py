#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

import os
import subprocess
from std_srvs.srv import Trigger
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import jtop

from bot_jetson_stats_interfaces.srv import NVPModel, JetsonClocks, Fan

from bot_jetson_stats.utils import (
    other_status,
    board_status,
    disk_status,
    cpu_status,
    fan_status,
    gpu_status,
    ram_status,
    swap_status,
    power_status,
    temp_status,
    emc_status,
)


class JTOPPublisher(LifecycleNode):
    def __init__(self):
        super().__init__('jtop_publisher')

        self.declare_parameter('interval', 2) 
        self.declare_parameter('level_error', 60)
        self.declare_parameter('level_warning', 40)
        self.declare_parameter('level_ok', 20)

        self.declare_parameter('use_sysrq_fallback', False)

        self.diag_pub = self.create_lifecycle_publisher(
            DiagnosticArray, 'diagnostics', 1
        )
        self.human_pub = self.create_lifecycle_publisher(
            String, 'diagnostic_stats', 1
        )

        self.timer = None
        self.level_options = {}
        self.jetson = None
        self.hardware = None
        self.board_status_cached = None

        self.fan_srv = None
        self.nvpmodel_srv = None
        self.jetson_clocks_srv = None
        self.reboot_srv = None

        self.arr = DiagnosticArray()


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring JTOPLifecycleNode...")
        try:

            interval_param = self.get_parameter('interval')
            interval = float(interval_param.value)  # simple & robust

            level_error   = int(self.get_parameter('level_error').value)
            level_warning = int(self.get_parameter('level_warning').value)
            level_ok      = int(self.get_parameter('level_ok').value)

            self.level_options = {
                level_error: DiagnosticStatus.ERROR,
                level_warning: DiagnosticStatus.WARN,
                level_ok:     DiagnosticStatus.OK,
            }

            self.jetson = jtop.jtop(interval=0.5)
            self.jetson.start()

            board = self.jetson.board
            self.hardware = board["platform"]["Machine"]
            self.board_status_cached = board_status(self.hardware, board, 'board')

            self.fan_srv            = self.create_service(Fan, 'jtop/fan', self.fan_service)
            self.nvpmodel_srv       = self.create_service(NVPModel, 'jtop/nvpmodel', self.nvpmodel_service)
            self.jetson_clocks_srv  = self.create_service(JetsonClocks, 'jtop/jetson_clocks', self.jetson_clocks_service)
            self.reboot_srv         = self.create_service(Trigger, 'jtop/reboot', self.reboot_service)

            self.get_logger().info(f"Jetson Stats configured (interval={interval}s).")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"on_configure() failed: {e}")
            return TransitionCallbackReturn.FAILURE


    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating JTOPLifecycleNode...")
        try:
  
            ret = super().on_activate(state)
            if ret != TransitionCallbackReturn.SUCCESS:
                return ret

            interval = float(self.get_parameter('interval').value)
            self.timer = self.create_timer(interval, self.jetson_callback)
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Failed to activate JTOPLifecycleNode: {e}")
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating JTOPLifecycleNode...")
        if self.timer:
            self.timer.cancel()
            self.timer = None

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up JTOPLifecycleNode...")

        for srv in (self.fan_srv, self.nvpmodel_srv, self.jetson_clocks_srv, self.reboot_srv):
            if srv is not None:
                try:
                    self.destroy_service(srv)
                except Exception:
                    pass
        self.fan_srv = self.nvpmodel_srv = self.jetson_clocks_srv = self.reboot_srv = None

        if self.diag_pub is not None:
            try:
                self.destroy_publisher(self.diag_pub)
            except Exception:
                pass
            self.diag_pub = None

        if self.human_pub is not None:
            try:
                self.destroy_publisher(self.human_pub)
            except Exception:
                pass
            self.human_pub = None

        if self.jetson is not None:
            try:
                close_fn = getattr(self.jetson, 'close', None) or getattr(self.jetson, 'stop', None)
                if close_fn:
                    close_fn()
            except Exception:
                pass
            self.jetson = None

        self.hardware = None
        self.board_status_cached = None
        self.level_options = {}

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down JTOPLifecycleNode...")
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if self.jetson is not None:
            try:
                (getattr(self.jetson, 'close', None) or getattr(self.jetson, 'stop', None))()
            except Exception:
                pass
            self.jetson = None
        return TransitionCallbackReturn.SUCCESS

    def fan_service(self, req, resp):
        if self.jetson is None:
            self.get_logger().warn("fan_service called while node not configured.")
            return resp

        fan_mode = req.mode
        fan_speed = req.speed
        try:
            self.jetson.fan.profile = fan_mode
            self.jetson.fan.speed = fan_speed
        except jtop.JtopException:
            fan_mode = str(self.jetson.fan.profile)
            fan_speed = self.jetson.fan.speed

        while self.jetson.ok():
            if self.jetson.fan.speed == fan_speed:
                break

        resp.set_fan_mode = fan_mode
        resp.set_fan_speed = fan_speed
        self.get_logger().info(
            f"Fan request: mode={req.mode} speed={req.speed}; "
            f"applied mode={resp.set_fan_mode} speed={resp.set_fan_speed}"
        )
        return resp

    def jetson_clocks_service(self, req, resp):
        if self.jetson is None:
            self.get_logger().warn("jetson_clocks_service called while node not configured.")
            return resp
        self.jetson.jetson_clocks = req.status
        resp.done = req.status
        self.get_logger().info(f"Jetson clocks set to: {resp.done}")
        return resp

    def nvpmodel_service(self, req, resp):
        if self.jetson is None:
            self.get_logger().warn("nvpmodel_service called while node not configured.")
            return resp

        cur_nvpmodel = self.jetson.nvpmodel
        self.get_logger().info(f"NVPModel request: {req.nvpmodel}; current: {cur_nvpmodel}")
        nvpmodel = req.nvpmodel
        try:
            self.jetson.nvpmodel = nvpmodel
        except jtop.JtopException:
            nvpmodel = self.jetson.nvpmodel

        while self.jetson.ok():
            if self.jetson.nvpmodel.id == nvpmodel:
                break

        resp.power_mode = str(self.jetson.nvpmodel)
        return resp

    def reboot_service(self, req, resp):
        if self.jetson is None:
            self.get_logger().warn("reboot_service called while node not configured.")
            resp.success = False
            resp.message = "Node not configured"
            return resp

        self.get_logger().warn("Reboot requested via /jtop/reboot")

        def run(cmd, env=None):
            try:
                p = subprocess.run(
                    cmd, shell=True, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE
                )
                return p.returncode, p.stdout.decode(errors="ignore"), p.stderr.decode(errors="ignore")
            except Exception as e:
                return 127, "", str(e)

        # 1) Preferred: call login1 over the system bus (bypasses chroot heuristics)
        rc, out, err = run(
            "dbus-send --system --print-reply --dest=org.freedesktop.login1 "
            "/org/freedesktop/login1 org.freedesktop.login1.Manager.Reboot boolean:true"
        )
        if rc == 0:
            resp.success = True
            resp.message = "Reboot requested via D-Bus (login1)."
            return resp
        self.get_logger().error(f"D-Bus reboot failed (rc={rc}): {err or out}")

        # 2) Try systemctl with chroot override
        env = os.environ.copy()
        env["SYSTEMD_IGNORE_CHROOT"] = "1"
        rc, out, err = run("systemctl reboot", env=env)
        if rc == 0:
            resp.success = True
            resp.message = "Reboot requested via systemctl (IGNORE_CHROOT=1)."
            return resp
        self.get_logger().error(f"systemctl reboot failed (rc={rc}): {err or out}")

        # 3) Try loginctl (also talks to login1)
        rc, out, err = run("loginctl reboot")
        if rc == 0:
            resp.success = True
            resp.message = "Reboot requested via loginctl."
            return resp
        self.get_logger().error(f"loginctl reboot failed (rc={rc}): {err or out}")

        # 4) Optional last resort: kernel sysrq (immediate, unsafe for filesystems)
        if bool(self.get_parameter('use_sysrq_fallback').value):
            try:
                # ensure sysrq is enabled
                with open("/proc/sys/kernel/sysrq", "w") as f:
                    f.write("1")
                with open("/proc/sysrq-trigger", "w") as f:
                    f.write("b")
                resp.success = True
                resp.message = "Emergency reboot triggered via /proc/sysrq-trigger (unsafe)."
                return resp
            except Exception as e:
                self.get_logger().error(f"sysrq fallback failed: {e}")

        resp.success = False
        resp.message = ("Failed to invoke reboot via D-Bus/systemctl/loginctl. "
                        "Check: container has /run and /etc/machine-id mounted, pid: host, "
                        "polkit rule permits login1 reboot, and dbus tools are installed.")
        return resp


    def jetson_callback(self):

        self.arr.header.stamp = self.get_clock().now().to_msg()
        self.arr.status = [other_status(self.hardware, self.jetson, jtop.__version__)]
        self.arr.status += [cpu_status(self.hardware, name, cpu)
                            for name, cpu in enumerate(self.jetson.cpu['cpu'])]
        self.arr.status += [gpu_status(self.hardware, name, self.jetson.gpu[name])
                            for name in self.jetson.gpu]
        self.arr.status += [ram_status(self.hardware, self.jetson.memory['RAM'], 'mem')]
        self.arr.status += [swap_status(self.hardware, self.jetson.memory['SWAP'], 'mem')]
        self.arr.status += [emc_status(self.hardware, self.jetson.memory['EMC'], 'mem')]
        self.arr.status += [temp_status(self.hardware, self.jetson.temperature, self.level_options)]
        self.arr.status += [power_status(self.hardware, self.jetson.power)]
        if self.jetson.fan is not None:
            self.arr.status += [fan_status(self.hardware, key, value) for key, value in self.jetson.fan.items()]
        self.arr.status += [self.board_status_cached]
        self.arr.status += [disk_status(self.hardware, self.jetson.disk, 'board')]

        self.diag_pub.publish(self.arr)

        human_msg = String()
        human_msg.data = self.generate_human_readable_summary()
        self.human_pub.publish(human_msg)

    def generate_human_readable_summary(self) -> str:
        summary = []
        summary.append("\n" + "=" * 50)
        summary.append("\n=== Jetson Diagnostic Stats ===\n")
        for status in self.arr.status:
            if not status.values:
                continue
            summary.append(f"\n[{status.name}] {status.message}")
            for kv in status.values:
                summary.append(f" - {kv.key}: {kv.value}")
        summary.append("\n" + "=" * 50 + "\n")
        return "\n".join(summary)


def main(args=None):
    rclpy.init(args=args)
    node = JTOPPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()