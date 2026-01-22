#!/usr/bin/env python3

import os
import shlex
import subprocess
import time
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.timer import Timer

from bot_jetson_stats_interfaces.srv import GetAvailableNetworks, ConnectWifi, ForgetNetwork
from std_srvs.srv import SetBool, Trigger


ENV_BASE = {
    "LC_ALL": "C",
    "LANG": "C",
    "TERM": "dumb",
    "NM_CLI_NO_COLOR": "1",
}


def _run(cmd: str, timeout: float = 15.0) -> str:
    """Run a shell command safely and return stdout (merged with stderr)."""
    args = shlex.split(cmd)
    env = {**os.environ, **ENV_BASE}
    try:
        return subprocess.check_output(
            args, text=True, stderr=subprocess.STDOUT, env=env, timeout=timeout
        )
    except subprocess.CalledProcessError as e:
        return e.output or ""
    except Exception:
        return ""


def _nm_running() -> bool:
    return _run("nmcli -t -f RUNNING general").strip() == "running"


def _pick_iface() -> str:
    for line in _run("nmcli -t -f DEVICE,TYPE dev").splitlines():
        parts = line.split(":")
        if len(parts) >= 2 and parts[1] == "wifi":
            return parts[0]
    return ""


def _current_ip(iface: str) -> str:
    ip = _run(f"nmcli -g IP4.ADDRESS dev show {shlex.quote(iface)}").strip()
    if not ip:
        out = _run(f"ip -4 -o addr show dev {shlex.quote(iface)}").strip()
        if out:
            parts = out.split()
            ip = next((p for p in parts if "/" in p and p.count('.') == 3), "")
    return ip.split("/")[0] if "/" in ip else ip

def _device_state(iface: str) -> str:
    """
    Return a stable state string for iface: 'connected', 'disconnected', 'connecting', etc.
    Tries 'dev status' first (2 fields), then falls back to 'device show' numeric state.
    """

    line = _run("nmcli -t -f DEVICE,STATE dev status").strip()
    for l in line.splitlines():
        if l.startswith(f"{iface}:"):
            parts = l.split(":")
            if len(parts) >= 2:
                return parts[1].strip()


    line = _run(f"nmcli -t -f GENERAL.STATE device show {shlex.quote(iface)}").strip()
    if line:
        after_colon = line.split(":", 1)[-1].strip()  
        if "(" in after_colon and ")" in after_colon:
            human = after_colon.split("(", 1)[1].split(")", 1)[0].strip()
            if human:
                return human
        code = after_colon.split()[0]
        return "connected" if code == "100" else "disconnected" if code == "20" else "unknown"
    return ""


def _active_wifi_ssid() -> str:

    line = _run("nmcli -t -f active,ssid dev wifi").strip()
    for l in line.splitlines():
        if l.startswith("yes:"):
            return l.split(":", 1)[1]
    return ""


def _wifi_radio_state() -> str:

    return _run("nmcli radio wifi").strip().lower()


def _has_ip(iface: str) -> bool:
    return bool(_run(f"ip addr show {shlex.quote(iface)} | grep 'inet '").strip())


def _write_status_file(path: str, text: str) -> bool:
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(text)
        os.chmod(path, 0o644)
        return True
    except Exception:
        return False

def _is_saved_wifi(ssid: str) -> bool:
    """Retorna True se existir um perfil Wi-Fi salvo com esse nome (SSID)."""
    if not ssid:
        return False
    out = _run("nmcli -t -f NAME,TYPE connection show").strip()
    for l in out.splitlines():
        parts = l.split(":")
        if len(parts) >= 2 and parts[1].strip().lower() in ("802-11-wireless", "wifi"):
            if parts[0].strip() == ssid:
                return True
    return False


class WifiServicesLifecycle(LifecycleNode):

    def __init__(self):
        super().__init__("wifi_services")

        self.declare_parameters(
            "",
            [
                ("iface", ""),                
                ("default_timeout", 30.0),    
                ("force_rescan", False),     
                ("use_sudo", False),         
                ("status_file", ""),          
                ("watchdog_enabled", False),
                ("check_interval", 5.0),
                ("wifi_ssid", ""),           
                ("wifi_pass", ""),          
                ("wifi_iface", ""),         
                ("fourg_iface", ""),          
                ("robot_iface", ""),          
                ("robot_conn_name", ""),      
            ],
        )

        self._srv_available = None
        self._srv_connect = None
        self._srv_radio = None
        self._srv_saved = None
        self._srv_forget = None
        self._srv_check4g = None
        self._srv_update_status = None
        self._watchdog_timer: Optional[Timer] = None


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        if not _nm_running():
            self.get_logger().error(
                "NetworkManager not running or unreachable via D-Bus. nmcli calls will fail."
            )
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info("Configured wifi_services lifecycle node.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating: creating services /available_networks and /connect_wifi")
        self._srv_available = self.create_service(
            GetAvailableNetworks, "available_networks", self._handle_available
        )
        self._srv_connect = self.create_service(
            ConnectWifi, "connect_wifi", self._handle_connect
        )
        self._srv_radio = self.create_service(SetBool, "wifi_radio", self._handle_wifi_radio)
        self._srv_saved = self.create_service(Trigger, "saved_networks", self._handle_saved_networks)
        self._srv_forget = self.create_service(ForgetNetwork, "forget_network", self._handle_forget_network)
        self._srv_check4g = self.create_service(Trigger, "check_4g", self._handle_check_4g)
        self._srv_update_status = self.create_service(Trigger, "update_network_status", self._handle_update_status)

        if self.get_parameter("watchdog_enabled").get_parameter_value().bool_value:
            interval = self.get_parameter("check_interval").get_parameter_value().double_value
            if interval <= 0:
                interval = 5.0
            self._watchdog_timer = self.create_timer(interval, self._watchdog_tick)
            self.get_logger().info(f"Watchdog enabled, interval={interval}s")

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating: destroying services & timers")
        self._destroy_services()
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._destroy_services()
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._destroy_services()
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _destroy_services(self):
        for s in (
            self._srv_available, self._srv_connect,
            self._srv_radio, self._srv_saved, self._srv_forget,
            self._srv_check4g, self._srv_update_status
        ):
            if s:
                s.destroy()
        self._srv_available = self._srv_connect = None
        self._srv_radio = self._srv_saved = self._srv_forget = None
        self._srv_check4g = self._srv_update_status = None


    def _handle_available(self, request, response):
        iface = self._resolve_iface(request.interface)
        do_rescan = bool(getattr(request, "rescan", False)) or \
            self.get_parameter("force_rescan").get_parameter_value().bool_value

        if do_rescan:
            rescan_cmd = "nmcli dev wifi rescan"
            if iface:
                rescan_cmd += f" ifname {shlex.quote(iface)}"
            _ = self._nmcli(rescan_cmd)

        if iface:
            cmd = f"nmcli dev wifi list ifname {shlex.quote(iface)}"
        else:
            cmd = "nmcli dev wifi list"

        out = self._nmcli(cmd)

        if not out or out.startswith("Error:"):
            alt = f"nmcli dev wifi ifname {shlex.quote(iface)}" if iface else "nmcli dev wifi"
            maybe = self._nmcli(alt)
            if maybe and not maybe.startswith("Error:"):
                out = maybe

        response.table = (out or "").rstrip("\n")
        response.message = "ok" if out and not out.startswith("Error:") else (out.strip() or "empty or error")
        return response

    def _handle_connect(self, request: ConnectWifi.Request, response: ConnectWifi.Response):
        if not _nm_running():
            response.success = False
            response.message = "NetworkManager not running"
            return response

        iface = self._resolve_iface(request.interface)
        if not iface:
            response.success = False
            response.message = "No Wi-Fi interface found"
            return response

        auth = (request.auth_type or "").strip().lower()
        ssid = request.ssid
        bssid = (request.bssid or "").strip()
        timeout = (
            request.timeout_s
            if (request.timeout_s and request.timeout_s > 0)
            else self.get_parameter("default_timeout").get_parameter_value().double_value
        )
        deadline = time.time() + float(timeout)

        con_name_for_cleanup = None

        try:

            _ = self._nmcli("nmcli dev wifi rescan")

            if (not auth or auth in ('psk',)) and (not request.psk) and _is_saved_wifi(ssid):
                cmd = f"nmcli connection up {shlex.quote(ssid)}"
                if iface:
                    cmd += f" ifname {shlex.quote(iface)}"
                _ = self._nmcli(cmd, 30)

            if auth in ("", "open"):
                cmd = f"nmcli dev wifi connect {shlex.quote(ssid)} ifname {shlex.quote(iface)}"
                if bssid:
                    cmd += f" bssid {shlex.quote(bssid)}"
                _ = self._nmcli(cmd, 25)

            elif auth in ("psk", "wpa-psk", "wpa2-psk"):
                cmd = (
                    f"nmcli dev wifi connect {shlex.quote(ssid)} ifname {shlex.quote(iface)} "
                    f"password {shlex.quote(request.psk)}"
                )
                if bssid:
                    cmd += f" bssid {shlex.quote(bssid)}"
                _ = self._nmcli(cmd, 30)

            elif auth in ("wpa2-enterprise", "enterprise", "eap"):
                
                con_name_for_cleanup = f"ros2-wifi-{int(time.time())}"
                _ = self._nmcli(
                    f"nmcli connection add type wifi ifname {shlex.quote(iface)} "
                    f"con-name {shlex.quote(con_name_for_cleanup)} ssid {shlex.quote(ssid)}"
                )
                _ = self._nmcli(
                    f"nmcli connection modify {shlex.quote(con_name_for_cleanup)} "
                    f"wifi-sec.key-mgmt wpa-eap 802-1x.eap peap "
                    f"802-1x.identity {shlex.quote(request.identity)} "
                    f"802-1x.password {shlex.quote(request.password)} "
                    f"802-1x.phase2-auth mschapv2"
                )
                if bssid:
                    _ = self._nmcli(
                        f"nmcli connection modify {shlex.quote(con_name_for_cleanup)} wifi.bssid {shlex.quote(bssid)}"
                    )
                if request.ca_cert_path:
                    _ = self._nmcli(
                        f"nmcli connection modify {shlex.quote(con_name_for_cleanup)} "
                        f"802-1x.ca-cert {shlex.quote(request.ca_cert_path)}"
                    )
                _ = self._nmcli(
                    f"nmcli connection up {shlex.quote(con_name_for_cleanup)} ifname {shlex.quote(iface)}",
                    40,
                )
            else:
                response.success = False
                response.message = f"Unsupported auth_type: {auth}"
                return response

            
            while time.time() < deadline:
                state = _device_state(iface)
                ip = _current_ip(iface)
                if state.startswith("connected") and ip:
                    response.ip_address = ip
                    break
                time.sleep(0.5)
            else:
                response.success = False
                response.message = "Timeout waiting for connection"
                return response

            response.success = True
            response.message = "Connected"
            return response

        finally:
            if auth in ("wpa2-enterprise", "enterprise", "eap") and not getattr(
                request, "save_profile", False
            ):
                if con_name_for_cleanup:
                    _ = self._nmcli(f"nmcli connection delete {shlex.quote(con_name_for_cleanup)}")


    def _handle_wifi_radio(self, request: SetBool.Request, response: SetBool.Response):
        """
        Turn Wi-Fi radio on/off.
        True  -> nmcli radio wifi on
        False -> nmcli radio wifi off
        """
        cmd = "nmcli radio wifi on" if request.data else "nmcli radio wifi off"
        out = self._nmcli(cmd)
        state = _wifi_radio_state()
        ok = (state == ("enabled" if request.data else "disabled"))
        response.success = ok
        response.message = f"wifi radio: {state} ({'ok' if ok else 'mismatch'})"
        return response

    def _handle_saved_networks(self, request, response):
        """
        List saved Wi-Fi profiles (names). Returns newline-separated list in message.
        Wi-Fi TYPE is usually '802-11-wireless'; accept 'wifi' as well for safety.
        """
        out = _run("nmcli -t -f NAME,TYPE connection show").strip()
        names = []
        for l in out.splitlines():
            parts = l.split(":")
            if len(parts) >= 2:
                name, ctype = parts[0], parts[1].strip().lower()
                if ctype in ("802-11-wireless", "wifi"):  # accept both
                    names.append(name)
        response.success = True
        response.message = "\n".join(names) if names else "(none)"
        return response

    def _handle_forget_network(self, request: ForgetNetwork.Request, response: ForgetNetwork.Response):
        """
        Remove um perfil Wi-Fi salvo no NetworkManager.
        Exemplo: nmcli connection delete "BotBot"
        """
        ssid = (request.ssid or "").strip()
        if not ssid:
            response.success = False
            response.message = "SSID não fornecido"
            return response

        out = self._nmcli(f"nmcli connection delete {shlex.quote(ssid)}")
        if "deleted" in out.lower() or not out.strip():
            response.success = True
            response.message = f"Rede esquecida: {ssid}"
        else:
            response.success = False
            response.message = f"Falha ao remover {ssid}: {out.strip()}"
        return response

    def _handle_check_4g(self, request: Trigger.Request, response: Trigger.Response):
        """
        Check 4G USB-ethernet interface (fourg_iface param). Success true if interface has IPv4.
        """
        fourg = self.get_parameter("fourg_iface").get_parameter_value().string_value.strip()
        if not fourg:
            response.success = False
            response.message = "fourg_iface parameter not set"
            return response
        has = _has_ip(fourg)
        ip = _current_ip(fourg) if has else ""
        response.success = has
        response.message = f"{fourg} {'UP' if has else 'DOWN'} {ip}"
        return response

    def _handle_update_status(self, request: Trigger.Request, response: Trigger.Response):
        """
        Write current network status to status_file param:
          - If connected to Wi-Fi:  'YYYY-mm-dd HH:MM:SS - WiFi: <SSID>'
          - Else if 4G has IP:      '... - 4G: <IFACE>'
          - Else:                   '... - No network'
        """
        path = self.get_parameter("status_file").get_parameter_value().string_value.strip()
        if not path:
            response.success = False
            response.message = "status_file parameter not set"
            return response

        now = time.strftime("%Y-%m-%d %H:%M:%S")
        ssid = _active_wifi_ssid()
        if ssid:
            ok = _write_status_file(path, f"{now} - WiFi: {ssid}")
        else:
            fourg = self.get_parameter("fourg_iface").get_parameter_value().string_value.strip()
            if fourg and _has_ip(fourg):
                ok = _write_status_file(path, f"{now} - 4G: {fourg}")
            else:
                ok = _write_status_file(path, f"{now} - No network")

        response.success = ok
        response.message = f"updated: {path}" if ok else "failed to write status file"
        return response

    def _watchdog_tick(self):
        """
        Mimics your bash loop when enabled via parameters:
          - Ensure Wi-Fi radio is on.
          - Ensure robot ethernet (robot_iface) is connected (nmcli connection up robot_conn_name).
          - If already on target Wi-Fi -> write status file and return.
          - If SSID present -> attempt connect with wifi_pass.
          - Else if 4G has IP -> write status '4G'.
          - Else do nothing.
        """

        if _wifi_radio_state() == "disabled":
            self.get_logger().info("Watchdog: Wi-Fi radio OFF, turning ON...")
            _ = self._nmcli("nmcli radio wifi on")
            time.sleep(1.0)

        robot_iface = self.get_parameter("robot_iface").get_parameter_value().string_value.strip()
        robot_conn = self.get_parameter("robot_conn_name").get_parameter_value().string_value.strip()
        if robot_iface:
            state_line = _run("nmcli device status")
            if robot_iface in state_line and " connected " not in state_line.split(robot_iface, 1)[-1][:100]:
                if robot_conn:
                    self.get_logger().info(f"Watchdog: Bringing up robot Ethernet {robot_iface} via '{robot_conn}'")
                    _ = self._nmcli(f"nmcli connection up {shlex.quote(robot_conn)}")

    
        status_path = self.get_parameter("status_file").get_parameter_value().string_value.strip()

      
        target_ssid = self.get_parameter("wifi_ssid").get_parameter_value().string_value.strip()
        target_psk = self.get_parameter("wifi_pass").get_parameter_value().string_value
        iface_pref = self.get_parameter("wifi_iface").get_parameter_value().string_value.strip()
        iface = iface_pref or self._resolve_iface("")

        current = _active_wifi_ssid()
        if current and status_path:
            _write_status_file(status_path, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - WiFi: {current}")
            return

        # Try connect to target Wi-Fi if found
        if target_ssid and target_psk and iface:
            table = self._nmcli("nmcli -t -f SSID dev wifi list")
            if target_ssid in [l.strip() for l in table.splitlines()]:
                self.get_logger().info(f"Watchdog: attempting connect to {target_ssid} on {iface}")
                _ = self._nmcli(
                    f"nmcli dev wifi connect {shlex.quote(target_ssid)} ifname {shlex.quote(iface)} password {shlex.quote(target_psk)}",
                    30,
                )
                time.sleep(3.0)
                current = _active_wifi_ssid()

    
        if not current and status_path:
            fourg = self.get_parameter("fourg_iface").get_parameter_value().string_value.strip()
            if fourg and _has_ip(fourg):
                _write_status_file(status_path, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - 4G: {fourg}")
            else:
                _write_status_file(status_path, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - No network")


    def _nmcli(self, cmd: str, timeout: float = 15.0) -> str:
        if self.get_parameter("use_sudo").get_parameter_value().bool_value:
            cmd = "sudo -n " + cmd
        return _run(cmd, timeout=timeout)

    def _resolve_iface(self, req_iface: Optional[str]) -> str:
        """Resolve interface: request -> parameter -> auto-detect."""
        if req_iface and isinstance(req_iface, str) and req_iface.strip():
            return req_iface.strip()
        iface_param = self.get_parameter("iface")
        if iface_param.type_ == Parameter.Type.STRING:
            val = iface_param.get_parameter_value().string_value.strip()
            if val:
                return val
        return _pick_iface()


def main():
    rclpy.init()
    node = WifiServicesLifecycle()
    rclpy.spin(node) 
    rclpy.shutdown()


if __name__ == "__main__":
    main()