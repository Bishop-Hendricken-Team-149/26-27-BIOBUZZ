"""
CLI client for HermesFusionBridge (http://127.0.0.1:18149).
Usage:
  python cad/fusion_cli.py status
  python cad/fusion_cli.py exec "print(root.name)"
  python cad/fusion_cli.py run-file path/to/script.py
  python cad/fusion_cli.py screenshot cad/tmp/fusion_view.png --fit
"""
import argparse
import ctypes
import json
import sys
import threading
import urllib.error
import urllib.request
from pathlib import Path

BASE_URL = "http://127.0.0.1:18149"


def _wake_fusion_message_loop(stop_event: threading.Event):
    user32 = ctypes.windll.user32
    WNDENUMPROC = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)
    hwnds = []

    def cb(hwnd, lparam):
        buf = ctypes.create_unicode_buffer(256)
        user32.GetWindowTextW(hwnd, buf, 256)
        if "Fusion" in buf.value or "Autodesk" in buf.value:
            hwnds.append(hwnd)
        return True

    user32.EnumWindows(WNDENUMPROC(cb), 0)
    while not stop_event.is_set():
        for h in hwnds:
            user32.PostMessageW(h, 0x0000, 0, 0)
        stop_event.wait(0.1)


def request_json(method: str, path: str, payload: dict | None = None, timeout: float = 300.0) -> dict:
    url = f"{BASE_URL}{path}"
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    headers = {"Content-Type": "application/json; charset=utf-8"} if data else {}
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    stop_ev = threading.Event()
    waker = threading.Thread(target=_wake_fusion_message_loop, args=(stop_ev,), daemon=True)
    waker.start()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        try:
            return json.loads(body)
        except Exception:
            return {"ok": False, "http_status": e.code, "error": body}
    except Exception as e:
        return {"ok": False, "error": str(e)}
    finally:
        stop_ev.set()


def main():
    parser = argparse.ArgumentParser(description="HermesFusionBridge CLI")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="Get active Fusion 360 document & component tree status")

    p_exec = sub.add_parser("exec", help="Execute inline Python inside Fusion 360 main thread")
    p_exec.add_argument("code", help="Python code to execute")
    p_exec.add_argument("--timeout", type=float, default=120.0, help="Main-thread timeout in seconds")

    p_file = sub.add_parser("run-file", help="Execute a Python file inside Fusion 360 main thread")
    p_file.add_argument("file", help="Path to Python script")
    p_file.add_argument("--timeout", type=float, default=120.0, help="Main-thread timeout in seconds")

    p_shot = sub.add_parser("screenshot", help="Save Fusion 360 active viewport image")
    p_shot.add_argument("path", help="Output PNG path")
    p_shot.add_argument("--width", type=int, default=1600)
    p_shot.add_argument("--height", type=int, default=1000)
    p_shot.add_argument("--fit", action="store_true")
    p_shot.add_argument("--eye", type=float, nargs=3, help="Camera eye X Y Z in mm")
    p_shot.add_argument("--target", type=float, nargs=3, help="Camera target X Y Z in mm")
    p_shot.add_argument("--up", type=float, nargs=3, default=[0, 0, 1], help="Camera up vector")

    args = parser.parse_args()

    if args.cmd == "status":
        res = request_json("GET", "/status")
        print(json.dumps(res, indent=2))
        sys.exit(0 if res.get("ok") else 1)

    elif args.cmd == "exec":
        res = request_json("POST", "/exec", {"code": args.code, "timeout": args.timeout}, timeout=args.timeout + 30.0)
        if res.get("stdout"):
            sys.stdout.write(res["stdout"])
        if res.get("stderr"):
            sys.stderr.write(res["stderr"])
        if not res.get("ok"):
            sys.stderr.write(f"ERROR: {res.get('error')}\n{res.get('traceback', '')}\n")
            sys.exit(1)
        if res.get("result") is not None:
            print(json.dumps(res["result"], indent=2, default=str))

    elif args.cmd == "run-file":
        code = Path(args.file).read_text(encoding="utf-8")
        res = request_json("POST", "/exec", {"code": code, "timeout": args.timeout}, timeout=args.timeout + 30.0)
        if res.get("stdout"):
            sys.stdout.write(res["stdout"])
        if res.get("stderr"):
            sys.stderr.write(res["stderr"])
        if not res.get("ok"):
            sys.stderr.write(f"ERROR: {res.get('error')}\n{res.get('traceback', '')}\n")
            sys.exit(1)
        if res.get("result") is not None:
            print(json.dumps(res["result"], indent=2, default=str))

    elif args.cmd == "screenshot":
        out_abs = str(Path(args.path).resolve())
        payload = {
            "path": out_abs,
            "width": args.width,
            "height": args.height,
            "fit": args.fit,
        }
        if args.eye and args.target:
            payload["eye_mm"] = list(args.eye)
            payload["target_mm"] = list(args.target)
            payload["up"] = list(args.up)
        res = request_json("POST", "/screenshot", payload)
        print(json.dumps(res, indent=2))
        sys.exit(0 if res.get("ok") else 1)


if __name__ == "__main__":
    main()
