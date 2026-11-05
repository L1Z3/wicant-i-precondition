#!/usr/bin/env python3
"""Follow the WiCAN RAM log over HTTP.

The device serves GET /logs as an endless chunked text stream: the body starts
at the oldest byte it still holds (or at ?after=<position>), and the X-Log-Start
header gives that position. Positions count bytes from 0 at boot, so a client
that counts the bytes it prints can resume exactly where a dropped connection
stopped by asking for ?after=<position> again.

Nothing is written to flash: the log is a RAM ring and old bytes are overwritten
as it fills. When that happens (or the device reboots) the stream jumps forward;
the client says so on stderr and continues.

Examples:
    tools/wican_logs.py                       # follow 192.168.0.248:80
    tools/wican_logs.py 192.168.0.223 --once  # dump what is buffered and exit
    tools/wican_logs.py 192.168.0.248 --test  # emit a test line, then follow
"""

import argparse
import http.client
import os
import socket
import sys
import time


def notice(msg):
    print(msg, file=sys.stderr, flush=True)


def follow(host, port, start, once, timeout, quiet):
    """Stream until --once finishes or the user interrupts; reconnect on errors."""
    pos = start
    conn = None
    while True:
        try:
            conn = http.client.HTTPConnection(host, port, timeout=timeout)
            path = "/logs?once=1" if once else "/logs"
            if pos is not None:
                path += ("&" if "?" in path else "?") + "after=%d" % pos
            conn.request("GET", path)
            resp = conn.getresponse()
            if resp.status != 200:
                notice("device answered HTTP %d: %s" % (resp.status, resp.reason))
                if once:
                    return 1
                time.sleep(2.0)
                continue

            header = resp.getheader("X-Log-Start")
            try:
                server_pos = int(header) if header is not None else None
            except ValueError:
                server_pos = None
            if pos is None:
                pos = server_pos if server_pos is not None else 0
            elif server_pos is not None and server_pos != pos:
                notice("log moved from %d to %d (ring overwrote bytes or the "
                       "device rebooted)" % (pos, server_pos))
                pos = server_pos

            dropped = resp.getheader("X-Log-Dropped")
            if not quiet:
                notice("following %s:%d from position %d%s"
                       % (host, port, pos,
                          " (%s bytes already dropped)" % dropped if dropped else ""))

            while True:
                data = resp.read1(4096)
                if not data:
                    break
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
                pos += len(data)

            if once:
                return 0
            notice("connection closed by the device; reconnecting")
        except KeyboardInterrupt:
            return 0
        except (http.client.HTTPException, OSError, socket.timeout) as err:
            if once:
                notice("connection failed: %s" % err)
                return 1
            notice("connection lost (%s); reconnecting" % err)
        finally:
            if conn is not None:
                conn.close()

        if not once:
            time.sleep(1.0)


def send_test(host, port, level="warn"):
    try:
        conn = http.client.HTTPConnection(host, port, timeout=5.0)
        conn.request("POST", "/test_log?level=" + level)
        resp = conn.getresponse()
        resp.read()
        conn.close()
        return resp.status == 200
    except (http.client.HTTPException, OSError) as err:
        notice("test line failed: %s" % err)
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", nargs="?",
                    default=os.environ.get("WICAN_HOST", "192.168.0.248"),
                    help="device address (default: $WICAN_HOST or 192.168.0.248)")
    ap.add_argument("-p", "--port", type=int, default=80, help="HTTP port (default 80)")
    ap.add_argument("--once", action="store_true",
                    help="print what is buffered now and exit instead of following")
    ap.add_argument("--after", type=int, default=None,
                    help="start at this absolute log position")
    ap.add_argument("--timeout", type=float, default=70.0,
                    help="socket timeout in seconds (default 70; the device sends "
                         "a keep-alive newline every 15 s)")
    ap.add_argument("--test", action="store_true",
                    help="emit one test warning line before following")
    ap.add_argument("-q", "--quiet", action="store_true", help="no progress notices")
    args = ap.parse_args()

    if args.test:
        send_test(args.host, args.port)

    try:
        return follow(args.host, args.port, args.after, args.once,
                      args.timeout, args.quiet)
    except BrokenPipeError:
        # Piped into head/less and it went away: not an error.
        try:
            sys.stdout.close()
        except OSError:
            pass
        return 0


if __name__ == "__main__":
    sys.exit(main())
