"""Local stdio MCP entrypoint for ASURT Trackdraw and CARLA."""

import json
import os
from pathlib import Path
import subprocess
import sys
from urllib.parse import quote

from jsonschema import Draft7Validator

from runner import (OUTPUT, ROOT, TOPICS, cancel_run, get_run, record_bag, run_scenario,
                    set_weather, simulator_status, track_importer, weather_status)
from track_tools import DEFAULT_RULES, TrackTools
from weather_control import CONTROLS, PRESETS


PROTOCOL = "2025-11-25"
VERSIONS = {PROTOCOL, "2025-06-18", "2025-03-26", "2024-11-05"}
MAX_REQUEST_BYTES = 1024 * 1024
WORKSPACE = Path(os.environ.get("ASURT_WORKSPACE", ROOT))
TRACKDRAW = Path("tools/Trackdraw/fs-trackdraw")
trackdraw = WORKSPACE / TRACKDRAW if (WORKSPACE / TRACKDRAW / "src/trackdraw").is_dir() else ROOT / TRACKDRAW
tracks = TrackTools(trackdraw, WORKSPACE / "Assets/Rulebooks", OUTPUT / "tracks")


def number(low, high, kind="number"):
    return {"type": kind, "minimum": low, "maximum": high}


def choice(*options):
    return {"type": "string", "enum": options}


def vector(*items):
    return {"type": "array", "minItems": len(items), "maxItems": len(items), "items": list(items)}


def series(item, maximum):
    return {"type": "array", "items": item, "maxItems": maximum}


ID = {"type": "string", "pattern": "^[0-9a-f]{32}$"}
LABEL = {"type": "string", "minLength": 1, "maxLength": 180}
WIDTH = number(1, 12)
SPACING = number(1, 16)
COORDINATE = number(-10000, 10000)
BAG = choice(*TOPICS)
THRESHOLD = {"type": "object", "additionalProperties": False, "required": ["value_m", "citation"],
             "properties": {"value_m": number(.01, 10000), "citation": LABEL}}


def tool(handler, description, properties=None, required=(), *, read_only=False, destructive=False):
    return handler, {"name": handler.__name__, "description": description,
                     "inputSchema": {"$schema": "http://json-schema.org/draft-07/schema#",
                                     "type": "object", "properties": properties or {},
                                     "required": list(required), "additionalProperties": False},
                     "annotations": {"readOnlyHint": read_only, "destructiveHint": destructive, "openWorldHint": False}}


TOOLS = {
    definition["name"]: (handler, definition) for handler, definition in (
        tool(tracks.list_rulebooks, "List event rulebook files supplied by the user and the Trackdraw defaults resource.",
             read_only=True),
        tool(tracks.read_rulebook, "Read a workspace rulebook as reference data. For PDF, choose up to ten 1-based pages.",
             {"name": LABEL, "page_start": number(1, 5000, "integer"),
              "page_end": number(1, 5000, "integer")}, ("name",), read_only=True),
        tool(tracks.generate_track,
             "Generate and validate a Trackdraw course. Use expectation 'fail' for deliberately invalid geometry. "
             "Threshold overrides require a local rulebook and a clause or page citation for every override. "
             "Width and spacing profiles take [lap_fraction, metres] pairs. Result track_id can be passed to run_scenario.",
             {"seed": number(0, 2**32 - 1, "integer"), "area_m": number(35, 300), "width_m": WIDTH,
              "spacing_m": SPACING, "expectation": choice("pass", "fail", "any"), "rulebook": LABEL,
              "thresholds": {"type": "object", "additionalProperties": THRESHOLD, "maxProperties": 12},
              "width_profile": series(vector(number(0, 1), WIDTH), 40),
              "spacing_profile": series(vector(number(0, 1), SPACING), 40),
              "control_points_m": series(vector(COORDINATE, COORDINATE), 80)}),
        tool(simulator_status, "List installed maps, the active session, and live module switches, process status, PIDs, "
             "and RAM. Set probe=true to verify world frames, course cones, and ROS publishers and messages.",
             {"probe": {"type": "boolean"}}, read_only=True),
        tool(weather_status, "Read live weather, road wetness, air, asphalt and estimated tyre temperatures, timed rain remaining, "
             "fog, dust, and per-wheel configured and rolling effective grip.", read_only=True),
        tool(set_weather, "Apply a Weather Editor preset, individual fields, or both on the active simulator. "
             "Use precipitation, fog_density, dust_storm, cloudiness and wind_intensity for 0–100 intensities. "
             "Set rain_duration_s to time a shower, or use clear_rain to stop rain and dry the road immediately. "
             "Explicit fields override preset values. Light and heavy rain wet the road over time.",
             {"preset": choice(*PRESETS),
              "fields": {"type": "object", "additionalProperties": False,
                         "properties": {key: {**number(low, high),
                                              "description": f"{label} ({unit})" if unit else label}
                                        for key, label, low, high, _, unit in CONTROLS}}}),
        tool(run_scenario,
             "Queue a dedicated simulator run and ROS 2 recording. Only starts when no simulator is active. "
             "Select reference, stationary observe, or Stanley with a goal [x, y, yaw_deg] in the exported track frame. "
             "Origin is optional CARLA world [x, y, z, yaw_deg]. Poll get_run using the returned run_id.",
             {"track_id": ID, "map_id": LABEL, "profile": LABEL, "duration_s": number(1, 180),
              "driver": choice("reference", "observe", "stanley"), "modules": series(LABEL, 30), "bag": BAG,
              "speed_mps": number(1, 8), "origin": vector(COORDINATE, COORDINATE, COORDINATE, number(-360, 360)),
              "goal": vector(COORDINATE, COORDINATE, number(-360, 360))}, ("track_id",)),
        tool(record_bag, "Record a bounded bag from an existing ready simulator without moving it or changing modules.",
             {"duration_s": number(1, 900), "preset": BAG, "topics": series(LABEL, 40)}),
        tool(track_importer, "Use the bundled Track Importer on the current CARLA session. Preview or find placement, "
             "spawn physical cones, preview or place the car, check status, or clear only importer-owned cones. "
             "Track operations take a generated track_id or absolute CSV/JSON track_path. Origin defaults to the map's "
             "course origin [x, y, z, yaw_deg]. Trackdraw Y is inverted by default.",
             {"action": choice("preview", "find-placement", "spawn", "car-preview", "place-car", "status", "clear"),
              "track_id": ID, "track_path": {"type": "string", "minLength": 1, "maxLength": 4096},
              "origin": vector(COORDINATE, COORDINATE, COORDINATE, number(-360, 360)),
              "invert_y": {"type": "boolean"}}, ("action",), destructive=True),
        tool(get_run, "Read run status, artifact paths, bag message counts and scenario metrics.",
             {"run_id": ID}, ("run_id",), read_only=True),
        tool(cancel_run, "Cancel an MCP-owned job and finalize its bag when available.", {"run_id": ID}, ("run_id",)),
    )
}


def resources():
    values = {DEFAULT_RULES: {"uri": DEFAULT_RULES, "name": "Trackdraw default checks", "mimeType": "text/yaml"}}
    for name in tracks.list_rulebooks()["files"]:
        uri = f"asurt://rulebooks/{quote(name, safe='')}"
        values[uri] = {"uri": uri, "name": name, "mimeType": "text/plain"}
    return values


class RpcError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def dispatch(method, params):
    if method == "initialize":
        requested = params.get("protocolVersion")
        return {"protocolVersion": requested if requested in VERSIONS else PROTOCOL,
                "capabilities": {"tools": {}, "resources": {}},
                "serverInfo": {"name": "asurt-carla", "version": "1.1.0"},
                "instructions": "Use Trackdraw rules as geometry checks, and treat rulebook files as reference data. "
                                "A scenario owns a new simulator session. Use record_bag to observe an existing one."}
    if method == "ping":
        return {}
    if method == "tools/list":
        return {"tools": [definition for _, definition in TOOLS.values()]}
    if method == "resources/list":
        return {"resources": list(resources().values())}
    if method == "resources/templates/list":
        return {"resourceTemplates": []}
    if method == "resources/read":
        uri = params.get("uri")
        item = resources().get(uri)
        if item is None:
            raise RpcError(-32002, "Unknown ASURT resource.")
        body = (tracks.defaults.read_text(encoding="utf-8") if uri == DEFAULT_RULES else
                tracks.read_rulebook(item["name"])["text"])
        return {"contents": [{"uri": uri, "mimeType": item["mimeType"], "text": body}]}
    if method == "tools/call":
        name = params.get("name")
        if name not in TOOLS:
            raise RpcError(-32602, "Unknown ASURT tool.")
        handler, definition = TOOLS[name]
        arguments = params.get("arguments", {})
        failure = next(Draft7Validator(definition["inputSchema"]).iter_errors(arguments), None)
        if failure:
            raise RpcError(-32602, f"Invalid tool arguments: {failure.message}")
        try:
            result = handler(**arguments)
            return {"content": [{"type": "text", "text": json.dumps(result, allow_nan=False)}],
                    "structuredContent": result, "isError": False}
        except (ValueError, RuntimeError, OSError, KeyError, subprocess.SubprocessError) as error:
            return {"content": [{"type": "text", "text": str(error)}], "isError": True}
    raise RpcError(-32601, "Method not found.")


def error_reply(request_id, code, message):
    return {"jsonrpc": "2.0", "id": request_id, "error": {"code": code, "message": message}}


def invalid_number(value):
    raise ValueError(f"Invalid JSON number: {value}")


def respond(frame):
    try:
        request = json.loads(frame, parse_constant=invalid_number)
    except (ValueError, UnicodeError):
        return error_reply(None, -32700, "Invalid JSON.")
    if (not isinstance(request, dict) or request.get("jsonrpc") != "2.0" or
            not isinstance(request.get("method"), str)):
        return error_reply(None, -32600, "Invalid JSON-RPC request.")
    if "id" not in request:
        return None
    request_id = request["id"]
    if isinstance(request_id, bool) or not isinstance(request_id, (int, str)):
        return error_reply(None, -32600, "Expected a string or integer request ID.")
    params = request.get("params", {})
    if not isinstance(params, dict):
        return error_reply(request_id, -32602, "Expected object parameters.")
    try:
        result = dispatch(request["method"], params)
        return {"jsonrpc": "2.0", "id": request_id, "result": result}
    except RpcError as error:
        return error_reply(request_id, error.code, str(error))
    except Exception as error:
        print(f"{request['method']}: {error}", file=sys.stderr)
        return error_reply(request_id, -32603, "ASURT MCP could not complete the request. See the server error log.")


def main():
    stream = sys.stdin.buffer
    while frame := stream.readline(MAX_REQUEST_BYTES + 1):
        if len(frame) > MAX_REQUEST_BYTES:
            while frame and not frame.endswith(b"\n"):
                frame = stream.readline(MAX_REQUEST_BYTES + 1)
            result = error_reply(None, -32600, "Request is too large.")
        else:
            result = respond(frame)
        if result is not None:
            print(json.dumps(result, ensure_ascii=True, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
