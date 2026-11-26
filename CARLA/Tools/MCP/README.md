# ASURT CARLA and Trackdraw MCP

The local MCP server lets an agent read event rulebooks, generate and check
Trackdraw courses, start CARLA scenarios, and record ROS 2 bags. It runs in
Ubuntu 22.04 or in the Ubuntu WSL distro installed with ASURT on Windows.

## Connect an agent

Update ASURT first. The server launches on demand through `asurt mcp` and
communicates over standard input and output. The client discovers its tools
after connecting. The server does not listen on a network port.

Run one of these commands where your Codex client runs.

**Ubuntu or a client running inside WSL**

```bash
codex mcp add asurt-carla -- "$HOME/.local/share/asurt/current/bin/asurt" mcp
codex mcp list
```

**Windows PowerShell**

```powershell
codex mcp add asurt-carla -- wsl.exe -d Ubuntu-22.04 -- bash -lc 'exec "$HOME/.local/share/asurt/current/bin/asurt" mcp'
codex mcp list
```

On Windows, choose the same distro you used to install ASURT. If its name is
different, use `wsl --list --quiet` to find it and change the `-d` argument.
Open a new Codex chat after adding the server. [Codex MCP documentation](https://developers.openai.com/codex/mcp)

For another local MCP client, configure a **stdio** server with the same
command and arguments. A Windows client that uses `mcpServers` JSON can use
this example.

```json
{
  "mcpServers": {
    "asurt-carla": {
      "command": "wsl.exe",
      "args": ["-d", "Ubuntu-22.04", "--", "bash", "-lc", "exec \"$HOME/.local/share/asurt/current/bin/asurt\" mcp"]
    }
  }
}
```

## Make a rule-based test

1. Copy the event rulebook to `Assets/Rulebooks` in the **ASURT Workspace**.
   Create the folder if needed. PDF, Markdown, text, YAML and JSON files are
   supported. The data stays in your workspace. `list_rulebooks` returns its
   location and `read_rulebook` can read selected PDF pages.
2. Have the agent call `generate_track` with the event's values and a page or
   clause citation for each threshold it overrides. The built-in Trackdraw
   geometry checks are available as the `asurt://rules/trackdraw` resource.
   The report records the rulebook hash and the cited values. Event rules that
   have no matching Trackdraw check still need review.
3. Use `expectation: "pass"` for a course that passes the configured geometry
   checks, or `expectation: "fail"` for a deliberate violation. Check the
   returned `findings` for the specific rule the case is meant to exercise.
   Use the same seed to compare layouts and vary one input at a time. For a
   local pinch, supply a `width_profile` made of `[lap_fraction, metres]`
   pairs. The finish reconnects to the starting width. The tool exports cone
   CSV and JSON files plus a project you can edit in Trackdraw.
4. Call `run_scenario` with the resulting `track_id` and poll `get_run` with the
   returned `run_id`. The simulator must be idle before a dedicated scenario
   starts. `simulator_status` lists installed maps, profiles, and module IDs.
   Start with FlatAsphalt. A Silverstone case may need a custom `origin` of
   `[x, y, z, yaw_degrees]` so the whole course fits the drivable area.

The default `reference` driver follows the generated centerline at a bounded
speed. It is useful for collecting sensor data and exercising perception and
planning. For a closed-loop Stanley test, choose `driver: "stanley"` and set
`goal: [x, y, yaw_degrees]` in the **exported track frame**. The runner starts
the track-goal planner and Stanley controller. Use `driver: "observe"` to
leave the car parked. A completed job can end because its duration expired.
Check `metrics.stop_reason` and `metrics.lap_completed` for the driving result.

Dedicated scenarios use their own ROS domain and module switches. They use
your saved **Local** builds when you request those module IDs. Omit `modules`
to start with FakeMAP. For camera or LiDAR runs, include `zed2i` or `velodyne`
and select the corresponding bag preset. The tools never require a simulator
recook.

### Example agent prompt

> Use the ASURT MCP tools to make a FlatAsphalt track with seed 7 and a
> 3.30 m width, then make a 2.95 m variant to test minimum track width.
> Confirm the second case flags the intended rule. Run the baseline for 15
> seconds with the reference driver, FakeMAP, and the planning bag preset.
> Give me the run report, bag path, and any missing topics.

## Record and use ROS bags

For an already running simulator, call `record_bag` with `duration_s` and one
of `planning`, `camera`, `lidar`, or `training`. You can also pass a list of
explicit `topics`. The recorder includes `/clock` and keeps the current
simulator and module settings. Enable the publishers you need before
recording. `get_run` reports message counts and missing topics.

MCP binds scenarios and recordings to the active installer-managed runtime.
Run reports include its release, runtime path, and archive SHA-256. If a live
track was placed with the old static importer, clear it and respawn it through
`track_importer` before starting a new bag. Existing bags stay on disk.

## Import a track into a running simulator

Start FlatAsphalt or Silverstone Split, then call `track_importer` with an
`action`. Use `preview`, `find-placement`, `spawn`, `car-preview`, or
`place-car` with either a generated `track_id` or an absolute local CSV or JSON
`track_path`. The default placement is the map's course origin. Supply
`origin` as `[x, y, z, yaw_degrees]` in CARLA coordinates to change it. Leave
`invert_y` enabled for Trackdraw exports and turn it off if the file already
uses CARLA's Y direction. Use `status` and `clear` without a track argument.
Clear uses the importer's actor ownership checks. The MCP tool runs the
bundled importer from the active installation.

Generated tracks and runs are written under
`~/.local/share/asurt/state/mcp/`. Dedicated scenarios write a copy of the
track, `job.json` with the selected module entrypoint hashes, `report.json`, simulator logs, a ROS bag, and
`ground-truth.jsonl`. The ground truth contains sampled CARLA frames, elapsed
simulation time, and the car and cone poses. The `training` preset samples
every received simulator tick. Poses are `[x, y, z, roll, pitch, yaw]` with
rotation in degrees. These poses use CARLA world
coordinates and are useful for labeling or evaluating detections. Creating
YOLO boxes or other training targets requires an offline export that aligns
the poses and sensor data and accounts for visibility. The standalone
`record_bag` tool captures the selected ROS topics only.

On Ubuntu or WSL, inspect a finished bag with the installed ROS environment.

```bash
ASURT="$HOME/.local/share/asurt/current/bin/asurt"
"$ASURT" exec ros2 bag info /absolute/path/to/run/bag
```

To replay planning inputs, start the module under test with simulation time
enabled in an isolated ROS domain, then play only its input topics in that
domain. Use `--clock` to generate the replay clock.

```bash
ASURT_ROS_DOMAIN_ID=87 "$ASURT" exec ros2 bag play /absolute/path/to/run/bag \
  --clock --topics /tf /tf_static /odom /map/global_cones
```

Replay is useful for regression testing open-loop inputs. Rerun a scenario
in CARLA to measure what a changed controller does to the vehicle and its
subsequent observations. [ROS 2 Humble rosbag2](https://raw.githubusercontent.com/ros2/rosbag2/humble/README.md)

## Tools

Call `simulator_status` for the live module states shown by System Manager,
including switches, waiting or failed modules, process IDs and memory.
Pass `{"probe": true}` to check CARLA and ROS directly. The probe reports
world frame advancement, imported cones, ROS publisher counts in the
session's domain, and received `/clock` and `/odom` samples. A provider
process can be running while its data is unavailable. Publisher discovery
is sampled for up to three seconds.

If your agent app keeps its MCP process running across ASURT updates,
restart the ASURT connection after updating. In OneCode, open the ASURT
plugin and click **Restart**.

| Tool | Use |
| --- | --- |
| `list_rulebooks`, `read_rulebook` | Find event references and read rule text. |
| `generate_track` | Build a seeded or manually specified track and validate it. |
| `simulator_status` | List maps, profiles, live modules and the active session. |
| `run_scenario` | Queue a dedicated simulator run with recording and metrics. |
| `record_bag` | Record topics from an existing simulator. |
| `track_importer` | Preview, spawn or clear physical cones, check status, and place the car. |
| `get_run`, `cancel_run` | Inspect a job or cancel one started by this MCP server. |

For failures, check `worker.log` first. `track-import.log` records ground
projection and start-placement checks. `rosbag.log` records subscription and
storage errors. A missing-topic entry can mean the topic was disabled or
event-driven, such as a collision topic in a clean run.
