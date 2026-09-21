#!/usr/bin/env bash
# Visualization only: run one workshop mission with overview cameras recording
# numbered frames. Outputs stay in results/debug; never study evidence.
set -o pipefail
cd /home/roboboat/morphology_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
out=results/debug/render_mission
rm -rf $out && mkdir -p $out/frames
python3 -c "from modular_robot_benchmarks.design import generate_workshop_design as g; g(replicates=1).write_frozen('$out/design.json')"
# Frame saver: every 4 s of wall time keep the latest image from each camera.
ros2 run ros_gz_bridge parameter_bridge \
  /overview/oblique@sensor_msgs/msg/Image[gz.msgs.Image \
  /overview/close@sensor_msgs/msg/Image[gz.msgs.Image \
  /overview/top@sensor_msgs/msg/Image[gz.msgs.Image > $out/bridge.log 2>&1 &
bridge_pid=$!
python3 - $out/frames <<'PY' > $out/saver.log 2>&1 &
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
out = sys.argv[1]
rclpy.init()
node = Node("overview_mission_saver")
latest = {}
for name in ("oblique", "close", "top"):
    node.create_subscription(Image, f"/overview/{name}", lambda m, n=name: latest.__setitem__(n, m), qos_profile_sensor_data)
index, next_save = 0, time.monotonic() + 4.0
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.2)
    if time.monotonic() >= next_save and latest:
        for name, msg in latest.items():
            channels = 4 if msg.encoding.endswith("a8") else 3
            data = bytes(msg.data)
            rows = [data[r * msg.step:r * msg.step + msg.width * channels] for r in range(msg.height)]
            pixels = bytearray()
            for line in rows:
                if channels == 3:
                    pixels += line
                else:
                    for i in range(0, len(line), 4):
                        pixels += line[i:i + 3]
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with open(f"{out}/{name}_{index:03d}_t{stamp:06.1f}.ppm", "wb") as f:
                f.write(f"P6 {msg.width} {msg.height} 255\n".encode() + pixels)
        index += 1
        next_save = time.monotonic() + 4.0
PY
saver_pid=$!
# Inject cameras into the exported world without changing repository code.
python3 - <<'PY'
import sys
sys.argv = ["mission_batch", "--design", "results/debug/render_mission/design.json",
            "--raw", "results/debug/render_mission/raw",
            "--artifacts", "results/debug/render_mission/artifacts",
            "--max-trials", "1", "--allow-dirty"]
sys.path.insert(0, "results/debug")
from xml.etree.ElementTree import ElementTree, parse
import render_world
import modular_robot_benchmarks.mission_batch as batch
original = batch.export_confirmatory_sdf
def export_with_cameras(family, layout_index, world_seed, output, catalog_path, friction_seed):
    world_path, manifest = original(family, layout_index, world_seed, output, catalog_path, friction_seed)
    tree = parse(world_path)
    render_world.inject(tree.getroot(), (2.15, 1.75))
    ElementTree(tree.getroot()).write(world_path, encoding="unicode", xml_declaration=True)
    return world_path, manifest
batch.export_confirmatory_sdf = export_with_cameras
batch.main()
PY
kill -INT $saver_pid $bridge_pid 2>/dev/null; sleep 2; kill -9 $saver_pid $bridge_pid 2>/dev/null
ls $out/frames | head -3; echo "frames: $(ls $out/frames | wc -l)"
python3 -c "import json,glob; r=json.load(open(glob.glob('$out/raw/*.json')[0])); print(r['spec']['trial_id'], r['terminal_status'], [(round(e['time_s'],1), e['state']) for e in r['execution_state_history']])"
