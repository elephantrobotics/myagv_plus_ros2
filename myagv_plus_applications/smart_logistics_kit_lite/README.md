# Introduction

The ROS2 package of the MyAGV Plus and Mecharm M5 smart logistics kit lite.

Route Server based logistics sorting on a 2x2 m map: the robot navigates to pickup point `O`, picks a package and scans its QR code with the arm, resolves the destination city to waypoint `A` or `B`, navigates there, aligns with ArUco vision parking, and places the package.

# User manual

## Run a mission

Three terminals, in order.

```
ros2 launch myagv_plus_bringup myagv_plus_bringup.launch.py enable_csi_camera:=true
```

```
ros2 launch smart_logistics_kit_lite navigation2_active.launch.py
```

```
ros2 launch smart_logistics_kit_lite logistics_mission.launch.py
```

`enable_csi_camera:=true` is required. The CSI camera publishes `/camera/image_raw` and `/camera/camera_info` for ArUco parking, and the mission node waits for it before departure.

The mission node checks the arm, the QR camera, `route_bridge`, the parking action server and the ArUco camera before it starts, and reports every second in red until all of them are ready.

## Log language

`language` selects the display language of the city name in QR resolution logs. Default is `zh`.

```
ros2 launch smart_logistics_kit_lite logistics_mission.launch.py language:=en
```

## Map and route graph

Route annotation with the official Route Tool RViz, without starting navigation:

```
ros2 launch smart_logistics_kit_lite route_tool.launch.py
```

The same RViz mode is available while navigation is running:

```
ros2 launch smart_logistics_kit_lite navigation2_active.launch.py rviz:=route
```

Record waypoints `O`, `A` and `B`:

```
scripts/record_waypoints.py
```

## Configuration

| File | Content |
|------|------|
| `scripts/waypoints.yaml` | Pickup point `O`, delivery points `A` / `B`, and the `city_to_waypoint` mapping |
| `graphs/map.geojson` | Route Server navigation graph |
| `map/map.yaml` | Occupancy grid map |
| `param/myagvplus.yaml` | Navigation parameters |

`waypoints.yaml` is read from the source tree first, so editing waypoints or the city mapping needs no rebuild. Editing any `.py` does.
