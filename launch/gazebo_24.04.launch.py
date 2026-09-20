from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET


DEFAULT_WORLD_FILE = "world.sdf"
DEFAULT_SPAWN_MODELS = "f3rc2026_map_v2:0,0,0.01,0,0,0;omni_robot:2,1.0,0.5,0,0,0"
DEFAULT_STATIC_MODELS = "f3rc2026_map_v2"


def launch_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


class SpawnSpec:
    def __init__(self, model_name, instance_name, pose):
        self.model_name = model_name
        self.instance_name = instance_name
        self.pose = pose


def parse_name_list(value):
    return {item.strip() for item in value.split(",") if item.strip()}


def parse_spawn_models(value):
    specs = []
    for raw_entry in value.split(";"):
        entry = raw_entry.strip()
        if not entry:
            continue

        name_part, pose_part = (entry.split(":", 1) + [""])[:2] if ":" in entry else (entry, "")
        if "@" in name_part:
            model_name, instance_name = name_part.split("@", 1)
        else:
            model_name = name_part
            instance_name = model_name

        pose_values = [part.strip() for part in pose_part.split(",") if part.strip()]
        if pose_values:
            if len(pose_values) != 6:
                raise RuntimeError(
                    f"spawn_models entry '{entry}' must use 6 pose values: x,y,z,roll,pitch,yaw"
                )
            pose = " ".join(pose_values)
        else:
            pose = "0 0 0 0 0 0"

        specs.append(SpawnSpec(model_name.strip(), instance_name.strip(), pose))

    if not specs:
        raise RuntimeError("At least one model must be listed in spawn_models")

    return specs


def is_static(spec, static_models):
    return spec.model_name in static_models or spec.instance_name in static_models


def validate_control_targets(specs, static_models):
    controlled_specs = [spec for spec in specs if not is_static(spec, static_models)]
    if len(controlled_specs) > 1:
        controlled_names = ", ".join(spec.instance_name for spec in controlled_specs)
        raise RuntimeError(
            "Multiple controlled models are not supported yet because controller_manager is global. "
            f"Add all but one to static_models, or split controller namespaces. Controlled models: {controlled_names}"
        )
    return controlled_specs


def remove_matching_plugins(element, predicate):
    for child in list(element):
        remove_matching_plugins(child, predicate)
        if child.tag == "plugin" and predicate(child):
            element.remove(child)


def strip_gz_ros2_control_plugins(sdf_text):
    root = ET.fromstring(sdf_text)

    def is_gz_ros2_control(plugin):
        name = plugin.attrib.get("name", "")
        filename = plugin.attrib.get("filename", "")
        return "gz_ros2_control" in name or "gz_ros2_control" in filename

    remove_matching_plugins(root, is_gz_ros2_control)
    return ET.tostring(root, encoding="unicode")


def normalize_mesh_uris(sdf_text):
    sdf_text = re.sub(r"model://.*?/meshes/", "meshes/", sdf_text)
    return sdf_text.replace("model://meshes/", "meshes/")


def write_model_config(model_dir, model_name):
    config_path = os.path.join(model_dir, "model.config")
    model_config = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>Hao</name>
  </author>
  <description>
    Auto-generated model configuration
  </description>
</model>
"""
    with open(config_path, "w", encoding="utf-8") as config_file:
        config_file.write(model_config)


def regenerate_model_sdf(ros2_ws, model_name, keep_control_plugins, xacro_arguments=None):
    model_dir = os.path.join(ros2_ws, f"src/gazebo_simulator/models/{model_name}")
    xacro_path = os.path.join(model_dir, "urdf", f"{model_name}.xacro")
    model_sdf_path = os.path.join(model_dir, "model.sdf")

    if not os.path.isdir(model_dir):
        raise RuntimeError(f"Model directory does not exist: {model_dir}")

    if not os.path.isfile(xacro_path):
        if not os.path.isfile(model_sdf_path):
            raise RuntimeError(f"Neither xacro nor model.sdf exists for model: {model_name}")
        return

    urdf_path = os.path.join(model_dir, f"{model_name}.urdf")
    xacro_command = ["xacro", xacro_path, f"robot_name:={model_name}"]
    for key, value in (xacro_arguments or {}).items():
        xacro_command.append(f"{key}:={value}")

    with open(urdf_path, "w", encoding="utf-8") as urdf_file:
        subprocess.run(
            xacro_command,
            check=True,
            stdout=urdf_file,
        )

    with open(model_sdf_path, "w", encoding="utf-8") as sdf_file:
        subprocess.run(["gz", "sdf", "-p", urdf_path], check=True, stdout=sdf_file)

    with open(model_sdf_path, "r", encoding="utf-8") as sdf_file:
        model_sdf = sdf_file.read()

    model_sdf = normalize_mesh_uris(model_sdf)
    if not keep_control_plugins:
        model_sdf = strip_gz_ros2_control_plugins(model_sdf)

    with open(model_sdf_path, "w", encoding="utf-8") as sdf_file:
        sdf_file.write(model_sdf)

    write_model_config(model_dir, model_name)


def make_include_block(spec, static_models):
    lines = [
        "    <include>",
        f"      <uri>model://{spec.model_name}</uri>",
    ]
    if spec.instance_name != spec.model_name:
        lines.append(f"      <name>{spec.instance_name}</name>")
    lines.append(f"      <pose>{spec.pose}</pose>")
    if is_static(spec, static_models):
        lines.append("      <static>true</static>")
    lines.append("    </include>")
    return "\n".join(lines)


def build_world_sdf(ros2_ws, world_file, include_blocks):
    world_template_path = os.path.join(ros2_ws, f"src/gazebo_simulator/models/world/{world_file}")
    with open(world_template_path, "r", encoding="utf-8") as template_file:
        world_sdf = template_file.read()

    includes = "\n".join(include_blocks)
    placeholder_include_pattern = re.compile(
        r"\s*<include>\s*<uri>model://__MODEL_NAME__</uri>.*?</include>",
        re.DOTALL,
    )
    if "__MODEL_NAME__" in world_sdf:
        world_sdf = placeholder_include_pattern.sub("\n" + includes, world_sdf)
        world_sdf = world_sdf.replace("__MODEL_NAME__", "")
    elif "__MODEL_INCLUDES__" in world_sdf:
        world_sdf = world_sdf.replace("__MODEL_INCLUDES__", includes)
    else:
        world_sdf = world_sdf.replace("</world>", includes + "\n  </world>")

    world_file_handle = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        delete=False,
        suffix=".sdf",
        prefix="gazebo_simulator_world_",
    )
    world_file_handle.write(world_sdf)
    world_file_handle.close()
    return world_file_handle.name


def append_env_path(current_value, path):
    values = [value for value in current_value.split(os.pathsep) if value]
    if path not in values:
        values.insert(0, path)
    return os.pathsep.join(values)


def launch_setup(context, *args, **kwargs):
    ros2_ws = os.environ.get("ROS2_WS")
    if not ros2_ws:
        raise RuntimeError("ROS2_WS environment variable is required")

    world_file = LaunchConfiguration("world_file").perform(context)
    spawn_models = LaunchConfiguration("spawn_models").perform(context)
    static_models = parse_name_list(LaunchConfiguration("static_models").perform(context))
    enable_localization = launch_bool(LaunchConfiguration("enable_localization").perform(context))
    enable_control = launch_bool(LaunchConfiguration("enable_control").perform(context))
    gui = launch_bool(LaunchConfiguration("gui").perform(context))
    headless_rendering = launch_bool(LaunchConfiguration("headless_rendering").perform(context))
    bridge_scan = launch_bool(LaunchConfiguration("bridge_scan").perform(context))
    rviz = launch_bool(LaunchConfiguration("rviz").perform(context))
    render_engine = LaunchConfiguration("render_engine").perform(context)
    render_engine_gui = LaunchConfiguration("render_engine_gui").perform(context)
    render_engine_server = LaunchConfiguration("render_engine_server").perform(context)
    lidar_options = {
        "enable_lidar": LaunchConfiguration("enable_lidar").perform(context),
        "lidar_visualize": LaunchConfiguration("lidar_visualize").perform(context),
        "lidar_update_rate": LaunchConfiguration("lidar_update_rate").perform(context),
        "lidar_samples": LaunchConfiguration("lidar_samples").perform(context),
    }

    specs = parse_spawn_models(spawn_models)
    controlled_specs = validate_control_targets(specs, static_models)
    active_controlled_specs = controlled_specs if enable_control else []
    model_names = sorted({spec.model_name for spec in specs})

    for name in model_names:
        keep_control_plugins = any(spec.model_name == name for spec in active_controlled_specs)
        model_xacro_arguments = lidar_options if name == "omni_robot" else None
        regenerate_model_sdf(ros2_ws, name, keep_control_plugins, model_xacro_arguments)

    world_path = build_world_sdf(
        ros2_ws,
        world_file,
        [make_include_block(spec, static_models) for spec in specs],
    )

    models_path = os.path.join(ros2_ws, "src/gazebo_simulator/models")
    gz_resource_path = append_env_path(os.environ.get("GZ_SIM_RESOURCE_PATH", ""), models_path)
    gz_model_path = append_env_path(os.environ.get("GZ_MODEL_PATH", ""), models_path)
    gz_plugin_path = append_env_path(os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""), "/opt/ros/jazzy/lib")

    gz_cmd = [
    # os.path.expanduser("~/.local/bin/run-mesa-d3d12-fixed"),
    "gz","sim","-r",]
    if not gui:
        gz_cmd.append("-s")
    if headless_rendering:
        gz_cmd.append("--headless-rendering")
    if render_engine:
        gz_cmd.extend(["--render-engine", render_engine])
    if render_engine_gui:
        gz_cmd.extend(["--render-engine-gui", render_engine_gui])
    if render_engine_server:
        gz_cmd.extend(["--render-engine-server", render_engine_server])
    gz_cmd.append(world_path)

    actions = [
        ExecuteProcess(
            cmd=gz_cmd,
            output="screen",
            additional_env={
                "GZ_SIM_RESOURCE_PATH": gz_resource_path,
                "GZ_MODEL_PATH": gz_model_path,
                "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            },
        )
    ]

    for spec in specs:
        xacro_path = os.path.join(
            ros2_ws,
            f"src/gazebo_simulator/models/{spec.model_name}/urdf/{spec.model_name}.xacro",
        )
        if not os.path.isfile(xacro_path):
            continue

        xacro_command = [
            "xacro ",
            xacro_path,
            " robot_name:=",
            spec.model_name,
        ]
        if spec.model_name == "omni_robot":
            for key, value in lidar_options.items():
                xacro_command.extend([f" {key}:=", value])

        robot_description = Command(xacro_command)
        node_kwargs = {}
        if spec not in active_controlled_specs:
            node_kwargs["namespace"] = spec.instance_name
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{
                    "robot_description": robot_description,
                    "use_sim_time": True,
                }],
                output="screen",
                **node_kwargs,
            )
        )

    bridge_arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    for spec in specs:
        bridge_arguments.append(
            f"/model/{spec.instance_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
        )
    if bridge_scan:
        bridge_arguments.append("/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=bridge_arguments,
            output="screen",
        )
    )

    controlled_spec = active_controlled_specs[0] if active_controlled_specs else None
    if controlled_spec:
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "60",
                "--service-call-timeout", "60",
                "--switch-timeout", "60",
            ],
        )
        forward_velocity_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "forward_velocity_controller",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "60",
                "--service-call-timeout", "60",
                "--switch-timeout", "60",
            ],
        )
        control_nodes = [
            Node(
                package="gazebo_simulator",
                executable="can_to_gazebo",
                parameters=[{"robot_name": controlled_spec.model_name}],
                output="screen",
            ),
            Node(
                package="gazebo_simulator",
                executable="gazebo_to_uodom",
                parameters=[{
                    "input_topic": f"/model/{controlled_spec.instance_name}/pose",
                    "target_child_frame_id": controlled_spec.instance_name,
                }],
                output="screen",
            ),
            Node(
                package="uec_core",
                executable="omni",
                parameters=[
                    {"update_freq": 0.05},
                    {"wheel_radius": 0.05},
                    {"wheel_base": 0.23},
                    {"min_v": 0.05},
                    {"max_v": 1.0},
                    {"max_a": 1.0},
                    {"bulk_id": [1, 2, 3, 4]},
                ],
                output="screen",
            ),
            Node(
                package="gazebo_simulator",
                executable="teleop_twist_keyboard",
                output="screen",
                prefix="xterm -e",
            ),
        ]
        if enable_localization:
            localization_config_path = os.path.join(
                ros2_ws,
                "src/ROSLib2/uec_localization/config/localization.yaml",
            )
            rviz_config_path = os.path.join(
                ros2_ws,
                "src/ROSLib2/uec_localization/rviz/bag_fastlio2d.rviz",
            )
            control_nodes.extend([
                Node(
                    package="uec_localization",
                    executable="fastlio2d_node",
                    name="fastlio2d_node",
                    output="screen",
                    parameters=[
                        {"config_path": localization_config_path},
                        {"publish_pointcloud": True},
                        {"use_sim_time": True},
                        {"tf_mode": "map_to_odom"},
                        {"tf_map_frame_id": "map"},
                        {"tf_odom_frame_id": "odom"},
                        {"tf_base_frame_id": "base_link"},
                    ],
                ),
                Node(
                    package="uec_localization",
                    executable="view_map",
                    name="view_map",
                    output="screen",
                    parameters=[
                        {"config_path": localization_config_path},
                        {"use_sim_time": True},
                    ],
                ),
            ])
            if rviz:
                control_nodes.append(
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=["-d", rviz_config_path],
                        parameters=[{"use_sim_time": True}],
                    )
                )
        actions.append(
            TimerAction(
                period=5.0,
                actions=[
                    joint_state_broadcaster_spawner,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=joint_state_broadcaster_spawner,
                            on_exit=[forward_velocity_controller_spawner],
                        )
                    ),
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=forward_velocity_controller_spawner,
                            on_exit=control_nodes,
                        )
                    ),
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world_file",
            default_value=DEFAULT_WORLD_FILE,
            description="models/world/ 内のワールドテンプレートファイル",
        ),
        DeclareLaunchArgument(
            "spawn_models",
            default_value=DEFAULT_SPAWN_MODELS,
            description=(
                "読み込むモデルをセミコロン区切りで指定します。"
                "model[:x,y,z,roll,pitch,yaw] または "
                "model@instance[:x,y,z,roll,pitch,yaw] の形式で指定します。"
            ),
        ),
        DeclareLaunchArgument(
            "static_models",
            default_value=DEFAULT_STATIC_MODELS,
            description="静的モデルとして読み込むモデル名またはインスタンス名をカンマ区切りで指定します",
        ),
        DeclareLaunchArgument(
            "enable_localization",
            default_value="false",
            description="map->odom TF のために uec_localization の fastlio2d_node/view_map を起動します",
        ),
        DeclareLaunchArgument(
            "enable_control",
            default_value="true",
            description="controller spawner、can_to_gazebo、gazebo_to_uodom、omni、teleop を起動します",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Gazebo GUI を起動します。false にすると gz sim のサーバーのみ（-s）を起動します",
        ),
        DeclareLaunchArgument(
            "headless_rendering",
            default_value="false",
            description="gz sim に --headless-rendering を渡します。サーバーのみで gpu_lidar をテストする場合などに使用します",
        ),
        DeclareLaunchArgument(
            "bridge_scan",
            default_value="true",
            description="Gazebo の /scan を ROS にブリッジします",
        ),
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="true",
            description="生成する SDF に omni_robot の gpu_lidar センサーを含めます",
        ),
        DeclareLaunchArgument(
            "lidar_visualize",
            default_value="false",
            description="omni_robot の gpu_lidar のレーザーを Gazebo 上で可視化します",
        ),
        DeclareLaunchArgument(
            "lidar_update_rate",
            default_value="40",
            description="omni_robot の gpu_lidar の更新周波数（Hz）",
        ),
        DeclareLaunchArgument(
            "lidar_samples",
            default_value="1440",
            description="omni_robot の gpu_lidar の水平方向のサンプル数",
        ),
        DeclareLaunchArgument(
            "render_engine",
            default_value="",
            description="gz sim の --render-engine に渡す値を指定します（例: ogre2、ogre）。省略可能です",
        ),
        DeclareLaunchArgument(
            "render_engine_gui",
            default_value="",
            description="gz sim の --render-engine-gui に渡す値を指定します。省略可能です",
        ),
        DeclareLaunchArgument(
            "render_engine_server",
            default_value="",
            description="gz sim の --render-engine-server に渡す値を指定します。省略可能です",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="uec_localization の FastLIO2D 設定を使用して RViz を起動します",
        ),
        OpaqueFunction(function=launch_setup),
    ])




# ### 今回定義されている設定

# | 設定                     |       デフォルト | 意味                            |
# | ---------------------- | ----------: | ----------------------------- |
# | `world_file`           | `world.sdf` | 使用するGazeboワールド                |
# | `spawn_models`         |       定数で指定 | 出現させるモデル・位置・姿勢                |
# | `static_models`        |       定数で指定 | 動かない固定モデル                     |
# | `enable_localization`  |      `true` | FastLIO2Dなど自己位置推定を起動          |
# | `enable_control`       |      `true` | ロボット制御系を起動                    |
# | `gui`                  |      `true` | Gazeboの画面を表示                  |
# | `headless_rendering`   |     `false` | GUIなしでもレンダリングする               |
# | `bridge_scan`          |      `true` | Gazeboの `/scan` をROS 2へBridge |
# | `enable_lidar`         |      `true` | LiDARをモデルに含める                 |
# | `lidar_visualize`      |     `false` | Gazebo上でLiDARの光線を可視化          |
# | `lidar_update_rate`    |        `10` | LiDAR更新周期（10 Hz）              |
# | `lidar_samples`        |       `720` | LiDARの水平方向サンプル数               |
# | `render_engine`        |           空 | Gazebo全体の描画エンジン指定             |
# | `render_engine_gui`    |           空 | GUI側の描画エンジン指定                 |
# | `render_engine_server` |           空 | サーバー側の描画エンジン指定                |
# | `rviz`                 |     `false` | RVizを起動するか                    |
