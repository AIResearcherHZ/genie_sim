# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0
#
# Script to convert Taks_T1_omnipicker URDF to USD/USDA assets.
# Run this script inside Isaac Sim:
#   ./python.sh scripts/convert_taks_t1_urdf_to_usd.py
#
# This will generate:
#   source/geniesim/assets/robot/Taks_T1_omnipicker/robot.usd
#   source/geniesim/assets/robot/Taks_T1_omnipicker/robot.usda
#   source/geniesim/assets/robot/Taks_T1_omnipicker/configuration/

import os
import sys
import argparse


def convert_urdf_to_usd(urdf_path, output_dir, robot_name="Taks_T1_omnipicker"):
    """Convert URDF to USD using Isaac Sim's UrdfConverter."""
    try:
        from omni.isaac.urdf import _urdf as urdf_interface
    except ImportError:
        try:
            from omni.importer.urdf import _urdf as urdf_interface
        except ImportError:
            print("ERROR: Cannot import URDF converter. Run this script inside Isaac Sim.")
            print("Usage: <isaac_sim_path>/python.sh scripts/convert_taks_t1_urdf_to_usd.py")
            sys.exit(1)

    import omni.kit.app
    import carb

    # Configure URDF import settings
    import_config = urdf_interface.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.make_instanceable = True
    import_config.self_collision = False
    import_config.density = 0.0
    import_config.import_inertia_tensor = True
    import_config.convex_decomp = False
    import_config.create_physics_scene = False
    import_config.default_drive_type = urdf_interface.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 100.0
    import_config.default_position_drive_damping = 1.0
    import_config.distance_scale = 1.0

    # Set output path
    output_usd = os.path.join(output_dir, "robot.usd")

    print(f"Converting URDF: {urdf_path}")
    print(f"Output USD: {output_usd}")

    # Import URDF
    result = urdf_interface.acquire_urdf_interface()
    parsed_urdf = result.parse_urdf(urdf_path, import_config)

    if parsed_urdf is None:
        print("ERROR: Failed to parse URDF file.")
        sys.exit(1)

    dest_path = result.import_robot(
        urdf_path,
        robot_name,
        import_config,
        "",
    )

    if dest_path:
        print(f"Successfully converted URDF to USD: {dest_path}")
        # Save as USDA (text format)
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        usda_path = os.path.join(output_dir, "robot.usda")
        stage.GetRootLayer().Export(usda_path)
        print(f"Exported USDA: {usda_path}")
    else:
        print("ERROR: URDF import failed.")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Convert Taks_T1_omnipicker URDF to USD")
    parser.add_argument(
        "--urdf",
        type=str,
        default=None,
        help="Path to Taks_T1_omnipicker URDF file",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output directory for USD files",
    )
    args = parser.parse_args()

    # Determine paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)

    urdf_path = args.urdf or os.path.join(
        project_root,
        "source", "geniesim", "app", "robot_cfg",
        "Taks_T1_omnipicker", "Taks_T1_omnipicker.urdf",
    )
    output_dir = args.output or os.path.join(
        project_root,
        "source", "geniesim", "assets", "robot", "Taks_T1_omnipicker",
    )

    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF file not found: {urdf_path}")
        sys.exit(1)

    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(os.path.join(output_dir, "configuration"), exist_ok=True)

    print(f"Project root: {project_root}")
    print(f"URDF path: {urdf_path}")
    print(f"Output dir: {output_dir}")

    convert_urdf_to_usd(urdf_path, output_dir)
    print("Done! Taks_T1_omnipicker USD assets generated.")


if __name__ == "__main__":
    main()
