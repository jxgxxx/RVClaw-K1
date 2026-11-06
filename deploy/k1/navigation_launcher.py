#!/usr/bin/env python3
"""Start the reviewed launch graph without repeated generic ROS CLI discovery."""
import argparse
from pathlib import Path
import time

OWNERS = {'/rvclaw_stm32_hardware', '/x10/lslidar_driver_node', '/amcl',
          '/controller_server', '/rvclaw_localization_runtime'}


def use_native_parameter_yaml():
    """Use installed LibYAML in this launcher only, preserving YAML types."""
    import importlib
    import yaml
    if not getattr(yaml, '__with_libyaml__', False):
        return False

    class NativeYaml:
        def __getattr__(self, name):
            return getattr(yaml, name)

        def safe_load(self, stream):
            return yaml.load(stream, Loader=yaml.CSafeLoader)

        def dump(self, data, stream=None, **options):
            options.setdefault('Dumper', yaml.CDumper)
            return yaml.dump(data, stream, **options)

    # Do not alter the shared yaml module or any installed ROS file. Child
    # nodes start independent interpreters and retain their original loaders.
    native=NativeYaml()
    for name in ('nav2_common.launch.rewritten_yaml',
                 'launch_ros.actions.node', 'launch_ros.parameter_descriptions',
                 'launch_ros.utilities.evaluate_parameters',
                 'launch_ros.utilities.normalize_parameters'):
        importlib.import_module(name).yaml=native
    return True


def verify_graph():
    import rclpy
    rclpy.init()
    node = None
    try:
        node = rclpy.create_node('rvclaw_navigation_start_check',
                                 start_parameter_services=False,
                                 enable_rosout=False)
        deadline = time.monotonic() + 1.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline-time.monotonic())))
        names = {namespace.rstrip('/') + '/' + name
                 for name, namespace in node.get_node_names_and_namespaces()}
        duplicates = OWNERS & names
        if duplicates:
            raise RuntimeError('refusing duplicate persistent runtime: ' + ', '.join(sorted(duplicates)))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def launch_navigation(arguments, *, native_yaml=True):
    if native_yaml:
        use_native_parameter_yaml()
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    launch_file = Path(get_package_share_directory('rvclaw_nav2_bringup')) / 'launch/persistent_navigation.launch.py'
    service = LaunchService(argv=[key+':='+value for key, value in arguments.items()])
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)),
                                 launch_arguments=arguments.items())]))
    return service.run()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--launch-only', action='store_true')
    parser.add_argument('arguments', nargs='*')
    args = parser.parse_args()
    if args.check_only and args.launch_only:
        parser.error('select only one mode')
    arguments = {}
    for argument in args.arguments:
        key, separator, value = argument.partition(':=')
        if not separator or not key or not value:
            parser.error('expected name:=value')
        arguments[key] = value
    started = time.monotonic()
    if not args.launch_only:
        verify_graph()
        print('navigation_launch_timing graph_verified_s=%.3f' % (time.monotonic()-started), flush=True)
    if args.check_only:
        return 0
    return launch_navigation(arguments)


if __name__ == '__main__':
    raise SystemExit(main())
