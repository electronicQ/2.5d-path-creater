"""Cross-runtime boundaries must not load Melodic's Python 2 YAML in Python 3."""
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import yaml
from PIL import Image

from tools.ground_vehicle_tcp.pipeline import PipelineRunner
from tools.pgm_saver import save_continuous_map as saver


class MelodicRuntimeTest(unittest.TestCase):
    def test_ros_and_planner_python_environments_are_separate(self):
        runner = PipelineRunner(Path('.'), env={
            'ROS_DISTRO': 'melodic',
            'PYTHONPATH': '/unwanted/python3/site-packages',
            'VEHICLE_ROS_PYTHONPATH': '/opt/ros/melodic/lib/python2.7/dist-packages',
            'PYTHONHOME': '/unwanted/home',
        })
        self.assertEqual(runner.ros_python, '/usr/bin/python2.7')
        self.assertIn('python2.7', runner.env['PYTHONPATH'])
        self.assertNotIn('PYTHONPATH', runner.planner_env)
        self.assertNotIn('PYTHONHOME', runner.planner_env)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'out'
            with patch('tools.ground_vehicle_tcp.pipeline.subprocess.run') as run:
                run.return_value.returncode = 1
                with self.assertRaisesRegex(Exception, 'planner exited'):
                    runner.plan(Path(directory) / 'config.yaml', output, Path(directory) / 'job.log')
            self.assertEqual(run.call_args[0][0][0], sys.executable)
            self.assertNotIn('PYTHONPATH', run.call_args[1]['env'])

    def test_real_planner_subprocess_generates_all_ten_outputs(self):
        from tools.ground_vehicle_tcp.workflow import NavigationWorkflow
        from tools.path_creater.geo import GeoPoint, enu_to_geodetic
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            Image.new('L', (40, 40), 254).save(str(work / 'map.pgm'))
            (work / 'map.yaml').write_text(
                'image: map.pgm\nresolution: 0.5\norigin: [0, 0, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
            origin = GeoPoint(22.39758935, 113.30942461, 25.8618)
            metadata = work / 'meta.yaml'
            metadata.write_text(yaml.safe_dump({'enu_origin': dict(
                latitude=origin.latitude, longitude=origin.longitude, altitude=origin.altitude)}))
            config = yaml.safe_load((root / 'tools/path_creater/config/config.yaml').read_text())
            config['map_yaml'] = str(work / 'map.yaml')
            config['map_meta'] = str(metadata)
            config['output_dir'] = str(work / 'output')
            for name, xy in [('start', (3, 3)), ('goal', (15, 15))]:
                point = enu_to_geodetic(xy[0], xy[1], 0, origin)
                config[name] = dict(latitude=point.latitude, longitude=point.longitude,
                                    altitude=point.altitude)
            (work / 'config.yaml').write_text(yaml.safe_dump(config))
            poison = work / 'python2-path'
            poison.mkdir()
            (poison / 'yaml.py').write_text('raise RuntimeError("Python 2 path leaked")\n')
            runner = PipelineRunner(root, timeout_s=30, env={
                'ROS_DISTRO': 'melodic', 'VEHICLE_ROS_PYTHONPATH': str(poison)})
            outputs = runner.plan(work / 'config.yaml', work / 'output', work / 'job.log')
            self.assertEqual(len(outputs), 10)
            NavigationWorkflow._validate_outputs(work / 'output')

    def test_noetic_default_and_explicit_saver_interpreter(self):
        self.assertEqual(PipelineRunner(Path('.'), env={'ROS_DISTRO': 'noetic'}).ros_python,
                         '/usr/bin/python3')
        self.assertEqual(PipelineRunner(Path('.'), env={'VEHICLE_ROS_PYTHON': '/custom/python'}).ros_python,
                         '/custom/python')

    def test_map_conversion_starts_nodes_without_topic_gates(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / 'map_enu'
            Path(str(output) + '.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
            Path(str(output) + '.yaml').write_text(
                'image: map_enu.pgm\nresolution: 0.5\norigin: [0, 0, 0]\n',
                encoding='utf-8',
            )
            runner = PipelineRunner(root, env={'ROS_DISTRO': 'noetic'})
            commands = []

            class FakeProcess:
                returncode = 0

                def poll(self):
                    return 0

            def record_start(command, log_path):
                commands.append(command)
                return FakeProcess()

            with patch.object(runner, '_start', side_effect=record_start), \
                    patch.object(runner, '_wait_for_ros_master') as wait_for_master:
                runner.convert('scans_Global_ENU.pcd', output, root / 'job.log')

            wait_for_master.assert_called_once()
            self.assertEqual(commands[0], [
                'roslaunch', 'grid_map_pcl', 'grid_map_pcl_loader_node.launch',
                'pcd_filename:=scans_Global_ENU.pcd',
                'enable_rviz:=false',
                'publish_point_cloud:=false',
            ])
            self.assertEqual(commands[1], [
                'roslaunch', 'grid_map_demos', 'filters_demo_gai.launch',
                'enable_rviz:=false',
            ])
            self.assertEqual(commands[2], [
                '/usr/bin/python3', 'tools/pgm_saver/save_continuous_map.py', '-n', str(output),
            ])

    def test_binary_pgm_preserves_grayscale_flip_and_yaml_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            node = saver.ContinuousMapSaver.__new__(saver.ContinuousMapSaver)
            node.output_base = Path(directory) / 'map with spaces'
            msg = SimpleNamespace(
                data=[-1, 100, 0, 50],
                info=SimpleNamespace(width=2, height=2, resolution=0.2,
                    origin=SimpleNamespace(position=SimpleNamespace(x=12, y=-5),
                        orientation=SimpleNamespace(w=1, x=0, y=0, z=0))))
            with patch.object(saver, 'rospy'):
                node.map_callback(msg)
            with Image.open(str(node.output_base) + '.pgm') as image:
                self.assertEqual(list(image.getdata()), [254, 127, 205, 0])
            metadata = yaml.safe_load(Path(str(node.output_base) + '.yaml').read_text())
            self.assertEqual(metadata['image'], 'map with spaces.pgm')
            self.assertEqual(metadata['origin'], [12, -5, 0.0])
            self.assertEqual(metadata['resolution'], 0.2)


if __name__ == '__main__':
    unittest.main()
