"""PCD -> real ROS -> TCP -> XML trajectories and overlay. Use a dedicated test workspace.

Run only after deployment, with ROS Python paths preserved separately:
  source devel/setup.bash
  export VEHICLE_ROS_PYTHONPATH="$PYTHONPATH"
  export VEHICLE_ROS_PYTHON=/usr/bin/python2.7
  env -u PYTHONPATH -u PYTHONHOME .runtime/planner/bin/python \
      tests/deployment/smoke_melodic_pipeline.py

Uses the standard conversion node names/topics and loader bag output. Do not run
alongside an active vehicle conversion workflow. The PCD and job outputs are
synthetic, and planning does not command vehicle motion.
"""
import asyncio
import os
from pathlib import Path
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.ground_vehicle_tcp.protocol import NavigationRequest, encode_request, decode_response
from tools.ground_vehicle_tcp.server import NavigationServer
from tools.ground_vehicle_tcp.workflow import NavigationWorkflow


async def smoke(work, pcd):
    metadata = work / 'metadata.yaml'
    metadata.write_text('enu_origin:\n  latitude: 22.39758935\n  longitude: 113.30942461\n  altitude: 25.8618\n')
    config = yaml.safe_load((ROOT / 'tools/path_creater/config/config.yaml').read_text())
    config['output_dir'] = str(work / 'published')
    planner_config = work / 'planner.yaml'
    planner_config.write_text(yaml.safe_dump(config))
    workflow = NavigationWorkflow(ROOT, work / 'state', pcd, pcd.name,
                                  metadata, planner_config, conversion_timeout_s=180)
    server = NavigationServer(workflow, host='127.0.0.1', port=0)
    listener = await server.serve()
    port = listener.sockets[0].getsockname()[1]
    reader, writer = await asyncio.open_connection('127.0.0.1', port)
    try:
        previous_map_state = None
        for goal in (15000, 14000):
            request = NavigationRequest(5000, 5000, 0, goal, goal, 0, 60)
            writer.write(encode_request(request))
            await writer.drain()
            response = await asyncio.wait_for(reader.readexactly(8), timeout=300)
            assert decode_response(response) == (True, 0), response.hex()
            NavigationWorkflow._validate_outputs(work / 'published')
            maps = sorted((work / 'state/maps').glob('*/map_enu.pgm'))
            assert len(maps) == 1, maps
            state = (maps[0].name, maps[0].stat().st_mtime_ns)
            if previous_map_state is not None:
                assert state == previous_map_state, 'map unexpectedly reconverted'
            previous_map_state = state
        print('PASS: real ROS conversion, two TCP requests, valid XML/overlay outputs, cache reuse')
    except Exception:
        for log in (work / 'state/jobs').glob('*/job.log'):
            print(log.read_text(errors='replace')[-12000:], file=sys.stderr)
        raise
    finally:
        writer.close()
        await writer.wait_closed()
        await server.close()


if __name__ == '__main__':
    assert os.environ.get('ROS_DISTRO') == 'melodic', 'This smoke test targets Melodic'
    assert 'VEHICLE_ROS_PYTHONPATH' in os.environ, 'Preserve ROS Python path as documented'
    os.chdir(str(ROOT))
    pcd_dir = ROOT / 'map/pcd'
    pcd_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', prefix='vehicle_smoke_', suffix='.pcd',
                                     dir=str(pcd_dir), delete=False) as stream:
        pcd = Path(stream.name)
        stream.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                     'TYPE F F F\nCOUNT 1 1 1\nWIDTH 10201\nHEIGHT 1\n'
                     'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 10201\nDATA ascii\n')
        for row in range(101):
            for col in range(101):
                stream.write('{:.2f} {:.2f} 0.0\n'.format(col * 0.2, row * 0.2))
    try:
        with tempfile.TemporaryDirectory(prefix='vehicle-melodic-smoke-') as directory:
            asyncio.run(smoke(Path(directory), pcd))
    finally:
        pcd.unlink()
