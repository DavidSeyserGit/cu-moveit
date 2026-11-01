#!/usr/bin/env python3
"""Headless MuJoCo transport for the Copper ReBeL example."""
import argparse
import json
import socket
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def load_model(path):
    tree = ET.parse(path)
    # This example uses Copper's collision model and needs no visual meshes.
    for link in tree.getroot().iter('link'):
        for child in list(link):
            if child.tag in ('visual', 'collision'):
                link.remove(child)
    with tempfile.NamedTemporaryFile(suffix='.urdf') as source:
        tree.write(source.name)
        spec = mujoco.MjSpec.from_file(source.name)
    spec.option.timestep = 0.002
    joints = list(spec.joints)
    if [joint.name for joint in joints] != [f'joint{i}' for i in range(1, 7)]:
        raise ValueError('URDF must contain joint1 through joint6 in order')
    for joint in joints:
        spec.add_actuator(name='motor_' + joint.name,
                          type=mujoco.mjtActuator.mjACTUATOR_MOTOR,
                          trntype=mujoco.mjtTrn.mjTRN_JOINT, target=joint.name,
                          gear=[1, 0, 0, 0, 0, 0], forcelimited=True,
                          forcerange=[-60, 60])
    return spec.compile()


def send_state(conn, data):
    message = {'type': 'state', 'q': data.qpos[:6].tolist(),
               'qd': data.qvel[:6].tolist(), 'time': float(data.time)}
    conn.sendall((json.dumps(message, allow_nan=False) + '\n').encode())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', required=True, help='igus ReBeL URDF')
    parser.add_argument('--port', type=int, default=55091)
    args = parser.parse_args()
    model = load_model(args.urdf)
    data = mujoco.MjData(model)
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', args.port))
        server.listen(1)
        print(f'MuJoCo ready on 127.0.0.1:{args.port}', flush=True)
        while True:
            conn, _ = server.accept()
            try:
                serve(conn, model, data)
            except (json.JSONDecodeError, UnicodeDecodeError, KeyError) as error:
                # Not the Copper bridge (e.g. a local HTTP port probe): drop it.
                print(f'ignoring non-protocol client: {error}', flush=True)
            except ConnectionError:
                print('bridge disconnected', flush=True)


def serve(conn, model, data):
    with conn, conn.makefile('rb') as reader:
        send_state(conn, data)
        for line in reader:
            message = json.loads(line)
            q = np.asarray(message['q'], dtype=float)
            qd = np.asarray(message.get('qd', [0] * 6), dtype=float)
            if q.shape != (6,) or qd.shape != (6,) or not np.isfinite(q).all() or not np.isfinite(qd).all():
                raise ValueError('invalid joint command')
            for _ in range(10):
                mujoco.mj_forward(model, data)
                data.ctrl[:] = np.clip(80 * (q - data.qpos[:6]) + 10 * (qd - data.qvel[:6]) + data.qfrc_bias[:6], -60, 60)
                mujoco.mj_step(model, data)
            send_state(conn, data)

if __name__ == '__main__':
    main()
