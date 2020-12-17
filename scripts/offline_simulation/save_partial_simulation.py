# Copyright (c) Facebook, Inc. and its affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import functools
import multiprocessing
import os
import signal
import sys
import phyre.interface.scene.ttypes as scene_if
import joblib
import numpy as np

import phyre

def sample_gaussian(mu,sigma,n = 1,rng=np.random.default_rng()):
    return rng.normal(mu,sigma,n)

# skewed physics parameters, no noise
def inaccurate_physics():
    physics = scene_if.Physics()
    physics.friction = 0.1
    physics.restitution = 0.5
    physics.density = 0.5
    physics.gravity = -5
    return physics

# default phyre physics
def default_physics():
    # all noise params set to 0, and physics params set to default values
    physics = scene_if.Physics()
    return physics

def inaccurate_noisy_physics(rng=np.random.default_rng()):
    physics = scene_if.Physics()
    physics.friction = sample_gaussian(0.1,0.05,rng=rng)
    physics.restitution = sample_gaussian(1,0.05,rng=rng)
    physics.density = sample_gaussian(0.5,0.05,rng=rng)
    physics.angularDamping = sample_gaussian(0.1,0.01,rng=rng)
    physics.linearDamping = sample_gaussian(0.1,0.01,rng=rng)
    physics.gravity = sample_gaussian(-9.8,0.05,rng=rng)

    return physics


# default phrye physics with added noise
def default_noisy_physics(rng=np.random.default_rng()):
    physics = scene_if.Physics()
    physics.friction = sample_gaussian(physics.friction,0.1,rng=rng)
    physics.angularDamping = sample_gaussian(physics.angularDamping,0.05,rng=rng)
    physics.linearDamping = sample_gaussian(physics.linearDamping,0.05,rng=rng)
    physics.density = sample_gaussian(physics.density,0.1,rng=rng)
    physics.restitution = sample_gaussian(physics.restitution,0.1,rng=rng)
    physics.gravity = sample_gaussian(physics.gravity,0.1,rng=rng)
    return physics

def _worker(tier, task_id, num_jobs, num_actions, job_id):
    action_path = (
        phyre.simulation_cache.get_partial_cache_folder(num_actions) / tier /
        phyre.simulation_cache.ACTION_FILE_NAME)
    actions = joblib.load(action_path)
    sim = phyre.initialize_simulator([task_id], tier)

    actions = np.array_split(actions, num_jobs)[job_id]
    statuses = [
        int(sim.simulate_action(0, action, need_images=False,physics = default_noisy_physics()).status)
        for action in actions
    ]
    return statuses


def get_job_id():
    if 'SLURM_ARRAY_JOB_ID' in os.environ:
        return '%s_%s' % (os.environ['SLURM_ARRAY_JOB_ID'],
                          os.environ['SLURM_ARRAY_TASK_ID'])
    else:
        return os.environ['SLURM_JOB_ID']


def get_task_id_slurm(task_list_fpath):
    assert 'SLURM_ARRAY_TASK_ID' in os.environ
    with open(task_list_fpath) as stream:
        task_list = stream.read().split()
    return task_list[int(os.environ['SLURM_ARRAY_TASK_ID'])]


def sig_handler(signum, frame):
    del frame  # Unused.
    print("Signal handler called with signal " + str(signum))
    prod_id = int(os.environ['SLURM_PROCID'])
    job_id = get_job_id()
    if prod_id == 0:
        print("Requeuing job " + job_id)
        os.system('scontrol requeue ' + job_id)
    else:
        print("Not the master process, no need to requeue.")
    sys.exit(-1)


def term_handler(signum, frame):
    del frame  # Unused.
    print("Signal handler called with signal " + str(signum))
    print("Bypassing SIGTERM.")


def init_signal_handler():
    """
    Handle signals sent by SLURM for time limit / pre-emption.
    """
    signal.signal(signal.SIGUSR1, sig_handler)
    signal.signal(signal.SIGTERM, term_handler)
    print("Signal handler installed.")


def main(task_id, tier, num_workers, num_actions):
    if '/' in task_id:
        task_id = get_task_id_slurm(task_id)
        init_signal_handler()

    print('task_id =', task_id)
    sim_dir = (phyre.simulation_cache.get_partial_cache_folder(num_actions) /
               tier / task_id.split(':')[0])
    assert sim_dir.exists(), sim_dir
    sim_path = sim_dir / (task_id + '.gz')

    if os.path.exists(sim_path):
        print('Already computed', sim_path)
        return

    worker = functools.partial(_worker, tier, task_id, num_workers, num_actions)
    pool = multiprocessing.Pool(num_workers)
    all_statuses = sum(pool.map(worker, range(num_workers)), [])
    all_statuses = np.array(all_statuses, 'int8')
    print('Writing to', sim_path)
    joblib.dump(all_statuses, sim_path)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--num-workers', type=int, required=True)
    parser.add_argument('--task-id', required=True)
    parser.add_argument('--tier',
                        choices=phyre.simulation_cache.TIERS,
                        required=True)
    parser.add_argument('--num-actions',
                        type=int,
                        default=phyre.simulation_cache.DEFAULT_NUM_ACTIONS)
    main(**vars(parser.parse_args()))
