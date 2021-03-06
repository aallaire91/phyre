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
"""Template task in which one ball must be hit into a container."""
import numpy as np
import phyre.creator as creator_lib
import phyre.virtual_tools as vt


@creator_lib.define_task_template(
    seed=range(1000),
    version="2",
    search_params=dict(
        max_search_tasks=300,
        required_flags=['BALL:GOOD_STABLE'],
        excluded_flags=['BALL:TRIVIAL'],
    ),
)
def build_task(C, seed):
    rng = np.random.RandomState(seed=seed)
    tableWidth = [100, 400]
    tableHeight = [100, 400]
    ballSize = [10, 30]
    goalHeight = [50, 200]
    flip_lr = rng.uniform(0, 1) < 0.5

    tableDims = [
        rng.uniform(tableWidth[0], tableWidth[1]),
        rng.uniform(tableHeight[0], tableHeight[1])
    ]
    ballRad = rng.uniform(ballSize[0], ballSize[1])
    ballPos = rng.uniform(ballRad + 5, tableDims[0] - ballRad - 5)
    goalDims = [
        rng.uniform(ballRad * 2 + 5, vt.VT_SCALE - tableDims[0] - 30),
        rng.uniform(goalHeight[0], tableDims[1] - 10)
    ]
    goalXPos = rng.uniform(tableDims[0] + 10, (vt.VT_SCALE - goalDims[0] - 10))

    table = vt.add_box(C, [0, 0, tableDims[0], tableDims[1]],
                       False,
                       flip_lr=flip_lr)

    if flip_lr:
        center_x = vt.flip_left_right(ballPos)
    else:
        center_x = ballPos

    ball = C.add('dynamic ball',
                 ballRad * 2 / vt.VT_SCALE,
                 center_x=center_x * C.scene.width / vt.VT_SCALE,
                 bottom=tableDims[1] * C.scene.height / vt.VT_SCALE)
    goalVerts = [[goalXPos, goalDims[1]], [goalXPos, 5],
                 [goalXPos + goalDims[0], 5],
                 [goalXPos + goalDims[0], goalDims[1]]]
    container_id, bbid = vt.add_container(C,
                                          goalVerts,
                                          10,
                                          False,
                                          True,
                                          flip_lr=flip_lr)
    C.update_task(body1=ball,
                  body2=container_id,
                  relationships=[C.SpatialRelationship.TOUCHING])

    C.set_meta(C.SolutionTier.VIRTUAL_TOOLS)
