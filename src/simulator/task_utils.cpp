// Copyright (c) Facebook, Inc. and its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "task_utils.h"
#include "task_validation.h"
#include "thrift_box2d_conversion.h"
#include <random>
#include <iostream>

namespace {
    std::default_random_engine generator;

struct SimulationRequest {
  int maxSteps;
  int stride;
};

double sample_gaussian(double mean,double std) {
    std::normal_distribution<double> distribution(mean,std);
    return distribution(generator);
}

double truncate(double x,double min,double max) {
    if (x<min) {
        x = min;
    } else if (x > max) {
        x = max;
    }
    return x;
}

// Runs simulation for the scene. If task is not nullptr, is-task-solved checks
// are performed.
::task::TaskSimulation simulateTask(const ::scene::Scene &scene,
                                    const SimulationRequest &request,
                                    const ::task::Task *task,::scene::NoisyPhysics noisy_physics) {

    ::scene::Physics physics = ::scene::Physics();
    if (noisy_physics.noise_friction>0) {
        physics.friction = truncate(sample_gaussian(noisy_physics.friction,noisy_physics.noise_friction),0.0,1.0);
    } else {
        physics.friction = DEFAULT_FRICTION;
    }
    if (noisy_physics.noise_restitution>0) {
        physics.restitution = truncate(sample_gaussian(noisy_physics.restitution,noisy_physics.noise_restitution),0.0,1.0);
    } else {
        physics.restitution = DEFAULT_RESTITUTION;
    }
    if (noisy_physics.noise_density>0) {
        physics.density = truncate(sample_gaussian(noisy_physics.density,noisy_physics.noise_density),0.0,10.0);
    } else {
        physics.density = DEFAULT_DENSITY;
    }
    if (noisy_physics.noise_gravity>0) {
        physics.gravity = truncate(sample_gaussian(noisy_physics.gravity,noisy_physics.noise_gravity),-20,0.0);
    } else {
        physics.gravity = DEFAULT_GRAVITY;
    }
    if (noisy_physics.noise_damping>0) {
        physics.angularDamping=  truncate(sample_gaussian(noisy_physics.angularDamping,noisy_physics.noise_damping),0.0,0.1);
        physics.linearDamping= truncate(sample_gaussian(noisy_physics.linearDamping,noisy_physics.noise_damping),0.0,0.1);
    } else {
        physics.angularDamping=  DEFAULT_ANGULAR_DAMPING;
        physics.linearDamping= DEFAULT_LINEAR_DAMPING;
    }

  std::unique_ptr<b2WorldWithData> world = convertSceneToBox2dWorld_with_bounding_boxes(scene,physics);

  unsigned int continuousSolvedCount = 0;
  std::vector<::scene::Scene> scenes;
  std::vector<bool> solveStateList;
  bool solved = false;
  int step = 0;

  // For different relations number of steps the condition should hold varies.
  // For NOT_TOUCHING relation one of three should be true:
  //   1. Objects are touching at the beginning and then not touching for
  //   kStepsForSolution steps.
  //   2. Objects are not touching at the beginning, touching at some point of
  //   simulation and then not touching for kStepsForSolution steps.
  //   3. Objects are not touching whole sumulation.
  // For TOUCHING_BRIEFLY a single touching is allowed.
  // For all other relations the condition must hold for kStepsForSolution
  // consequent steps.
  bool lookingForSolution =
      (task == nullptr || !isTaskInSolvedState(*task, *world) ||
       task->relationships.size() != 1 ||
       task->relationships[0] != ::task::SpatialRelationship::NOT_TOUCHING);
  const bool allowInstantSolution =
      (task != nullptr && task->relationships.size() == 1 &&
       task->relationships[0] == ::task::SpatialRelationship::TOUCHING_BRIEFLY);
  for (; step < request.maxSteps; step++) {
    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    world->Step(kTimeStep, kVelocityIterations, kPositionIterations);
    if (request.stride > 0 && step % request.stride == 0) {
      scenes.push_back(updateSceneFromWorld(scene, *world));
    }
    if (task == nullptr) {
      solveStateList.push_back(false);
    } else {
      solveStateList.push_back(isTaskInSolvedState(*task, *world));
      if (solveStateList.back()) {
        continuousSolvedCount++;
        if (lookingForSolution) {
          if (continuousSolvedCount >= kStepsForSolution ||
              allowInstantSolution) {
            solved = true;
            break;
          }
        }
      } else {
        lookingForSolution = true;  // Task passed through non-solved state.
        continuousSolvedCount = 0;
      }
    }
  }

  if (!lookingForSolution && continuousSolvedCount == solveStateList.size()) {
    // See condition 3) for NOT_TOUCHING relation above.
    solved = true;
  }

  {
    std::vector<bool> stridedSolveStateList;
    if (request.stride > 0) {
      for (size_t i = 0; i < solveStateList.size(); i += request.stride) {
        stridedSolveStateList.push_back(solveStateList[i]);
      }
    }
    stridedSolveStateList.swap(solveStateList);
  }

  ::task::TaskSimulation taskSimulation;
  taskSimulation.__set_sceneList(scenes);
  taskSimulation.__set_stepsSimulated(step);
  if (task != nullptr) {
    taskSimulation.__set_solvedStateList(solveStateList);
    taskSimulation.__set_isSolution(solved);
  }

  return taskSimulation;
}
}  // namespace



std::vector<::scene::Scene> simulateScene(const ::scene::Scene &scene,
                                          const int num_steps,::scene::NoisyPhysics noisy_physics) {
  const SimulationRequest request{num_steps, 1};
  const auto simulation = simulateTask(scene, request, /*task=*/nullptr,noisy_physics);
  return simulation.sceneList;
}

::task::TaskSimulation simulateTask(const ::task::Task &task,
                                    const int num_steps, const int stride,::scene::NoisyPhysics noisy_physics) {
  const SimulationRequest request{num_steps, stride};
  return simulateTask(task.scene, request, &task,noisy_physics);
}
