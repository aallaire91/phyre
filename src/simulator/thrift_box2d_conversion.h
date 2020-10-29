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
#ifndef THRIFT_BOX2D_CONVERSION_H
#define THRIFT_BOX2D_CONVERSION_H
#include <memory>

#include "Box2D/Box2D.h"
#include "gen-cpp/scene_types.h"
#include "image_to_box2d.h"

constexpr float PIXELS_IN_METER = 6.0;
constexpr float DEFAULT_GRAVITY = -9.8;
constexpr float DEFAULT_DENSITY = 0.25;
// Friction is used to make objects slide along each other realistically. Box2D
// supports static and dynamic friction, but uses the same parameter for both.
// The friction parameter is usually set between 0 and 1, but can be any
// non-negative value. A friction value of 0 turns off friction and a value of
// 1 makes the friction strong.
constexpr float DEFAULT_FRICTION = 0.5;
// How bouncy the objects are from 0 to 1.
constexpr float DEFAULT_RESTITUTION = 0.2;
// Damping is used to reduce the world velocity of bodies. Damping is different
// than friction because friction only occurs with contact. Damping is not a
// replacement for friction and the two effects should be used together.
// Damping parameters should be between 0 and infinity, with 0 meaning no
// damping, and infinity meaning full damping. Normally you will use a damping
// value between 0 and 0.1. I generally do not use linear damping because it
// makes bodies look like they are floating.
constexpr float DEFAULT_ANGULAR_DAMPING = 0.01;
constexpr float DEFAULT_LINEAR_DAMPING = 0.0;

::scene::Physics default_physics() ;

struct Box2dData {
  enum ObjectType { GENERAL, USER, BOUNDING_BOX };
  size_t object_id;
  ObjectType object_type;
};

class b2WorldWithData : public b2World {
 public:
  using b2World::b2World;

  ~b2WorldWithData() {}

  Box2dData* CreateData() {
    _data.emplace_back(new Box2dData);
    return _data.back().get();
  }

 private:
  std::vector<std::unique_ptr<Box2dData>> _data;
};

std::unique_ptr<b2WorldWithData> convertSceneToBox2dWorld(
    const ::scene::Scene& scene, ::scene::Physics physics);

std::unique_ptr<b2WorldWithData> convertSceneToBox2dWorld_with_bounding_boxes(
    const ::scene::Scene& scene,::scene::Physics physics);

::scene::Scene updateSceneFromWorld(const ::scene::Scene& scene,
                                    const b2WorldWithData& world);

std::vector<::scene::Body> convertInputToSceneBodies(
    const std::vector<::scene::IntVector>& input_points,
    const std::vector<::scene::Body>& scene_bodies, const unsigned int& height,
    const unsigned int& width,::scene::Physics physics);

::scene::Shape p2mShape(const ::scene::Shape& shape);

class NoisyContactListener : public b2ContactListener
{
   // TODO
};

#endif  // THRIFT_BOX2D_CONVERSION_H
