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

include "shared.thrift"

namespace cpp scene

// General objects.
struct Vector {
  1: required double x,
  2: required double y,
}

struct IntVector {
  1: required i32 x,
  2: required i32 y,
}

enum BodyType {
  STATIC = 1,
  DYNAMIC = 2,
}

enum ShapeType {
  UNDEFINED = 0,
  BALL = 1,
  BAR = 2,
  JAR = 3,
  STANDINGSTICKS = 4,
}

// Shapes.
struct Polygon {
  // Vertices are relative for the position. Must be in counter-clockwise
  // order.
  1: required list<Vector> vertices,
}

struct Circle {
  1: required double radius,
}

union Shape {
  1: Polygon polygon,
  2: Circle circle,
}

struct Body {
  1: required Vector position,
  2: required BodyType bodyType,
  // Angle relative to position in radians.
  3: optional double angle = 0.0,
  4: optional Vector linVelocity,
  5: optional double angVelocity = 0.0,
  6: optional list<Shape> shapes,
  7: optional shared.Color color,
  8: optional ShapeType shapeType = SHAPE_TYPE.UNDEFINED,
  9: optional double diameter

}


// Convex polygon with absolutely positions vertices.
struct AbsoluteConvexPolygon {
  1: optional list<Vector> vertices,
}

// For compatibility reasons this is a separate class that used only for
// UserInput.
struct CircleWithPosition {
  1: optional Vector position,
  2: optional double radius,
}

struct UserInput {
  1: list<AbsoluteConvexPolygon> polygons,
  2: list<CircleWithPosition> balls,
  3: list<i32> flattened_point_list, // x[0], y[0], x[1], y[1], ...
}

enum UserInputStatus {
  UNDEFINED = 0,
  NO_OCCLUSIONS = 1,
  HAD_OCCLUSIONS = 2,
}

struct Scene {
  1: optional list<Body> bodies,
  2: optional list<Body> user_input_bodies,
  3: required i32 width,
  4: required i32 height,
  5: optional UserInputStatus user_input_status = UserInputStatus.UNDEFINED,

}

struct Solution {
  // Points marked by the user.
  1: optional list<IntVector> points,
}

struct Image {
  // Row major matrix of colors.
  1: optional list<i32> values,
  2: optional i32 height,
  3: optional i32 width,
}

struct Physics {
  6: double friction =  0.5 ,
  7: double restitution = 0.2,
  8: double density = 0.25,
  9: double angularDamping = 0.01,
  10:double linearDamping =  0.0,
  11: double gravity = -9.8,
}

struct NoisyPhysics {
   1: double noise_friction = 0.0,
   2: double noise_restitution  = 0.0,
   3: double noise_density  = 0.0,
   4: double noise_damping = 0.0
   5: double noise_gravity  = 0.0,
   6: double noise_contact_mag  = 0.0,
   7: double noise_contact_dir  = 0.0,
  8: double friction = 0.5 ,
  9: double restitution = 0.2,
  10: double density = 0.25,
  11: double angularDamping = 0.01,
  12:double linearDamping = 0.0,
  13: double gravity = -9.8,
}

