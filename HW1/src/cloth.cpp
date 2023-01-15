#include "cloth.h"
#include <Eigen/Geometry>

#include "configs.h"
#include "sphere.h"

//Include math library
#include <math.h>
//Include iostream library
#include <iostream>
#include <string> 
using namespace std;

Cloth::Cloth() : Shape(particlesPerEdge * particlesPerEdge, particleMass) {
  initializeVertex();
  initializeSpring();
}

void Cloth::draw(DrawType type) const {
  vao.bind();
  positionBuffer.load(0, 4 * particlesPerEdge * particlesPerEdge * sizeof(GLfloat), _particles.getPositionData());
  const ElementArrayBuffer* currentEBO = nullptr;
  switch (type) {
    case DrawType::PARTICLE: [[fallthrough]];
    case DrawType::FULL: currentEBO = &ebo; break;
    case DrawType::STRUCTURAL: currentEBO = &structuralSpring; break;
    case DrawType::SHEAR: currentEBO = &shearSpring; break;
    case DrawType::BEND: currentEBO = &bendSpring;
  }
  currentEBO->bind();
  GLsizei indexCount = static_cast<GLsizei>(currentEBO->size() / sizeof(GLuint));
  if (type == DrawType::FULL)
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
  else if (type == DrawType::PARTICLE)
    glDrawArrays(GL_POINTS, 0, particlesPerEdge * particlesPerEdge);
  else
    glDrawElements(GL_LINES, indexCount, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Cloth::initializeVertex() {
  float wStep = 2.0f * clothWidth / (particlesPerEdge - 1);
  float hStep = 2.0f * clothHeight / (particlesPerEdge - 1);

  int current = 0;
  for (int i = 0; i < particlesPerEdge; ++i) {
    for (int j = 0; j < particlesPerEdge; ++j) {
      _particles.position(current++) = Eigen::Vector4f(-clothWidth + j * wStep, 0, -clothHeight + i * hStep, 1);
    }
  }
  // Four corners will not move
  _particles.mass(0) = 0.0f;
  _particles.mass(particlesPerEdge - 1) = 0.0f;
  _particles.mass(particlesPerEdge * (particlesPerEdge - 1)) = 0.0f;
  _particles.mass(particlesPerEdge * particlesPerEdge - 1) = 0.0f;

  std::vector<GLuint> indices;
  indices.reserve((particlesPerEdge - 1) * (2 * particlesPerEdge + 1));
  for (int i = 0; i < particlesPerEdge - 1; ++i) {
    int offset = i * (particlesPerEdge);
    for (int j = 0; j < particlesPerEdge - 1; ++j) {
      indices.emplace_back(offset + j);
      indices.emplace_back(offset + j + particlesPerEdge);
      indices.emplace_back(offset + j + 1);

      indices.emplace_back(offset + j + 1);
      indices.emplace_back(offset + j + particlesPerEdge);
      indices.emplace_back(offset + j + particlesPerEdge + 1);
    }
  }

  int vboSize = particlesPerEdge * particlesPerEdge * sizeof(GLfloat);
  positionBuffer.allocate_load(vboSize * 4, _particles.getPositionData());
  normalBuffer.allocate(particlesPerEdge * particlesPerEdge * sizeof(float) * 4);

  ebo.allocate_load(indices.size() * sizeof(GLuint), indices.data());

  vao.bind();
  positionBuffer.bind();
  vao.enable(0);
  vao.setAttributePointer(0, 4, 4, 0);
  normalBuffer.bind();
  vao.enable(1);
  vao.setAttributePointer(1, 4, 4, 0);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


void Cloth::initializeSpring() {
  // TODO: Connect particles with springs.
  //   1. Compute spring length per type.
  //   2. Iterate the particles. Push spring objects into `_springs` vector
  // Note:
  //   1. The particles index:
  //   ===============================================
  //   0 1 2 3 ... particlesPerEdge
  //   particlesPerEdge + 1 ....
  //   ... ... particlesPerEdge * particlesPerEdge - 1
  //   ===============================================
  // Here is a simple example which connects the horizontal structrual springs.
  //float structrualLength = (_particles.position(0) - _particles.position(1)).norm();
  //sample code
  /*
  for (int i = 0; i < particlesPerEdge; ++i) {
    for (int j = 0; j < particlesPerEdge - 1; ++j) {
      int index = i * particlesPerEdge + j;
      _springs.emplace_back(index, index + 1, structrualLength, Spring::Type::STRUCTURAL);
    }
  }
  */
  // Write code here!
  
  //Struct
  float structrualLength = (_particles.position(0) - _particles.position(1)).norm();
  // This part connects the horizontal structrual springs. 
  for (int i = 0; i < particlesPerEdge; ++i) {
    for (int j = 0; j < particlesPerEdge - 1; ++j) {
      int index = i * particlesPerEdge + j;
      _springs.emplace_back(index, index + 1, structrualLength, Spring::Type::STRUCTURAL);
    }
  }
  // This part connects the vertical structrual springs. 
  for (int i = 0; i < particlesPerEdge - 1; ++i) {
    for (int j = 0; j < particlesPerEdge; ++j) {
       int index = i * particlesPerEdge + j;
       _springs.emplace_back(index, index + particlesPerEdge, structrualLength, Spring::Type::STRUCTURAL);
    }
  }

  //Shear
  float normal_length = (_particles.position(0) - _particles.position(1)).norm();
  float shearLength = (_particles.position(0) - _particles.position(particlesPerEdge + 1)).norm();
  // This part connects the spring from to upper left to lower rignt.
  for (int i = 0; i < particlesPerEdge - 1; ++i) {
    for (int j = 0; j < particlesPerEdge -1; ++j) {
       int index = i * particlesPerEdge + j;
       _springs.emplace_back(index, index + particlesPerEdge +1, shearLength, Spring::Type::SHEAR);
    }
  }
  // This part connects the spring from to upper right to lower left.
  for (int i = 0; i < particlesPerEdge - 1; ++i) {
    for (int j = 1; j < particlesPerEdge; ++j) {
       int index = i * particlesPerEdge + j;
       _springs.emplace_back(index, index + particlesPerEdge -1, shearLength, Spring::Type::SHEAR);
    }
  }

  //Bending
  float bendingLength = 2 * normal_length;
  // This part connects the horizontal bending springs. 
  for (int i = 0; i < particlesPerEdge; ++i) {
    for (int j = 0; j < particlesPerEdge - 2; ++j) {
      int index = i * particlesPerEdge + j;
      _springs.emplace_back(index, index + 2, bendingLength, Spring::Type::BEND);
    }
  }
  // This part connects the vertical bending springs. 
  for (int i = 0; i < particlesPerEdge - 2; ++i) {
    for (int j = 0; j < particlesPerEdge; ++j) {
       int index = i * particlesPerEdge + j;
       _springs.emplace_back(index, index + 2*particlesPerEdge, bendingLength, Spring::Type::BEND);
    }
  }


  std::vector<GLuint> structrualIndices, shearIndices, bendIndices;
  for (const auto& spring : _springs) {
    switch (spring.type()) {
      case Spring::Type::STRUCTURAL:
        structrualIndices.emplace_back(spring.startParticleIndex());
        structrualIndices.emplace_back(spring.endParticleIndex());
        break;
      case Spring::Type::SHEAR:
        shearIndices.emplace_back(spring.startParticleIndex());
        shearIndices.emplace_back(spring.endParticleIndex());
        break;
      case Spring::Type::BEND:
        bendIndices.emplace_back(spring.startParticleIndex());
        bendIndices.emplace_back(spring.endParticleIndex());
        break;
    }
  }
  structuralSpring.allocate_load(structrualIndices.size() * sizeof(GLuint), structrualIndices.data());
  shearSpring.allocate_load(shearIndices.size() * sizeof(GLuint), shearIndices.data());
  bendSpring.allocate_load(bendIndices.size() * sizeof(GLuint), bendIndices.data());
}


void Cloth::computeSpringForce() {
  // TODO: Compute spring force and damper force for each spring.
  //   1. Read the start and end index from spring
  //   2. Use _particles.position(i) to get particle i's position.
  //   3. Modify particles' acceleration a = F / m;
  // Note:
  //   1. Use _particles.inverseMass(i) to get 1 / m can deal with m == 0. Which will returns 0.
  // Hint:
  //   1. Use a.norm() to get length of a.
  //   2. Use a.normalize() to normalize a inplace.
  //          a.normalized() will create a new vector.
  //   3. Use a.dot(b) to get dot product of a and b.
  for (const auto& spring : _springs) {

    // Write code here!
    // 1. Read the start and end index from spring
    unsigned int a   = spring.startParticleIndex();
    unsigned int b   = spring.endParticleIndex();
    float r = 0;
    r = spring.length();

    // 2. Use _particles.position(i) to get particle i's position.
    Eigen::Vector4f  x_a = _particles.position(a);
    Eigen::Vector4f  x_b = _particles.position(b);
    Eigen::Vector4f  v_a = _particles.velocity(a);
    Eigen::Vector4f  v_b = _particles.velocity(b);

    //3.Modify particles' acceleration a = F / m and update the a of each particle.
    // n_ary_forces
    // a_spring forces
    Eigen::Vector4f a_spring_forces;
    a_spring_forces.setZero();
    Eigen::Vector4f x_a_b = x_a - x_b;
    a_spring_forces = (-1) * springCoef * ( x_a_b.norm() - r) * ( x_a_b / x_a_b.norm() );

    // a_damper forces
    Eigen::Vector4f a_damper_forces;
    a_damper_forces.setZero();
    Eigen::Vector4f v_a_b = v_a - v_b;
    float a_v_dot_x = v_a_b.dot(x_a_b);
    a_damper_forces = (-1) * damperCoef * ( a_v_dot_x / x_a_b.norm() ) * ( x_a_b / x_a_b.norm() );
   
    // calculate the a
    Eigen::Vector4f fa;
    fa.setZero();
    fa = a_spring_forces + a_damper_forces;
    float a_inverse_mass = _particles.inverseMass(a);      
    Eigen::Vector4f a_acceleration = fa * a_inverse_mass;


    // b_spring forces
    Eigen::Vector4f b_spring_forces;
    b_spring_forces.setZero();
    Eigen::Vector4f x_b_a = x_b - x_a;
    b_spring_forces = (-1) * springCoef * ( x_b_a.norm() - r) * ( x_b_a / x_b_a.norm() );

    // b_damper forces
    Eigen::Vector4f b_damper_forces;
    b_damper_forces.setZero();
    Eigen::Vector4f v_b_a = v_b - v_a;
    float b_v_dot_x = v_b_a.dot(x_b_a);
    b_damper_forces = (-1) * damperCoef * ( b_v_dot_x / x_b_a.norm() ) * ( x_b_a / x_b_a.norm() );

    // calculate the b
    Eigen::Vector4f fb;
    fb.setZero();
    fb = b_spring_forces + b_damper_forces;
    float b_inverse_mass = _particles.inverseMass(b);      
    Eigen::Vector4f b_acceleration = fb * b_inverse_mass;

    _particles.acceleration(a) += a_acceleration;
    _particles.acceleration(b) += b_acceleration;
  }
}

void Cloth::collide(Shape* shape) { shape->collide(this); }
void Cloth::collide(Spheres* sphere) { sphere->collide(this); }

void Cloth::computeNormal() {
  static Eigen::Matrix<float, 4, particlesPerEdge * particlesPerEdge> normals;
  normals.setZero();
  for (int i = 0; i < particlesPerEdge - 1; ++i) {
    int offset = i * (particlesPerEdge);
    for (int j = 0; j < particlesPerEdge - 1; ++j) {
      Eigen::Vector4f v1 = _particles.position(offset + j) - _particles.position(offset + j + particlesPerEdge);
      Eigen::Vector4f v2 = _particles.position(offset + j + 1) - _particles.position(offset + j + particlesPerEdge);
      Eigen::Vector4f n1 = v2.cross3(v1);
      normals.col(offset + j) += n1;
      normals.col(offset + j + 1) += n1;
      normals.col(offset + j + particlesPerEdge) += n1;

      Eigen::Vector4f v3 =
          _particles.position(offset + j + particlesPerEdge + 1) - _particles.position(offset + j + particlesPerEdge);
      Eigen::Vector4f n2 = v3.cross3(v2);
      normals.col(offset + j + 1) += n2;
      normals.col(offset + j + particlesPerEdge) += n2;
      normals.col(offset + j + particlesPerEdge + 1) += n2;
    }
  }
  normals.colwise().normalize();
  normalBuffer.load(0, particlesPerEdge * particlesPerEdge * sizeof(float) * 4, normals.data());
}
