#include "integrator.h"

#include "configs.h"

void ExplicitEuler::integrate(const std::vector<Particles *> &particles, std::function<void(void)>) const {
  // TODO: Integrate velocity and acceleration
  //   1. Integrate velocity.
  //   2. Integrate acceleration.
  //   3. You should not compute position using acceleration. Since some part only update velocity. (e.g. impulse)
  // Note:
  //   1. You don't need the simulation function in explicit euler.
  //   2. You should do this first because it is very simple. Then you can chech your collision is correct or not.
  //   3. This can be done in 2 lines. (Hint: You can add / multiply all particles at once since it is a large matrix.)
  
  for (const auto &p : particles) {
    // Write code here!
    // y(n+1) = y(n) + h * f(x(n), y(n)) 
    // where h = (x(n) – x(0)) / n 
    // x(t+h) = x(t) + h * f(x, t) => Xn+1 = Xn + h X'n
  
    float h = deltaTime;
    Eigen::Matrix4Xf x_y = p->position();
    Eigen::Matrix4Xf v_y = p->velocity();
    Eigen::Matrix4Xf a_y = p->acceleration();
    
    // x(t+h) = x(t) + h * f(x, t) => Xn+1 = Xn + h X'n
    Eigen::Matrix4Xf x_y_next = x_y + h * (v_y);
    Eigen::Matrix4Xf v_y_next = v_y + h * (a_y);

    // update the position and velocity
    p->position() = x_y_next;
    p->velocity() = v_y_next;
  }
  
}

void ImplicitEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // Write code here!
  //Particles initialCloth = cloth.particles();
  //Particles initialSpheres = spheres.particles();
  float h = deltaTime;
  // Back up the data
  auto particlesTmp = particles; 
  simulateOneStep();

  int i =0;
  for (const auto &p : particles) {

    Eigen::Matrix4Xf x_y = particlesTmp[i]->position();
    Eigen::Matrix4Xf v_y = particlesTmp[i]->velocity();
    Eigen::Matrix4Xf a_y = particlesTmp[i]->acceleration();
      
    Eigen::Matrix4Xf x_y_next = x_y + h * particles[i]->velocity();
    Eigen::Matrix4Xf v_y_next = v_y + h * particles[i]->acceleration();

    p->position() = x_y_next;
    p->velocity() = v_y_next;
    ++i;
  }
  
  return ;
}

void MidpointEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // Write code here!

  // init the deltaTime
  float h = deltaTime;
  float deltaTime_org = deltaTime;

  // Back up the data
  auto particlesTmp = particles; 
  //deltaTime = (deltaTime_org / 2);
      
  // Go to t0 + h/2
  for (const auto &p : particles) {
    p->position() = p->position() + p->velocity()*h/2; 
    p->velocity() = p->velocity() + p->acceleration()*h/2;
  }

  simulateOneStep();

  int i=0;
  for (const auto &p : particles) {

    // t0
    Eigen::Matrix4Xf x_y = particlesTmp[i]->position();
    Eigen::Matrix4Xf v_y = particlesTmp[i]->velocity();

    // t0 + h/2
    Eigen::Matrix4Xf fmid_x = p->velocity();
    Eigen::Matrix4Xf fmid_v = p->acceleration();

    // t0 + h
    Eigen::Matrix4Xf x_y_next = x_y + h * (fmid_x);
    Eigen::Matrix4Xf v_y_next = v_y + h * (fmid_v);

    // update the position and velocity
    p->position() = x_y_next;
    p->velocity() = v_y_next;
    ++i;
  }
  //deltaTime = deltaTime_org;
  //simulateOneStep();

  return ;
}

void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Compute k1, k2, k3, k4
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // Write code here!
  //float x0, y0, xn, h, yn, k1, k2, k3, k4, k;
  
  float h = deltaTime;
  float deltaTime_org = deltaTime;

  // k1
  auto particles_1 = particles; 

  // k2
  int i = 0;
  for (const auto &p : particles) {
   p->position() = particles_1[i]->position() + (h * particles_1[i]->velocity()) / 2;
   p->velocity() = p->velocity() + (h * particles_1[i]->acceleration()) / 2;
   ++i;
  }
  deltaTime = deltaTime_org / 2;
  simulateOneStep(); 
  auto particles_2 = particles; 

  // k3
  i = 0;
  for (const auto &p : particles) {
   p->position() = particles_1[i]->position() + (h * particles_1[i]->velocity()) / 2;
   p->velocity() = p->velocity() + (h * particles_2[i]->acceleration()) / 2;
   ++i;
  }
  deltaTime = 0;
  simulateOneStep(); 
  auto particles_3 = particles; 

  // k4
  i = 0;
  for (const auto &p : particles) {
   p->position() = particles_1[i]->position() + (h * particles_1[i]->velocity()) / 2;
   p->velocity() = p->velocity() + h * particles_3[i]->acceleration();
   ++i;
  }
  deltaTime = deltaTime_org / 2;
  simulateOneStep(); 
  auto particles_4 = particles; 

  deltaTime = deltaTime_org;

  // sum 
  Eigen::Matrix4Xf x_k1, x_k2, x_k3, x_k4;
  Eigen::Matrix4Xf v_k1, v_k2, v_k3, v_k4;
  Eigen::Matrix4Xf x_y_next;
  Eigen::Matrix4Xf v_y_next; 
  i = 0; 
  for (const auto &p : particles) {
    
    x_k1 = h * particles_1[i]->velocity();
    v_k1 = h * particles_1[i]->acceleration();
    
    x_k2 = h * particles_1[i]->velocity();
    v_k2 = h * particles_1[i]->acceleration();
    
    x_k3 = h * particles_1[i]->velocity();
    v_k3 = h * particles_1[i]->acceleration();
     
    x_k4 = h * particles_1[i]->velocity();
    v_k4 = h * particles_1[i]->acceleration();
    
    x_y_next = particles_1[i]->position() + (1/6) * (x_k1 + 2*x_k2 + 2*x_k3 + x_k4);
    v_y_next = particles_1[i]->velocity() + (1/6) * (v_k1 + 2*v_k2 + 2*v_k3 + v_k4);

    p->position() = x_y_next;
    p->velocity() = v_y_next; 
    ++i;
  }


  return ;
}
