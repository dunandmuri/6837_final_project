#include "pendulumsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>
// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
const float g = 9.8;
const float mass = 1.0;
const float drag = 0.2;
const float springK = 30.0;
const float rest = 0.25;
PendulumSystem::PendulumSystem()
{

    // TODO 4.2 Add particles for simple pendulum
    // TODO 4.3 Extend to multiple particles

    // To add a bit of randomness, use e.g.
    // float f = rand_uniform(-0.5f, 0.5f);
    // in your initial conditions.
	Vector3f initialPosition = { 0, 1, 0 };
	Vector3f decrement = { 0, -0.25, 0 };
	for (int i = 0; i < NUM_PARTICLES; i++) {
		m_vVecState.push_back(initialPosition + i * decrement); //position decreases every timestep
		m_vVecState.push_back(Vector3f{ 0.0, 0.0, 0.0 }); // velocity = 0
	}


}

std::vector<Vector3f> getSpringForces(std::vector<Vector3f> state) {
	std::vector<Vector3f> forces = {};

	for (int i = 0; i < state.size() - 2; i+=2) {
		Vector3f d = state[i]-state[i+2];
		Vector3f force = -1 *springK*(d.abs() - rest)*d / d.abs();
		forces.push_back(force);

	}
	return forces;
}
std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    //std::vector<Vector3f> f(state.size());
	std::vector<Vector3f> f={};

	// state is the m_v_vect !!!
	// state: pos, vel, pos, vel
	// f:     vel, accel, vel , accel
	// velocity, accel = 0 for first point
    // TODO 4.1: implement evalF
    //  - gravity
    //  - viscous drag
    //  - springs
	std::vector<Vector3f> springForce = getSpringForces(state);

	f.push_back(state[1]); // velocity for state[0]
	f.push_back(Vector3f{ 0.0, 0.0, 0.0 }); //acceleration for state[0]
	for (int i = 2; i < state.size(); i+=2) {
		Vector3f springForceBelow;
		if (i == state.size() - 2) {			
			springForceBelow = { 0.0, 0.0, 0.0 };
		}
		else {
			springForceBelow = springForce[i/2];
		}
		Vector3f springForceAbove = -1 * springForce[i/2 - 1];

		Vector3f gravityForce = { 0, -g, 0 };
		Vector3f dragForce = -1*drag*state[i+1];
		Vector3f accel = (springForceAbove + springForceBelow + gravityForce + dragForce) / mass;
		f.push_back(state[i+1]); // velocity
		f.push_back(accel); //acceleration

	}


	

    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{

    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // TODO 4.2, 4.3

    // example code. Replace with your own drawing  code
	//gl.updateModelMatrix(Matrix4f::translation(Vector3f(-0.5, 1.0, 0)));
    //drawSphere(0.075f, 10, 10);

	for (int i = 0; i < m_vVecState.size(); i+=2) {
		Vector3f pos(m_vVecState[i]); //YOUR PARTICLE POSITION
		gl.updateModelMatrix(Matrix4f::translation(pos));
		drawSphere(0.075f, 10, 10);

	}


}
