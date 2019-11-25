#include "timestepper.h"

#include <cstdio>

void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1 
	std::vector<Vector3f> deriv = particleSystem->evalF(particleSystem->getState());
	std::vector<Vector3f> finalState = {};
	for (int i = 0; i < deriv.size(); i += 1) {
		finalState.push_back(particleSystem->getState()[i] + stepSize * deriv[i]);
	}
	particleSystem->setState(finalState);
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1 
	std::vector<Vector3f> f0s = particleSystem->evalF(particleSystem->getState());
	std::vector<Vector3f> finalState = {};
	std::vector<Vector3f> f1s = {};

	for (int i = 0; i < f0s.size(); i += 1) {
		Vector3f f1 = particleSystem->getState()[i] + stepSize * f0s[i];
		f1s.push_back(f1);
	}
	std::vector<Vector3f> f1 = particleSystem->evalF(f1s);

	for (int i = 0; i < f0s.size(); i += 1) {
		Vector3f inter = particleSystem->getState()[i];
		finalState.push_back(particleSystem->getState()[i] + stepSize / 2.0*(f0s[i] + f1[i]));
	}
	particleSystem->setState(finalState);
}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	std::vector<Vector3f> k1 = particleSystem->evalF(particleSystem->getState()); //f(X(t))
	std::vector<Vector3f> k2 = {};
	for (int i = 0; i < k1.size(); i += 1) {
		k2.push_back(particleSystem->getState()[i] + stepSize * .5* k1[i]);
	}
	k2 = particleSystem->evalF(k2);


	std::vector<Vector3f> k3 = {};
	for (int i = 0; i < k1.size(); i += 1) {
		k3.push_back(particleSystem->getState()[i] + stepSize * .5* k2[i]);
	}
	k3 = particleSystem->evalF(k3);


	std::vector<Vector3f> k4 = {};
	for (int i = 0; i < k2.size(); i += 1) {
		k4.push_back(particleSystem->getState()[i] + stepSize * k3[i]);
	}
	k4 = particleSystem->evalF(k4);

	std::vector<Vector3f> total = {};
	for (int i = 0; i < k4.size(); i += 1) {
		total.push_back(particleSystem->getState()[i] + stepSize * 1 / 6.0*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]));
	}

	particleSystem->setState(total);


}

