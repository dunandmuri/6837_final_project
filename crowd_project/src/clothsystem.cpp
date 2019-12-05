#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

 // your system should at least contain 8x8 particles.
const int W = 8;
const int H = 8;
const float g = 5.0;
const float mass = 1.0;
const float drag = 3.0;
const float kStruct = 350.0;
const float kShear = 400.0;
const float kFlex = 50.0;
const float diff = 0.5;  // how far apart each adjacent particle is
const float restStruct = diff;
const float restShear = sqrt(diff*diff*2);
const float restFlex = diff * 2;

// small personal space = 0.4
// large personal space = 0.6
const float restPersonal = 0.6;

// low chaos = 2
// high chaos = 10
const int xVariation = 2;

ClothSystem::ClothSystem()
{
    // TODO 5. Initialize m_vVecState with cloth particles. 
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting
	Vector3f initialPosition = { 0, 0, 0 };
	Vector3f Ydecrement = { 0, -1*diff, 0 };
	Vector3f Xincrement = { diff, 0, 0 };
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			m_vVecState.push_back(initialPosition + j * Xincrement + i * Ydecrement ); //position decreases every timestep
			m_vVecState.push_back(Vector3f{ 0.0, 0.5, 0.0 }); // velocity = 0
		}
	}
}

Vector3f getPersonalSpaceForce(Vector3f boid, Vector3f other) { //find force acting on boid from other
	Vector3f d = boid - other;
	Vector3f force = { 0,0,0 };
	if (d.abs() < restPersonal) {
		force = -1 * (d.abs()-restPersonal)*d/ d.abs();
	}
	return force;
}

std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
    //std::vector<Vector3f> f(state.size());
	std::vector<Vector3f> f = {};
    // TODO 5. implement evalF
    // - gravity
    // - viscous drag
    // - structural springs
    // - shear springs
    // - flexion springs
	float t = 0;
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W*2; j+=2) {
			Vector3f totalF = { 0.0, 0.0, 0.0 };
			//totalF += {0, g, 0};
			//totalF += 1 * drag*state[t + 1]; //drag

			float tt = 0;
			for (int ii = 0; ii < H; ii++) {
				for (int jj = 0; jj < W * 2; jj += 2) {
					if (t != tt) {
						totalF += getPersonalSpaceForce(state[t], state[tt]);
					}
					tt += 2;
				}
			}

			// Random float between - xVariaion/2 and + xVariation/2
			float x_dir = xVariation/2. - (rand() % (xVariation * 100))/100.; 
			
			// Boid randomly speed up and slow down by 0.25 in the y direction
			float y_dir = 2.5 - (rand() % 50)/10.;

			// We are staying in one plane
			float z_dir = 0;

			f.push_back(state[t + 1]); //velocity

			f.push_back(Vector3f(x_dir, y_dir, z_dir) + totalF / mass); //acceleration

			t += 2;
			
		}
	}     
    return f;
}

void ClothSystem::draw(GLProgram& gl)
{
    //TODO 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh

    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);

    // EXAMPLE for how to render cloth particles.
    //  - you should replace this code.
    float w = 0.5f;
    Vector3f O(0, 0, 0);
    /*gl.updateModelMatrix(Matrix4f::translation(O));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, 0, 0)));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, -w, 0)));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(0, -w, 0)));
    drawSphere(0.04f, 8, 8);*/

	float t = 0;
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W * 2; j += 2) {
			Vector3f pos(m_vVecState[t]); //YOUR PARTICLE POSITION
			gl.updateModelMatrix(Matrix4f::translation(pos));
			drawSphere(0.075f, 10, 10);
			gl.disableLighting();
			gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
			VertexRecorder rec;
			if (i != 0) { //if not at top edge
				//totalF += getSpringForceStruct(state[t], state[t - 2 * W]); //get spring 1 above
				rec.record(m_vVecState[t], CLOTH_COLOR);
				rec.record(m_vVecState[t - 2 * W], CLOTH_COLOR);
			}
			if (i != H - 1) { //if not at bottom edge
				//totalF += getSpringForceStruct(state[t], state[t + 2 * W]); // get spring 1 below
				rec.record(m_vVecState[t], CLOTH_COLOR);
				rec.record(m_vVecState[t + 2 * W], CLOTH_COLOR);
			}

			if (j != 0) { //if not at left edge
				//totalF += getSpringForceStruct(state[t], state[t - 2]); //get spring 1 to the left
				rec.record(m_vVecState[t], CLOTH_COLOR);
				rec.record(m_vVecState[t - 2], CLOTH_COLOR);

			}

			if (j != W * 2 - 2) { //if not at right edge
				//totalF += getSpringForceStruct(state[t], state[t + 2]); // get spring 1 to the right
				rec.record(m_vVecState[t], CLOTH_COLOR);
				rec.record(m_vVecState[t + 2], CLOTH_COLOR);
			}
			glLineWidth(3.0f);
			//rec.draw(GL_LINES);

			gl.enableLighting(); // reset to default lighting model
			t += 2;
		}
	}


    // EXAMPLE: This shows you how to render lines to debug the spring system.
    //
    //          You should replace this code.
    //
    //          Since lines don't have a clearly defined normal, we can't use
    //          a regular lighting model.
    //          GLprogram has a "color only" mode, where illumination
    //          is disabled, and you specify color directly as vertex attribute.
    //          Note: enableLighting/disableLighting invalidates uniforms,
    //          so you'll have to update the transformation/material parameters
    //          after a mode change.
    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    /*VertexRecorder rec;
    rec.record(O, CLOTH_COLOR);
    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
    rec.record(O, CLOTH_COLOR);
    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);

    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);

    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);
    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);
    glLineWidth(3.0f);
    rec.draw(GL_LINES);*/

    gl.enableLighting(); // reset to default lighting model
    // EXAMPLE END
}

