/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Originally Written by: Marten Svanfeldt
ReWritten by: Francisco Le�n
*/



#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "GenericJointDemo.h"

#include "Ragdoll.h"

#include <iostream>



GLDebugDrawer debugDrawer;




void motorPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	GenericJointDemo* jointDemo = (GenericJointDemo*)world->getWorldUserInfo();

	jointDemo->setMotorTargets(timeStep);

}

void GenericJointDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	m_Time = 0;
	m_fCyclePeriod = 2000.f; // in milliseconds
	m_fMuscleStrength = 0.5f;

	// Setup the basic world

	btDefaultCollisionConfiguration * collision_config = new btDefaultCollisionConfiguration();

	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collision_config);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	btBroadphaseInterface* overlappingPairCache = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	btConstraintSolver* constraintSolver = new btSequentialImpulseConstraintSolver;


	m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,constraintSolver,collision_config);

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	m_dynamicsWorld->setDebugDrawer(&debugDrawer);

	m_dynamicsWorld->setInternalTickCallback(motorPreTickCallback,this,true);

	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-15,0));
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
	}

	// Spawn one ragdoll
	spawnRagdoll();

	clientResetScene();
}

void GenericJointDemo::spawnRagdoll(bool random)
{
	RagDoll* ragDoll = new RagDoll (m_dynamicsWorld, btVector3 (0,0,10),5.f);
	m_ragdolls.push_back(ragDoll);
}

void GenericJointDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	renderme();

	glFlush();

	swapBuffers();
}

void GenericJointDemo::setMotorTargets(btScalar deltaTime)
{

	float ms = deltaTime*1000000.;
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	m_Time += ms;

	//
	// set per-frame sinusoidal position targets using angular motor (hacky?)
	//
	for (int r=0; r<m_ragdolls.size(); r++)
	{
		for (int i=0; i<(RagDoll::JOINT_COUNT-1); i++)
		{
			std::cout << "Updating joint " << i << std::endl;
			btGeneric6DofConstraint* hingeC = static_cast<btGeneric6DofConstraint*>(m_ragdolls[r]->GetJoints()[i]);
			if ( hingeC != NULL)
			{
				btRotationalLimitMotor * rotMotor = hingeC->getRotationalLimitMotor(0);
				std::cout << "motor for A is " << (rotMotor->m_targetVelocity) << std::endl;
				// std::cout << "motor for A is " << (rotMotor->) << std::endl;
				rotMotor->m_enableMotor = true;
				rotMotor->m_maxMotorForce = 10.0f;
				// hingeC->setL
				btScalar fCurAngle      = hingeC->getAngle(0);
				btVector3 angleLowerLimit;
				hingeC->getAngularLowerLimit(angleLowerLimit);
				btVector3 angleUpperLimit;
				hingeC->getAngularUpperLimit(angleUpperLimit);
				btScalar fTargetPercent = (int(m_Time / 2000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
				btScalar fTargetAngle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent));
				btScalar fTargetLimitAngle = angleLowerLimit.x() + fTargetAngle * (angleUpperLimit.x() - angleLowerLimit.x());
				btScalar fAngleError  = fTargetLimitAngle - fCurAngle;
				btScalar fDesiredAngularVel = 1000000.f * fAngleError/ms;
				// std::cout << "desired angle: " << fTargetAngle << " desired angle Vel: " << fDesiredAngularVel << std::endl;
				rotMotor->m_targetVelocity = btScalar(fDesiredAngularVel);
				// hingeC->setAxis(btVector3(fTargetAngle, 0.0, 0.0), btVector3(fTargetAngle, 0.0, 0.0));
				// hingeC->enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
			}
		}
	}


}

void GenericJointDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	renderme();

	glFlush();
	swapBuffers();
}

void GenericJointDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		spawnRagdoll(true);
		break;
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}


}
