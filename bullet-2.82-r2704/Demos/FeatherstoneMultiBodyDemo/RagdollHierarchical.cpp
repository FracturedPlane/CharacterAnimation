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

Written by: Marten Svanfeldt
*/

#include <iostream>
#include "RagdollHierarchical.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
// #include "LinearMath/btQuaternion.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "FeatherstoneMultiBodyDemo.h"

//#define RIGID 1

RagDollHierarchical::RagDollHierarchical (class btMultiBodyDynamicsWorld* ownerWorld, const btVector3& positionOffset,
	btScalar scale_ragdoll)	: m_ownerWorld (ownerWorld)
{
	float scaling = scale_ragdoll;
	float friction = 1.0f;

	btMultiBodySettings settings;

	settings.m_numLinks = 3;
	settings.m_basePosition = btVector3 (-25,14.5,20);
	settings.m_isFixedBase = true;
	settings.m_usePrismatic = false;

	// Setup the geometry

	m_shapes[BODYPART_PELVIS] = new btEmptyShape();
	m_shapes[BODYPART_PELVIS_XY] = new btEmptyShape();
	m_shapes[BODYPART_PELVIS_YZ] = new btBoxShape(btVector3(scale_ragdoll*0.16f,scale_ragdoll*0.14f,scale_ragdoll*0.08f));

	m_shapes[BODYPART_SPINE] = new btEmptyShape();
	m_shapes[BODYPART_SPINE_XY] = new btEmptyShape();
	m_shapes[BODYPART_SPINE_YZ] = new btBoxShape(btVector3(scale_ragdoll*0.06f,scale_ragdoll*0.30f,scale_ragdoll*0.06f));

	m_shapes[BODYPART_LEFT_UPPER_LEG] = new btEmptyShape();
	m_shapes[BODYPART_LEFT_UPPER_LEG_XY] = new btEmptyShape();
	m_shapes[BODYPART_LEFT_UPPER_LEG_YZ] = new btBoxShape(btVector3(scale_ragdoll*0.06f,scale_ragdoll*0.27f,scale_ragdoll*0.06f));


	int n_links = settings.m_numLinks;
	float mass = 13.5*scaling;
	btVector3 inertia = btVector3 (91,344,253)*scaling*scaling;


	btMultiBody * bod = new btMultiBody(n_links, mass, inertia, settings.m_isFixedBase, settings.m_canSleep);
//		bod->setHasSelfCollision(false);

	//btQuaternion orn(btVector3(0,0,1),-0.25*SIMD_HALF_PI);//0,0,0,1);
	btQuaternion orn(0,0,0,1);
	bod->setBasePos(settings.m_basePosition);
	bod->setWorldToBaseRot(orn);
	btVector3 vel(0,0,0);
	bod->setBaseVel(vel);

	m_world_to_local.resize(n_links+1);

	m_local_to_origin.resize(n_links+1);
	m_world_to_local[JOINT_ROOT_PELVIS_X] = bod->getWorldToBaseRot();
	m_local_to_origin[JOINT_ROOT_PELVIS_X] = bod->getBasePos();
	{

		float pos[4]={m_local_to_origin[JOINT_ROOT_PELVIS_X].x(),m_local_to_origin[JOINT_ROOT_PELVIS_X].y(),m_local_to_origin[JOINT_ROOT_PELVIS_X].z(),1};
		float quat[4]={-m_world_to_local[JOINT_ROOT_PELVIS_X].x(),-m_world_to_local[JOINT_ROOT_PELVIS_X].y(),
				-m_world_to_local[JOINT_ROOT_PELVIS_X].z(),m_world_to_local[JOINT_ROOT_PELVIS_X].w()};


		if (1)
		{
			btCollisionShape* box = new btEmptyShape();
			// btCollisionShape* box = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2])*scaling);
			btRigidBody* body = new btRigidBody(mass,0,box,inertia);
			btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(bod,-1);

			body->setCollisionShape(box);
			col->setCollisionShape(box);

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(m_local_to_origin[JOINT_ROOT_PELVIS_X]);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
			body->setWorldTransform(tr);
			col->setWorldTransform(tr);

			m_ownerWorld->addCollisionObject(col, 2,1+2);
			col->setFriction(friction);
			// bod->setBaseCollider(col);

		}
	}

	{

		btVector3 joint_axis_hinge(1,0,0); // axis of revolution
		btQuaternion parent_to_child = orn.inverse(); // transformation from parent to child
		btVector3 joint_axis_child_hinge = quatRotate(parent_to_child , joint_axis_hinge);
		btVector3 pos = btVector3 (0,0,0.0)*scaling; // parent joint position?
		btVector3 joint_axis_position = btVector3 (0,0,0.0)*scaling;// child joint position?

		float initial_joint_angle=0.0;
		int child_link_num=0;

		// Initial root pelvis joint X
		bod->setupRevolute(child_link_num, mass, inertia, child_link_num-1,parent_to_child, joint_axis_child_hinge,
									joint_axis_position,quatRotate(parent_to_child , (pos - joint_axis_position)),settings.m_disableParentCollision);
		bod->setJointPos(child_link_num, initial_joint_angle+0.5f);

		m_world_to_local[child_link_num+1] = bod->getParentToLocalRot(child_link_num) * m_world_to_local[bod->getParent(child_link_num)+1];
		m_local_to_origin[child_link_num+1] = m_local_to_origin[bod->getParent(child_link_num)+1] + (quatRotate(m_world_to_local[child_link_num+1].inverse() , bod->getRVector(child_link_num)));
		{
			btVector3 posr = m_local_to_origin[child_link_num];
			float pos[4]={posr.x(),posr.y(),posr.z(),1};

			float quat[4]={-m_world_to_local[child_link_num].x(),-m_world_to_local[child_link_num].y(),-m_world_to_local[child_link_num].z(),m_world_to_local[child_link_num].w()};

			btCollisionShape* box = m_shapes[BODYPART_PELVIS];
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod,child_link_num-1);

			col->setCollisionShape(box);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
			col->setWorldTransform(tr);
			col->setFriction(friction);
			m_ownerWorld->addCollisionObject(col,2,1+2);

			// bod->getLink(child_link_num).m_collider=col;
			//app->drawBox(halfExtents, pos,quat);
		}
		child_link_num++;


		// pelvis joint Y  //
		joint_axis_hinge = btVector3(0,1,0);
		parent_to_child = orn.inverse();
		joint_axis_child_hinge = quatRotate(parent_to_child , joint_axis_hinge);
		pos = btVector3 (0,0,0.0)*scaling;
		joint_axis_position = btVector3 (0,0,0.0)*scaling;

		bod->setupRevolute(child_link_num, mass, inertia, child_link_num-1,parent_to_child, joint_axis_child_hinge,
									joint_axis_position,quatRotate(parent_to_child , (pos - joint_axis_position)),settings.m_disableParentCollision);
		bod->setJointPos(child_link_num, initial_joint_angle+0.5f);

		m_world_to_local[child_link_num+1] = bod->getParentToLocalRot(child_link_num) * m_world_to_local[bod->getParent(child_link_num)+1];
		m_local_to_origin[child_link_num+1] = m_local_to_origin[bod->getParent(child_link_num)+1] + (quatRotate(m_world_to_local[child_link_num+1].inverse() , bod->getRVector(child_link_num)));
		{
			btVector3 posr = m_local_to_origin[child_link_num];
			float pos[4]={posr.x(),posr.y(),posr.z(),1};

			float quat[4]={-m_world_to_local[child_link_num].x(),-m_world_to_local[child_link_num].y(),-m_world_to_local[child_link_num].z(),m_world_to_local[child_link_num].w()};

			btCollisionShape* box = m_shapes[BODYPART_PELVIS_XY];
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod,child_link_num-1);

			col->setCollisionShape(box);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
			col->setWorldTransform(tr);
			col->setFriction(friction);
			m_ownerWorld->addCollisionObject(col,2,1+2);

			// bod->getLink(child_link_num).m_collider=col;
			//app->drawBox(halfExtents, pos,quat);
		}
		child_link_num++;

		// pelvis joint Z  //
		joint_axis_hinge = btVector3(0,0,1);
		parent_to_child = orn.inverse();
		joint_axis_child_hinge = quatRotate(parent_to_child , joint_axis_hinge);
		pos = btVector3 (0,0,0.0)*scaling;
		joint_axis_position = btVector3 (0,0,0.0)*scaling;

		bod->setupRevolute(child_link_num, mass, inertia, child_link_num-1,parent_to_child, joint_axis_child_hinge,
									joint_axis_position,quatRotate(parent_to_child , (pos - joint_axis_position)),settings.m_disableParentCollision);
		bod->setJointPos(child_link_num, initial_joint_angle+0.5f);

		m_world_to_local[child_link_num+1] = bod->getParentToLocalRot(child_link_num) * m_world_to_local[bod->getParent(child_link_num)+1];
		m_local_to_origin[child_link_num+1] = m_local_to_origin[bod->getParent(child_link_num)+1] + (quatRotate(m_world_to_local[child_link_num+1].inverse() , bod->getRVector(child_link_num)));

		{
			btVector3 posr = m_local_to_origin[child_link_num];
			float pos[4]={posr.x(),posr.y(),posr.z(),1};

			float quat[4]={-m_world_to_local[child_link_num].x(),-m_world_to_local[child_link_num].y(),-m_world_to_local[child_link_num].z(),m_world_to_local[child_link_num].w()};

			btCollisionShape* box = m_shapes[BODYPART_PELVIS_YZ];
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod,child_link_num-1);

			col->setCollisionShape(box);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
			col->setWorldTransform(tr);
			col->setFriction(friction);
			m_ownerWorld->addCollisionObject(col,2,1+2);

			bod->getLink(child_link_num).m_collider=col;
			//app->drawBox(halfExtents, pos,quat);
		}
		child_link_num++;

		/*
		// spine joint X  //
		joint_axis_hinge = btVector3(1,0,0);
		parent_to_child = orn.inverse();
		joint_axis_child_hinge = quatRotate(parent_to_child , joint_axis_hinge);
		pos = btVector3 (0,0,0.0)*scaling;
		joint_axis_position = btVector3 (0,0.3,0.0)*scaling;

		bod->setupRevolute(child_link_num, mass, inertia, child_link_num-1,parent_to_child, joint_axis_child_hinge,
									joint_axis_position,quatRotate(parent_to_child , (pos - joint_axis_position)),settings.m_disableParentCollision);
		bod->setJointPos(child_link_num, initial_joint_angle+0.5f);

		m_world_to_local[child_link_num] = bod->getParentToLocalRot(child_link_num) * m_world_to_local[bod->getParent(child_link_num)];
		m_local_to_origin[child_link_num] = m_local_to_origin[bod->getParent(child_link_num)] + (quatRotate(m_world_to_local[child_link_num].inverse() , bod->getRVector(child_link_num-1)));

		{
			btVector3 posr = m_local_to_origin[child_link_num];
			float pos[4]={posr.x(),posr.y(),posr.z(),1};

			float quat[4]={-m_world_to_local[child_link_num].x(),-m_world_to_local[child_link_num].y(),-m_world_to_local[child_link_num].z(),m_world_to_local[child_link_num].w()};

			btCollisionShape* box = m_shapes[BODYPART_SPINE];
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod,child_link_num-1);

			col->setCollisionShape(box);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
			col->setWorldTransform(tr);
			col->setFriction(friction);
			m_ownerWorld->addCollisionObject(col,2,1+2);

			bod->getLink(child_link_num).m_collider=col;
			//app->drawBox(halfExtents, pos,quat);
		}
		child_link_num++;


		// Spine joint Y  //
		joint_axis_hinge = btVector3(0,1,0);
		parent_to_child = orn.inverse();
		joint_axis_child_hinge = quatRotate(parent_to_child , joint_axis_hinge);
		pos = btVector3 (0,0,0.0)*scaling;
		joint_axis_position = btVector3 (0,0.0,0.0)*scaling;

		bod->setupRevolute(child_link_num, mass, inertia, child_link_num-1,parent_to_child, joint_axis_child_hinge,
									joint_axis_position,quatRotate(parent_to_child , (pos - joint_axis_position)),settings.m_disableParentCollision);
		bod->setJointPos(child_link_num, initial_joint_angle+0.5f);

		m_world_to_local[child_link_num] = bod->getParentToLocalRot(child_link_num) * m_world_to_local[bod->getParent(child_link_num)];
		m_local_to_origin[child_link_num] = m_local_to_origin[bod->getParent(child_link_num)] + (quatRotate(m_world_to_local[child_link_num].inverse() , bod->getRVector(child_link_num-1)));

		{
			btVector3 posr = m_local_to_origin[child_link_num];
			float pos[4]={posr.x(),posr.y(),posr.z(),1};

			float quat[4]={-m_world_to_local[child_link_num].x(),-m_world_to_local[child_link_num].y(),-m_world_to_local[child_link_num].z(),m_world_to_local[child_link_num].w()};

			btCollisionShape* box = m_shapes[BODYPART_SPINE_XY];
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod,child_link_num-1);

			col->setCollisionShape(box);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
			col->setWorldTransform(tr);
			col->setFriction(friction);
			m_ownerWorld->addCollisionObject(col,2,1+2);

			bod->getLink(child_link_num).m_collider=col;
			//app->drawBox(halfExtents, pos,quat);
		}
		child_link_num++;


		// spine joint Z  //
		joint_axis_hinge = btVector3(0,0,1);
		parent_to_child = orn.inverse();
		joint_axis_child_hinge = quatRotate(parent_to_child , joint_axis_hinge);
		pos = btVector3 (0,0,0.0)*scaling;
		joint_axis_position = btVector3 (0,0.0,0.0)*scaling;

		bod->setupRevolute(child_link_num, mass, inertia, child_link_num-1,parent_to_child, joint_axis_child_hinge,
									joint_axis_position,quatRotate(parent_to_child , (pos - joint_axis_position)),settings.m_disableParentCollision);
		bod->setJointPos(child_link_num, initial_joint_angle+0.5f);

		m_world_to_local[child_link_num] = bod->getParentToLocalRot(child_link_num) * m_world_to_local[bod->getParent(child_link_num)];
		m_local_to_origin[child_link_num] = m_local_to_origin[bod->getParent(child_link_num)] + (quatRotate(m_world_to_local[child_link_num].inverse() , bod->getRVector(child_link_num-1)));

		{
			btVector3 posr = m_local_to_origin[child_link_num];
			float pos[4]={posr.x(),posr.y(),posr.z(),1};

			float quat[4]={-m_world_to_local[child_link_num].x(),-m_world_to_local[child_link_num].y(),-m_world_to_local[child_link_num].z(),m_world_to_local[child_link_num].w()};

			btCollisionShape* box = m_shapes[BODYPART_SPINE_YZ];
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod,child_link_num-1);

			col->setCollisionShape(box);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
			col->setWorldTransform(tr);
			col->setFriction(friction);
			m_ownerWorld->addCollisionObject(col,2,1+2);

			bod->getLink(child_link_num).m_collider=col;
			//app->drawBox(halfExtents, pos,quat);
		}
		child_link_num++;
		*/



	}
	m_ownerWorld->addMultiBody(bod);

	// Setup all the rigid bodies


}


RagDollHierarchical::~RagDollHierarchical()
{
	int i;

	// Remove all constraints
	for (i = 0; i < JOINT_COUNT; ++i)
	{
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = 0;
	}

	// Remove all bodies and shapes
	for (i = 0; i < BODYPART_COUNT; ++i)
	{
		m_ownerWorld->removeRigidBody(m_bodies[i]);

		delete m_bodies[i]->getMotionState();

		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	}
}


btRigidBody* RagDollHierarchical::localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	rbInfo.m_additionalDamping = true;
	btRigidBody* body = new btRigidBody(rbInfo);

	m_ownerWorld->addRigidBody(body);

	return body;
}
