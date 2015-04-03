//test addJointTorque
#include "MultiBodyVehicle.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

#include <iostream>

/*
 * Play with these constants until you find a setup that exhibits an explosion
 * of energy.
 */
const btScalar FIXED_STEP = 1.0f / 500.0;
const int NUM_MULTIBODIES = 1;

/*
btVector3 flipYandZ(const btVector3 & v)
{
	return btVector3(v.getX(), v.getZ(), v.getY());
	// return btVector3(v.getX(), v.getY(), v.getZ());
}
*/

bool Comparator::operator()(const btVector3 & u, const btVector3 & v) const
{
  if (u.getX() != v.getX())
    return u.getX() < v.getX();
  else if (u.getY() != v.getY())
    return u.getY() < v.getY();
  else
    return u.getZ() < v.getZ();
}

btBoxShape * MultiBodyVehicleSetup::getBoxShape(const btVector3 & halfExtents)
{
  if (this->boxShapes.find(halfExtents) == this->boxShapes.end())
  {
    this->boxShapes[halfExtents] = new btBoxShape(halfExtents);
  }
  return this->boxShapes[halfExtents];
}

MultiBodyVehicleSetup::MultiBodyVehicleSetup()
{

	_rigth_foot_contact = false;
	_left_foot_contact =  false;
	_lastTransitionFrameNum = 0;
}

MultiBodyVehicleSetup::~MultiBodyVehicleSetup()
{
  for (std::map<btVector3, btBoxShape *, Comparator>::iterator it = this->boxShapes.begin();
      it != this->boxShapes.end();
      it++)
  {
    delete it->second;
  }
  for (unsigned i = 0; i < this->colliders.size(); i++)
  {
    delete this->colliders[i];
  }
}



class btMultiBody* MultiBodyVehicleSetup::createMultiBodyVehicle(const ModelConstructionInfo & info, GraphicsPhysicsBridge& gfxBridge)
{
  class btMultiBodyDynamicsWorld* world = m_dynamicsWorld;

  const int NUM_LINKS = static_cast<int>(info.childLinks.size());
  btVector4 colors[4] =
  {
    btVector4(1,0,0,1),
    btVector4(0,1,0,1),
    btVector4(0,1,1,1),
    btVector4(1,1,0,1),
  };
  int curColor = 0;

  btCollisionShape * baseCollisionShape = getBoxShape(info.baseHalfExtents);
  btVector3 inertia(0, 0, 0);
  baseCollisionShape->calculateLocalInertia(static_cast<btScalar>(info.baseMass), inertia);

  bool fixedBase = false;
  btMultiBody * multiBody = new btMultiBody(NUM_LINKS,
      static_cast<btScalar>(info.baseMass),
      inertia,
      fixedBase,
      true,
      true);

  multiBody->setBasePos(info.basePosition);
  this->colliders.push_back(new btMultiBodyLinkCollider(multiBody,
        -1));
  btMultiBodyLinkCollider * baseCollider = this->colliders.back();
  baseCollider->setCollisionShape(baseCollisionShape);
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(multiBody->getBasePos());
  baseCollider->setWorldTransform(transform);
  world->addCollisionObject(baseCollider, 1, 1+2); //needed to collide with the ground plane
  multiBody->setBaseCollider(baseCollider);

  for (int i = 0; i < NUM_LINKS; i++)
  {
    const BodyConstructionInfo & LINK = info.childLinks.at(i);
    // btCollisionShape * collisionShape = getBoxShape(LINK.halfExtents);
    btCollisionShape * collisionShape = new btBoxShape(LINK.halfExtents);
    std::cout << "collision shape: " << *LINK.halfExtents << std::endl;
    // gfxBridge.createCollisionShapeGraphicsObject(collisionShape);
    this->colliders.push_back(new btMultiBodyLinkCollider(multiBody,
          i));
    btMultiBodyLinkCollider * collider = this->colliders.back();
    collider->setCollisionShape(collisionShape);
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(LINK.position);
    // tr.rotation3Drad()
    collider->setWorldTransform(tr);
    world->addCollisionObject(collider, 1, 1+2);
    // btRigidBody* body =  createRigidBody(0,start,box);
    multiBody->getLink(i).m_collider = collider;
    btVector4 colour = colors[curColor];
	curColor++;
	curColor&=3;
	// gfxBridge.createCollisionShapeGraphicsObject(collider->getCollisionShape());
    // gfxBridge.createCollisionObjectGraphicsObject(collider,colour);

    collisionShape->calculateLocalInertia(static_cast<btScalar>(LINK.mass), inertia);
  }

  for (int i = 0; i < static_cast<int>(info.joints.size()); i++)
  {
    const JointConstructionInfo & JOINT = info.joints.at(i);
    if (JOINT.jointType == JointConstructionInfo::REVOLUTE)
    {
      btVector3 parentPosition;
      if (JOINT.parentLinkIndex == -1)
      {
        parentPosition = info.basePosition;
      }
      else
      {
        parentPosition = info.childLinks.at(JOINT.parentLinkIndex).position;
      }

      const BodyConstructionInfo & LINK = info.childLinks.at(JOINT.linkIndex);

      btVector3 localInertia(0, 0, 0);
      multiBody->getLink(JOINT.linkIndex).m_collider->getCollisionShape()->calculateLocalInertia(static_cast<btScalar>(LINK.mass), localInertia);
      btVector3 linkToParent = parentPosition - JOINT.worldPosition;
      btVector3 linkToThis = LINK.position - JOINT.worldPosition;
      btQuaternion rotParentToThis = shortestArcQuatNormalize2(linkToParent, linkToThis);
      rotParentToThis = btQuaternion(JOINT.hingeAxis, JOINT.initAngle);
      multiBody->setupRevolute(JOINT.linkIndex,
          LINK.mass,
          localInertia,
          JOINT.parentLinkIndex,
          // btQuaternion(JOINT.hingeAxis.x(),JOINT.hingeAxis.y(),JOINT.hingeAxis.z(),0),
          rotParentToThis,
          // btQuaternion(0,0,0,1).inverse(),
          JOINT.hingeAxis,
          JOINT.worldPosition - parentPosition,
          LINK.position - JOINT.worldPosition,
          true);
    }
    else if (JOINT.jointType == JointConstructionInfo::SPHERICAL)
    {
      btVector3 parentPosition;
      if (JOINT.parentLinkIndex == -1)
      {
        parentPosition = info.basePosition;
      }
      else
      {
        parentPosition = info.childLinks.at(JOINT.parentLinkIndex).position;
      }

      const BodyConstructionInfo & LINK = info.childLinks.at(JOINT.linkIndex);

      btVector3 localInertia(0, 0, 0);
      multiBody->getLink(JOINT.linkIndex).m_collider->getCollisionShape()->calculateLocalInertia(static_cast<btScalar>(LINK.mass), localInertia);

      multiBody->setupSpherical(JOINT.linkIndex,
          LINK.mass,
          localInertia,
          JOINT.parentLinkIndex,
          btQuaternion(0,0,0,1).inverse(),
          JOINT.worldPosition - parentPosition,
          LINK.position - JOINT.worldPosition,
          true);
    }
    else
    {
    }
    // btMultiBodyJointMotor* con = new btMultiBodyJointMotor(multiBody,i,0,500000);
	// world->addMultiBodyConstraint(con);

  }

  multiBody->finalizeMultiDof();
  world->addMultiBody(multiBody);
  return multiBody;
}

void MultiBodyVehicleSetup::checkGroundContact(size_t frameNum, float dt)
{
	btMultibodyLink rFoot = this->m_multiBody->getLink(RIGHT_FOOT);
	btMultibodyLink lFoot = this->m_multiBody->getLink(LEFT_FOOT);

	// if ( rFoot.m_collider->checkCollideWithOverride(this->_ground) )
	{
		// std::cout << "Right foot contact" << std::endl;
	}

	int numManifolds = this->m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  this->m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = dynamic_cast<const btCollisionObject*>(contactManifold->getBody0());
		const btCollisionObject* obB = dynamic_cast<const btCollisionObject*>(contactManifold->getBody1());

		if ( (obA == rFoot.m_collider && obB == this->_ground) ||
				(obB == rFoot.m_collider && obA == this->_ground))
		{
			if ( _rigth_foot_contact == false )
			{ // transition from state STANDING_ON_RIGHT_FOOT => START_WALKING_ON_LEFT_FOOT
				std::cout << "Found A RIGHT foot collision with ground" << std::endl;
				transitionControllerStates();

			}
			_rigth_foot_contact = true;
		}

		if (  (obA == lFoot.m_collider && obB == this->_ground) ||
				(obB == lFoot.m_collider && obA == this->_ground))
		{
			if ( _left_foot_contact == false )
			{ // transition from state STANDING_ON_LEFT_FOOT => START_WALKING_ON_RIGHT_FOOT
				std::cout << "Found A LEFT foot collision with ground" << std::endl;
				transitionControllerStates();
			}
			_left_foot_contact = true;
		}

		/*
		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance()<0.f)
			{
				const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;
			}
		}*/
	}

	// deactivate rFoot contact

	size_t r_i=0;
	for (; r_i < numManifolds ;r_i++)
	{
		btPersistentManifold* contactManifold =  this->m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(r_i);
		const btCollisionObject* obA = dynamic_cast<const btCollisionObject*>(contactManifold->getBody0());
		const btCollisionObject* obB = dynamic_cast<const btCollisionObject*>(contactManifold->getBody1());

		if ( (obA == rFoot.m_collider && obB == this->_ground) ||
				(obB == rFoot.m_collider && obA == this->_ground))
		{
			break;
		}
	}
	if ( r_i == numManifolds)
	{// found no right foot ground manifolds (contact)
		_rigth_foot_contact = false;
	}

	// deactivate lFoot contact
	size_t l_i=0;
	for (; l_i < numManifolds ;l_i++)
	{
		btPersistentManifold* contactManifold =  this->m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(l_i);
		const btCollisionObject* obA = dynamic_cast<const btCollisionObject*>(contactManifold->getBody0());
		const btCollisionObject* obB = dynamic_cast<const btCollisionObject*>(contactManifold->getBody1());

		if (  (obA == lFoot.m_collider && obB == this->_ground) ||
				(obB == lFoot.m_collider && obA == this->_ground))
		{
			break;
		}

	}
	if ( l_i == numManifolds)
	{// found no left foot ground manifolds (contact)
		_left_foot_contact = false;
	}


	// m_dynamicsWorld->contactPairTest(rFoot, this->_ground, NULL);
	// btMultibodyLink lFoot = this->m_multiBody->;


}

void MultiBodyVehicleSetup::checkControllerStates(size_t frameNum, float dt)
{

	if ( ((((frameNum + 1)-_lastTransitionFrameNum) % 150) == 0) &&
			(( this->_controllerState == START_WALKING_ON_LEFT_FOOT) ||
				this->_controllerState == START_WALKING_ON_RIGHT_FOOT)) // 500 * 0.3 = 150 frames
	{ // Transition to another state
		this->transitionControllerStates();
	}
}

void MultiBodyVehicleSetup::transitionControllerStates()
{
	_lastTransitionFrameNum = this->_frameNum;
	std::cout << "Transitioning between states, current state: " << this->_controllerState << std::endl;
	if ( this->_controllerState == START_WALKING_ON_RIGHT_FOOT )
	{
		this->_controllerState = STANDING_ON_RIGHT_FOOT;
	}
	else if ( this->_controllerState == STANDING_ON_RIGHT_FOOT )
	{
		this->_controllerState = START_WALKING_ON_LEFT_FOOT;
	}
	else if (this->_controllerState == START_WALKING_ON_LEFT_FOOT)
	{
		this->_controllerState = STANDING_ON_LEFT_FOOT;
	}
	else if (this->_controllerState == STANDING_ON_LEFT_FOOT)
	{
		this->_controllerState = START_WALKING_ON_RIGHT_FOOT;
	}
	else
	{
		std::cout << "**** PROBLEM TRANSITIONING STATES ****" << std::endl;
	}
}

void MultiBodyVehicleSetup::initControllerStates()
{

	this->_controllerState = STANDING_ON_RIGHT_FOOT;

	this->_init_config_states.resize(4);
	for (size_t item=0; item < this->_init_config_states.size(); item++)
	{
		this->_init_config_states.at(item).resize(7);
	}


    // Desired joint angles

	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][HIPS_TO_TOURSO] = 0.0; // Base
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][HIP_TO_LEFT_THIGH_JOINT] = M_PI_2; // left hip
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][LEFT_THIGH_TO_LEFT_CHIN_JOINT] = M_PI_2; // left knee
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][LEFT_CHIN_TO_LEFT_FOOT_JOINT] = M_PI_2; // left ankle
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][HIP_TO_RIGHT_THIGH_JOINT] = 2.6f; // right hip
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = 3.1f; // right knee
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = 1.9f; // right ankle
	/*
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][HIPS] = 3.9; // Base
    this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][HIP_TO_LEFT_THIGH_JOINT] = M_PI_2*3; // left hip
    this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][LEFT_THIGH_TO_LEFT_CHIN_JOINT] = M_PI_2*3; // left knee
    this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][LEFT_CHIN_TO_LEFT_FOOT_JOINT] = M_PI_2*3; // left ankle
    this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][HIP_TO_RIGHT_THIGH_JOINT] = 2.6f; // right hip
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = 3.1f; // right knee
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = 1.4f; // right ankle
*/
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][HIPS_TO_TOURSO] = 0.0; // Base
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][HIP_TO_LEFT_THIGH_JOINT] = 1.0f; // left hip
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][LEFT_THIGH_TO_LEFT_CHIN_JOINT] = M_PI_2; // left knee
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][LEFT_CHIN_TO_LEFT_FOOT_JOINT] = M_PI_2; // left ankle
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][HIP_TO_RIGHT_THIGH_JOINT] = M_PI_2; // right hip
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = M_PI_2; // right knee
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = M_PI_2; // right ankle

	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][HIPS_TO_TOURSO] = 0.0; // Base
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][HIP_TO_LEFT_THIGH_JOINT] = 2.6f; // left hip
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][LEFT_THIGH_TO_LEFT_CHIN_JOINT] = 3.1f; // left knee
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][LEFT_CHIN_TO_LEFT_FOOT_JOINT] = 1.9f; // left ankle
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][HIP_TO_RIGHT_THIGH_JOINT] = M_PI_2; // right hip
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = M_PI_2; // right knee
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = M_PI_2; // right ankle

	this->_init_config_states[STANDING_ON_LEFT_FOOT][HIPS_TO_TOURSO] = 0.0; // Base
	this->_init_config_states[STANDING_ON_LEFT_FOOT][HIP_TO_LEFT_THIGH_JOINT] = M_PI_2; // left hip
	this->_init_config_states[STANDING_ON_LEFT_FOOT][LEFT_THIGH_TO_LEFT_CHIN_JOINT] = M_PI_2; // left knee
	this->_init_config_states[STANDING_ON_LEFT_FOOT][LEFT_CHIN_TO_LEFT_FOOT_JOINT] = M_PI_2; // left ankle
	this->_init_config_states[STANDING_ON_LEFT_FOOT][HIP_TO_RIGHT_THIGH_JOINT] = 1.0f; // right hip
	this->_init_config_states[STANDING_ON_LEFT_FOOT][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = M_PI_2; // right knee
	this->_init_config_states[STANDING_ON_LEFT_FOOT][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = M_PI_2; // right ankle


}

void MultiBodyVehicleSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
  int upAxis = 1;

  this->_frameNum = 0;

  btVector4 colors[4] =
  {
    btVector4(1,0,0,1),
    btVector4(0,1,0,1),
    btVector4(0,1,1,1),
    btVector4(1,1,0,1),
  };
  int curColor = 0;

  this->initControllerStates();

    gfxBridge.setUpAxis(upAxis);

	this->createEmptyDynamicsWorld();
    gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
        //btIDebugDraw::DBG_DrawConstraints
        +btIDebugDraw::DBG_DrawWireframe
        +btIDebugDraw::DBG_DrawContactPoints
        +btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);

    
  for (int i = 0; i < NUM_MULTIBODIES; i++)
  {
    const btVector3 OFFSET((btVector3(1 * i, 0.05, 1 * i)));
    ModelConstructionInfo modelInfo;
    modelInfo.modelName = "M";//  + std::to_string(i);
    modelInfo.baseName = "hips";
    modelInfo.baseHalfExtents = (btVector3(0.05f, 0.05f, 0.15f));
    modelInfo.basePosition = (btVector3(-0.3f, 1.025, 0.0f)) + OFFSET;
    modelInfo.baseMass = 1.01f;


    BodyConstructionInfo stomachInfo;
    stomachInfo.name = "stomach";
    stomachInfo.halfExtents = (btVector3(0.05f, 0.25f, 0.15f));
    stomachInfo.position = (btVector3(-0.07f, 1.05f, 0.0f)) + OFFSET;
    stomachInfo.mass = 25.0f;
    stomachInfo.parentIndex = -1;

    BodyConstructionInfo leftUpperLegInfo;
    leftUpperLegInfo.name = "leftUpperLeg";
    leftUpperLegInfo.halfExtents = (btVector3(0.2, 0.05, 0.05));
    leftUpperLegInfo.position = (btVector3(0.2f, 0.525, 0.1)) + OFFSET;
    leftUpperLegInfo.mass = 11.0f;
    leftUpperLegInfo.parentIndex = 0;

    BodyConstructionInfo leftLowerLegInfo;
    leftLowerLegInfo.name = "leftLowerLeg";
    leftLowerLegInfo.halfExtents = (btVector3(0.05, 0.2, 0.05));
    leftLowerLegInfo.position = (btVector3(0.41f, 0.325, 0.1)) + OFFSET;
    leftLowerLegInfo.mass = 8.0f;
    leftLowerLegInfo.parentIndex = 1;

    BodyConstructionInfo leftFootInfo;
    leftFootInfo.name = "leftFoot";
    leftFootInfo.halfExtents = (btVector3(0.025, 0.105, 0.05));
    leftFootInfo.position = (btVector3(0.4f, 0.025, 0.1)) + OFFSET;
    leftFootInfo.mass = 2.0f;
    leftFootInfo.parentIndex = 2;

    BodyConstructionInfo rightUpperLegInfo;
    rightUpperLegInfo.name = "rightUpperLeg";
    rightUpperLegInfo.halfExtents = (btVector3(0.2, 0.05, 0.05));
    rightUpperLegInfo.position = (btVector3(0.2f, 0.525, -0.1)) + OFFSET;
    rightUpperLegInfo.mass = 11.0f;
    rightUpperLegInfo.parentIndex = 0;

    BodyConstructionInfo rightLowerLegInfo;
    rightLowerLegInfo.name = "rightLowerLeg";
    rightLowerLegInfo.halfExtents = (btVector3(0.05, 0.2, 0.05));
    rightLowerLegInfo.position = (btVector3(0.41f, 0.325, -0.1)) + OFFSET;
    rightLowerLegInfo.mass = 8.0f;
    rightLowerLegInfo.parentIndex = 4;

    BodyConstructionInfo rightFootInfo;
    rightFootInfo.name = "rightFoot";
    rightFootInfo.halfExtents = (btVector3(0.025, 0.105, 0.05));
    rightFootInfo.position = (btVector3(0.4f, 0.025, -0.1)) + OFFSET;
    rightFootInfo.mass = 2.0f;
    rightFootInfo.parentIndex = 5;


    /*
    BodyConstructionInfo chestInfo;
    chestInfo.name = "chest";
    chestInfo.halfExtents = flipYandZ(btVector3(0.05f, 0.15f, 0.05f));
    chestInfo.position = flipYandZ(btVector3(0.0f, 0.0f, 1.35)) + OFFSET;
    chestInfo.mass = 1.0f;
    chestInfo.parentIndex = 6;

    BodyConstructionInfo rightArmInfo;
    rightArmInfo.name = "rightArm";
    rightArmInfo.halfExtents = flipYandZ(btVector3(0.05, 0.15, 0.05));
    rightArmInfo.position = flipYandZ(btVector3(0.0f, -0.3, 1.35)) + OFFSET;
    rightArmInfo.mass = 1.0f;
    rightArmInfo.parentIndex = 7;

	BodyConstructionInfo leftArmInfo;
	leftArmInfo.name = "leftArm";
	leftArmInfo.halfExtents = flipYandZ(btVector3(0.05, 0.15, 0.05));
	leftArmInfo.position = flipYandZ(btVector3(0.0f, 0.3, 1.35)) + OFFSET;
	leftArmInfo.mass = 1.0f;
	leftArmInfo.parentIndex = 7;

    BodyConstructionInfo rightForearmInfo;
    rightForearmInfo.name = "rightForearm";
    rightForearmInfo.halfExtents = flipYandZ(btVector3(0.04, 0.14, 0.04));
    rightForearmInfo.position = flipYandZ(btVector3(0.0f, -0.55, 1.35)) + OFFSET;
    rightForearmInfo.mass = 1.0f;
    rightForearmInfo.parentIndex = 8;

	BodyConstructionInfo leftForearmInfo;
	leftForearmInfo.name = "leftForearm";
	leftForearmInfo.halfExtents = flipYandZ(btVector3(0.04, 0.14, 0.04));
	leftForearmInfo.position = flipYandZ(btVector3(0.0f, 0.55, 1.35)) + OFFSET;
	leftForearmInfo.mass = 1.0f;
	leftForearmInfo.parentIndex = 9;

    BodyConstructionInfo rightHandInfo;
    rightHandInfo.name = "rightHand";
    rightHandInfo.halfExtents = flipYandZ(btVector3(0.04, 0.06, 0.02));
    rightHandInfo.position = flipYandZ(btVector3(0.0f, -0.76, 1.35)) + OFFSET;
    rightHandInfo.mass = 1.0f;
    rightHandInfo.parentIndex = 10;

	BodyConstructionInfo leftHandInfo;
	leftHandInfo.name = "leftHand";
	leftHandInfo.halfExtents = flipYandZ(btVector3(0.04, 0.06, 0.02));
	leftHandInfo.position = flipYandZ(btVector3(0.0f, 0.76, 1.35)) + OFFSET;
	leftHandInfo.mass = 1.0f;
	leftHandInfo.parentIndex = 11;

	BodyConstructionInfo headInfo;
	headInfo.name = "head";
	headInfo.halfExtents = flipYandZ(btVector3(0.07, 0.06, 0.085));
	headInfo.position = flipYandZ(btVector3(0.0f, 0.0, 1.53)) + OFFSET;
	headInfo.mass = 1.0f;
	headInfo.parentIndex = 7;
*/


    JointConstructionInfo hipToStomachJointInfo;
    hipToStomachJointInfo.name = "hipToStomachJoint";
    hipToStomachJointInfo.linkIndex = 0;
    hipToStomachJointInfo.parentLinkIndex = -1;
    hipToStomachJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    hipToStomachJointInfo.worldPosition = (btVector3(0.0f, 1.275, 0.0f)) + OFFSET;
    hipToStomachJointInfo.hingeAxis = (btVector3(0, 0, 1));
    hipToStomachJointInfo.initAngle = this->_init_config_states[this->_controllerState][TOURSO];


    JointConstructionInfo leftHipJointInfo;
    leftHipJointInfo.name = "leftHipJoint";
    leftHipJointInfo.linkIndex = 1;
    leftHipJointInfo.parentLinkIndex = 0;
    leftHipJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    leftHipJointInfo.worldPosition = (btVector3(0.00f, 0.625, 0.1)) + OFFSET;
    leftHipJointInfo.hingeAxis = (btVector3(0, 0, 1));
    leftHipJointInfo.initAngle = this->_init_config_states[this->_controllerState][HIP_TO_LEFT_THIGH_JOINT];

    JointConstructionInfo leftKneeJointInfo;
    leftKneeJointInfo.name = "leftKneeJoint";
    leftKneeJointInfo.linkIndex = 2;
    leftKneeJointInfo.parentLinkIndex = 1;
    leftKneeJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    leftKneeJointInfo.worldPosition = (btVector3(0.45f, 0.56,  0.1)) + OFFSET;
    leftKneeJointInfo.hingeAxis = (btVector3(0, 0, -1));
    leftKneeJointInfo.initAngle = this->_init_config_states[this->_controllerState][LEFT_THIGH_TO_LEFT_CHIN_JOINT];

    JointConstructionInfo leftAnkleJointInfo;
    leftAnkleJointInfo.name = "leftAnkleJoint";
    leftAnkleJointInfo.linkIndex = 3;
    leftAnkleJointInfo.parentLinkIndex = 2;
    leftAnkleJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    leftAnkleJointInfo.worldPosition = (btVector3(0.4f, 0.05, 0.1)) + OFFSET;
    leftAnkleJointInfo.hingeAxis = (btVector3(0, 0, 1));
    leftAnkleJointInfo.initAngle = this->_init_config_states[this->_controllerState][LEFT_CHIN_TO_LEFT_FOOT_JOINT];

    JointConstructionInfo rightHipJointInfo;
    rightHipJointInfo.name = "rightHipJoint";
    rightHipJointInfo.linkIndex = 4;
    rightHipJointInfo.parentLinkIndex = 0;
    rightHipJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    rightHipJointInfo.worldPosition = (btVector3(0.0f, 0.625, -0.1)) + OFFSET;
    rightHipJointInfo.hingeAxis = (btVector3(0, 0, 1));
    rightHipJointInfo.initAngle = this->_init_config_states[this->_controllerState][HIP_TO_RIGHT_THIGH_JOINT];

    JointConstructionInfo rightKneeJointInfo;
    rightKneeJointInfo.name = "rightKneeJoint";
    rightKneeJointInfo.linkIndex = 5;
    rightKneeJointInfo.parentLinkIndex = 4;
    rightKneeJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    rightKneeJointInfo.worldPosition = (btVector3(0.45f, 0.56, -0.1)) + OFFSET;
    rightKneeJointInfo.hingeAxis = (btVector3(0, 0, -1));
    rightKneeJointInfo.initAngle = this->_init_config_states[this->_controllerState][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT];

    JointConstructionInfo rightAnkleJointInfo;
    rightAnkleJointInfo.name = "rightAnkleJoint";
    rightAnkleJointInfo.linkIndex = 6;
    rightAnkleJointInfo.parentLinkIndex = 5;
    rightAnkleJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    rightAnkleJointInfo.worldPosition = (btVector3(0.4f, 0.05, -0.1)) + OFFSET;
    rightAnkleJointInfo.hingeAxis = (btVector3(0, 0, 1));
    rightAnkleJointInfo.initAngle = this->_init_config_states[this->_controllerState][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT];


    /*
    JointConstructionInfo stomachToChestJointInfo;
    stomachToChestJointInfo.name = "stomachToChestJoint";
    stomachToChestJointInfo.linkIndex = 7;
    stomachToChestJointInfo.parentLinkIndex = 6;
    stomachToChestJointInfo.jointType = JointConstructionInfo::SPHERICAL;
    stomachToChestJointInfo.worldPosition = flipYandZ(btVector3(0.0f, 0.0f, 1.275)) + OFFSET;

    JointConstructionInfo chestToRightArmJoint;
    chestToRightArmJoint.name = "chestToRightArmJoint";
    chestToRightArmJoint.linkIndex = 8;
    chestToRightArmJoint.parentLinkIndex = 7;
    chestToRightArmJoint.jointType = JointConstructionInfo::SPHERICAL;
    chestToRightArmJoint.worldPosition = flipYandZ(btVector3(0.0f, -0.175, 1.4)) + OFFSET;

	JointConstructionInfo chestToLeftArmJoint;
	chestToLeftArmJoint.name = "chestToLeftArmJoint";
	chestToLeftArmJoint.linkIndex = 9;
	chestToLeftArmJoint.parentLinkIndex = 7;
	chestToLeftArmJoint.jointType = JointConstructionInfo::SPHERICAL;
	chestToLeftArmJoint.worldPosition = flipYandZ(btVector3(0.0f, 0.175, 1.4)) + OFFSET;

    JointConstructionInfo rightArmToRightForearmJoint;
    rightArmToRightForearmJoint.name = "rightArmToRightForearmJoint";
    rightArmToRightForearmJoint.linkIndex = 10;
    rightArmToRightForearmJoint.parentLinkIndex = 8;
    rightArmToRightForearmJoint.jointType = JointConstructionInfo::REVOLUTE;
    rightArmToRightForearmJoint.worldPosition = flipYandZ(btVector3(0.0f, -0.43, 1.35)) + OFFSET;
    rightArmToRightForearmJoint.hingeAxis = flipYandZ(btVector3(0, 0, 1));

	JointConstructionInfo leftArmToLeftForearmJoint;
	leftArmToLeftForearmJoint.name = "leftArmToLeftForearmJoint";
	leftArmToLeftForearmJoint.linkIndex = 11;
	leftArmToLeftForearmJoint.parentLinkIndex = 9;
	leftArmToLeftForearmJoint.jointType = JointConstructionInfo::REVOLUTE;
	leftArmToLeftForearmJoint.worldPosition = flipYandZ(btVector3(0.0f, 0.43, 1.35)) + OFFSET;
	leftArmToLeftForearmJoint.hingeAxis = flipYandZ(btVector3(0, 0, 1));

    JointConstructionInfo rightForearmToRightHandJoint;
    rightForearmToRightHandJoint.name = "rightForearmToRightHandJoint";
    rightForearmToRightHandJoint.linkIndex = 12;
    rightForearmToRightHandJoint.parentLinkIndex = 10;
    rightForearmToRightHandJoint.jointType = JointConstructionInfo::SPHERICAL;
    rightForearmToRightHandJoint.worldPosition = flipYandZ(btVector3(0.0f, -0.72, 1.35)) + OFFSET;

	JointConstructionInfo leftForearmToLeftHandJoint;
	leftForearmToLeftHandJoint.name = "leftForearmToLeftHandJoint";
	leftForearmToLeftHandJoint.linkIndex = 13;
	leftForearmToLeftHandJoint.parentLinkIndex = 11;
	leftForearmToLeftHandJoint.jointType = JointConstructionInfo::SPHERICAL;
	leftForearmToLeftHandJoint.worldPosition = flipYandZ(btVector3(0.0f, 0.72, 1.35)) + OFFSET;

	JointConstructionInfo chestToHeadJoint;
	chestToHeadJoint.name = "chestToHeadJoint";
	chestToHeadJoint.linkIndex = 14;
	chestToHeadJoint.parentLinkIndex = 7;
	chestToHeadJoint.jointType = JointConstructionInfo::SPHERICAL;
	chestToHeadJoint.worldPosition = flipYandZ(btVector3(0.0f, 0.0, 1.45)) + OFFSET;
*/

    modelInfo.childLinks.push_back(stomachInfo);
    modelInfo.childLinks.push_back(leftUpperLegInfo);
    modelInfo.childLinks.push_back(leftLowerLegInfo);
    modelInfo.childLinks.push_back(leftFootInfo);
    modelInfo.childLinks.push_back(rightUpperLegInfo);
    modelInfo.childLinks.push_back(rightLowerLegInfo);
    modelInfo.childLinks.push_back(rightFootInfo);
    /*
    modelInfo.childLinks.push_back(chestInfo);
    modelInfo.childLinks.push_back(rightArmInfo);
    modelInfo.childLinks.push_back(leftArmInfo);
    modelInfo.childLinks.push_back(rightForearmInfo);
	modelInfo.childLinks.push_back(leftForearmInfo);
	modelInfo.childLinks.push_back(rightHandInfo);
	modelInfo.childLinks.push_back(leftHandInfo);
	modelInfo.childLinks.push_back(headInfo);
*/

    modelInfo.joints.push_back(hipToStomachJointInfo);
    modelInfo.joints.push_back(leftHipJointInfo);
    modelInfo.joints.push_back(leftKneeJointInfo);
    modelInfo.joints.push_back(leftAnkleJointInfo);
    modelInfo.joints.push_back(rightHipJointInfo);
    modelInfo.joints.push_back(rightKneeJointInfo);
    modelInfo.joints.push_back(rightAnkleJointInfo);
    /*
    modelInfo.joints.push_back(stomachToChestJointInfo);
    modelInfo.joints.push_back(chestToRightArmJoint);
    modelInfo.joints.push_back(chestToLeftArmJoint);
    modelInfo.joints.push_back(rightArmToRightForearmJoint);
	modelInfo.joints.push_back(leftArmToLeftForearmJoint);
    modelInfo.joints.push_back(rightForearmToRightHandJoint);
	modelInfo.joints.push_back(leftForearmToLeftHandJoint);
	modelInfo.joints.push_back(chestToHeadJoint);
*/

    m_multiBody = createMultiBodyVehicle(modelInfo, gfxBridge);
    this->config.resize(m_multiBody->getNumLinks());
    m_multiBody->setUseGyroTerm(true);
  }    
  

    btVector4 color = colors[curColor];
    gfxBridge.createCollisionShapeGraphicsObject(m_multiBody->getBaseCollider()->getCollisionShape());
    gfxBridge.createCollisionObjectGraphicsObject(m_multiBody->getBaseCollider(),color);
    curColor++;
	curColor&=3;
    for (size_t link=0; link < m_multiBody->getNumLinks(); link++)
    {
    	btVector4 color = colors[curColor];
		curColor++;
		curColor&=3;
    	gfxBridge.createCollisionShapeGraphicsObject(m_multiBody->getLink(link).m_collider->getCollisionShape());
    	gfxBridge.createCollisionObjectGraphicsObject(m_multiBody->getLink(link).m_collider,color);
    }

    if (1)
      { // Add Ground
          btVector3 groundHalfExtents(10,0.5,10);
          // groundHalfExtents[upAxis]=1.f;
          btBoxShape* box = new btBoxShape(groundHalfExtents);
          box->initializePolyhedralFeatures();

          gfxBridge.createCollisionShapeGraphicsObject(box);
          btTransform start; start.setIdentity();
          btVector3 groundOrigin(0,0,0);
          groundOrigin[upAxis]=-1.0;
          start.setOrigin(groundOrigin);
          btRigidBody* body =  createRigidBody(0,start,box);
          this->_ground = body;
          btVector4 color = colors[curColor];
          curColor++;
          curColor&=3;
          gfxBridge.createRigidBodyGraphicsObject(body,color);
      }


    this->_base_config = m_multiBody->getWorldToBaseRot().getAngle();
    std::cout << "Initial Angle for joint ROOT is " << this->_base_config << std::endl;
    for (size_t con=0; con< this->config.size(); con++)
    {
    	this->config[con] = m_multiBody->getParentToLocalRot(con).getAngle();
    	std::cout << "Initial Angle for joint " << con << " is " << this->config[con] << std::endl;
    }
    m_multiBody->setLinearDamping(1.10);
    m_multiBody->setAngularDamping(1.10);
    std::cout << "Linear dampening: " << m_multiBody->getLinearDamping() << std::endl;
    std::cout << "Angular dampening: " << m_multiBody->getAngularDamping() << std::endl;

    // Set up kd and kp
    this->_Kds.resize(7);
    this->_Kps.resize(7);

    this->_root_Kp = 500.0f;
    this->_Kps[HIP_TO_LEFT_THIGH_JOINT] = 50.0f;
    this->_Kps[LEFT_THIGH_TO_LEFT_CHIN_JOINT] = 80.0f;
    this->_Kps[LEFT_CHIN_TO_LEFT_FOOT_JOINT] = 20.0f;

    this->_Kps[HIP_TO_RIGHT_THIGH_JOINT] = this->_Kps[HIP_TO_LEFT_THIGH_JOINT];
	this->_Kps[RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = this->_Kps[LEFT_THIGH_TO_LEFT_CHIN_JOINT];
	this->_Kps[RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = this->_Kps[LEFT_CHIN_TO_LEFT_FOOT_JOINT];
	this->_Kps[HIPS_TO_TOURSO] = 50;


	this->_root_Kd = 50.0f;
	this->_Kds[HIP_TO_LEFT_THIGH_JOINT] = 350.0f;
	this->_Kds[LEFT_THIGH_TO_LEFT_CHIN_JOINT] = 400.0f;
	this->_Kds[LEFT_CHIN_TO_LEFT_FOOT_JOINT] = 100.0f;

	this->_Kds[HIP_TO_RIGHT_THIGH_JOINT] = this->_Kds[HIP_TO_LEFT_THIGH_JOINT];
	this->_Kds[RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = this->_Kds[LEFT_THIGH_TO_LEFT_CHIN_JOINT];
	this->_Kds[RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = this->_Kds[LEFT_CHIN_TO_LEFT_FOOT_JOINT];
	this->_Kds[HIPS_TO_TOURSO] = 30.0;


	btMultibodyLink rFoot = this->m_multiBody->getLink(RIGHT_FOOT);

	ContactSensorCallback callback(*rFoot.m_collider, *this->_ground);
	m_dynamicsWorld->contactPairTest((rFoot.m_collider), this->_ground, callback);



}

/*
 * Angles in Bullet wrap around at M_PI * 2 to 0
 * i.e. that are between 0 and 2*M_PI
 * error = desiredValue - currentValue
 */
float angleError(float desiredAngle, float currentAngle)
{
	if ( desiredAngle < currentAngle )
	{
		float error1 = desiredAngle - currentAngle;
		float error2 = (desiredAngle - currentAngle)+(M_PI*2);
		if ( fabs(error2) < fabs(error1))
		{
			return error2;
		}
		else
		{
			return error1;
		}
	}
	else
	{
		float error1 = desiredAngle - currentAngle;
		float error2 = (desiredAngle - currentAngle)-(M_PI*2);
		if ( fabs(error2) < fabs(error1))
		{
			return error2;
		}
		else
		{
			return error1;
		}
	}

}

float angleError2(float desiredAngle, float currentAngle)
{
	float error = desiredAngle - currentAngle;
	if ( fabs(error) > M_PI )
	{
		if ( desiredAngle < currentAngle )
		{
			return ((desiredAngle+(2*M_PI)) - currentAngle);
		}
		else
		{
			return ((desiredAngle) - (currentAngle+(2*M_PI)));
		}
	}
	return error;

}


void MultiBodyVehicleSetup::stepSimulation(float deltaTime)
{

      size_t frameNum = m_dynamicsWorld->stepSimulation(deltaTime, 1, FIXED_STEP);
      this->_frameNum++;
      this->checkControllerStates(this->_frameNum, deltaTime);
      this->checkGroundContact(this->_frameNum, deltaTime);
      //m_dynamicsWorld->stepSimulation(deltaTime);
      // std::cout << "frameNum: " << frameNum << " delta time:  " << deltaTime << std::endl;
      // m_multiBody->
      float desiredAngle = 0.0f;
      size_t joint=2;
      float torqueLimit = 200.0;

      // Duh torque limits

/*
      desiredAngle = this->_init_config_states[this->_controllerState][HIPS];

      btQuaternion angleQ =  m_multiBody->getWorldToBaseRot();
      float _angleCurrent = angleQ.getAngle();

      float _errorDerivative =  this->_root_Kp * ((_angleCurrent - this->config[HIPS] )/deltaTime);
      float _angleError = -angleError2(_angleCurrent, desiredAngle);
	  float _errorDifference =  (this->_root_Kd * (_angleError)/deltaTime);
	  // float _errorDifference =  (kp * ((desiredAngle - _angleCurrent))/deltaTime);
	  float appliedTourque = ((_errorDifference) + (_errorDerivative));// * m_multiBody->getLinkMass(joint);
	  if (appliedTourque > torqueLimit )
	  {
		  appliedTourque = torqueLimit;
	  }
	  if (appliedTourque < -torqueLimit )
	  {
		  appliedTourque = -torqueLimit;
	  }
	  m_multiBody->addBaseTorque(btVector3(0,0,appliedTourque));
	  this->config[HIPS] = _angleCurrent;
	  // std::cout << "desiredAngle: " << desiredAngle << " currentAngle for joint " << HIPS << " is " << _angleCurrent << " torque is " <<
	     // 	appliedTourque << " Error: " << _angleError << " ErrorDt: " << _errorDerivative << std::endl;
*/


      for (size_t joint=0; joint < m_multiBody->getNumLinks(); joint++)
      // for (size_t joint=0; joint < 3; joint++)
      {
    	  desiredAngle = this->_init_config_states[this->_controllerState][joint];
    	  btQuaternion angleQ =  m_multiBody->getParentToLocalRot(joint);
    	  float angleCurrent = angleQ.getAngle();
    	  // std::cout << "Angle for joint " << joint << " is " << angleCurrent << std::endl;
    	  // btVector3 getAngularMomentum()
    	  // float errorDerivative =  kd * (((angleCurrent - desiredAngle)) - (this->config[joint] - desiredAngle)/deltaTime);
    	  float errorDerivative =  _Kds[joint] * ((angleCurrent - this->config[joint] )/deltaTime);
    	  // float errorDerivative =  kd * ((this->config[joint+1] - angleCurrent )/deltaTime);
    	  float errorDifference =  (_Kps[joint] * (angleError2(angleCurrent, desiredAngle))/deltaTime);
    	  // float errorDifference =  (kp * ((desiredAngle - angleCurrent))/deltaTime);
    	  // float errorDifference =  (kp * ((angleCurrent - desiredAngle))/deltaTime);
    	  float appliedTourque = ((errorDifference) + (errorDerivative));// * m_multiBody->getLinkMass(joint);
    	  if (appliedTourque > torqueLimit )
    	  {
    		  appliedTourque = torqueLimit;
    	  }
    	  if (appliedTourque < -torqueLimit )
		  {
			  appliedTourque = -torqueLimit;
		  }
    	  m_multiBody->addJointTorque(joint, appliedTourque);
    	  // std::cout << "Angle for joint " << joint << " is " << angleCurrent << " torque is " <<
    		//	appliedTourque << " Error: " << errorDifference << " ErrorDt: " << errorDerivative << std::endl;
    	  this->config[joint] = angleCurrent;
      }



      if ( (this->_controllerState == STANDING_ON_LEFT_FOOT) ||
    		  ( this->_controllerState == START_WALKING_ON_RIGHT_FOOT))
      { // Stance foot is LEFT FOOT

    	  // Balance Feedback
    	  desiredAngle = this->_init_config_states[this->_controllerState][joint];
    	  // Need to change joint torque for HIP_TO_LEFT_THIGH_JOINT
    	  m_multiBody->addJointTorque(HIP_TO_LEFT_THIGH_JOINT,
    			  -m_multiBody->getJointTorque(TOURSO) -
    			  m_multiBody->getJointTorque(HIP_TO_RIGHT_THIGH_JOINT));
      }
      else
      { // Stance foot is RIGHT foot
    	  m_multiBody->addJointTorque(HIP_TO_RIGHT_THIGH_JOINT,
    			  -m_multiBody->getJointTorque(TOURSO) -
				  m_multiBody->getJointTorque(HIP_TO_LEFT_THIGH_JOINT));

      }

      // btScalar * q = btScalar[3];


      btVector3 basePos = m_multiBody->getBasePos();
      basePos.setZ(0.0f);
      m_multiBody->setBasePos(basePos);
      btVector3 baseVel = m_multiBody->getBaseVel();
      baseVel.setZ(0.0);
	  m_multiBody->setBaseVel(baseVel);
	  btVector3 baseOmg = m_multiBody->getBaseOmega();
	  baseOmg.setX(0);
	  baseOmg.setY(0);
	  m_multiBody->setBaseOmega(baseOmg);

      // m_multiBody->setJointPos(0,config[0]);

      // config[0] = (config[0] + (deltaTime));
}

