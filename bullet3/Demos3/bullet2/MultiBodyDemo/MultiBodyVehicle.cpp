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

void MultiBodyVehicleSetup::checkControllerStates(size_t frameNum, float dt)
{

	if ( ((frameNum + 1) % 150) == 0) // 500 * 0.3 = 150 frames
	{ // Transition to another state
		this->transitionControllerStates();
		std::cout << "Transitioning between states, current state: " << this->_controllerState << std::endl;
	}
}

void MultiBodyVehicleSetup::transitionControllerStates()
{
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

	this->_controllerState = START_WALKING_ON_RIGHT_FOOT;

	this->_init_config_states.resize(4);
	for (size_t item=0; item < this->_init_config_states.size(); item++)
	{
		this->_init_config_states.at(item).resize(6);
	}


    // Desired joint angles
    this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][HIP_TO_LEFT_THIGH_JOINT] = M_PI_2; // left hip
    this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][LEFT_THIGH_TO_LEFT_CHIN_JOINT] = M_PI_2; // left knee
    this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][LEFT_CHIN_TO_LEFT_FOOT_JOINT] = M_PI_2; // left ankle
    this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][HIP_TO_RIGHT_THIGH_JOINT] = 2.6f; // right hip
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = 2.7f; // right knee
	this->_init_config_states[START_WALKING_ON_RIGHT_FOOT][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = 1.9f; // right ankle

	this->_init_config_states[STANDING_ON_RIGHT_FOOT][HIP_TO_LEFT_THIGH_JOINT] = 1.0f; // left hip
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][LEFT_THIGH_TO_LEFT_CHIN_JOINT] = M_PI_2; // left knee
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][LEFT_CHIN_TO_LEFT_FOOT_JOINT] = M_PI_2; // left ankle
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][HIP_TO_RIGHT_THIGH_JOINT] = M_PI_2; // right hip
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = M_PI_2; // right knee
	this->_init_config_states[STANDING_ON_RIGHT_FOOT][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = M_PI_2; // right ankle

	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][HIP_TO_LEFT_THIGH_JOINT] = 2.6f; // left hip
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][LEFT_THIGH_TO_LEFT_CHIN_JOINT] = 2.7f; // left knee
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][LEFT_CHIN_TO_LEFT_FOOT_JOINT] = 1.9f; // left ankle
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][HIP_TO_RIGHT_THIGH_JOINT] = M_PI_2; // right hip
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT] = M_PI_2; // right knee
	this->_init_config_states[START_WALKING_ON_LEFT_FOOT][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT] = M_PI_2; // right ankle

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
  this->initControllerStates();
  this->_frameNum = 0;

  btVector4 colors[4] =
  {
    btVector4(1,0,0,1),
    btVector4(0,1,0,1),
    btVector4(0,1,1,1),
    btVector4(1,1,0,1),
  };
  int curColor = 0;



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
    modelInfo.baseHalfExtents = (btVector3(0.05f, 0.22f, 0.15f));
    modelInfo.basePosition = (btVector3(-0.03f, 0.825, 0.0f)) + OFFSET;
    modelInfo.baseMass = 10.0f;

    BodyConstructionInfo leftUpperLegInfo;
    leftUpperLegInfo.name = "leftUpperLeg";
    leftUpperLegInfo.halfExtents = (btVector3(0.2, 0.05, 0.05));
    leftUpperLegInfo.position = (btVector3(0.2f, 0.525, 0.1)) + OFFSET;
    leftUpperLegInfo.mass = 1.0f;
    leftUpperLegInfo.parentIndex = -1;

    BodyConstructionInfo leftLowerLegInfo;
    leftLowerLegInfo.name = "leftLowerLeg";
    leftLowerLegInfo.halfExtents = (btVector3(0.05, 0.2, 0.05));
    leftLowerLegInfo.position = (btVector3(0.41f, 0.325, 0.1)) + OFFSET;
    leftLowerLegInfo.mass = 1.0f;
    leftLowerLegInfo.parentIndex = 0;

    BodyConstructionInfo leftFootInfo;
    leftFootInfo.name = "leftFoot";
    leftFootInfo.halfExtents = (btVector3(0.025, 0.105, 0.05));
    leftFootInfo.position = (btVector3(0.4f, 0.025, 0.1)) + OFFSET;
    leftFootInfo.mass = 1.0f;
    leftFootInfo.parentIndex = 1;

    BodyConstructionInfo rightUpperLegInfo;
    rightUpperLegInfo.name = "rightUpperLeg";
    rightUpperLegInfo.halfExtents = (btVector3(0.2, 0.05, 0.05));
    rightUpperLegInfo.position = (btVector3(0.2f, 0.525, -0.1)) + OFFSET;
    rightUpperLegInfo.mass = 1.0f;
    rightUpperLegInfo.parentIndex = -1;

    BodyConstructionInfo rightLowerLegInfo;
    rightLowerLegInfo.name = "rightLowerLeg";
    rightLowerLegInfo.halfExtents = (btVector3(0.05, 0.2, 0.05));
    rightLowerLegInfo.position = (btVector3(0.41f, 0.325, -0.1)) + OFFSET;
    rightLowerLegInfo.mass = 1.0f;
    rightLowerLegInfo.parentIndex = 3;

    BodyConstructionInfo rightFootInfo;
    rightFootInfo.name = "rightFoot";
    rightFootInfo.halfExtents = (btVector3(0.025, 0.105, 0.05));
    rightFootInfo.position = (btVector3(0.4f, 0.025, -0.1)) + OFFSET;
    rightFootInfo.mass = 1.0f;
    rightFootInfo.parentIndex = 4;

    /*
    BodyConstructionInfo stomachInfo;
    stomachInfo.name = "stomach";
    stomachInfo.halfExtents = flipYandZ(btVector3(0.05f, 0.15f, 0.05f));
    stomachInfo.position = flipYandZ(btVector3(0.0f, 0.0f, 1.2)) + OFFSET;
    stomachInfo.mass = 1.0f;
    stomachInfo.parentIndex = -1;

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

    JointConstructionInfo leftHipJointInfo;
    leftHipJointInfo.name = "leftHipJoint";
    leftHipJointInfo.linkIndex = 0;
    leftHipJointInfo.parentLinkIndex = -1;
    leftHipJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    leftHipJointInfo.worldPosition = (btVector3(0.0f, 0.56, 0.1)) + OFFSET;
    leftHipJointInfo.hingeAxis = (btVector3(0, 0, 1));
    leftHipJointInfo.initAngle = this->_init_config_states[this->_controllerState][HIP_TO_LEFT_THIGH_JOINT];

    JointConstructionInfo leftKneeJointInfo;
    leftKneeJointInfo.name = "leftKneeJoint";
    leftKneeJointInfo.linkIndex = 1;
    leftKneeJointInfo.parentLinkIndex = 0;
    leftKneeJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    leftKneeJointInfo.worldPosition = (btVector3(0.45f, 0.56,  0.1)) + OFFSET;
    leftKneeJointInfo.hingeAxis = (btVector3(0, 0, -1));
    leftKneeJointInfo.initAngle = this->_init_config_states[this->_controllerState][LEFT_THIGH_TO_LEFT_CHIN_JOINT];

    JointConstructionInfo leftAnkleJointInfo;
    leftAnkleJointInfo.name = "leftAnkleJoint";
    leftAnkleJointInfo.linkIndex = 2;
    leftAnkleJointInfo.parentLinkIndex = 1;
    leftAnkleJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    leftAnkleJointInfo.worldPosition = (btVector3(0.4f, 0.05, 0.1)) + OFFSET;
    leftAnkleJointInfo.hingeAxis = (btVector3(0, 0, 1));
    leftAnkleJointInfo.initAngle = this->_init_config_states[this->_controllerState][LEFT_CHIN_TO_LEFT_FOOT_JOINT];

    JointConstructionInfo rightHipJointInfo;
    rightHipJointInfo.name = "rightHipJoint";
    rightHipJointInfo.linkIndex = 3;
    rightHipJointInfo.parentLinkIndex = -1;
    rightHipJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    rightHipJointInfo.worldPosition = (btVector3(0.0f, 0.56, -0.1)) + OFFSET;
    rightHipJointInfo.hingeAxis = (btVector3(0, 0, 1));
    rightHipJointInfo.initAngle = this->_init_config_states[this->_controllerState][HIP_TO_RIGHT_THIGH_JOINT];

    JointConstructionInfo rightKneeJointInfo;
    rightKneeJointInfo.name = "rightKneeJoint";
    rightKneeJointInfo.linkIndex = 4;
    rightKneeJointInfo.parentLinkIndex = 3;
    rightKneeJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    rightKneeJointInfo.worldPosition = (btVector3(0.45f, 0.56, -0.1)) + OFFSET;
    rightKneeJointInfo.hingeAxis = (btVector3(0, 0, -1));
    rightKneeJointInfo.initAngle = this->_init_config_states[this->_controllerState][RIGHT_THIGH_TO_RIGHT_CHIN_JOINT];

    JointConstructionInfo rightAnkleJointInfo;
    rightAnkleJointInfo.name = "rightAnkleJoint";
    rightAnkleJointInfo.linkIndex = 5;
    rightAnkleJointInfo.parentLinkIndex = 4;
    rightAnkleJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    rightAnkleJointInfo.worldPosition = (btVector3(0.4f, 0.05, -0.1)) + OFFSET;
    rightAnkleJointInfo.hingeAxis = (btVector3(0, 0, 1));
    rightAnkleJointInfo.initAngle = this->_init_config_states[this->_controllerState][RIGHT_CHIN_TO_RIGHT_FOOT_JOINT];

/*
    JointConstructionInfo hipToStomachJointInfo;
    hipToStomachJointInfo.name = "hipToStomachJoint";
    hipToStomachJointInfo.linkIndex = 6;
    hipToStomachJointInfo.parentLinkIndex = -1;
    hipToStomachJointInfo.jointType = JointConstructionInfo::SPHERICAL;
    hipToStomachJointInfo.worldPosition = flipYandZ(btVector3(0.0f, 0.0f, 1.125)) + OFFSET;

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

    modelInfo.childLinks.push_back(leftUpperLegInfo);
    modelInfo.childLinks.push_back(leftLowerLegInfo);
    modelInfo.childLinks.push_back(leftFootInfo);
    modelInfo.childLinks.push_back(rightUpperLegInfo);
    modelInfo.childLinks.push_back(rightLowerLegInfo);
    modelInfo.childLinks.push_back(rightFootInfo);
  /*
    modelInfo.childLinks.push_back(stomachInfo);
    modelInfo.childLinks.push_back(chestInfo);
    modelInfo.childLinks.push_back(rightArmInfo);
    modelInfo.childLinks.push_back(leftArmInfo);
    modelInfo.childLinks.push_back(rightForearmInfo);
	modelInfo.childLinks.push_back(leftForearmInfo);
	modelInfo.childLinks.push_back(rightHandInfo);
	modelInfo.childLinks.push_back(leftHandInfo);
	modelInfo.childLinks.push_back(headInfo);
*/

    modelInfo.joints.push_back(leftHipJointInfo);
    modelInfo.joints.push_back(leftKneeJointInfo);
    modelInfo.joints.push_back(leftAnkleJointInfo);
    modelInfo.joints.push_back(rightHipJointInfo);
    modelInfo.joints.push_back(rightKneeJointInfo);
    modelInfo.joints.push_back(rightAnkleJointInfo);
  /*
    modelInfo.joints.push_back(hipToStomachJointInfo);
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
    this->config[0] = 0.0;
    m_multiBody->setUseGyroTerm(true);
  }    
  
    if (1)
    {
        btVector3 groundHalfExtents(20,20,20);
        groundHalfExtents[upAxis]=1.f;
        btBoxShape* box = new btBoxShape(groundHalfExtents);
        box->initializePolyhedralFeatures();
        
        gfxBridge.createCollisionShapeGraphicsObject(box);
        btTransform start; start.setIdentity();
        btVector3 groundOrigin(0,0,0);
        groundOrigin[upAxis]=-1.5;
        start.setOrigin(groundOrigin);
        btRigidBody* body =  createRigidBody(0,start,box);
        btVector4 color = colors[curColor];
        curColor++;
        curColor&=3;
        gfxBridge.createRigidBodyGraphicsObject(body,color);
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

    for (size_t con=0; con< this->config.size(); con++)
    {
    	this->config[con] = m_multiBody->getParentToLocalRot(con).getAngle();
    	std::cout << "Initial Angle for joint " << con << " is " << this->config[con] << std::endl;
    }
    m_multiBody->setLinearDamping(0.10);
    m_multiBody->setAngularDamping(0.10);
    std::cout << "Linear dampening: " << m_multiBody->getLinearDamping() << std::endl;
    std::cout << "Angular dampening: " << m_multiBody->getAngularDamping() << std::endl;

}

void MultiBodyVehicleSetup::stepSimulation(float deltaTime)
{

      size_t frameNum = m_dynamicsWorld->stepSimulation(deltaTime, 1, FIXED_STEP);
      this->_frameNum++;
      this->checkControllerStates(this->_frameNum, deltaTime);
      //m_dynamicsWorld->stepSimulation(deltaTime);
      // std::cout << "frameNum: " << frameNum << " delta time:  " << deltaTime << std::endl;
      // m_multiBody->
      float kp = 10.f;
      float kd = 70.0f;
      float desiredAngle = 0.0f;
      size_t joint=2;
      float torqueLimit = 100.0;

      // Duh torque limits
      for (size_t joint=0; joint < m_multiBody->getNumLinks(); joint++)
      // for (size_t joint=0; joint < 3; joint++)
      {
    	  desiredAngle = this->_init_config_states[this->_controllerState][joint];
    	  btQuaternion angleQ =  m_multiBody->getParentToLocalRot(joint);
    	  float angleCurrent = angleQ.getAngle();
    	  // std::cout << "Angle for joint " << joint << " is " << angleCurrent << std::endl;
    	  // btVector3 getAngularMomentum()
    	  // float errorDerivative =  kd * (((angleCurrent - desiredAngle)) - (this->config[joint] - desiredAngle)/deltaTime);
    	  float errorDerivative =  kd * ((angleCurrent - this->config[joint] )/deltaTime);
    	  float errorDifference =  (kp * ((angleCurrent - desiredAngle))/deltaTime);
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
      // std::cout << "joint tourqe is " << m_multiBody->getJointTorque(1) << std::endl;
      // config[0] = (config[0] + (deltaTime));
}

