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

btVector3 flipYandZ(const btVector3 & v)
{
	return btVector3(v.getX(), v.getZ(), v.getY());
	// return btVector3(v.getX(), v.getY(), v.getZ());
}

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

  btMultiBody * multiBody = new btMultiBody(NUM_LINKS,
      static_cast<btScalar>(info.baseMass),
      inertia,
      true,
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

      multiBody->setupRevolute(JOINT.linkIndex,
          LINK.mass,
          localInertia,
          JOINT.parentLinkIndex,
          btQuaternion(0,0,0,1).inverse(),
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


void MultiBodyVehicleSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
  int upAxis = 1;

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
    const btVector3 OFFSET(flipYandZ(btVector3(1 * i, 1 * i, 0.05)));
    ModelConstructionInfo modelInfo;
    modelInfo.modelName = "M";//  + std::to_string(i);
    modelInfo.baseName = "hips";
    modelInfo.baseHalfExtents = flipYandZ(btVector3(0.05f, 0.15f, 0.05f));
    modelInfo.basePosition = flipYandZ(btVector3(0.0f, 0.0f, 1.05)) + OFFSET;
    modelInfo.baseMass = 1.0f;

    BodyConstructionInfo leftUpperLegInfo;
    leftUpperLegInfo.name = "leftUpperLeg";
    leftUpperLegInfo.halfExtents = flipYandZ(btVector3(0.05, 0.05, 0.2));
    leftUpperLegInfo.position = flipYandZ(btVector3(0.0f, 0.1, 0.75)) + OFFSET;
    leftUpperLegInfo.mass = 1.0f;
    leftUpperLegInfo.parentIndex = -1;

    BodyConstructionInfo leftLowerLegInfo;
    leftLowerLegInfo.name = "leftLowerLeg";
    leftLowerLegInfo.halfExtents = flipYandZ(btVector3(0.05, 0.05, 0.2));
    leftLowerLegInfo.position = flipYandZ(btVector3(0.0f, 0.1, 0.3)) + OFFSET;
    leftLowerLegInfo.mass = 1.0f;
    leftLowerLegInfo.parentIndex = 0;

    BodyConstructionInfo leftFootInfo;
    leftFootInfo.name = "leftFoot";
    leftFootInfo.halfExtents = flipYandZ(btVector3(0.105, 0.05, 0.025));
    leftFootInfo.position = flipYandZ(btVector3(0.055f, 0.1, 0.025)) + OFFSET;
    leftFootInfo.mass = 1.0f;
    leftFootInfo.parentIndex = 1;

    BodyConstructionInfo rightUpperLegInfo;
    rightUpperLegInfo.name = "rightUpperLeg";
    rightUpperLegInfo.halfExtents = flipYandZ(btVector3(0.05, 0.05, 0.2));
    rightUpperLegInfo.position = flipYandZ(btVector3(0.0f, -0.1, 0.75)) + OFFSET;
    rightUpperLegInfo.mass = 1.0f;
    rightUpperLegInfo.parentIndex = -1;

    BodyConstructionInfo rightLowerLegInfo;
    rightLowerLegInfo.name = "rightLowerLeg";
    rightLowerLegInfo.halfExtents = flipYandZ(btVector3(0.05, 0.05, 0.2));
    rightLowerLegInfo.position = flipYandZ(btVector3(0.0f, -0.1, 0.3)) + OFFSET;
    rightLowerLegInfo.mass = 1.0f;
    rightLowerLegInfo.parentIndex = 3;

    BodyConstructionInfo rightFootInfo;
    rightFootInfo.name = "rightFoot";
    rightFootInfo.halfExtents = flipYandZ(btVector3(0.105, 0.05, 0.025));
    rightFootInfo.position = flipYandZ(btVector3(0.055f, -0.1, 0.025)) + OFFSET;
    rightFootInfo.mass = 1.0f;
    rightFootInfo.parentIndex = 4;

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


    JointConstructionInfo leftHipJointInfo;
    leftHipJointInfo.name = "leftHipJoint";
    leftHipJointInfo.linkIndex = 0;
    leftHipJointInfo.parentLinkIndex = -1;
    leftHipJointInfo.jointType = JointConstructionInfo::SPHERICAL;
    leftHipJointInfo.worldPosition = flipYandZ(btVector3(0.0f, 0.1, 0.975)) + OFFSET;

    JointConstructionInfo leftKneeJointInfo;
    leftKneeJointInfo.name = "leftKneeJoint";
    leftKneeJointInfo.linkIndex = 1;
    leftKneeJointInfo.parentLinkIndex = 0;
    leftKneeJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    leftKneeJointInfo.worldPosition = flipYandZ(btVector3(0.0f, 0.1, 0.525)) + OFFSET;
    leftKneeJointInfo.hingeAxis = flipYandZ(btVector3(0, 1, 0));

    JointConstructionInfo leftAnkleJointInfo;
    leftAnkleJointInfo.name = "leftAnkleJoint";
    leftAnkleJointInfo.linkIndex = 2;
    leftAnkleJointInfo.parentLinkIndex = 1;
    leftAnkleJointInfo.jointType = JointConstructionInfo::SPHERICAL;
    leftAnkleJointInfo.worldPosition = flipYandZ(btVector3(0.0f, 0.1, 0.075)) + OFFSET;

    JointConstructionInfo rightHipJointInfo;
    rightHipJointInfo.name = "rightHipJoint";
    rightHipJointInfo.linkIndex = 3;
    rightHipJointInfo.parentLinkIndex = -1;
    rightHipJointInfo.jointType = JointConstructionInfo::SPHERICAL;
    rightHipJointInfo.worldPosition = flipYandZ(btVector3(0.0f, -0.1, 0.975)) + OFFSET;

    JointConstructionInfo rightKneeJointInfo;
    rightKneeJointInfo.name = "rightKneeJoint";
    rightKneeJointInfo.linkIndex = 4;
    rightKneeJointInfo.parentLinkIndex = 3;
    rightKneeJointInfo.jointType = JointConstructionInfo::REVOLUTE;
    rightKneeJointInfo.worldPosition = flipYandZ(btVector3(0.0f, -0.1, 0.525)) + OFFSET;
    rightKneeJointInfo.hingeAxis = flipYandZ(btVector3(0, 1, 0));

    JointConstructionInfo rightAnkleJointInfo;
    rightAnkleJointInfo.name = "rightAnkleJoint";
    rightAnkleJointInfo.linkIndex = 5;
    rightAnkleJointInfo.parentLinkIndex = 4;
    rightAnkleJointInfo.jointType = JointConstructionInfo::SPHERICAL;
    rightAnkleJointInfo.worldPosition = flipYandZ(btVector3(0.0f, -0.1, 0.075)) + OFFSET;

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


    modelInfo.childLinks.push_back(leftUpperLegInfo);
    modelInfo.childLinks.push_back(leftLowerLegInfo);
    modelInfo.childLinks.push_back(leftFootInfo);
    modelInfo.childLinks.push_back(rightUpperLegInfo);
    modelInfo.childLinks.push_back(rightLowerLegInfo);
    modelInfo.childLinks.push_back(rightFootInfo);
    modelInfo.childLinks.push_back(stomachInfo);
    modelInfo.childLinks.push_back(chestInfo);
    modelInfo.childLinks.push_back(rightArmInfo);
    modelInfo.childLinks.push_back(leftArmInfo);
    modelInfo.childLinks.push_back(rightForearmInfo);
	modelInfo.childLinks.push_back(leftForearmInfo);
	modelInfo.childLinks.push_back(rightHandInfo);
	modelInfo.childLinks.push_back(leftHandInfo);
	modelInfo.childLinks.push_back(headInfo);


    modelInfo.joints.push_back(leftHipJointInfo);
    modelInfo.joints.push_back(leftKneeJointInfo);
    modelInfo.joints.push_back(leftAnkleJointInfo);
    modelInfo.joints.push_back(rightHipJointInfo);
    modelInfo.joints.push_back(rightKneeJointInfo);
    modelInfo.joints.push_back(rightAnkleJointInfo);
    modelInfo.joints.push_back(hipToStomachJointInfo);
    modelInfo.joints.push_back(stomachToChestJointInfo);
    modelInfo.joints.push_back(chestToRightArmJoint);
    modelInfo.joints.push_back(chestToLeftArmJoint);
    modelInfo.joints.push_back(rightArmToRightForearmJoint);
	modelInfo.joints.push_back(leftArmToLeftForearmJoint);
    modelInfo.joints.push_back(rightForearmToRightHandJoint);
	modelInfo.joints.push_back(leftForearmToLeftHandJoint);
	modelInfo.joints.push_back(chestToHeadJoint);

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


}

void MultiBodyVehicleSetup::stepSimulation(float deltaTime)
{

      m_dynamicsWorld->stepSimulation(deltaTime, 1, FIXED_STEP);
      //m_dynamicsWorld->stepSimulation(deltaTime);
      // std::cout << "MultiBody: " << m_multiBody << " delta time:  " << deltaTime << std::endl;
      // m_multiBody->
      // this->config[0] = 10.0f;
      m_multiBody->addJointTorque(1, this->config[0]);
      // btScalar * q = btScalar[3];
      // m_multiBody->setJointPos(0,config[0]);
      // std::cout << "joint tourqe is " << m_multiBody->getJointTorque(1) << std::endl;
      config[0] = (config[0] + (deltaTime));
}

