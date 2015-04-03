#ifndef TEST_MULTIBODY_VEHICLE_SETUP_H
#define TEST_MULTIBODY_VEHICLE_SETUP_H

#include "Bullet3AppSupport/CommonMultiBodySetup.h"

#include <map>
#include <string>
#include <vector>

struct Comparator
{
  bool operator()(const btVector3 & u, const btVector3 & v) const;
};

struct MultiBodyVehicleSetup : public CommonMultiBodySetup
{
  struct BodyConstructionInfo
  {
    std::string name;
    btVector3 halfExtents;
    btVector3 position;
    float mass;
    int parentIndex;
  };

  struct JointConstructionInfo
  {
    enum JointType
    {
      SPHERICAL,
      REVOLUTE
    };

    std::string name;

    /*
     * The link in the BodyConstructionInfo vector to join.
     */
    int linkIndex;

    /*
     * The link in the BodyConstructionInfo vector to join linkIndex to.
     */
    int parentLinkIndex;

    JointType jointType;
    btVector3 worldPosition;

    /*
       * Axis of rotation if using hinge joint.
       */
      btVector3 hingeAxis;

      btScalar initAngle;
    };

    struct ModelConstructionInfo
    {
      std::string modelName;

      /*
       * The base link.
       */
      std::string baseName;
      btVector3 baseHalfExtents;
      btVector3 basePosition;
      float baseMass;

      std::vector<BodyConstructionInfo> childLinks;
      std::vector<JointConstructionInfo> joints;
    };

    enum ControlerState
    {
    	START_WALKING_ON_RIGHT_FOOT,
    	STANDING_ON_RIGHT_FOOT,
    	START_WALKING_ON_LEFT_FOOT,
    	STANDING_ON_LEFT_FOOT
    };

    enum jointIndex
    {
    	HIPS,
    	HIP_TO_LEFT_THIGH_JOINT,
    	LEFT_THIGH_TO_LEFT_CHIN_JOINT,
    	LEFT_CHIN_TO_LEFT_FOOT_JOINT,
		HIP_TO_RIGHT_THIGH_JOINT,
		RIGHT_THIGH_TO_RIGHT_CHIN_JOINT,
		RIGHT_CHIN_TO_RIGHT_FOOT_JOINT
    };

	ControlerState _controllerState;

    btMultiBody* m_multiBody;
    std::vector<float> config;
    std::vector<std::vector<float> > _init_config_states;
    size_t _frameNum;

public:

    MultiBodyVehicleSetup();
    virtual ~MultiBodyVehicleSetup();

    virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

    virtual void stepSimulation(float deltaTime);
    // Initilizes the poses and sets the start pose
    virtual void initControllerStates();
    // checks the current state, see if controller should transition
    virtual void checkControllerStates(size_t frameNum, float dt);
    virtual void transitionControllerStates();
    
    class btMultiBody* createMultiBodyVehicle(const ModelConstructionInfo & info, GraphicsPhysicsBridge& gfxBridge);

private:
    std::map<btVector3, btBoxShape *, Comparator> boxShapes;
    std::vector<btMultiBodyLinkCollider *> colliders;
    btBoxShape * getBoxShape(const btVector3 & halfExtents);
};

#endif //TEST_MULTIBODY_VEHICLE_SETUP_H

