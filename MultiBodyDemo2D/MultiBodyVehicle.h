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
    	// HIPS,
    	// HIPS_TO_TOURSO,
    	HIP_TO_LEFT_THIGH_JOINT,
    	LEFT_THIGH_TO_LEFT_CHIN_JOINT,
    	LEFT_CHIN_TO_LEFT_FOOT_JOINT,
		HIP_TO_RIGHT_THIGH_JOINT,
		RIGHT_THIGH_TO_RIGHT_CHIN_JOINT,
		RIGHT_CHIN_TO_RIGHT_FOOT_JOINT,
    };

    enum linkIndex
	{
		// HIPS,
		// TOURSO,
		LEFT_THIGH,
		LEFT_CHIN,
		LEFT_FOOT,
		RIGHT_THIGH,
		RIGHT_CHIN,
		RIGHT_FOOT,
	};

	ControlerState _controllerState;

    btMultiBody* m_multiBody;
    std::vector<float> config;
    std::vector<float> _Kds;
    std::vector<float> _Kps;

    float _base_config;
    float _root_Kp;
    float _root_Kd;

    std::vector<std::vector<float> > _init_config_states;
    std::vector<float> _init_base_config_states;
    size_t _frameNum;
    size_t _lastTransitionFrameNum;

    btRigidBody * _ground;
    bool _rigth_foot_contact;
    bool _left_foot_contact;



public:

    MultiBodyVehicleSetup();
    virtual ~MultiBodyVehicleSetup();

    virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

    virtual void stepSimulation(float deltaTime);
    // Initilizes the poses and sets the start pose
    virtual void initControllerStates();
    // checks the current state, see if controller should transition
    virtual void checkControllerStates(size_t frameNum, float dt);
    virtual void checkGroundContact(size_t frameNum, float dt);
    virtual void transitionControllerStates();
    
    // void rightFootContactCallback(btRigidBody& rFoot , btRigidBody& ground )
       //      : btCollisionWorld::ContactResultCallback(), body(rFoot)  { }

    class btMultiBody* createMultiBodyVehicle(const ModelConstructionInfo & info, GraphicsPhysicsBridge& gfxBridge);

private:
    std::map<btVector3, btBoxShape *, Comparator> boxShapes;
    std::vector<btMultiBodyLinkCollider *> colliders;
    btBoxShape * getBoxShape(const btVector3 & halfExtents);
};

#include <iostream>

struct ContactSensorCallback : public btCollisionWorld::ContactResultCallback {

    //! Constructor, pass whatever context you want to have available when processing contacts
    /*! You may also want to set m_collisionFilterGroup and m_collisionFilterMask
     *  (supplied by the superclass) for needsCollision() */
    ContactSensorCallback(btCollisionObject& rFoot , btRigidBody& ground  /*, ... */)
        : btCollisionWorld::ContactResultCallback(), _rFoot(rFoot), _ground(ground) { }

    btCollisionObject& _rFoot; //!< The body the sensor is monitoring
    btRigidBody& _ground; //!< The body the sensor is monitoring
    // YourContext& ctxt; //!< External information for contact processing

    //! If you don't want to consider collisions where the bodies are joined by a constraint, override needsCollision:
    /*! However, if you use a btCollisionObject for #body instead of a btRigidBody,
     *  then this is unnecessary—checkCollideWithOverride isn't available */
    virtual bool needsCollision(btBroadphaseProxy* proxy) const {
        // superclass will check m_collisionFilterGroup and m_collisionFilterMask
    	std::cout << "Found SOME?? Foot Ground collision" << std::endl;
        if(!btCollisionWorld::ContactResultCallback::needsCollision(proxy))
            return false;
        // if passed filters, may also want to avoid contacts between constraints
        return _rFoot.checkCollideWithOverride(static_cast<btCollisionObject*>(proxy->m_clientObject));
    }

    //! Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
    virtual btScalar addSingleResult(btManifoldPoint& cp,
        const btCollisionObjectWrapper* colObj0,int partId0,int index0,
        const btCollisionObjectWrapper* colObj1,int partId1,int index1)
    {
        btVector3 pt; // will be set to point of collision relative to body
        if(colObj0->m_collisionObject==&_rFoot) {
            pt = cp.m_localPointA;
        } else {
            assert(colObj1->m_collisionObject==&_rFoot && "body does not match either collision object");
            pt = cp.m_localPointB;
        }
        // do stuff with the collision point
        std::cout << "Found Foot Ground collision" << std::endl;
        return 0; // There was a planned purpose for the return value of addSingleResult, but it is not used so you can ignore it.
    }
};
#endif //TEST_MULTIBODY_VEHICLE_SETUP_H

