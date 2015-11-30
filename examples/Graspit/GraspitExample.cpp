#include "GraspitExample.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"

#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include <vector>


class GraspitExample : public CommonMultiBodyBase
{

public:

    GraspitExample(GUIHelperInterface* helper);
    virtual ~GraspitExample();

    virtual void	initPhysics();

    virtual void	stepSimulation(float deltaTime);

    virtual void resetCamera()
    {
        float dist = 1;
        float pitch = -118;
        float yaw = 38;
        float targetPos[3]={0.244,0,-0.3};
        m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
    }
private:

    btMultiBody* m_multiBody;
};

static btScalar kp = 100;
static btScalar kd = 20;
static btScalar maxForce = 100;
static bool g_floatingBase = false;
static bool g_firstInit = true;
static float scaling = 0.4f;
static float friction = 1.;
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)


#define START_POS_X -5
//#define START_POS_Y 12
#define START_POS_Y 2
#define START_POS_Z -3



GraspitExample::GraspitExample(GUIHelperInterface* helper)
:CommonMultiBodyBase(helper)
{
    m_guiHelper->setUpAxis(2);
}
GraspitExample::~GraspitExample()
{
}

static btAlignedObjectArray<btScalar> qDesiredArray;
void	GraspitExample::stepSimulation(float deltaTime)
{
    //use a smaller internal timestep, there are stability issues
    float internalTimeStep = 1./240.f;
    m_dynamicsWorld->stepSimulation(deltaTime,10,internalTimeStep);

    btScalar target= 0.8;
    qDesiredArray.resize(m_multiBody->getNumLinks(),target);
    for (int joint = 0; joint<m_multiBody->getNumLinks();joint++)
    {
        btScalar qActual = m_multiBody->getJointPos(joint);
        printf("current %d pos: %f\n", joint, qActual);
        btScalar qdActual = m_multiBody->getJointVel(joint);
        btScalar positionError = (qDesiredArray[joint]-qActual);
        double desiredVelocity = 0;
        btScalar velocityError = (desiredVelocity-qdActual);
        btScalar force = kp * positionError + kd*velocityError;
        btClamp(force,-maxForce,maxForce);
        m_multiBody->addJointTorque(joint, force);
        printf("add %d joint torque: %f\n", joint, force);
    }

}


void	GraspitExample::initPhysics()
{
    m_guiHelper->setUpAxis(2);

    if(g_firstInit)
    {
        m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraDistance(btScalar(15));
        m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraPitch(-57.2);
        m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraYaw(42.6);
        g_firstInit = false;
    }
    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();

    //Use the btMultiBodyConstraintSolver for Featherstone btMultiBody support
    btMultiBodyConstraintSolver* sol = new btMultiBodyConstraintSolver;
    m_solver = sol;

    //use btMultiBodyDynamicsWorld for Featherstone btMultiBody support
    btMultiBodyDynamicsWorld* world = new btMultiBodyDynamicsWorld(m_dispatcher,m_broadphase,sol,m_collisionConfiguration);
    m_dynamicsWorld = world;
//	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->setGravity(btVector3(0,0,-0.98));

    ///create a few basic rigid bodies
    btVector3 groundHalfExtents(50,50,50);
    btCollisionShape* groundShape = new btBoxShape(groundHalfExtents);

    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,0,-50));

    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////

    bool damping = false;
    bool gyro = true;
    bool multibodyOnly = false;
    bool canSleep = false;
    bool selfCollide = true;

    BulletURDFImporter u2b(m_guiHelper);
    bool loadOk =  u2b.loadURDF("kinova_kg3/kinova_kg3.urdf");
    if (loadOk)
    {
        int rootLinkIndex = u2b.getRootLinkIndex();
        b3Printf("urdf root link index = %d\n",rootLinkIndex);
        MyMultiBodyCreator creation(m_guiHelper);
        btTransform identityTrans;
        identityTrans.setIdentity();
        ConvertURDF2Bullet(u2b,creation, identityTrans,m_dynamicsWorld,true,u2b.getPathPrefix());
        m_multiBody = creation.getBulletMultiBody();
    }

    g_floatingBase = ! g_floatingBase;
    m_multiBody->setCanSleep(canSleep);
    m_multiBody->setHasSelfCollision(selfCollide);
    m_multiBody->setUseGyroTerm(gyro);

    btQuaternion world_to_base_rot;
    world_to_base_rot.setEulerZYX(0, -SIMD_PI/2 ,0);
    m_multiBody->setWorldToBaseRot(world_to_base_rot);

    //
    if(!damping)
    {
        m_multiBody->setLinearDamping(0.f);
        m_multiBody->setAngularDamping(0.f);
    }else
    {	m_multiBody->setLinearDamping(0.2f);
        m_multiBody->setAngularDamping(0.2f);
    }
    //
    //////////////////////////////////////////////
    /*
    if(numLinks > 0)
    {
        btScalar q0 = 45.f * SIMD_PI/ 180.f;
        if(!spherical)
        {
            m_multiBody->setJointPosMultiDof(0, &q0);
        }
        else
        {
            btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
            quat0.normalize();
            m_multiBody->setJointPosMultiDof(0, quat0);
        }
    }
    */
    ///


    /////////////////////////////////////////////////////////////////
    btScalar groundHeight = -51.55;
    if (!multibodyOnly)
    {
        btScalar mass(0.);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0,0,0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass,localInertia);

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0,0,groundHeight));
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        //add the body to the dynamics world
        m_dynamicsWorld->addRigidBody(body,1,1+2);//,1,1+2);

    }
    /////////////////////////////////////////////////////////////////
    /*
    if(!multibodyOnly)
    {
        btVector3 halfExtents(0.055,0.035, 0.13);
        btBoxShape* colShape = new btBoxShape(halfExtents);
        //btCollisionShape* colShape = new btSphereShape(btScalar(1.));
        m_collisionShapes.push_back(colShape);

        /// Create Dynamic Objects
        btTransform startTransform;
        startTransform.setIdentity();

        btScalar	mass(0.f);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0,0,0);
        if (isDynamic)
            colShape->calculateLocalInertia(mass,localInertia);

        startTransform.setOrigin(btVector3(
                            btScalar(0.12),
                            btScalar(0.0),
                            btScalar(0.0)));


        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        m_dynamicsWorld->addRigidBody(body);//,1,1+2);
    }
    */

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

    /////////////////////////////////////////////////////////////////
}




class CommonExampleInterface*   GraspitExampleCreateFunc(struct CommonExampleOptions& options)
{
    return new GraspitExample(options.m_guiHelper);
}
