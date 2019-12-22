/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "VoxelDemo.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletCollision/CollisionShapes/btVoxelShape.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"


struct VoxelDemo : public CommonRigidBodyBase
{
	VoxelDemo(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~VoxelDemo(){}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 41;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};

struct VoxelWorld : public btVoxelContentProvider
{
	VoxelWorld() {
	}

	void getVoxel(int x, int y, int z,btVoxelInfo& info) const {
		if (y > 0 || abs(x) > 5 || abs(z) > 5) {
			info.m_blocking = false;
            info.m_tracable = false;
		}
        info.m_blocking = true;
        info.m_voxelTypeId = 1;
        info.m_tracable = true;
        info.m_collisionShape = new btBoxShape((btVector3(btScalar(.25), btScalar(.25), btScalar(0.25))));
        info.m_friction = 0.7;
        info.m_restitution = 0.5;
        info.m_rollingFriction = 0.7;
        info.m_collisionOffset = btVector3(0, 0, 0);
	}
};

void VoxelDemo::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	btVoxelContentProvider* provider = new VoxelWorld();

	btVoxelShape* voxelWorld = new btVoxelShape(provider, btVector3(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT), btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT));
	voxelWorld->setLocalScaling(btVector3(0.5, 0.5, 0.5));
		

	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(voxelWorld);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,0,0));
	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, voxelWorld, btVector4(0,0,0,0));
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = new btBoxShape(btVector3(1, 1, 1));
		

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);


		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										btScalar(2.0*i),
										btScalar(20+2.0*k),
										btScalar(2.0*j)));

			
					createRigidBody(mass,startTransform,colShape);
					

				}
			}
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void VoxelDemo::renderScene()
{
	CommonRigidBodyBase::renderScene();
	
}







CommonExampleInterface*    VoxelDemoCreateFunc(CommonExampleOptions& options)
{
	return new VoxelDemo(options.m_guiHelper);
}



