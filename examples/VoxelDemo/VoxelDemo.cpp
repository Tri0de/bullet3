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

#include <tuple>
#include <map>
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
	btVoxelInfo emptyInfo;
	btVoxelInfo filledInfo;

    std::map<std::tuple<int, int, int>, bool> blocksMap;

	VoxelWorld() {
		emptyInfo.m_blocking = false;
		emptyInfo.m_tracable = false;
		filledInfo.m_blocking = true;
		filledInfo.m_voxelTypeId = 1;
		filledInfo.m_tracable = true;
		filledInfo.m_collisionShape = new btBoxShape((btVector3(btScalar(.5), btScalar(.5), btScalar(.5))));
		filledInfo.m_friction = 0.7;
		filledInfo.m_restitution = 0.5;
		filledInfo.m_rollingFriction = 0.7;
		filledInfo.m_collisionOffset = btVector3(0, 0, 0);

		int range = 15;
		for (int x = -range; x <= range; x++) {
		    for (int z = -range; z <= range; z++) {
		        int height = abs(x) + abs(z);
		        std::tuple<int, int, int> blockPos(x, height, z);
		        blocksMap[blockPos] = true;
		    }
		}
	}

	btVoxelInfo getVoxel(int x, int y, int z) const override {
	    if (blocksMap.find(std::tuple<int, int, int>(x, y, z)) != blocksMap.end()) {
            return filledInfo;
	    }
		return emptyInfo;
	}

    std::map<std::tuple<int, int, int>, bool>::iterator begin() override {
	    return blocksMap.begin();
	}

    std::map<std::tuple<int, int, int>, bool>::iterator end() override {
	    return blocksMap.end();
	}
};

btRigidBody* worldBody;

void VoxelDemo::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	btVoxelContentProvider* provider = new VoxelWorld();

    btVector3 aabbMin = btVector3(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
    btVector3 aabbMax = btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);

	btVoxelShape* voxelWorld = new btVoxelShape(provider, aabbMin, aabbMax);
	// voxelWorld->setLocalScaling(btVector3(0.5, 0.5, 0.5));


	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(voxelWorld);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,0,0));
	{
		btScalar mass(0.);
		worldBody = createRigidBody(mass, groundTransform, voxelWorld, btVector4(0,0,0,0));
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
    btQuaternion initialRotation = worldBody->getWorldTransform().getRotation();
    btQuaternion rotChange;
    rotChange.setRotation(btVector3(0, 1, 0), .01);
    btQuaternion finalRotation = initialRotation * rotChange;

    btTransform newTransform = worldBody->getWorldTransform();
    newTransform.setRotation(finalRotation);
    worldBody->setWorldTransform(newTransform);
	CommonRigidBodyBase::renderScene();
	
}







CommonExampleInterface*    VoxelDemoCreateFunc(CommonExampleOptions& options)
{
	return new VoxelDemo(options.m_guiHelper);
}



