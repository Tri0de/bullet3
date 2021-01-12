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

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include<unordered_set>
#include <BulletCollision/CollisionShapes/btVoxelShapeData.h>

// Bad style, but I need to get a reference to the ground object somehow.
btRigidBody* groundRigidBody = nullptr;

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
		const float dist = 80;
		const float pitch = -25;
		const float yaw = 45;
		const float targetPos[3]={0,0.5,0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	void stepSimulation(float deltaTime) override
	{
		if (m_dynamicsWorld)
		{
			// Enabling this makes the ground move
			if (false) {
				// Before phys ticking, rotate the ground slightly
				btTransform groundBodyTransform = groundRigidBody->getWorldTransform();

				btTransform deltaTransform;
				deltaTransform.setOrigin(btVector3(0, 0, 0));
				deltaTransform.setRotation(btQuaternion(btVector3(0, 1, 1).normalize(), deltaTime * 1));

				// Delta transform will rotate us slightly
				groundBodyTransform = groundBodyTransform * deltaTransform;
				groundRigidBody->setWorldTransform(groundBodyTransform);
			}

			// Now run the phys tick
			m_dynamicsWorld->stepSimulation(deltaTime);
		}
	}
};

void VoxelDemo::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	btVoxelShapeData* littleTowerProvider = new btVoxelShapeData(btVector3i(-128, -10, -128), btVector3i(127, 245, 127));
	littleTowerProvider->initializeLittleTower();

	btVoxelShapeData* sphereProvider = new btVoxelShapeData(btVector3i(-128, -10, -128), btVector3i(127, 245, 127));
	sphereProvider->initializeSphere();

	// Make voxel shapes
	auto* voxelWorldShape = new btVoxelShape(littleTowerProvider, btVector3(-50, -1.5, -50), btVector3(50, 6.5, 50));
	auto* voxelSphereShape = new btVoxelShape(sphereProvider, btVector3(-11.5, -8, -11.5), btVector3(11.5, 8, 11.5));

	// Keep track of the shapes for memory deletion/cleanup once the demo ends
	m_collisionShapes.push_back(voxelWorldShape);
	m_collisionShapes.push_back(voxelSphereShape);


	btVector3 rotationAxis(1, 0, 0);
	rotationAxis.normalize();

	float rotationAngle = 0; // 3.14 / 2; // .3; // 3.14 / 2;
	btQuaternion rotationQuaternion(rotationAxis, rotationAngle);

	// For now, the ground transform is just the origin no rotation transform. Must btVoxelCollisionAlgorithm to support
	// cooler transforms.
	btTransform groundTransform;
	groundTransform.setRotation(rotationQuaternion);
	groundTransform.setOrigin(btVector3(0,0,0));
	{
		btScalar mass(0.);

		groundRigidBody = createRigidBody(mass, groundTransform, voxelWorldShape, btVector4(0, 0, 0, 0));

		// Create another voxel world, make this one fall
		btScalar fallingVoxelWorldMass(10.);
		btTransform fallingTransform;
		fallingTransform.setIdentity();
		fallingTransform.setOrigin(btVector3(0, 200, 0));


		auto* fallingVoxelBody1 = createRigidBody(fallingVoxelWorldMass, fallingTransform, voxelSphereShape, btVector4(0, 0, 0, 0));

		btTransform fallingTransform2;
		fallingTransform2.setIdentity();
		fallingTransform2.setOrigin(btVector3(0, 30, 0));

		auto* fallingVoxelBody2 = createRigidBody(fallingVoxelWorldMass, fallingTransform2, voxelSphereShape, btVector4(0, 0, 0, 0));

		btTransform fallingTransform3;
		fallingTransform3.setIdentity();
		fallingTransform3.setOrigin(btVector3(0, 40, -20));

		auto* fallingVoxelBody3 = createRigidBody(fallingVoxelWorldMass, fallingTransform3, voxelSphereShape, btVector4(0, 0, 0, 0));

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

		/*
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
		 */
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void VoxelDemo::renderScene()
{
	CommonRigidBodyBase::renderScene();
	
}







CommonExampleInterface*	VoxelDemoCreateFunc(CommonExampleOptions& options)
{
	return new VoxelDemo(options.m_guiHelper);
}



