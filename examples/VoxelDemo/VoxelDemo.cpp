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

#include<unordered_set >

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
		float dist = 20;
		float pitch = -45;
		float yaw = 45;
		float targetPos[3]={0,0.5,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
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

struct VoxelWorld : public btVoxelContentProvider
{
	std::vector<btVector3i> setOfBlocks;

	uint8_t* voxelData;
	int minX, maxX, minY, maxY, minZ, maxZ;

	// This is only correct assuming scaling is <1,1,1>
	// Should always be equal to scaling / 2
	btBoxShape* typicalBox = new btBoxShape((btVector3(btScalar(.5), btScalar(.5), btScalar(.5))));

	VoxelWorld() {
		minX = minY = minZ = -128;
		maxX = maxY = maxZ = 127;
		voxelData = (uint8_t*) calloc((maxX - minX + 1) * (maxY - minY + 1) * (maxZ - minZ + 1), sizeof(uint8_t));
		int radius = 10;
		for (int y = 0; y < 5; y++) {
			for (int x = -radius; x <= radius; x++) {
				for (int z = -radius; z <= radius; z++) {
					int yShapeThing = y;
					if (abs(x) + abs(z) > 2) {
						yShapeThing = 0;
					}
						// Make a checkerboard pattern in the voxel world
					else if ((x + z) % 2 == 0) {
						continue;
					}

					btVector3i blockPos(x, yShapeThing, z);
					setOfBlocks.push_back(blockPos);
					voxelData[convertPosToIndex(x, yShapeThing, z)] = 1;

					for (int xOff = -1; xOff <= 1; xOff++) {
						for (int yOff = -1; yOff <= 1; yOff++) {
							for (int zOff = -1; zOff <= 1; zOff++) {
								if (xOff == 0 && yOff == 0 && zOff == 0) {
									continue;
								}
								// Promote type to hasNeighbors
								voxelData[convertPosToIndex(x + xOff, yShapeThing + yOff, z + zOff)] |= 128;
							}
						}
					}
				}
			}
		}
	}

	bool isSurfaceOrSet(int x, int y, int z) const override {
		uint8_t data = voxelData[convertPosToIndex(x, y, z)];
		return ((data & 1) == 1) || ((data & 128) == 128);
	}

	void getVoxel(int x, int y, int z,btVoxelInfo& info) const override {
		btVector3i blockPos(x, y, z);

		if ((voxelData[convertPosToIndex(x, y, z)] & 1) == 1) {
			info.m_blocking = true;
			info.m_voxelTypeId = 1;
			info.m_tracable = true;
			info.m_collisionShape = typicalBox;
			info.m_friction = 0.7;
			info.m_restitution = 0.5;
			info.m_rollingFriction = 0.7;
			info.m_collisionOffset = btVector3(0, 0, 0);
		} else {
			info.m_blocking = false;
			info.m_tracable = false;
		}
	}

	int convertPosToIndex(int x, int y, int z) const {
		size_t xLen = maxX - minX + 1;
		size_t yLen = maxY - minY + 1;
		return (x - minX) + xLen * (y - minY) + xLen * yLen * (z - minZ);
	}

	std::vector<btVector3i>::const_iterator begin() const override {
		return setOfBlocks.begin();
	}

	std::vector<btVector3i>::const_iterator end() const override {
		return setOfBlocks.end();
	}

	bool isProximity(int x, int y, int z) const override {
		return (voxelData[convertPosToIndex(x, y, z)] & 128u) != 0;
	}

	bool isSurface(int x, int y, int z) const override {
		return (voxelData[convertPosToIndex(x, y, z)] & 1u) != 0;
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

	auto* voxelWorld = new btVoxelShape(provider, btVector3(-11.5, -1.5, -11.5), btVector3(11.5, 6.5, 11.5));

	// For now, just don't support scaling
	// voxelWorld->setLocalScaling(btVector3(0.5, 0.5, 0.5));
		

	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(voxelWorld);

	btVector3 rotationAxis(1, 0, 0);
	rotationAxis.normalize();

	float rotationAngle = 0;
	btQuaternion rotationQuaternion(rotationAxis, rotationAngle);

	// For now, the ground transform is just the origin no rotation transform. Must btVoxelCollisionAlgorithm to support
	// cooler transforms.
	btTransform groundTransform;
	groundTransform.setRotation(rotationQuaternion);
	groundTransform.setOrigin(btVector3(0,0,0));
	{
		btScalar mass(0.);


		// Create another voxel world, make this one fall
		btScalar fallingVoxelWorldMass(10.);
		btTransform fallingTransform;
		fallingTransform.setIdentity();
		fallingTransform.setOrigin(btVector3(0, 50, 0));

		// auto* fallingVoxelWorld = createRigidBody(fallingVoxelWorldMass, fallingTransform, voxelWorld, btVector4(0,0,0,0));

		btTransform fallingTransform2;
		fallingTransform2.setIdentity();
		fallingTransform2.setOrigin(btVector3(3, 30, 0));

		// auto* fallingVoxelWorld2 = createRigidBody(fallingVoxelWorldMass, fallingTransform2, voxelWorld, btVector4(0,0,0,0));

		btTransform fallingTransform3;
		fallingTransform3.setIdentity();
		fallingTransform3.setOrigin(btVector3(0, 20, -4));



		auto* fallingVoxelWorld3 = createRigidBody(fallingVoxelWorldMass, fallingTransform3, voxelWorld, btVector4(0,0,0,0));

		groundRigidBody = createRigidBody(mass, groundTransform, voxelWorld, btVector4(0,0,0,0));
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



