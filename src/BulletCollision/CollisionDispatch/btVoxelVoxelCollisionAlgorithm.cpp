/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#include <limits.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <cassert>
#include <assert.h>
#include "BulletCollision/CollisionDispatch/btVoxelVoxelCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btVoxelShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAabbUtil2.h"
#include "btManifoldResult.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "btBoxBoxDetector.h"

btVoxelVoxelCollisionAlgorithm::btVoxelVoxelCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, bool isSwapped)
	: btCollisionAlgorithm(ci),
	  m_ownManifold(false),
	  m_sharedManifold(ci.m_manifold)
{
	btAssert(body0Wrap->getCollisionShape()->isVoxel());
	btAssert(body1Wrap->getCollisionShape()->isVoxel());
}

btVoxelVoxelCollisionAlgorithm::~btVoxelVoxelCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_sharedManifold)
			m_dispatcher->releaseManifold(m_sharedManifold);
	}
}

#define BT_VOXEL_NEGATIVE_INFINITY -999.
#define BT_VOXEL_NORMAL_CONFLICT_THRESHOLD .3

void btVoxelVoxelCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	// These are swapped because I must've mixed up the pointshell and voxmap shapes in the implementation
	// Thankfully this seems to work fine
	const auto* pointShellCollisionObject = body1Wrap;
	const auto* voxMapCollisionObject = body0Wrap;

	// Sanity checks
	btAssert(pointShellCollisionObject->getCollisionShape()->isVoxel());
	btAssert(voxMapCollisionObject->getCollisionShape()->isVoxel());

	// Setup the collision manifold
	// I'm not sure what this code does, but other collision algorithms do this exactly setup this.
	if (!m_sharedManifold) {
		m_sharedManifold = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
	m_sharedManifold->clearManifold();
	resultOut->setPersistentManifold(m_sharedManifold);

	// Setup constants
	const auto* pointShellShape = static_cast<const btVoxelShape*>(pointShellCollisionObject->getCollisionShape());
	const auto* voxMapShape = static_cast<const btVoxelShape*>(voxMapCollisionObject->getCollisionShape());

	const auto& pointShellShapeTransform = pointShellCollisionObject->getWorldTransform();
	const auto& pointShellShapeInverseTransform = pointShellShapeTransform.inverse();

	const auto& voxMapShapeInverseTransform = voxMapCollisionObject->getWorldTransform().inverse();
	const auto& voxMapShapeTransform = voxMapCollisionObject->getWorldTransform();

	const auto pointShellShapeContentProvider = pointShellShape->getContentProvider();
	const auto voxMapShapeContentProvider = voxMapShape->getContentProvider();

	// The box shape used to determine collision points and normals
	// const btBoxShape* boxShape = new btBoxShape(btVector3(.5, .5, .5));

	// Voxel offset directions array
	const btVector3 offsetDirections[6] = {
			btVector3(1, 0, 0),
			btVector3(0, 1, 0),
			btVector3(0, 0, 1),
			btVector3(-1, 0, 0),
			btVector3(0, -1, 0),
			btVector3(0, 0, -1)
	};

	// The possible normals for the point in the point shell shape
	const btVector3 pointNormals[6] = {
			(pointShellShapeTransform.getBasis() * offsetDirections[0]),
			(pointShellShapeTransform.getBasis() * offsetDirections[1]),
			(pointShellShapeTransform.getBasis() * offsetDirections[2]),
			(pointShellShapeTransform.getBasis() * offsetDirections[3]),
			(pointShellShapeTransform.getBasis() * offsetDirections[4]),
			(pointShellShapeTransform.getBasis() * offsetDirections[5])
	};

	// Iterate over every "surface" point in the pointshell, and add collisions to the manifold if any are found.
	for (auto it = pointShellShapeContentProvider->begin(); it != pointShellShapeContentProvider->end(); it++) {
		// In every iteration we are handling collisions between the point (*it) and the voxel it collides with (in other),
		// if there is one.
		const btVector3i blockPos = *it;

		// Run the collision for all 24 points
		for (int i = 0; i < 6; i++) {
			const btVector3 pointNormal = pointNormals[i];
			const btVector3 offsetDirection = offsetDirections[i];

			// Don't add points if there's another block there
			const btVector3i blockPosNextToMe(blockPos.x + (int) offsetDirection.x(), blockPos.y + (int) offsetDirection.y(), blockPos.z + (int) offsetDirection.z());
			const uint8_t blockNextToMeType = pointShellShape->getContentProvider()->getVoxelType(blockPosNextToMe.x, blockPosNextToMe.y, blockPosNextToMe.z);

			// Skip these points
			if (blockNextToMeType == VOX_TYPE_INTERIOR || blockNextToMeType == VOX_TYPE_SURFACE) {
				continue;
			}

			btVector3 facePoints[4];

			const btScalar pointDiameter = .5;

			switch (i) {
				case 0:
					facePoints[0] = btVector3(.75, .25, .25);
					facePoints[1] = btVector3(.75, .25, .75);
					facePoints[2] = btVector3(.75, .75, .25);
					facePoints[3] = btVector3(.75, .75, .75);
					break;
				case 1:
					facePoints[0] = btVector3(.25, .75, .25);
					facePoints[1] = btVector3(.25, .75, .75);
					facePoints[2] = btVector3(.75, .75, .25);
					facePoints[3] = btVector3(.75, .75, .75);
					break;
				case 2:
					facePoints[0] = btVector3(.25, .25, .75);
					facePoints[1] = btVector3(.25, .75, .75);
					facePoints[2] = btVector3(.75, .25, .75);
					facePoints[3] = btVector3(.75, .75, .75);
					break;
				case 3:
					facePoints[0] = btVector3(.25, .25, .25);
					facePoints[1] = btVector3(.25, .25, .75);
					facePoints[2] = btVector3(.25, .75, .25);
					facePoints[3] = btVector3(.25, .75, .75);
					break;
				case 4:
					facePoints[0] = btVector3(.25, .25, .25);
					facePoints[1] = btVector3(.25, .25, .75);
					facePoints[2] = btVector3(.75, .25, .25);
					facePoints[3] = btVector3(.75, .25, .75);
					break;
				case 5:
					facePoints[0] = btVector3(.25, .25, .25);
					facePoints[1] = btVector3(.25, .75, .25);
					facePoints[2] = btVector3(.75, .25, .25);
					facePoints[3] = btVector3(.75, .75, .25);
					break;
			}

			for (auto collisionOffset : facePoints) {
				const btVector3 offsetPos(blockPos.x + collisionOffset.x() - .5, blockPos.y + collisionOffset.y() - .5, blockPos.z + collisionOffset.z() - .5);

				// The position of the point within the local space of the voxel shape.
				const btVector3 positionInOther = voxMapShapeInverseTransform * pointShellShapeTransform * offsetPos; // btVector3(blockPos.x, blockPos.y, blockPos.z);
				// Convert "positionInOther" to a btVector3i to determine the individual voxel we are colliding with
				const btVector3i otherBlockPos((int) round(positionInOther.x()), (int) round(positionInOther.y()), (int) round(positionInOther.z()));

				// The type of the point
				const uint8_t pointType = pointShellShape->getContentProvider()->getVoxelType(blockPos.x, blockPos.y, blockPos.z);
				// The type of the colliding voxel
				const uint8_t voxelType = voxMapShape->getContentProvider()->getVoxelType(otherBlockPos.x, otherBlockPos.y, otherBlockPos.z);


				if (voxelType == VOX_TYPE_AIR) {
					// No collision, move on to the next point
					continue;
				}

				const btVector3 normalInLocal = voxMapShapeInverseTransform.getBasis() * pointNormal;
				const btVector3i destinationBlockPos(
						(int) round(positionInOther.x() - normalInLocal.x()),
						(int) round(positionInOther.y() - normalInLocal.y()),
						(int) round(positionInOther.z() - normalInLocal.z())
				);

				const uint8_t destinationVoxelType = voxMapShape->getContentProvider()->getVoxelType(destinationBlockPos.x, destinationBlockPos.y, destinationBlockPos.z);

				if (destinationBlockPos == otherBlockPos) {
					// TODO: This isn't good, what do we do here?
					continue;
				}

				// Only push points OUTWARDS, do not push inwards!
				if (voxelType == VOX_TYPE_PROXIMITY) {
					if (destinationVoxelType != VOX_TYPE_AIR) {
						continue;
					}
				} else if (voxelType == VOX_TYPE_SURFACE) {
					if (destinationVoxelType != VOX_TYPE_AIR && destinationVoxelType != VOX_TYPE_PROXIMITY) {
						continue;
					}
				} else if (voxelType == VOX_TYPE_INTERIOR) {
					if (destinationVoxelType != VOX_TYPE_AIR && destinationVoxelType != VOX_TYPE_PROXIMITY && destinationVoxelType != VOX_TYPE_SURFACE) {
						continue;
					}
				}


				const auto& pointPosInGlobal = pointShellShapeTransform * offsetPos;
				const auto& voxelCenterInGlobal = voxMapShapeTransform * btVector3(otherBlockPos.x, otherBlockPos.y, otherBlockPos.z);
				const auto& pointVoxelPositionDifference = pointPosInGlobal - voxelCenterInGlobal;


				btScalar collisionDepth = -pointNormal.dot(pointVoxelPositionDifference);

				if (voxelType == VOX_TYPE_PROXIMITY) {
					collisionDepth += .25;
				}

				// Prioritize points in surface and interior voxels by increasing the collision depth
				// TODO: This isn't necessarily right, it depends on the normal direction; but this works for now.
				if (i == 1 || i == 4) {
					if (voxelType == VOX_TYPE_SURFACE) {
						collisionDepth -= .75;
					}
					if (voxelType == VOX_TYPE_INTERIOR) {
						collisionDepth -= 1.75;
					}
				}

				// Check if there was a collision
				if (collisionDepth < 0) {
					resultOut->addContactPoint(pointNormal, pointPosInGlobal, collisionDepth);
				}
			}
		}
	}

	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}
}

btScalar btVoxelVoxelCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	return 0;
}
