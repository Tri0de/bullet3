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
#include "BulletCollision/CollisionDispatch/btVoxelVoxelCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btVoxelShape.h"
#include "LinearMath/btIDebugDraw.h"
#include "btManifoldResult.h"

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
	// I'm not sure what this code does, but other collision algorithms do this to setup their manifold.
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

	// The center of the 4 collision spheres used for each face
	const btVector3 cubeFaceSphereCenters[6][4] = {
			{
					btVector3(.25, -.25, -.25),
					btVector3(.25, -.25, .25),
					btVector3(.25, .25, -.25),
					btVector3(.25, .25, .25)
			},
			{
					btVector3(-.25, .25, -.25),
					btVector3(-.25, .25, .25),
					btVector3(.25, .25, -.25),
					btVector3(.25, .25, .25)
			},
			{
					btVector3(-.25, -.25, .25),
					btVector3(-.25, .25, .25),
					btVector3(.25, -.25, .25),
					btVector3(.25, .25, .25)
			},
			{
					btVector3(-.25, -.25, -.25),
					btVector3(-.25, -.25, .25),
					btVector3(-.25, .25, -.25),
					btVector3(-.25, .25, .25)
			},
			{
					btVector3(-.25, -.25, -.25),
					btVector3(-.25, -.25, .25),
					btVector3(.25, -.25, -.25),
					btVector3(.25, -.25, .25)
			},
			{
					btVector3(-.25, -.25, -.25),
					btVector3(-.25, .25, -.25),
					btVector3(.25, -.25, -.25),
					btVector3(.25, .25, -.25)
			}
	};

	// The size of each sphere we're testing
	const btScalar sphereRadius = .25;

	// Iterate over every "surface" voxel in the pointshell shape, and add collisions to the manifold if any are found.
	for (auto it = pointShellShapeContentProvider->begin(); it != pointShellShapeContentProvider->end(); it++) {
		// The block position of the surface voxel we are testing.
		const btVector3i blockPos = *it;

		// For each surface voxel, process collision for all 6 faces of the voxel
		for (size_t i = 0; i < 6; i++) {
			const btVector3 pointNormal = pointNormals[i];
			const btVector3 offsetDirection = offsetDirections[i];

			// Don't run collision on this face if there is another voxel covering this face
			const btVector3i blockPosNextToMe(blockPos.x + (int) offsetDirection.x(), blockPos.y + (int) offsetDirection.y(), blockPos.z + (int) offsetDirection.z());
			const uint8_t blockNextToMeType = pointShellShapeContentProvider->getVoxelType(btVector3i(blockPosNextToMe.x, blockPosNextToMe.y, blockPosNextToMe.z));
			if (blockNextToMeType == VOX_TYPE_INTERIOR || blockNextToMeType == VOX_TYPE_SURFACE) {
				// Another voxel is covering this face, so skip collision on this face.
				continue;
			}

			// Process collision for the 4 spheres used to approximate this face
			for (const auto collisionOffset : cubeFaceSphereCenters[i]) {
				const btVector3 pointPosInPointShell(blockPos.x + collisionOffset.x(), blockPos.y + collisionOffset.y(), blockPos.z + collisionOffset.z());

				// The position of the point within the local space of the voxel shape.
				const btVector3 pointPosInVoxMap = voxMapShapeInverseTransform * pointShellShapeTransform * pointPosInPointShell; // btVector3(blockPos.x, blockPos.y, blockPos.z);
				// Convert "positionInOther" to a btVector3i to determine the individual voxel we are colliding with
				const btVector3i collidingVoxelPos((int) round(pointPosInVoxMap.x()), (int) round(pointPosInVoxMap.y()), (int) round(pointPosInVoxMap.z()));

				// The type of the colliding voxel
				const uint8_t voxelType = voxMapShapeContentProvider->getVoxelType(btVector3i(collidingVoxelPos.x, collidingVoxelPos.y, collidingVoxelPos.z));

				// If we're colliding with an AIR voxel then there is no collision
				if (voxelType == VOX_TYPE_AIR) {
					continue;
				}

				// Make sure that this collision point won't push the pointshell inside of the voxmap. (We only want to PUSH, not PULL)
				const btVector3 normalInLocal = voxMapShapeInverseTransform.getBasis() * pointNormal;
				const btVector3i destinationBlockPos(
						(int) round(pointPosInVoxMap.x() - normalInLocal.x()),
						(int) round(pointPosInVoxMap.y() - normalInLocal.y()),
						(int) round(pointPosInVoxMap.z() - normalInLocal.z())
				);

				// The type of voxel this collision pushes the point towards
				const uint8_t destinationVoxelType = voxMapShapeContentProvider->getVoxelType(btVector3i(destinationBlockPos.x, destinationBlockPos.y, destinationBlockPos.z));

				if (destinationBlockPos == collidingVoxelPos) {
					// TODO: This isn't good, what do we do here?
					continue;
				}

				// Only push points OUTWARDS, do not push inwards!
				if (voxelType == VOX_TYPE_PROXIMITY) {
					// If we're a PROXIMITY voxel, then we can only push points towards AIR voxels
					if (destinationVoxelType != VOX_TYPE_AIR) {
						continue;
					}
				} else if (voxelType == VOX_TYPE_SURFACE) {
					// If we're a SURFACE voxel, then we can push points towards AIR and PROXIMITY voxels
					if (destinationVoxelType != VOX_TYPE_AIR && destinationVoxelType != VOX_TYPE_PROXIMITY) {
						continue;
					}
				} else if (voxelType == VOX_TYPE_INTERIOR) {
					// If we're an interior voxel, then we can push points towards AIR, PROIXMITY and SURFACE voxels.
					if (destinationVoxelType != VOX_TYPE_AIR && destinationVoxelType != VOX_TYPE_PROXIMITY && destinationVoxelType != VOX_TYPE_SURFACE) {
						continue;
					}
				}

				// At this point we've established that this collision will push the objects apart, not together.
				//
				// Now check if this point is actually colliding.
				const auto& pointPosInGlobal = pointShellShapeTransform * pointPosInPointShell;
				const auto& voxelCenterInGlobal = voxMapShapeTransform * btVector3(collidingVoxelPos.x, collidingVoxelPos.y, collidingVoxelPos.z);
				const auto& pointVoxelPositionDifference = pointPosInGlobal - voxelCenterInGlobal;

				// We compute the collision depth using the tangent plane model, making sure to compensate for the size of the spheres used to approximate the cube
				btScalar collisionDepth = -pointNormal.dot(pointVoxelPositionDifference) + .5 - sphereRadius;

				// Prioritize points in surface and interior voxels by increasing the collision depth
				if (voxelType == VOX_TYPE_SURFACE) {
					// Only increase the collision depth if the collision normal pushes outwards
					if (destinationVoxelType == VOX_TYPE_PROXIMITY) {
						collisionDepth -= 1;
					}
				}
				if (voxelType == VOX_TYPE_INTERIOR) {
					// Only increase the collision depth if the collision normal pushes outwards
					if (destinationVoxelType == VOX_TYPE_SURFACE) {
						collisionDepth -= 2;
					}
				}

				// If a collision occurs at this point, then add it to the contact manifold.
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
