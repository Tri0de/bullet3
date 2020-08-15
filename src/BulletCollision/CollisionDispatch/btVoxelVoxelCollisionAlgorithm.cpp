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
			btVector3(0, 1, 0), // btVector3(1, 0, 0),
			btVector3(0, 1, 0), // btVector3(0, 1, 0),
			btVector3(0, 1, 0), // btVector3(0, 0, 1),
			btVector3(0, -1, 0), // btVector3(-1, 0, 0),
			btVector3(0, -1, 0), // btVector3(0, -1, 0),
			btVector3(0, -1, 0), // btVector3(0, 0, -1)
	};

	// The possible normals for the point in the point shell shape
	const btVector3 pointNormals[6] = {
			pointShellShapeTransform.getBasis() * offsetDirections[0],
			pointShellShapeTransform.getBasis() * offsetDirections[1],
			pointShellShapeTransform.getBasis() * offsetDirections[2],
			pointShellShapeTransform.getBasis() * offsetDirections[3],
			pointShellShapeTransform.getBasis() * offsetDirections[4],
			pointShellShapeTransform.getBasis() * offsetDirections[5]
	};

	// The possible normals for the voxels in the voxel shape
	const btVector3 voxelNormals[6] = {
			(voxMapShapeTransform.getBasis() * offsetDirections[0]),
			(voxMapShapeTransform.getBasis() * offsetDirections[1]),
			(voxMapShapeTransform.getBasis() * offsetDirections[2]),
			(voxMapShapeTransform.getBasis() * offsetDirections[3]),
			(voxMapShapeTransform.getBasis() * offsetDirections[4]),
			(voxMapShapeTransform.getBasis() * offsetDirections[5])
	};

	// Even more sanity checks
	// btAssert(sizeof(pointNormals) == sizeof(offsetDirections))
	// btAssert(sizeof(voxelNormals) == sizeof(offsetDirections))

	// Iterate over every "surface" point in the pointshell, and add collisions to the manifold if any are found.
	for (auto it = pointShellShapeContentProvider->begin(); it != pointShellShapeContentProvider->end(); it++) {
		// In every iteration we are handling collisions between the point (*it) and the voxel it collides with (in other),
		// if there is one.
		const btVector3i blockPos = *it;

		// The position of the point within the local space of the voxel shape.
		const btVector3 positionInOther = voxMapShapeInverseTransform * pointShellShapeTransform * btVector3(blockPos.x, blockPos.y, blockPos.z);
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

		// Sanity check
		// btAssert(pointType == VOX_TYPE_SURFACE);
		// btAssert(voxelType != VOX_TYPE_AIR);

		bool pointAllowedNormals[6] = {false, false, false, false, false, false};
		bool voxelAllowedNormals[6] = {false, false, false, false, false, false};

		// Determine allowed normals for the point
		for (size_t index = 0; index < 6; index++) {
			const btVector3 offsetDirection = offsetDirections[index];
			const uint8_t offsetVoxelType = pointShellShape->getContentProvider()->getVoxelType(blockPos.x + (int) offsetDirection.x(), blockPos.y - (int) offsetDirection.y(), blockPos.z + (int) offsetDirection.z());
			// Allow pushing from surface to proximity
			if (pointType == VOX_TYPE_SURFACE && offsetVoxelType == VOX_TYPE_PROXIMITY) {
				// We can push in this direction
				pointAllowedNormals[index] = true;
			}
		}

		// Determine allowed normals for the voxel
		for (size_t index = 0; index < 6; index++) {
			const btVector3 offsetDirection = offsetDirections[index];
			const uint8_t offsetVoxelType = voxMapShape->getContentProvider()->getVoxelType(otherBlockPos.x + (int) offsetDirection.x(), otherBlockPos.y + (int) offsetDirection.y(), otherBlockPos.z + (int) offsetDirection.z());

			// Allow pushing from surface to proximity
			if (voxelType == VOX_TYPE_SURFACE && offsetVoxelType == VOX_TYPE_PROXIMITY) {
				// We can push in this direction
				voxelAllowedNormals[index] = true;
			}

			// Allow pushing from proximity to air
			if (voxelType == VOX_TYPE_PROXIMITY && offsetVoxelType == VOX_TYPE_AIR) {
				// We can push in this direction
				voxelAllowedNormals[index] = true;
			}

			// Allow pushing from interior to surface (idk about this one)
			if (voxelType == VOX_TYPE_INTERIOR && offsetVoxelType == VOX_TYPE_SURFACE) {
				// We can push in this direction
				voxelAllowedNormals[index] = true;
			}
		}

		// Then determine the normals that are compatible with each other
		//
		// "SecondPass" as in these are the old allowed normals, except verified that they don't push in the direction
		// of a normal that is not allowed.
		bool pointAllowedNormalsSecondPass[6] = {false, false, false, false, false, false};
		bool voxelAllowedNormalsSecondPass[6] = {false, false, false, false, false, false};

		// Determine which point potential normals are compatible with the voxel potential normals
		for (size_t pointIndex = 0; pointIndex < 6; pointIndex++) {
			if (pointAllowedNormals[pointIndex]) {
				const auto potentialPointNormal = pointNormals[pointIndex];
				// Determine if this point potential normal conflicts with the allowed normals of the voxel
				bool doesThisNormalConflict = false;

				for (size_t voxelIndex = 0; voxelIndex < 6; voxelIndex++) {
					if (!voxelAllowedNormals[voxelIndex]) {
						const btScalar dotProduct = potentialPointNormal.dot(voxelNormals[voxelIndex]);
						if (dotProduct > BT_VOXEL_NORMAL_CONFLICT_THRESHOLD) {
							doesThisNormalConflict = true;
							break;
						}
					}
				}

				if (!doesThisNormalConflict) {
					pointAllowedNormalsSecondPass[pointIndex] = true;
				}
			}
		}

		// Determine which voxel potential normals are compatible with the point potential normals
		for (size_t voxelIndex = 0; voxelIndex < 6; voxelIndex++) {
			if (voxelAllowedNormals[voxelIndex]) {
				const auto potentialVoxelNormal = voxelNormals[voxelIndex];
				// Determine if this point potential normal conflicts with the allowed normals of the voxel
				bool doesThisNormalConflict = false;

				for (size_t pointIndex = 0; pointIndex < 6; pointIndex++) {
					if (!pointAllowedNormals[pointIndex]) {
						const btScalar dotProduct = potentialVoxelNormal.dot(pointNormals[pointIndex]);
						if (dotProduct < -BT_VOXEL_NORMAL_CONFLICT_THRESHOLD) {
							doesThisNormalConflict = true;
							break;
						}
					}
				}

				if (!doesThisNormalConflict) {
					voxelAllowedNormalsSecondPass[voxelIndex] = true;
				}
			}
		}


		// This isn't good style, or good for performance; but it does make the code simpler
		const btVector3 allNormals[12] = {
				pointNormals[0],
				pointNormals[1],
				pointNormals[2],
				pointNormals[3],
				pointNormals[4],
				pointNormals[5],
				voxelNormals[0],
				voxelNormals[1],
				voxelNormals[2],
				voxelNormals[3],
				voxelNormals[4],
				voxelNormals[5]
		};

		const bool isNormalUsable[12] = {
				false, // pointAllowedNormals[0], // pointAllowedNormalsSecondPass[0],
				false, // pointAllowedNormals[1], // pointAllowedNormalsSecondPass[1],
				false, // pointAllowedNormals[2], // pointAllowedNormalsSecondPass[2],
				false, // pointAllowedNormals[3], // pointAllowedNormalsSecondPass[3],
				false, // pointAllowedNormals[4], // pointAllowedNormalsSecondPass[4],
				false, // pointAllowedNormals[5], // pointAllowedNormalsSecondPass[5],
				voxelAllowedNormals[0], // true, // pointAllowedNormals[0], // false, // voxelAllowedNormalsSecondPass[0],
				voxelAllowedNormals[1], // true, // pointAllowedNormals[1], // false, // voxelAllowedNormalsSecondPass[1],
				voxelAllowedNormals[2], // true, // pointAllowedNormals[2], // false, // voxelAllowedNormalsSecondPass[2],
				voxelAllowedNormals[3], // true, // pointAllowedNormals[3], // false, // voxelAllowedNormalsSecondPass[3],
				voxelAllowedNormals[4], // true, // pointAllowedNormals[4], // false, // voxelAllowedNormalsSecondPass[4],
				voxelAllowedNormals[5], // true, // pointAllowedNormals[5], // false, // voxelAllowedNormalsSecondPass[5],
		};

		// Calculate the offset between the point and voxel
		const auto& pointPosInGlobal = pointShellShapeTransform * btVector3(blockPos.x, blockPos.y, blockPos.z);
		const auto& voxelCenterInGlobal = voxMapShapeTransform * btVector3(otherBlockPos.x, otherBlockPos.y, otherBlockPos.z);
		const auto& pointVoxelPositionDifference = pointPosInGlobal - voxelCenterInGlobal;

		int minNormalIndex = -1;
		btScalar minCollisionDistance = -999;

		for (size_t normalIndex = 0; normalIndex < 12; normalIndex++) {
			if (!isNormalUsable[normalIndex]) {
				// Skip normals that will push the 2 shapes further inside each other
				continue;
			}
			const auto collisionNormal = allNormals[normalIndex];
			btScalar collisionDepth = collisionNormal.dot(pointVoxelPositionDifference);

			// Prioritize points in surface and interior voxels by increasing the collision depth
			if (voxelType == VOX_TYPE_SURFACE) {
				collisionDepth -= .5;
			}
			if (voxelType == VOX_TYPE_INTERIOR) {
				collisionDepth -= 1.5;
			}

			// Check if there was a collision
			if (collisionDepth < 0) {
				// Check if this normal collision has a lower penetration distance than the current min normal
				if (collisionDepth > minCollisionDistance) {
					minNormalIndex = normalIndex;
					minCollisionDistance = collisionDepth;

					// resultOut->addContactPoint(-allNormals[minNormalIndex], pointPosInGlobal, minCollisionDistance);
				}
			}
		}

		// Make sure we actually had a collision
		if (minNormalIndex != -1) {
			// Use the normal with the smallest penetration distance
			resultOut->addContactPoint(-allNormals[minNormalIndex], pointPosInGlobal, minCollisionDistance);
		}


		/*

		auto func = [pointShellShape, voxMapShape, blockPos, pointShellShapeTransform, voxMapShapeTransform, voxMapShapeInverseTransform, resultOut] (const btVector3 offset, const btVector3 normal) {

			btVector3 positionInOther(blockPos.x, blockPos.y, blockPos.z);
			positionInOther += offset;

			positionInOther = voxMapShapeInverseTransform * pointShellShapeTransform * positionInOther;

			btVector3 otherVoxelPosition(round(positionInOther.x()), round(positionInOther.y()), round(positionInOther.z()));

			if ((voxMapShape->getContentProvider()->getVoxelType((int) otherVoxelPosition.x(), (int) otherVoxelPosition.y(), (int) otherVoxelPosition.z()) == VOX_TYPE_SURFACE) || (voxMapShape->getContentProvider()->getVoxelType((int) otherVoxelPosition.x(), (int) otherVoxelPosition.y(), (int) otherVoxelPosition.z()) == VOX_TYPE_PROXIMITY)) {

				btVector3 rotatedNormal = normal;

				btVector3 pointPositionInGlobal((btScalar) blockPos.x + offset.x(), (btScalar) blockPos.y + offset.y(),
												(btScalar) blockPos.z + offset.z());
				pointPositionInGlobal = pointShellShapeTransform * pointPositionInGlobal;

				btVector3 forceVoxelPositionInGlobal = voxMapShapeTransform * otherVoxelPosition;


				btVector3 positionDif = pointPositionInGlobal - forceVoxelPositionInGlobal;
				// The collision depth of the contact
				btScalar collisionDepth = positionDif.dot(rotatedNormal);

				bool isSurface = voxMapShape->getContentProvider()->getVoxelType((int) otherVoxelPosition.x(), (int) otherVoxelPosition.y(), (int) otherVoxelPosition.z()) == VOX_TYPE_SURFACE;
				bool isProximity = voxMapShape->getContentProvider()->getVoxelType((int) otherVoxelPosition.x(), (int) otherVoxelPosition.y(), (int) otherVoxelPosition.z()) == VOX_TYPE_PROXIMITY;

				// If we are a surface voxel then the point must always be outside, so we add .5 to the collision depth
				if (isSurface) {
					collisionDepth -= .5;
				}

				// If collisionDepth < 0 then the point is intersecting with the voxel
				if (collisionDepth < 0) {
					// Now make sure that pushing in this direction will actually push the voxel shapes apart, not towards each other
					btVector3i offsets[] = { btVector3i(1, 0, 0), btVector3i(-1, 0, 0), btVector3i(0, 1, 0),
							  btVector3i(0, -1, 0), btVector3i(0, 0, 1), btVector3i(0, 0, -1) };
					for (btVector3i offset : offsets) {
						bool isOffsetSurface = voxMapShape->getContentProvider()->getVoxelType((int) otherVoxelPosition.x() + offset.x, (int) otherVoxelPosition.y() + offset.y, (int) otherVoxelPosition.z() + offset.z) == VOX_TYPE_SURFACE;
						bool isOffsetProximity = voxMapShape->getContentProvider()->getVoxelType((int) otherVoxelPosition.x() + offset.x, (int) otherVoxelPosition.y() + offset.y, (int) otherVoxelPosition.z() + offset.z) == VOX_TYPE_PROXIMITY;

						btVector3 lazy(offset.x, offset.y, offset.z);
						lazy = voxMapShapeTransform.getBasis() * lazy;

						bool isNormalTowards = rotatedNormal.dot(lazy) > 0.3;

						if (isNormalTowards) {
							if (isSurface && isOffsetSurface) {
								// If we're a surface voxel, then don't push towards other surface voxels
								return BT_VOXEL_NEGATIVE_INFINITY;
							}
							if (isProximity && (isOffsetSurface || isOffsetProximity)) {
								// If we're a proximity voxel, then don't push towards surface voxels or other proximity voxels.
								return BT_VOXEL_NEGATIVE_INFINITY;
							}
							break;
						}

					}

					// resultOut->addContactPoint(-rotatedNormal, pointPositionInGlobal, collisionDepth);
					return (double) collisionDepth;
				}
			}
			return BT_VOXEL_NEGATIVE_INFINITY;
		};

		const btVector3 normals[] = { btVector3(1, 0, 0), btVector3(-1, 0, 0), btVector3(0, 1, 0),
								 btVector3(0, -1, 0), btVector3(0, 0, 1), btVector3(0, 0, -1) };

		btVector3 idealNormal(0, 0, 0);
		double collisionDepth = BT_VOXEL_NEGATIVE_INFINITY;


		btVector3i otherVoxelPosition((int) round(positionInOther.x()), (int) round(positionInOther.y()), (int) round(positionInOther.z()));


		// Test 12 possible normals (There are 6 normals per voxel shape, and 2 voxel shapes)
		for (auto normal : normals) {
			// Test the normal on the point shell
			{
				// Don't use surface normals that point towards neighbor blocks
				if (!pointShellShape->getContentProvider()->getVoxelType(blockPos.x - (int) normal.x(), blockPos.y - (int) normal.y(), blockPos.z - (int) normal.z()) == VOX_TYPE_SURFACE) {
					btVector3 rotatedInFirst = pointShellShapeTransform.getBasis() * normal;
					double depthAtNormal = func(btVector3(0, 0, 0), rotatedInFirst);
					// We wan to minimize collision depth
					if (depthAtNormal > collisionDepth) {
						collisionDepth = depthAtNormal;
						idealNormal = rotatedInFirst;
					}
				}
			}
			// Test the normal on the voxel map
			{
				btVector3 rotatedInFirst = voxMapShapeTransform.getBasis() * normal;
				double depthAtNormal = func(btVector3(0, 0, 0), rotatedInFirst);
				// We wan to minimize collision depth
				if (depthAtNormal > collisionDepth) {
					collisionDepth = depthAtNormal;
					idealNormal = rotatedInFirst;
				}
			}
		}

		// Check if we found a valid collision normal
		if (collisionDepth != BT_VOXEL_NEGATIVE_INFINITY) {
			// If we did then add the collision point to the manifold
			btVector3 pointPositionInGlobal((btScalar) blockPos.x, (btScalar) blockPos.y,
											(btScalar) blockPos.z);
			pointPositionInGlobal = pointShellShapeTransform * pointPositionInGlobal;

			resultOut->addContactPoint(-idealNormal, pointPositionInGlobal, collisionDepth);
		}
		*/

		/*
		btTransform pointTranslationTransform;
		pointTranslationTransform.setIdentity();
		pointTranslationTransform.setOrigin(btVector3(blockPos.x, blockPos.y, blockPos.z));

		btTransform voxelTranslationTransform;
		pointTranslationTransform.setIdentity();
		pointTranslationTransform.setOrigin(btVector3(otherBlockPos.x, otherBlockPos.y, otherBlockPos.z));

		btDiscreteCollisionDetectorInterface::ClosestPointInput input;
		input.m_maximumDistanceSquared = BT_LARGE_FLOAT;
		input.m_transformA = pointTranslationTransform * body0Wrap->getWorldTransform();
		input.m_transformB = voxelTranslationTransform * body1Wrap->getWorldTransform();

		btBoxBoxDetector detector(boxShape, boxShape);
		detector.getClosestPoints(input, *resultOut, dispatchInfo.m_debugDraw);
		 */

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
