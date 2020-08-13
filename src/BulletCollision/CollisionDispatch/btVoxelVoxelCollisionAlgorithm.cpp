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

void btVoxelVoxelCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	// These are swapped because I must've mixed up the pointshell and voxmap shapes in the implementation
	// Thankfully this seems to work fine
	const btCollisionObjectWrapper* colObjWrap = body1Wrap;
	const btCollisionObjectWrapper* otherObjWrap = body0Wrap;

	// Create the persistent manifold
	// if (true) { // resultOut->getPersistentManifold() == nullptr) {
		// btPersistentManifold* persistentManifold = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
		// resultOut->setPersistentManifold(persistentManifold);
		// printf("Created a manifold\n");
	// }
	if (!m_sharedManifold)
	{
		//swapped?
		m_sharedManifold = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
	// if (m_sharedManifold->getNumContacts() != 0) {
		m_sharedManifold->clearManifold();
	// }
	resultOut->setPersistentManifold(m_sharedManifold);

	btAssert(colObjWrap->getCollisionShape()->isVoxel());
	const auto* voxelShape = static_cast<const btVoxelShape*>(colObjWrap->getCollisionShape());
	const auto* otherVoxelShape = static_cast<const btVoxelShape*>(otherObjWrap->getCollisionShape());

	btTransform inverseOtherTransform = otherObjWrap->getWorldTransform().inverse();

	btTransform voxelWorldTransform = colObjWrap->getWorldTransform();
	btTransform inverseVoxelWorldTransform = voxelWorldTransform.inverse();

	btTransform otherTransform = otherObjWrap->getWorldTransform();

	btVector3 aabbMin;
	btVector3 aabbMax;

	// When creating the AABB to check, first put the object in voxel world space, then create the AABB
	otherObjWrap->getCollisionShape()->getAabb(inverseVoxelWorldTransform * otherTransform, aabbMin, aabbMax);

	btVector3 scale = voxelShape->getLocalScaling();
	btVector3i regionMin(static_cast <int> (floor(aabbMin.x() / scale.x() + .5)),
							 static_cast <int> (floor(aabbMin.y() / scale.y() + .5)),
							 static_cast <int> (floor(aabbMin.z() / scale.z() + .5)));
	btVector3i regionMax(static_cast <int> (floor(aabbMax.x() / scale.x() + .5)),
							 static_cast <int> (floor(aabbMax.y() / scale.y() + .5)),
							 static_cast <int> (floor(aabbMax.z() / scale.z() + .5)));
	// Remove out of bounds collision info
	int i = 0;
	int numChildren = 0;

	const auto voxelShapeIterator = voxelShape->getContentProvider()->begin();
	const auto voxelShapeIteratorEnd = voxelShape->getContentProvider()->end();

	for (auto it = voxelShapeIterator; it != voxelShapeIteratorEnd; it++) {
		const btVector3i blockPos = *it;

		btVector3 positionInOther(blockPos.x, blockPos.y, blockPos.z);

		positionInOther = inverseOtherTransform * voxelWorldTransform * positionInOther;


		auto func = [voxelShape, otherVoxelShape, blockPos, voxelWorldTransform, otherTransform, inverseOtherTransform, resultOut] (const btVector3 offset, const btVector3 normal) {

			btVector3 positionInOther(blockPos.x, blockPos.y, blockPos.z);
			positionInOther += offset;

			positionInOther = inverseOtherTransform * voxelWorldTransform * positionInOther;

			btVector3 otherVoxelPosition(round(positionInOther.x()), round(positionInOther.y()), round(positionInOther.z()));

			if (otherVoxelShape->getContentProvider()->isSurface((int) otherVoxelPosition.x(), (int) otherVoxelPosition.y(), (int) otherVoxelPosition.z()) || otherVoxelShape->getContentProvider()->isProximity((int) otherVoxelPosition.x(), (int) otherVoxelPosition.y(), (int) otherVoxelPosition.z())) {

				btVector3 rotatedNormal = normal;

				btVector3 pointPositionInGlobal((btScalar) blockPos.x + offset.x(), (btScalar) blockPos.y + offset.y(),
												(btScalar) blockPos.z + offset.z());
				pointPositionInGlobal = voxelWorldTransform * pointPositionInGlobal;

				btVector3 forceVoxelPositionInGlobal = otherTransform * otherVoxelPosition;


				btVector3 positionDif = pointPositionInGlobal - forceVoxelPositionInGlobal;
				// The collision depth of the contact
				btScalar collisionDepth = positionDif.dot(rotatedNormal);

				bool isSurface = otherVoxelShape->getContentProvider()->isSurface((int) otherVoxelPosition.x(), (int) otherVoxelPosition.y(), (int) otherVoxelPosition.z());
				bool isProximity = otherVoxelShape->getContentProvider()->isProximity((int) otherVoxelPosition.x(), (int) otherVoxelPosition.y(), (int) otherVoxelPosition.z());

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
						bool isOffsetSurface = otherVoxelShape->getContentProvider()->isSurface((int) otherVoxelPosition.x() + offset.x, (int) otherVoxelPosition.y() + offset.y, (int) otherVoxelPosition.z() + offset.z);
						bool isOffsetProximity = otherVoxelShape->getContentProvider()->isProximity((int) otherVoxelPosition.x() + offset.x, (int) otherVoxelPosition.y() + offset.y, (int) otherVoxelPosition.z() + offset.z);

						btVector3 lazy(offset.x, offset.y, offset.z);
						lazy = otherTransform.getBasis() * lazy;

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
				if (!voxelShape->getContentProvider()->isSurface(blockPos.x - (int) normal.x(), blockPos.y - (int) normal.y(), blockPos.z - (int) normal.z())) {
					btVector3 rotatedInFirst = voxelWorldTransform.getBasis() * normal;
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
				btVector3 rotatedInFirst = otherTransform.getBasis() * normal;
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
			pointPositionInGlobal = voxelWorldTransform * pointPositionInGlobal;

			resultOut->addContactPoint(-idealNormal, pointPositionInGlobal, collisionDepth);
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
