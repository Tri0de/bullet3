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

		if (otherVoxelShape->getContentProvider()->isSurfaceOrSet(round(positionInOther.x()), round(positionInOther.y()), round(positionInOther.z()))) {


			// First compute the point's position in global
			btVector3 pointPositionInGlobal(blockPos.x, blockPos.y, blockPos.z);
			pointPositionInGlobal = voxelWorldTransform * pointPositionInGlobal;

			// Then compute the point's position in the other voxel world space
			btVector3 pointPositionInOtherLocal = inverseOtherTransform * pointPositionInGlobal;

			// Then compute the force voxel's position in global
			btVector3 forceVoxelPositionInGlobal(round(pointPositionInOtherLocal.x()), round(pointPositionInOtherLocal.y()), round(pointPositionInOtherLocal.z()));
			forceVoxelPositionInGlobal = otherTransform * forceVoxelPositionInGlobal;

			// The normal vector of the pointshell point
			btVector3 normal(0, 1, 0);
			normal = voxelWorldTransform.getBasis() * normal;

			btVector3 positionDif = pointPositionInGlobal - forceVoxelPositionInGlobal;
			// The collision depth of the contact
			btScalar collisionDepth = positionDif.dot(normal);

			if (collisionDepth < 0) {
				resultOut->addContactPoint(-normal, pointPositionInGlobal, collisionDepth);
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
