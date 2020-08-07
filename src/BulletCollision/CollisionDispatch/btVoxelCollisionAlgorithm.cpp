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
#include "BulletCollision/CollisionDispatch/btVoxelCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btVoxelShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAabbUtil2.h"
#include "btManifoldResult.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

btVoxelCollisionAlgorithm::btVoxelCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, bool isSwapped)
	: btCollisionAlgorithm(ci),
	  m_isSwapped(isSwapped),
	  m_lastMin(INT_MAX, INT_MAX, INT_MAX),
	  m_lastMax(INT_MIN, INT_MIN, INT_MIN),
	  m_sharedManifold(ci.m_manifold)
{
	const btCollisionObjectWrapper* colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
	btAssert(colObjWrap->getCollisionShape()->isVoxel());

	const btVoxelShape* voxelShape = static_cast<const btVoxelShape*>(colObjWrap->getCollisionShape());

	m_voxelCollisionInfo.reserve(30);
}

btVoxelCollisionAlgorithm::~btVoxelCollisionAlgorithm()
{
	int numChildren = m_voxelCollisionInfo.size();
	for (int i = 0; i < numChildren; i++)
	{
		if (m_voxelCollisionInfo[i].algorithm)
		{
			m_voxelCollisionInfo[i].algorithm->~btCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm(m_voxelCollisionInfo[i].algorithm);
		}
	}
}

void btVoxelCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	const btCollisionObjectWrapper* colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
	const btCollisionObjectWrapper* otherObjWrap = m_isSwapped ? body0Wrap : body1Wrap;

	btAssert(colObjWrap->getCollisionShape()->isVoxel());
	const btVoxelShape* voxelShape = static_cast<const btVoxelShape*>(colObjWrap->getCollisionShape());

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

	// Add new in-bounds collision info
	for (int x = regionMin.x; x <= regionMax.x; ++x)
	{
		for (int y = regionMin.y; y <= regionMax.y; ++y)
		{
			for (int z = regionMin.z; z <= regionMax.z; ++z)
			{
				if ((x < m_lastMin.x || x > m_lastMax.x) || (y < m_lastMin.y || y > m_lastMax.y) || (z < m_lastMin.z || z > m_lastMax.z))
				{
					int newIndex = m_voxelCollisionInfo.size();
					m_voxelCollisionInfo.resize(newIndex + 1);
					m_voxelCollisionInfo[newIndex].position.x = x;
					m_voxelCollisionInfo[newIndex].position.y = y;
					m_voxelCollisionInfo[newIndex].position.z = z;
					m_voxelCollisionInfo[newIndex].algorithm = nullptr;
				}
			}
		}
	}

	btVoxelInfo childInfo;
	numChildren = m_voxelCollisionInfo.size();
	btVoxelContentProvider* contentProvider = voxelShape->getContentProvider();

	while (i < numChildren)
	{
		btVoxelCollisionInfo& info = m_voxelCollisionInfo[i];

		// If we're outside of the region, delete info.
		if (info.position.x < regionMin.x || info.position.x > regionMax.x ||
			info.position.y < regionMin.y || info.position.y > regionMax.y ||
			info.position.z < regionMin.z || info.position.z > regionMax.z)
		{
			if (info.algorithm)
			{
				btCollisionAlgorithm* algo = info.algorithm;
				info.algorithm = nullptr;
				algo->~btCollisionAlgorithm();
				m_dispatcher->freeCollisionAlgorithm(algo);
			}
			if (numChildren > 1)
			{
				m_voxelCollisionInfo[i] = m_voxelCollisionInfo[numChildren - 1];
			}
			--numChildren;
		}
		else
		{
			contentProvider->getVoxel(info.position.x, info.position.y, info.position.z, childInfo);
			if(childInfo.m_collisionShape != nullptr)
			{
				if (info.algorithm != nullptr && (childInfo.m_collisionShape->getShapeType() != info.shapeType || !childInfo.m_blocking))
				{
					btCollisionAlgorithm* algo = info.algorithm;
					info.algorithm = nullptr;
					algo->~btCollisionAlgorithm();
					m_dispatcher->freeCollisionAlgorithm(algo);
				}
				if (childInfo.m_blocking)
				{
					btTransform voxelTranform;

					voxelTranform.setIdentity();
					voxelTranform.setOrigin(btVector3(info.position.x * scale.x() + childInfo.m_collisionOffset.x(),
													  info.position.y * scale.y() + childInfo.m_collisionOffset.y(),
													  info.position.z * scale.z() + childInfo.m_collisionOffset.z()));

					// The transform for the individual voxel shape is its own local transform, followed by the voxel world transform.
					voxelTranform = voxelWorldTransform * voxelTranform;

					btCollisionObjectWrapper voxelWrap(colObjWrap, childInfo.m_collisionShape, colObjWrap->getCollisionObject(),
													   voxelTranform, -1, -1);
					if (info.algorithm == nullptr)
					{
						info.algorithm = m_dispatcher->findAlgorithm(&voxelWrap, otherObjWrap, m_sharedManifold,
																	 BT_CONTACT_POINT_ALGORITHMS);
						info.shapeType = childInfo.m_collisionShape->getShapeType();
						info.voxelTypeId = childInfo.m_voxelTypeId;
					}


					btCollisionObject* tmpCollision = const_cast<btCollisionObject*>(colObjWrap->getCollisionObject());
					tmpCollision->setFriction(childInfo.m_friction);
					tmpCollision->setRestitution(childInfo.m_restitution);
					tmpCollision->setRollingFriction(childInfo.m_rollingFriction);
					tmpCollision->setVoxelPosition(info.position);

					const btCollisionObjectWrapper* tmpWrap = nullptr;
					if (resultOut->getBody0Internal() == colObjWrap->getCollisionObject())
					{
						tmpWrap = resultOut->getBody0Wrap();
						resultOut->setBody0Wrap(&voxelWrap);
						resultOut->setShapeIdentifiersA(-1, i);
					}
					else
					{
						tmpWrap = resultOut->getBody1Wrap();
						resultOut->setBody1Wrap(&voxelWrap);
						resultOut->setShapeIdentifiersB(-1, i);
					}

					info.algorithm->processCollision(&voxelWrap, otherObjWrap, dispatchInfo, resultOut);

					if (resultOut->getBody0Internal() == colObjWrap->getCollisionObject())
					{
						resultOut->setBody0Wrap(tmpWrap);
					}
					else
					{
						resultOut->setBody1Wrap(tmpWrap);
					}
				}
			}

			++i;
		}
	}
	if (numChildren < m_voxelCollisionInfo.size())
	{
		m_voxelCollisionInfo.resize(numChildren);
	}

	m_lastMin = regionMin;
	m_lastMax = regionMax;
}

btScalar btVoxelCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	return 0;
}
