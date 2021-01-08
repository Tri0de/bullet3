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

#ifndef BT_VOXEL_COLLISION_ALGORITHM_H
#define BT_VOXEL_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "BulletCollision/CollisionShapes/btVector3i.h"

class btDispatcher;

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "btCollisionCreateFunc.h"
#include "LinearMath/btAlignedObjectArray.h"

class btDispatcher;
class btCollisionObject;

class btCollisionShape;

ATTRIBUTE_ALIGNED16(struct) btVoxelCollisionInfo
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btVector3i position;
	long voxelTypeId;
	int shapeType = INVALID_SHAPE_PROXYTYPE;
	btCollisionAlgorithm* algorithm;
};

/// btVoxelCollisionAlgorithm  supports collision between VoxelCollisionShapes and other collision shapes
class btVoxelCollisionAlgorithm : public btCollisionAlgorithm
{
protected:
	b3AlignedObjectArray<btVoxelCollisionInfo> m_voxelCollisionInfo;
	bool m_isSwapped;
	btVector3i m_lastMin;
	btVector3i m_lastMax;
	class btPersistentManifold*	m_sharedManifold;

public:

	btVoxelCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci,const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,bool isSwapped);

	virtual ~btVoxelCollisionAlgorithm();

	virtual void processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	btScalar	calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray)
	{
		int i;
		for (i=0;i<m_voxelCollisionInfo.size();i++)
		{
		    if(m_voxelCollisionInfo[i].algorithm) {
			    m_voxelCollisionInfo[i].algorithm->getAllContactManifolds(manifoldArray);
			}
		}
	}


	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btVoxelCollisionAlgorithm));
			return new(mem) btVoxelCollisionAlgorithm(ci,body0Wrap,body1Wrap,false);
		}
	};

	struct SwappedCreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btVoxelCollisionAlgorithm));
			return new(mem) btVoxelCollisionAlgorithm(ci,body0Wrap,body1Wrap,true);
		}
	};

};

#endif //BT_VOXEL_COLLISION_ALGORITHM_H
