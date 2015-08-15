/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_VOXEL_SHAPE_H
#define BT_VOXEL_SHAPE_H

#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"

/// Information on the contents of a single voxel
ATTRIBUTE_ALIGNED16(struct) btVoxelInfo
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	/// Whether there is any collision content to this voxel. If false, then no other information is required
	bool				m_colliding;
	/// This id is used to detect when a voxel has changed, dropping and recalculating the physics interactions. It should uniquely identify the collision shape.
	/// It is somewhat optional, even with the same id the collision algorithm will attempt to detect changes
	long				m_voxelTypeId;
	/// Generic location for additional information to be attached to the voxel, which will be returned by raycasts/collisions
	void*				m_userPointer;
	/// The shape of the voxel
	btCollisionShape*	m_collisionShape;
	/// The offset of the shape from the center of the voxel
	btVector3			m_collisionOffset;
	/// Whether the voxel blocks rigid bodies
	bool				m_blocking;
	/// The friction of the voxel
	btScalar			m_friction;
	/// The resititution (bounciness) of the voxel
	btScalar			m_restitution;
	/// The rolling friction of the voxel
	btScalar			m_rollingFriction;
};

/// Provider of voxel information for a given voxel position
struct btVoxelContentProvider
{
	virtual ~btVoxelContentProvider()
	{}
	// return true when pairs need collision
	virtual btVoxelInfo getVoxel(int x, int y, int z) const = 0;
};

/// The btVoxelShape is a three dimensional grid of arbitrary size, with each cell containing a voxel. The contents of each cell is delegated to a
/// voxelContentProvider.
/// This shape is only intended for static objects (kinematic rigid bodies)
ATTRIBUTE_ALIGNED16(class) btVoxelShape	: public btCollisionShape
{
protected:
	btVector3						m_localAabbMin;
	btVector3						m_localAabbMax;
	btVoxelContentProvider*         m_contentProvider;
	
	btScalar	m_collisionMargin;
	btVector3	m_localScaling;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	explicit btVoxelShape(btVoxelContentProvider* contentProvider, btVector3& aabbMin, btVector3& aabbMax);

	virtual ~btVoxelShape();
	
	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	virtual	void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

	virtual void	setLocalScaling(const btVector3& scaling);

	virtual const btVector3& getLocalScaling() const
	{
		return m_localScaling;
	}

	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const;

	virtual void	setMargin(btScalar margin)
	{
		m_collisionMargin = margin;
	}
	virtual btScalar	getMargin() const
	{
		return m_collisionMargin;
	}
	virtual const char*	getName()const
	{
		return "Voxel";
	}

	virtual btVoxelContentProvider* getContentProvider() const {
		return m_contentProvider;
	}
	
};

#endif //BT_VOXEL_SHAPE_H
