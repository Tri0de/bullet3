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

#include <tuple>
#include <map>
#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"

/// Information on the contents of a single voxel
ATTRIBUTE_ALIGNED16(struct) btVoxelInfo
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	/// Whether this voxel can be hit by ray traces
	bool				m_tracable;
	/// Whether the voxel blocks rigid bodies
	bool				m_blocking;
	/// If the voxel is neither tracable nor blocking, the remaining information can be left blank

	/// This id is used to detect when a voxel has changed, dropping and recalculating the physics interactions. It should uniquely identify the collision shape.
	/// It is somewhat optional, even with the same id the collision algorithm will attempt to detect changes
	long				m_voxelTypeId;
	/// Generic location for additional information to be attached to the voxel, which will be returned by raycasts/collisions
	void*				m_userPointer;
	/// The shape of the voxel
	btCollisionShape*	m_collisionShape;
	/// The offset of the shape from the center of the voxel
	btVector3			m_collisionOffset;
	
	/// The friction of the voxel
	btScalar			m_friction;
	/// The resititution (bounciness) of the voxel
	btScalar			m_restitution;
	/// The rolling friction of the voxel
	btScalar			m_rollingFriction;

	/**@brief No initialization constructor */
	SIMD_FORCE_INLINE btVoxelInfo()
	{
	}

	/**@brief Constructor from scalars
	* @param x X value
	* @param y Y value
	* @param z Z value
	*/
	SIMD_FORCE_INLINE btVoxelInfo(const bool& _traceable, const bool& _blocking, const long& _voxelTypeId, void* const _userPointer, btCollisionShape* const _collisionShape, 
		const btVector3& _collisionOffset, const btScalar& _friction, const btScalar& _restitution, const btScalar& _rollingFriction)
	{
		m_tracable = _traceable;
		m_blocking = _blocking;
		m_voxelTypeId = _voxelTypeId;
		m_userPointer = _userPointer;
		m_collisionShape = _collisionShape;
		m_collisionOffset = _collisionOffset;
		m_friction = _friction;
		m_restitution = _restitution;
		m_rollingFriction = _rollingFriction;
	}
};

/// Provider of voxel information for a given voxel position
struct btVoxelContentProvider
{
	virtual ~btVoxelContentProvider() {}
	
	virtual btVoxelInfo getVoxel(int x, int y, int z) const = 0;

	virtual std::map<std::tuple<int, int, int>, bool>::iterator begin() = 0;

    virtual std::map<std::tuple<int, int, int>, bool>::iterator end() = 0;
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
