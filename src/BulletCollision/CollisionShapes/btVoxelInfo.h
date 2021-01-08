#ifndef BT_VOXEL_INFO_H
#define BT_VOXEL_INFO_H

/// Information on the contents of a single voxel
ATTRIBUTE_ALIGNED16(struct) btVoxelInfo
{
BT_DECLARE_ALIGNED_ALLOCATOR();
/// Whether this voxel can be hit by ray traces
bool				m_tracable;
/// Whether the voxel blocks rigid bodies
bool				m_blocking;
/// This id is used to detect when a voxel has changed, dropping and recalculating the physics interactions. It should uniquely identify the collision shape.
/// It is somewhat optional, even with the same id the collision algorithm will attempt to detect changes
unsigned short	    m_voxelTypeId;
/// The shape of the voxel
const btCollisionShape*	m_collisionShape;
/// The offset of the shape from the center of the voxel
btVector3			m_collisionOffset;


/// The friction of the voxel
btScalar			m_friction;
/// The resititution (bounciness) of the voxel
btScalar			m_restitution;
/// The rolling friction of the voxel
btScalar			m_rollingFriction;

/**@brief No initialization constructor */
SIMD_FORCE_INLINE btVoxelInfo():
m_tracable(0),
m_blocking(0),
m_voxelTypeId(0),
m_collisionShape(nullptr),
m_collisionOffset(btVector3(0,0,0)),
m_friction(0),
m_restitution(0),
m_rollingFriction(0)
{}

/**@brief Constructor from scalars
* @param x X value
* @param y Y value
* @param z Z value
*/
SIMD_FORCE_INLINE btVoxelInfo(const bool& _traceable, const bool& _blocking, const long& _voxelTypeId, btCollisionShape* const _collisionShape,
							  const btVector3& _collisionOffset, const btScalar& _friction, const btScalar& _restitution, const btScalar& _rollingFriction)
{
	m_tracable = _traceable;
	m_blocking = _blocking;
	m_voxelTypeId = _voxelTypeId;
	m_collisionShape = _collisionShape;
	m_collisionOffset = _collisionOffset;
	m_friction = _friction;
	m_restitution = _restitution;
	m_rollingFriction = _rollingFriction;
}

/**@brief Copy constructor */
SIMD_FORCE_INLINE btVoxelInfo (const btVoxelInfo& other)
: m_tracable(other.m_tracable),
m_blocking(other.m_blocking),
m_voxelTypeId(other.m_voxelTypeId),
m_collisionShape(other.m_collisionShape),
m_friction(other.m_friction),
m_restitution(other.m_restitution),
m_rollingFriction(other.m_rollingFriction),
m_collisionOffset(other.m_collisionOffset)
{
}

//	/**@brief Constructor from scalars
//	* @param x X value
//	* @param y Y value
//	* @param z Z value
//	*/
//	SIMD_FORCE_INLINE btVoxelInfo(bool _traceable,bool _blocking, long _voxelTypeId, btCollisionShape* const _collisionShape, const btVector3& _collisionOffset, btScalar _friction, btScalar _restitution, btScalar _rollingFriction)
//			: m_tracable(_traceable),m_blocking(_blocking),m_voxelTypeId(_voxelTypeId), m_collisionShape(_collisionShape),m_friction(_friction),m_restitution(_restitution),m_rollingFriction(_rollingFriction),m_collisionOffset(_collisionOffset)
//	{}

SIMD_FORCE_INLINE bool isEmpty(){return m_voxelTypeId == -1;}
};

#endif //BT_VOXEL_INFO_H