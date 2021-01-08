//
// Created by Alexander on 1/8/2021.
//

#ifndef BT_VOXEL_SHAPE_DATA_H
#define BT_VOXEL_SHAPE_DATA_H

#include <vector>
#include "btBoxShape.h"
#include "btVoxelInfo.h"
#include "btVector3i.h"

// An air voxel surrounded entirely by air voxels
#define VOX_TYPE_AIR 0
// An air voxel with surrounded by at least 1 solid voxel
#define VOX_TYPE_PROXIMITY 1
// A solid voxel surrounded by at least 1 air voxel
#define VOX_TYPE_SURFACE 2
// A solid voxel surrounded entirely by solid voxels
#define VOX_TYPE_INTERIOR 3


ATTRIBUTE_ALIGNED16(class) btVoxelShapeData {

private:
	// List of all set voxel positions
	std::vector<btVector3i> setOfBlocks;
	// Array of all voxel data
	uint8_t * const voxelData;
	// Min and max positions of voxels in this world
	const btVector3i minPos, maxPos;
	// This is only correct assuming scaling is <1,1,1>
	// Should always be equal to scaling / 2
	const btBoxShape *const singleVoxelShape;

	virtual size_t convertPosToIndex(btVector3i pos) const;

public:
	btVoxelShapeData(btVector3i minPos, btVector3i maxPos);

	btVoxelShapeData();

	/// Provider of voxel information for a given voxel position
	virtual ~btVoxelShapeData();

	virtual void getVoxel(btVector3i pos, btVoxelInfo &) const;

	// Used to iterate over all BlockPos in this voxel shape
	// Should only ever be used for rendering in demos
	virtual std::vector<btVector3i>::const_iterator begin() const;

	virtual std::vector<btVector3i>::const_iterator end() const;

	virtual uint8_t getVoxelType(btVector3i pos) const;

	virtual void setVoxelType(btVector3i pos, bool voxelType);

	// TODO: Remove these two, eventually
	virtual void initializeLittleTower();
	virtual void initializeSphere();
};


#endif //BT_VOXEL_SHAPE_DATA_H
