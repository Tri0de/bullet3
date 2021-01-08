//
// Created by Alexander on 1/8/2021.
//

#include "btVoxelShapeData.h"


btVoxelShapeData::btVoxelShapeData(const btVector3i minPos, const btVector3i maxPos) :
		minPos(minPos), maxPos(maxPos),
		singleVoxelShape(new btBoxShape((btVector3(btScalar(.5), btScalar(.5), btScalar(.5))))),
		voxelData((uint8_t *) calloc(
				(maxPos.x - minPos.x + 1) * (maxPos.y - minPos.y + 1) * (maxPos.z - minPos.z + 1),
				sizeof(uint8_t))) {
}

btVoxelShapeData::btVoxelShapeData::btVoxelShapeData() : btVoxelShapeData(btVector3i(-128, -128, -128),
																		  btVector3i(127, 127, 127)) {
}


void btVoxelShapeData::initializeLittleTower() {
	int radius = 30;
	for (int y = 0; y < 5; y++) {
		for (int x = -radius; x <= radius; x++) {
			for (int z = -radius; z <= radius; z++) {
				int yShapeThing = y;
				if (abs(x) + abs(z) > 2) {
					yShapeThing = 0;
				} else if ((x + z) % 2 == 0) {
					// Make a checkerboard pattern in the voxel world
					continue;
				}
				setVoxelType(btVector3i(x, yShapeThing, z), true);
			}
		}
	}
}

void btVoxelShapeData::initializeSphere() {
	const int radius = 8;
	for (int x = -radius; x <= radius; x++) {
		for (int y = -radius; y <= radius; y++) {
			for (int z = -radius; z <= radius; z++) {
				int distSq = x * x + y * y + z * z;
				if (distSq <= radius * radius) {
					setVoxelType(btVector3i(x, y, z), true);
				}
			}
		}
	}
}


void btVoxelShapeData::setVoxelType(const btVector3i pos, const bool voxelType) {
	// The index of the voxel data in the voxel data array
	const uint32_t voxelDataIndex = btVoxelShapeData::convertPosToIndex(pos);

	// The old voxel data
	const uint8_t oldVoxelData = voxelData[voxelDataIndex];
	const bool oldVoxelType = (oldVoxelData & 128u) == 128u;
	if (oldVoxelType != voxelType) {
		// Invert the set/unset bit of the voxel data
		const uint8_t newVoxelData = oldVoxelData ^128u;
		// Update the voxel data array
		voxelData[voxelDataIndex] = newVoxelData;

		// Update the setOfBlocks
		if (voxelType) {
			// Add blockPos to setOfBlocks
			setOfBlocks.push_back(pos);
		} else {
			// Remove blockPos from setOfBlocks
			for (auto iter = setOfBlocks.begin(); iter != setOfBlocks.end(); ++iter) {
				if (*iter == pos) {
					setOfBlocks.erase(iter);
					break;
				}
			}
		}

		// Update the neighbor count of the neighbors
		for (int32_t xOff = -1; xOff <= 1; xOff++) {
			for (int32_t yOff = -1; yOff <= 1; yOff++) {
				for (int32_t zOff = -1; zOff <= 1; zOff++) {
					if (xOff == 0 && yOff == 0 && zOff == 0) {
						// Don't update our own neighbor count
						continue;
					}
					const int32_t neighborX = pos.x + xOff;
					const int32_t neighborY = pos.y + yOff;
					const int32_t neighborZ = pos.z + zOff;

					if (neighborX < minPos.x || neighborX > maxPos.x || neighborY < minPos.y || neighborY > maxPos.y ||
						neighborZ < minPos.z || neighborZ > maxPos.z) {
						// This position is outside of the voxel shape, don't do anything
						continue;
					}

					// Increase the neighbor count of this voxel data by 1
					const uint32_t neighborVoxelIndex = btVoxelShapeData::convertPosToIndex(btVector3i(neighborX, neighborY, neighborZ));
					const uint8_t neighborOldVoxelData = voxelData[neighborVoxelIndex];
					const uint8_t oldNeighborCount = neighborOldVoxelData & 127u;
					// If voxelType is true, increase neighbor count by 1, otherwise decrease it by 1
					uint8_t newNeighborCount;
					if (voxelType) {
						newNeighborCount = oldNeighborCount + 1;
					} else {
						newNeighborCount = oldNeighborCount - 1;
					}
					voxelData[neighborVoxelIndex] = (neighborOldVoxelData & 128u) | (newNeighborCount);
				}
			}
		}
	}
}

btVoxelShapeData::~btVoxelShapeData() {
	free(voxelData);
}

void btVoxelShapeData::getVoxel(const btVector3i pos, btVoxelInfo &info) const {
	if (voxelData[btVoxelShapeData::convertPosToIndex(pos)] & 128u) {
		info.m_blocking = true;
		info.m_voxelTypeId = 1;
		info.m_tracable = true;
		info.m_collisionShape = singleVoxelShape;
		info.m_friction = 0.7;
		info.m_restitution = 0.5;
		info.m_rollingFriction = 0.7;
		info.m_collisionOffset = btVector3(0, 0, 0);
	} else {
		info.m_blocking = false;
		info.m_tracable = false;
	}
}

size_t btVoxelShapeData::convertPosToIndex(const btVector3i pos) const {
	const size_t xLen = maxPos.x - minPos.x + 1;
	size_t yLen = maxPos.y - minPos.y + 1;
	return (pos.x - minPos.x) + xLen * (pos.y - minPos.y) + xLen * yLen * (pos.z - minPos.z);
}

std::vector<btVector3i>::const_iterator btVoxelShapeData::begin() const {
	return setOfBlocks.begin();
}

std::vector<btVector3i>::const_iterator btVoxelShapeData::end() const {
	return setOfBlocks.end();
}

uint8_t btVoxelShapeData::getVoxelType(const btVector3i pos) const {
	const uint8_t voxData = voxelData[convertPosToIndex(pos)];
	const bool isSet = (voxData & 128u) == 128u;
	const uint8_t neighborCount = voxData & 127u;
	if (isSet) {
		if (neighborCount != 26) {
			return VOX_TYPE_SURFACE;
		} else {
			return VOX_TYPE_INTERIOR;
		}
	} else {
		if (neighborCount != 0) {
			return VOX_TYPE_PROXIMITY;
		} else {
			return VOX_TYPE_AIR;
		}
	}
}