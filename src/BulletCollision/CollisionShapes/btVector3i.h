#ifndef BT_VECTOR_3I_H
#define BT_VECTOR_3I_H

ATTRIBUTE_ALIGNED16(struct) btVector3i {

	BT_DECLARE_ALIGNED_ALLOCATOR();

	int x;
	int y;
	int z;

	/**@brief No initialization constructor */
	SIMD_FORCE_INLINE btVector3i() {
	}

	/**@brief Constructor from scalars
	* @param x X value
	* @param y Y value
	* @param z Z value
	*/
	SIMD_FORCE_INLINE btVector3i(const int &_x, const int &_y, const int &_z) {
		x = _x;
		y = _y;
		z = _z;
	}

	SIMD_FORCE_INLINE bool operator==(const btVector3i &other) const {
		return (this->x == other.x && this->y == other.y && this->z == other.z);
	}

	SIMD_FORCE_INLINE bool operator!=(const btVector3i &other) const {
		return !(*this == other);
	}

	SIMD_FORCE_INLINE unsigned int getHash() const {
		int a = static_cast<unsigned int>(x & 0xff);
		int b = static_cast<unsigned int>((y & 0xff) << 8);
		int c = static_cast<unsigned int>((z & 0xff) << 16);
		long int key = a + b + c;

		// Thomas Wang's hash
		key += ~(key << 15);
		key ^= (key >> 10);
		key += (key << 3);
		key ^= (key >> 6);
		key += ~(key << 11);
		key ^= (key >> 16);
		return (int) key;
	}

	bool equals(const btVector3i &other) const {
		return ((x == other.x) && (y == other.y) && (z == other.z));
	}

};

#endif //BT_VECTOR_3I_H
