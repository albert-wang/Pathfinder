#pragma once
#include <vector>
#include <boost/cstdint.hpp>
#include <boost/static_assert.hpp>
#include <iostream>

enum Direction
{
	NONE = 0,
	SOUTHWEST, SOUTH, SOUTHEAST,
	WEST, CENTER, EAST,
	NORTHWEST, NORTH, NORTHEAST,
	UP, DOWN
};

//Represents a 4 dimensional uint vector. Generally used to refer to positions in the map.
//8 bytes.
struct Index
{
	static Index fromIndex(size_t index, size_t width);

	Index();
	Index(boost::uint16_t x, boost::uint16_t y, boost::uint16_t z = 0, boost::uint16_t w = 0);
	size_t index(size_t width) const;

	bool operator==(const Index&) const;

	boost::uint16_t x;
	boost::uint16_t y;
	boost::uint16_t z;
	boost::uint16_t w;
};

BOOST_STATIC_ASSERT(sizeof(Index) == 8);
std::ostream& operator<<(std::ostream&, const Index&);


//A portal between two blocks. Links start and start+direction.
struct Portal
{
	Index start;
	boost::uint8_t direction;
	boost::uint8_t passibility;
	boost::uint8_t size;

	bool operator==(const Portal& other) const;
};

std::ostream& operator<<(std::ostream&, const Portal&);

//A node represents a path beteween two portals. Bidirectional.
struct GraphVertex
{
	Portal start;
	Portal end;

	size_t length;
};

struct Block
{
	size_t index;
	std::vector<Portal> portals;
};

struct OriginAndGoal
{
	Index origin;
	Index goal;
};

class Map
{
	//Private access for the pathfinding policies
	friend struct PortalPathfindPolicy;
	friend struct BlockFindPolicy;

	//3 bits.
	static const size_t MaximumWidth = 7;

	//Bit allocation
	/*
		3: Ground passability
		3: Water passability
		3: Air Passability
		3: Ground|Water passability
		3: All passability

		1: leftover bit.
	*/

	//These are the valid capability combinations:
	// Assumption: Air units can fly anywhere where a ground unit could walk.
	// Ground, Water, Air, {Ground | Water}, {Ground, Water, Air}
	// The {Ground | Air} is simply 'Air' and {Air | Water} is {Ground | Water | Air}

	static const size_t BlockSize = 16;
public:
	OriginAndGoal loadFromStream(std::istream& i);

	size_t walkable(const Index& index, Direction dir) const;
	size_t passable(const Index& index) const;

	void debugDisplay(const OriginAndGoal& g) const;
	std::vector<Portal> linkPositionAndPortals(const Index& ind, size_t size) const;
	size_t portalPathfind(const std::vector<Portal>& origin, const std::vector<Portal>& goals) const;
private:
	size_t blockIndex(const Index& ind) const;

	void createBlockData(size_t block);
	size_t maximumPassibility(size_t index) const;

	void preprocess();
	void createPortalsInBlock(Block& b, const Index& start, const Index& iter, Direction d, size_t iterations);
	void simplifyBlockPortals(Block& p);
	void simplifyGraph();

	void innerblockPathfind(const Block& block);

	size_t blockpathfind(size_t blockIndex, const Index& start, const Index& end, size_t size, std::vector<Index> * path) const;

	std::vector<boost::uint8_t> map;
	std::vector<Block> blocks;
	std::vector<GraphVertex> graph;

	size_t width;
};