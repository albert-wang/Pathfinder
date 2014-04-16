#pragma once
#include <vector>
#include <boost/cstdint.hpp>
#include <iostream>

enum Direction
{
	NONE = 0,
	SOUTHWEST, SOUTH, SOUTHEAST,
	WEST, CENTER, EAST,
	NORTHWEST, NORTH, NORTHEAST,
	UP, DOWN
};

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

std::ostream& operator<<(std::ostream&, const Index&);

struct Portal
{
	Index start;
	Direction direction;
	size_t passibility;
	size_t size;

	bool operator==(const Portal& other) const;
};

std::ostream& operator<<(std::ostream&, const Portal&);

//A node represents a path beteween two portals.
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
	std::vector<Portal> linkPositionAndPortals(const Index& ind) const;
	size_t portalPathfind(const std::vector<Portal>& origin, const std::vector<Portal>& goals) const;
private:
	size_t blockIndex(const Index& ind) const;

	void createBlockData(size_t block);
	size_t maximumPassibility(size_t index) const;

	void preprocess();
	void createPortalsInBlock(Block& b, const Index& start, const Index& iter, Direction d, size_t iterations);
	void innerblockPathfind(const Block& block);

	size_t blockpathfind(size_t blockIndex, const Index& start, const Index& end) const;

	std::vector<boost::uint8_t> map;
	std::vector<Block> blocks;
	std::vector<GraphVertex> graph;

	size_t width;
};