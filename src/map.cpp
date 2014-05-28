#include "map.hpp"
#include <string>
#include <algorithm>
#include <cassert>
#include <set>
#include <map>

#include "logger.h"
#include "genericastar.h"

namespace
{
	bool isSeperator(char c)
	{
		return c == '|' || c == '-';
	}

	static const int offsetX[] = {0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, 0};
	static const int offsetY[] = {0, 1, 1, 1, 0, 0, 0, -1, -1, -1, 0, 0};
	static const char * directionName[] = {"", "SW", "S", "SE", "W", "C", "E", "NW", "N", "NW", "U", "D"};
	static const Direction invert[] = {
		NONE, NORTHEAST, NORTH, NORTHWEST, EAST, CENTER, WEST, SOUTHEAST, SOUTH, SOUTHWEST, DOWN, UP
	};
}

Index::Index()
{}

Index::Index(boost::uint16_t x, boost::uint16_t y, boost::uint16_t z, boost::uint16_t w)
	:x(x), y(y), z(z), w(w)
{}

bool Index::operator==(const Index& i) const
{
	return x == i.x && y == i.y && z == i.z && w == i.w;
}

Index Index::fromIndex(size_t i, size_t w)
{
	return Index(i % w, i / w, 0, 0);
}

size_t Index::index(size_t w) const
{
	return x + y * w;
}

bool Portal::operator==(const Portal& other) const
{
	return start == other.start;
}

std::ostream& operator<<(std::ostream& o, const Index& ind)
{
	o << "(" << ind.x << ", " << ind.y << ")";
	return o;
}

std::ostream& operator<<(std::ostream& o, const Portal& ind)
{
	o << ind.start << " " << directionName[ind.direction] << " " << ind.size;
	return o;
}

OriginAndGoal Map::loadFromStream(std::istream& input)
{
	std::string s;
	std::getline(input, s);
	s.erase(std::remove_if(s.begin(), s.end(), isSeperator), s.end());

	width = s.size();
	map.reserve(width * 32);

	OriginAndGoal res;
	do
	{
		s.erase(std::remove_if(s.begin(), s.end(), isSeperator), s.end());
		for (size_t i = 0; i < s.size(); ++i)
		{
			size_t value = MaximumWidth;
			if (s[i] == '0')
			{
				value = 0;
			}

			if (s[i] == 'G')
			{
				res.goal = Index::fromIndex(map.size(), width);
			}

			if (s[i] == 'S')
			{
				res.origin = Index::fromIndex(map.size(), width);
			}

			map.push_back(value);
		}

		std::getline(input, s);
	} while (input);

	preprocess();
	return res;
}

size_t Map::walkable(const Index& ind, Direction direction) const
{
	int x = static_cast<int>(ind.x);
	int y = static_cast<int>(ind.y);
	size_t height = map.size() / width;

	x += offsetX[direction];
	y += offsetY[direction];

	if (x < 0 || y < 0)
	{
		return 0;
	}

	if (x >= width || y >= height)
	{
		return 0;
	}

	return map[Index(x, y).index(width)];
}

size_t Map::passable(const Index& ind) const
{
	if (ind.x >= width || ind.x < 0)
	{
		return 0;
	}

	if (ind.y >= map.size() / width || ind.y < 0)
	{
		return 0;
	}

	return map[ind.index(width)];
}

size_t Map::blockIndex(const Index& ind) const
{
	size_t blockWidth = width / BlockSize;
	size_t blockX = (ind.x / BlockSize);
	size_t blockY = (ind.y / BlockSize);

	return blockX + blockY * blockWidth;
}

size_t Map::maximumPassibility(size_t index) const
{
	size_t height = map.size() / width;
	size_t baseX = index % width;
	size_t baseY = index / width;

	if (map[index] == 0)
	{
		return 0;
	}

	for (size_t i = 1; i < MaximumWidth; ++i)
	{
		if (baseX + i >= width)
		{
			return i;
		}

		if (baseY + i >= height)
		{
			return i;
		}

		//The furthest corner.
		if (!map[index + i * width + i])
		{
			return i;
		}

		//The edges.
		for (size_t j = 0; j < i; ++j)
		{
			if (!map[index + j * width + i])
			{
				return i;
			}

			if (!map[index + i * width + j])
			{
				return i;
			}
		}
	}

	return MaximumWidth;
}

void Map::createPortalsInBlock(Block& out, const Index& start, const Index& iterate, Direction d, size_t iterations)
{
	for (size_t i = 0; i < iterations; ++i)
	{
		Index current = Index(start.x + iterate.x * i, start.y + iterate.y * i);
		if (size_t pass = walkable(current, d))
		{
			if (!passable(current))
			{
				continue;
			}

			Portal p;
			p.start = current;
			p.direction = d;

			bool found = false;
			size_t maximumPortalPassability = 10;
			for (size_t w = 1; w < iterations - i; ++w)
			{
				Index next = Index(current.x + iterate.x * w, current.y + iterate.y * w);
				size_t nextPassable = std::min(passable(next), walkable(next, d));
				if (nextPassable == 0)
				{
					p.size = w;
					i += w;
					found = true;
					break;
				}

				maximumPortalPassability = std::min(maximumPortalPassability, nextPassable);
			}

			if (!found)
			{
				p.size = iterations - i;
				i += iterations - i;
			}

			p.passibility = maximumPortalPassability;
			out.portals.push_back(p);

			GraphVertex vertex;
			vertex.start = p;

			Portal next;
			next.start = Index(current.x + offsetX[d], current.y + offsetY[d]);
			next.size = p.size;
			next.direction = invert[d];
			vertex.end = next;

			graph.push_back(vertex);
		}
	}
}

void Map::simplifyBlockPortals(Block& block)
{
	Rawr::log << "Starting portal simplification for index [" << block.index << "], Initial size: " << block.portals.size();
	std::sort(block.portals.begin(), block.portals.end(), [this](const Portal& a, const Portal& b)
	{
		return a.start.index(width) < b.start.index(width);
	});

	//Remove identical portals
	auto predicate = [](const Portal& a, const Portal& b)
	{
		return a.start == b.start &&
			a.direction == b.direction &&
			a.size == b.size;
	};

	block.portals.erase(std::unique(block.portals.begin(), block.portals.end(), predicate), block.portals.end());

	//Collapse portals with
	Rawr::log << " Resulting Size: " << block.portals.size();
}

void Map::simplifyGraph()
{

}

//Block pathfinding policy.
struct BlockFindPolicy : public Pathfinder::PathfindPolicy<BlockFindPolicy>
{
	typedef Index Element;
	typedef size_t Hash;

	BlockFindPolicy(const Index& start, const Index& end, size_t blockIndex, size_t width, size_t size, const Map * map)
		:start(start)
		,end(end)
		,blockIndex(blockIndex)
		,width(width)
		,size(size)
		,map(map)
	{}

	size_t startingCount()
	{
		return 1;
	}

	const Index& startingPoint(size_t)
	{
		return start;
	}

	bool finished(const Index& s)
	{
		return s == end;
	}

	Hash hash(const Index& ind)
	{
		return ind.index(width);
	}

	size_t predict(const Index& s)
	{
		int diffx = static_cast<int>(s.x) - static_cast<int>(end.x);
		int diffy = static_cast<int>(s.y) - static_cast<int>(end.y);
		int diffz = static_cast<int>(s.z) - static_cast<int>(end.z);
		return diffx * diffx + diffy * diffy + diffz * diffz;
	}

	size_t neighborCount(const Index& elem)
	{
		return 8;
	}

	Index neighbor(const Index& elem, size_t i)
	{
		Direction possible[8] = {
			SOUTHWEST, SOUTH, SOUTHEAST, WEST, EAST, NORTHWEST, NORTH, NORTHEAST
		};

		return Index(elem.x + offsetX[possible[i]], elem.y + offsetY[possible[i]]);
	}

	bool passable(const Index& elem)
	{
		size_t w = map->passable(elem);
		if (w < size)
		{
			return false;
		}

		if (map->blockIndex(elem) != blockIndex)
		{
			return false;
		}

		return true;
	}

	Index start;
	Index end;

	size_t blockIndex;
	size_t width;
	size_t size;
	const Map * map;
};

/*
struct PortalPathfindPolicy : public Pathfinder::PathfindPolicy<PortalPathfindPolicy>
{
	typedef Portal Element;
	typedef size_t Hash;

	BlockFindPolicy(const std::vector<Portal>& start, const std::vector<Portal>& end, size_t width, const Map * map)
		:start(start)
		,end(end)
		,width(width)
		,map(map)
	{}

	size_t startingCount()
	{
		return start.size();
	}

	const Index& startingPoint(size_t i)
	{
		return start[i];
	}

	bool finished(const Portal& s)
	{
		return std::find(end.begin(), end.end(), s) != end.end();
	}

	Hash hash(const Portal& ind)
	{
		return ind.start.index(width);
	}

	size_t score(const Index& s, const Index& e)
	{
		int diffx = static_cast<int>(s.x) - static_cast<int>(e.x);
		int diffy = static_cast<int>(s.y) - static_cast<int>(e.y);
		int diffz = static_cast<int>(s.z) - static_cast<int>(e.z);
		return diffx * diffx + diffy * diffy + diffz * diffz;
	}

	size_t predict(const Index& s)
	{
		size_t best = score(s, end[0]);

		for (size_t i = 1; i < end.size(); ++i)
		{
			best = std::min(best, score(s, end[i]));
		}

		return best;
	}

	size_t neighborCount(const Portal& elem)
	{

	}

	Index neighbor(const Index& elem, size_t i)
	{
		Direction possible[8] = {
			SOUTHWEST, SOUTH, SOUTHEAST, WEST, EAST, NORTHWEST, NORTH, NORTHEAST
		};

		return Index(elem.x + offsetX[possible[i]], elem.y + offsetY[possible[i]]);
	}

	bool passable(const Index& elem)
	{
		size_t w = map->passable(elem);
		if (w < size)
		{
			return false;
		}

		if (map->blockIndex(elem) != blockIndex)
		{
			return false;
		}

		return true;
	}

	const std::vector<Portal>& start;
	const std::vector<Portal>& end;

	size_t width;
	const Map * map;
};
*/

//This finds a path completely within a single block.
//Generally used to create a path between portals or between a point and another portal.
size_t Map::blockpathfind(size_t bi, const Index& start, const Index& end, size_t size, std::vector<Index> * foundPath) const
{
	BlockFindPolicy policy(start, end, bi, width, size, this);
	return Pathfinder::pathfind(policy, foundPath);
}


//This finds a path between two sets of portals.
//TODO: Passability and size.
size_t Map::portalPathfind(const std::vector<Portal>& origins, const std::vector<Portal>& goals) const
{
	std::vector<Portal> open = origins;
	std::vector<Portal> closed;

	while (!open.empty())
	{
		Portal target = open.back();
		open.pop_back();

		if (std::find(goals.begin(), goals.end(), target) != goals.end())
		{
			return 1;
		}

		closed.push_back(target);

		std::vector<Portal> possible;
		for (size_t i = 0; i < graph.size(); ++i)
		{
			if (graph[i].start.start == target.start)
			{
				possible.push_back(graph[i].end);
			}
		}

		for (size_t i = 0; i < possible.size(); ++i)
		{
			bool found = false;
			for (size_t j = 0; j < closed.size(); ++j)
			{
				if (possible[i].start == closed[j].start)
				{
					found = true;
					break;
				}
			}

			if (!found)
			{
				open.push_back(possible[i]);
			}
		}
	}

	return 0;
}

//Generates all the portal links within a single block.
void Map::innerblockPathfind(const Block& block)
{
	//Uniform cost for moving, moving diagonally costs 1.5 movement.
	for (size_t i = 0; i < block.portals.size(); ++i)
	{
		for (size_t j = 0; j < block.portals.size(); ++j)
		{
			if (i != j)
			{
				size_t pathLength = blockpathfind(block.index, block.portals[i].start, block.portals[j].start, 1, nullptr);
				if (pathLength > 0)
				{
					GraphVertex vert;
					vert.start = block.portals[i];
					vert.end = block.portals[j];
					vert.length = pathLength;
					graph.push_back(vert);
				}
			}
		}
	}
}

std::vector<Portal> Map::linkPositionAndPortals(const Index& ind, size_t size) const
{
	size_t bi = blockIndex(ind);
	const Block& block = blocks[bi];

	std::vector<Portal> result;
	for (size_t i = 0; i < block.portals.size(); ++i)
	{
		size_t path = blockpathfind(bi, ind, block.portals[i].start, size, nullptr);
		if (path != 0)
		{
			result.push_back(block.portals[i]);
		}
	}

	return result;
}

void Map::createBlockData(size_t bi)
{
	size_t blockWidth = width / BlockSize;
	size_t startX = (bi % blockWidth) * BlockSize;
	size_t startY = (bi / blockWidth) * BlockSize;

	//For this block, compute the exits.
	//Compute exits for the given border, going in the border direction.
	Block block;
	block.index = bi;
	createPortalsInBlock(block, Index(startX, startY), Index(1, 0), NORTH, BlockSize);
	createPortalsInBlock(block, Index(startX, startY), Index(0, 1), WEST, BlockSize);
	createPortalsInBlock(block, Index(startX, startY + BlockSize - 1), Index(1, 0), SOUTH, BlockSize);
	createPortalsInBlock(block, Index(startX + BlockSize - 1, startY), Index(0, 1), EAST, BlockSize);

	//Compute exits for the corners.
	createPortalsInBlock(block, Index(startX, startY), Index(1, 0), NORTHWEST, 1);
	createPortalsInBlock(block, Index(startX + BlockSize - 1, startY), Index(1, 0), NORTHEAST, 1);
	createPortalsInBlock(block, Index(startX, startY + BlockSize - 1), Index(1, 0), SOUTHWEST, BlockSize);
	createPortalsInBlock(block, Index(startX + BlockSize - 1, startY + BlockSize - 1), Index(1, 0), SOUTHEAST, BlockSize);

	//Compute exits for all up/down transitions.
	createPortalsInBlock(block, Index(startX+1, startY), Index(1, 0), NORTHWEST, BlockSize - 1);
	createPortalsInBlock(block, Index(startX, startY), Index(1, 0), NORTHEAST, BlockSize - 1);

	createPortalsInBlock(block, Index(startX, startY+1), Index(0, 1), NORTHWEST, BlockSize - 1);
	createPortalsInBlock(block, Index(startX, startY), Index(0, 1), SOUTHWEST, BlockSize - 1);

	createPortalsInBlock(block, Index(startX+1, startY + BlockSize - 1), Index(1, 0), SOUTHWEST, BlockSize - 1);
	createPortalsInBlock(block, Index(startX, startY + BlockSize - 1), Index(1, 0), SOUTHEAST, BlockSize - 1);

	createPortalsInBlock(block, Index(startX + BlockSize - 1, startY+1), Index(0, 1), NORTHEAST, BlockSize - 1);
	createPortalsInBlock(block, Index(startX + BlockSize - 1, startY), Index(0, 1), SOUTHEAST, BlockSize - 1);

	//Inner-block path computation and vertex emittal.
	blocks.push_back(block);
}

void Map::preprocess()
{
	for (size_t i = 0; i < map.size(); ++i)
	{
		map[i] = maximumPassibility(i);
	}

	for (size_t i = 0; i < map.size() / (BlockSize * BlockSize); ++i)
	{
		createBlockData(i);
	}

	for (size_t i = 0; i < blocks.size(); ++i)
	{
		simplifyBlockPortals(blocks[i]);
	}

	for (size_t i = 0; i < blocks.size(); ++i)
	{
		innerblockPathfind(blocks[i]);
	}

	std::cout << "Graph: " << graph.size() << "\n";

	simplifyGraph();
}

void Map::debugDisplay(const OriginAndGoal& goal) const
{
	static const size_t ColorBase = 241;
	static const size_t colors[] = {
		1,
		ColorBase + 0,
		ColorBase + 0,
		ColorBase + 0,
		ColorBase + 0,
		ColorBase + 0,
		ColorBase + 0,
		ColorBase + 0
	};

	std::cout << "\033[0;0m";
	for (size_t i = 0; i < map.size(); ++i)
	{
		if (i % width == 0 && i != 0)
		{
			std::cout << " \033[0;0m" << i / width - 1 << "\n";
		}

		if ((i / width) % BlockSize == 0 && i % width == 0 && i != 0)
		{
			std::cout << "\n";
		}

		if (i % BlockSize == 0 && i % width != 0)
		{
			std::cout << " ";
		}

		if (i == goal.goal.index(width))
		{
			std::cout << "\033[38;5;3m" << "G";
		}
		else if (i == goal.origin.index(width))
		{
			std::cout << "\033[38;5;2m" << "S";
		}
		else if (map[i] == MaximumWidth)
		{
			std::cout << "\033[38;5;241m" << ".";
		} else
		{
			std::cout << "\033[38;5;" << colors[map[i]] << "m";
			std::cout << (int)map[i];
		}
	}

	std::cout << "\033[0;0m";
	if (width > 100)
	{
		std::cout << "\n";
		for (size_t i = 0; i < width; ++i)
		{
			if (i % BlockSize == 0 && i != 0)
			{
				std::cout << " ";
			}

			std::cout << i / 100;
		}
	}

	std::cout << "\n";
	for (size_t i = 0; i < width; ++i)
	{
		if (i % BlockSize == 0 && i != 0)
		{
			std::cout << " ";
		}

		std::cout << (i % 100) / 10;
	}

	std::cout << "\n";
	for (size_t i = 0; i < width; ++i)
	{
		if (i % BlockSize == 0 && i != 0)
		{
			std::cout << " ";
		}

		std::cout << i % 10;
	}

	std::cout << "\n";
	std::cout << "\033[0;0m";
	std::cout << "\n";
}