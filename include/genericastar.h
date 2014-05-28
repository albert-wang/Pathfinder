#pragma once
#include <vector>

namespace Pathfinder
{
	namespace detail
	{
		template<typename T>
		struct ElementAndScore
		{
			T element;
			size_t gscore;
			size_t fscore;

			bool operator<(const ElementAndScore& other) const
			{
				return fscore < other.fscore;
			}

			bool operator==(const ElementAndScore& other) const
			{
				return element == other.element;
			}
		};
	}

	template<typename T>
	struct PathfindPolicy
	{

	};

	template<typename U, typename Policy>
	size_t pathfind(Policy& info, std::vector<U> * path)
	{
		typedef typename Policy::Element T;
		typedef typename Policy::Hash Hash;

		std::vector<detail::ElementAndScore<T>> open;

		size_t startingPoints = info.startingCount();
		for (size_t i = 0; i < startingPoints; ++i) {
			T start = info.startingPoint(i);

			if (!info.passable(start))
			{
				return 0;
			}

			detail::ElementAndScore<T> initial;
			initial.element = start;
			initial.gscore = 0;
			initial.fscore = info.predict(start);

			open.push_back(initial);
		}

		std::set<Hash> closed;
		std::map<Hash, Hash> from;

		while (!open.empty())
		{
			detail::ElementAndScore<T> target = open.back();
			Hash targetHash = info.hash(target.element);

			open.pop_back();

			if (info.finished(target.element))
			{
				return 1;
			}

			closed.insert(targetHash);

			size_t neighbors = info.neighborCount(target.element);
			for (size_t i = 0; i < neighbors; ++i)
			{
				detail::ElementAndScore<T> score;
				score.element = info.neighbor(target.element, i);

				Hash h = info.hash(score.element);
				if (closed.find(h) != closed.end())
				{
					continue;
				}

				if (!info.passable(score.element))
				{
					continue;
				}

				score.gscore = target.gscore + 1;
				score.fscore = score.gscore + info.predict(score.element);

				auto it = std::find(open.begin(), open.end(), score);
				if (it == open.end())
				{
					from[targetHash] = h;
					open.push_back(score);
				}
				else
				{
					if (it->fscore > score.fscore)
					{
						from[targetHash] = h;
						it->fscore = score.fscore;
						it->gscore = score.gscore;
					}
				}
			}

			std::sort(open.begin(), open.end());
		}

		return 0;
	}
}
