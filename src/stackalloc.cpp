#include "stackalloc.h"
#include <memory.h>
#include <iostream>

namespace Engine
{
	//Stack Allocator
	namespace Memory
	{
		StackAllocator::StackAllocator(size_t memoryBytes)
			:avaliableSpace(((memoryBytes / 16) + 1) * 16)
		{
			assert(memoryBytes > 0);
			base = static_cast<boost::uint8_t *>(malloc(avaliableSpace));
			current = base;
		}

		StackAllocator::~StackAllocator()
		{
			free(base);
		}

		StackAllocator::Mark StackAllocator::mark() 
		{
			return reinterpret_cast<Incomplete *>(current);
		}

		void StackAllocator::release(StackAllocator::Mark b)
		{
			current = reinterpret_cast<boost::uint8_t *>(b);
		}

		const boost::uint8_t * StackAllocator::currentAllocation() const
		{
			return current;
		}

		void StackAllocator::reset(boost::uint8_t * point)
		{
			current = point;
		}

		size_t StackAllocator::remaining() const
		{
			return avaliableSpace - (current - base);
		}
	}

	//Stack Scope
	namespace Memory
	{
		StackScope::StackScope(StackAllocator * base)
			:base(base)
			,mark(NULL)
			,finalizer(NULL)
		{
			mark = base->mark();
		}

		StackScope::~StackScope()
		{
			close();
		}

		void StackScope::close()
		{
			FinalizerEntry * fin = finalizer;
			while (fin)
			{
				fin->finalizer(reinterpret_cast<boost::uint8_t *>(fin));
				fin = fin->next;
			}

			base->release(mark);
			mark = base->mark();
		}
	}
}
