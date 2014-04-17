#include <boost/cstdint.hpp>
#include <boost/noncopyable.hpp>
#include <boost/static_assert.hpp>
#include <boost/utility.hpp>
#include <boost/type_traits.hpp>
#include <boost/preprocessor.hpp>

#include <cassert>
#include <utility>

#pragma once
namespace Engine
{
	namespace Memory
	{
		//This creates 16-byte aligned allocations.
		class StackAllocator : boost::noncopyable
		{
		public:
			struct Incomplete;
			typedef Incomplete * Mark;

			//Will always allocate a multiple of 16 bytes.
			explicit StackAllocator(size_t memoryBytes);
			~StackAllocator();

			template<typename T>
			size_t computeActualAllocationSize(size_t n) const
			{
				if ((sizeof(T) * n) % 16 != 0)
				{	
					return (((sizeof(T) * n) / 16) + 1) * 16;
				}
				else
				{
					return sizeof(T) * n;
				}
			}

			template<typename T>
			T * allocate(size_t count)
			{
				//Make sure we have enough space to allocate.
				assert(sizeof(T) * count + (current - base) < avaliableSpace);
				assert(count > 0);

				T * result = reinterpret_cast<T *>(current);
				
				//The current pointer is always resting at a multiple of 16 bytes.
				current += computeActualAllocationSize<T>(count);
				return result;
			}

			template<typename T>
			bool wasMostRecentAllocation(const T * b, size_t size) const
			{
				//Expected next pointer: 
				const boost::uint8_t * base = reinterpret_cast<const boost::uint8_t *>(b);
				base += computeActualAllocationSize<T>(size);
				return base == current;
			}

			Mark mark();
			void release(Mark b);

			size_t remaining() const;
			const boost::uint8_t * currentAllocation() const;
			void reset(boost::uint8_t * point);
		private:
			boost::uint8_t * base;
			boost::uint8_t * current; 

			size_t avaliableSpace;
		};

		//This is an overlay on the stack allocator, which provides things like destructors.
		class StackScope : boost::noncopyable
		{
			//16 bytes for both x86 and x64.
			typedef void (*FinalizerMethod)(boost::uint8_t *);

			struct FinalizerEntry
			{
				FinalizerMethod finalizer;
				FinalizerEntry * next;

#if !defined(__LP64__)
				boost::uint8_t padding[8];
#endif
			};
			BOOST_STATIC_ASSERT(sizeof(FinalizerEntry) % 16 == 0);

			struct ArrayEntry
			{
				boost::uint32_t size;
				boost::uint8_t padding[12];
			};
			BOOST_STATIC_ASSERT(sizeof(ArrayEntry) % 16 == 0);

			template<typename T>
			static void destructor(boost::uint8_t * mem)
			{
				T * result = reinterpret_cast<T *>(mem + sizeof(FinalizerEntry));
				result->~T();
			}

			template<typename T>
			static void destructArray(boost::uint8_t * mem)
			{
				ArrayEntry * meta = reinterpret_cast<ArrayEntry *>(mem + sizeof(FinalizerEntry));
				T * base = reinterpret_cast<T *>(meta + 1);

				for (size_t i = 0; i < meta->size; ++i)
				{
					base[meta->size - 1 - i].~T();
				}
			}
		public:
			explicit StackScope(StackAllocator * base);
			~StackScope();

			void close();

//A few local defines to help out
#define ENABLE_POD(T) typename boost::enable_if<boost::is_pod<T>, T *>::type
#define DISABLE_POD(T) typename boost::disable_if<boost::is_pod<T>, T *>::type
#define FORWARD_ARG(z, n, data) BOOST_PP_CAT(A, n)&& BOOST_PP_CAT(a, n)

#define POD_CREATE(n) template<typename T BOOST_PP_COMMA_IF(n) BOOST_PP_ENUM_PARAMS(n, typename A)> \
	ENABLE_POD(T) create(BOOST_PP_ENUM(n, FORWARD_ARG, ~)) { T * result = base->allocate<T>(1); new (result) T(BOOST_PP_ENUM_PARAMS(n, a)); return result; } \
	template<typename T BOOST_PP_COMMA_IF(n) BOOST_PP_ENUM_PARAMS(n, typename A)> T * createPOD(BOOST_PP_ENUM(n, FORWARD_ARG, ~)) { T * result = base->allocate<T>(1); new (result) T(BOOST_PP_ENUM_PARAMS(n, a)); return result; }
			//Basic creates
			template<typename T>
			ENABLE_POD(T) create()
			{
				T * result = base->allocate<T>(1);
				return result;
			}

			template<typename T>
			T * createPOD() 
			{
				T * result = base->allocate<T>(1);
				return result;
			}

#define BOOST_PP_LOCAL_LIMITS (1, 9)
#define BOOST_PP_LOCAL_MACRO(n) POD_CREATE(n)
#include BOOST_PP_LOCAL_ITERATE()
#undef POD_CREATE
#undef BOOST_PP_LOCAL_MACRO
#undef BOOST_PP_LOCAL_LIMITS

			template<typename T>
			ENABLE_POD(T) createArray(size_t size)
			{
				T * result = base->allocate<T>(size);
				return result;
			}

			template<typename T>
			T * createArrayPOD(size_t size)
			{
				T * result = base->allocate<T>(size);
				return result;
			}

			template<typename T>
			bool unsafeResizeArray(T * pointer, size_t oldsize, size_t newsize)
			{
				assert(newsize > oldsize);
				if (base->remaining() > (newsize - oldsize) * sizeof(T))
				{
					if (base->wasMostRecentAllocation(pointer, oldsize))
					{
						base->reset(reinterpret_cast<boost::uint8_t *>(pointer) + base->computeActualAllocationSize<T>(newsize));
						return true;
					}
				}

				return false;
			}

#define NPOD_CREATE(n) template<typename T BOOST_PP_COMMA_IF(n) BOOST_PP_ENUM_PARAMS(n, typename A)> \
	DISABLE_POD(T) create(BOOST_PP_ENUM(n, FORWARD_ARG, ~)) { 										 \
		FinalizerEntry * entry = base->allocate<FinalizerEntry>(1); entry->finalizer = &destructor<T>; entry->next = finalizer; finalizer = entry; \
		T * result = base->allocate<T>(1); new (result) T(BOOST_PP_ENUM_PARAMS(n, a)); return result; }

#define BOOST_PP_LOCAL_LIMITS (0, 9)
#define BOOST_PP_LOCAL_MACRO(n) NPOD_CREATE(n)
#include BOOST_PP_LOCAL_ITERATE()
#undef NPOD_CREATE
#undef BOOST_PP_LOCAL_MACRO
#undef BOOST_PP_LOCAL_LIMITS

			template<typename T>
			DISABLE_POD(T) createArray(size_t size)
			{
				FinalizerEntry * entry = base->allocate<FinalizerEntry>(1);
				entry->finalizer = &destructArray<T>;
				entry->next = finalizer;
				finalizer = entry;

				ArrayEntry * arr = base->allocate<ArrayEntry>(1);
				arr->size = size;

				T * result = base->allocate<T>(size);
				for (size_t i = 0; i < size; ++i)
				{
					new (result + i) T();
				}

				return result;
			}
		private:
			StackAllocator * base;
			StackAllocator::Mark mark;
			FinalizerEntry * finalizer;
		};
	}
}
