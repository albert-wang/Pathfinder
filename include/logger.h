#pragma once
#include <sstream>

namespace Rawr
{
	enum Priority
	{
		Debug = 1 << 0,
		Warning = 1 << 1,
		Error = 1 << 2
	};

	namespace Detail
	{
		class LoggerStream
		{
		public:
			LoggerStream();
			LoggerStream(const LoggerStream&&);
			~LoggerStream();

			template<typename T>
			const LoggerStream& operator<<(const T& v) const
			{
				internalStream << v;
				return *this;
			}
		private:
			mutable std::stringstream internalStream;
		};
	}

	class Logger
	{
	public:
#ifdef DISABLE_LOGGING
		template<typename T>
		const Logger& operator<<(const T&) const { return *this; }

		inline const Logger& operator[](size_t) const { return *this; }
#else
		template<typename T>
		Detail::LoggerStream operator<<(const T& t) const
		{
			Detail::LoggerStream res;
			res << t;
			return res;
		}

		inline Detail::LoggerStream operator[](size_t) const
		{
			return Detail::LoggerStream();
		}
#endif
	};

	extern Logger log;
}