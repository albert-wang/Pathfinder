#include <iostream>

#include "logger.h"

namespace Rawr
{
	Logger log;

	namespace Detail
	{
		LoggerStream::LoggerStream()
		{}

		LoggerStream::LoggerStream(const LoggerStream&&)
		{}

		LoggerStream::~LoggerStream()
		{
			std::cerr << internalStream.str() << "\n";
		}
	}
}