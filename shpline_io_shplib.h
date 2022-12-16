#ifndef SHPLINE_IO_SHPLIB
#define SHPLINE_IO_SHPLIB

#include <vector>
#include <string>
#include <Eigen/Eigen>
#include <shapefil.h>

#define MAX_PATH  256

namespace utility
{
	class shpline_io_shplib
	{
	public:
		shpline_io_shplib();
		~shpline_io_shplib();

		static
			bool writePolylineShp(const std::string& filename,
				const std::vector<std::vector<Eigen::Vector3d>>& pts,
				const bool is_closed = false);

		bool writePolylineShpWithOffset(const std::string& filename,
			const std::vector<std::vector<Eigen::Vector3f>>& pts,
			const Eigen::Vector3d offset,
			const bool is_closed = false);

		static
			bool readPolylineShp(const std::string& filename, std::vector<std::vector<Eigen::Vector3d>>& pts);
	};
} // namespace utility


#endif