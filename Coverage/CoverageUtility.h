#include <string>
#include <sstream>

namespace Robotics
{
	namespace GameTheory 
	{
		const int MAX_NUMBER_NEUTRAL_AGENT = 0;
		const int DISCRETIZATION_COL = 31; //numero di quadrati in spazio discreto(era a 20)
		const int DISCRETIZATION_ROW = 31;

		template <typename T>
		std::string to_string(T const& value) {
			std::stringstream sstr;
			sstr << value;
			return sstr.str();
		}
	}
}