
// appleseed.foundation headers.
#include "foundation/utility/commandlineparser.h"

// Standard headers.
#include <string>

//
// Command line handler.
//

class CommandLineHandler
{
  public:
    // Constructor.
    CommandLineHandler();

    foundation::ValueOptionHandler<std::string> m_curve_name;
    foundation::ValueOptionHandler<float> m_rgb;
    foundation::ValueOptionHandler<float> m_control_points;

    void parse(
        const int argc,
		char* argv[]);

    foundation::CommandLineParser& parser();
    const foundation::CommandLineParser& parser() const;

  private:
    foundation::CommandLineParser m_parser;
    foundation::ParseResults m_parse_results;
    
    


	
};
