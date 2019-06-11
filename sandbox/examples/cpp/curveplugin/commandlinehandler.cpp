
// Interface header.
#include "commandlinehandler.h"

CommandLineHandler::CommandLineHandler() 
{
    m_parser.add_option_handler(
        &m_curve_name
		.add_name("--name")
		.add_name("-n")
		.set_description("curve object name")
		.set_syntax("name")
		.set_exact_value_count(1));

    m_parser.add_option_handler(
        &m_rgb
	    .add_name("--rgb")	
	    .add_name("-rgb")
	    .set_description("red, green, blue value")
	    .set_syntax("r g b")
	    .set_exact_value_count(3));

    m_parser.add_option_handler(
        &m_control_points
	    .add_name("--control-points")	
	    .add_name("-cps")
	    .set_description("control points")
	    .set_syntax("cp1x cp1y cp1z cp2x cp2y cp2z cp3x cp3y cp3z cp4x cp4y cp4z")
	    .set_exact_value_count(12));

}

void CommandLineHandler::parse(const int argc, char* argv[])
{
    m_parser.parse(argc, argv, m_parse_results);
}

foundation::CommandLineParser& CommandLineHandler::parser()
{
    return m_parser;
}

const foundation::CommandLineParser& CommandLineHandler::parser() const
{
    return m_parser;
}












