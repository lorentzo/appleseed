
// appleseed.cli headers.
#include "commandlinehandler.h"

// appleseed.renderer headers.
#include "renderer/modeling/object/curveobject.h"
#include "renderer/utility/paramarray.h"
#include "renderer/global/globaltypes.h"
#include "renderer/modeling/object/curveobjectwriter.h"

// appleseed.foundation headers.
#include "foundation/utility/autoreleaseptr.h"

// standard headers.
#include <string>
#include <vector>

using namespace renderer;
using namespace foundation;
using namespace std;

//
// Curve builder class.
//

class CurveBuilder
{

  private:

	string m_curve_object_name;
	const GVector3* m_control_points;
	vector<float> m_color;
	
  public:
	
	// Constructor. 
	CurveBuilder(const string& curve_object_name, const GVector3* control_points, const vector<float>& color)
	{
		m_curve_object_name = curve_object_name;
		m_control_points = control_points;
		m_color = color;
	}

	// Curve builder function.
	auto_release_ptr<CurveObject> build_curve()
	{

		// Create an curve object.
		auto_release_ptr<CurveObject> curve_object(
			CurveObjectFactory().create(
			m_curve_object_name.c_str(), 
			ParamArray()));

		// Add basis.
		curve_object->push_basis(CurveBasis::Bezier);

		// Add control points and type.
		curve_object->push_curve3(
		    Curve3Type(
		    m_control_points, // control points
		    GScalar(0.05),  // width
		    GScalar(1.0),  // opacity
		    Color3f(m_color[0], m_color[1], m_color[2])));

		return curve_object;

	}

};

//
// Main function.
//

int main(int argc, char* argv[])
{

	// Create command line handler.
    CommandLineHandler cl;

    // Parse the command line.
    cl.parse(argc, argv);

	// Curve object name.
    string name = cl.m_curve_name.value();

	// Curve object color.
	const vector<float> color = cl.m_rgb.values();

	// Curve object control points.
	const vector<float> cpp = cl.m_control_points.values();

	GVector3 ControlPoints[] = {
		GVector3(cpp[0], cpp[1], cpp[2]),
		GVector3(cpp[3], cpp[4], cpp[5]),
		GVector3(cpp[6], cpp[7], cpp[8]),
		GVector3(cpp[9], cpp[10], cpp[11])
	};
	
	// Create curve builder object.
	CurveBuilder curve_builder(name, ControlPoints, color);

	// Create curve object.
	auto_release_ptr<CurveObject> curve(curve_builder.build_curve());

	// Write curve object to file.
	CurveObjectWriter::write(curve.ref(), name.c_str());

}
