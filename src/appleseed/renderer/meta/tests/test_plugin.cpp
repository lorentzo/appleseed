
// appleseed.renderer headers.
#include "renderer/global/globaltypes.h"
#include "renderer/modeling/object/object.h"
#include "renderer/modeling/object/curveobject.h"
#include "renderer/modeling/object/curveobjectwriter.h"
#include "renderer/utility/paramarray.h"

// appleseed.foundation headers.
#include "foundation/platform/thread.h"
#include "foundation/utility/autoreleaseptr.h"
#include "foundation/utility/string.h"
#include "foundation/utility/searchpaths.h"
#include "foundation/utility/test.h"

// Boost headers.
#include "boost/filesystem.hpp"

// Standard
#include <string>
#include <cstring>

using namespace boost;
using namespace boost::filesystem;
using namespace foundation;
using namespace renderer;
using namespace std;

/*
TODO:
    + allow specification of control points
    + allow specification of basis
    + allow specification of curve type
    + allow specification of out dir
    + allow specification of name
*/

TEST_SUITE(PLUGIN)
{

	TEST_CASE(plugin1){

		// create an curve object.
		auto_release_ptr<CurveObject> curve_object(
		    CurveObjectFactory().create(
			"curve", 
			ParamArray()
		    )
		);

		// Define control points
		// GVector is defined in globaltypes.h
		static const GVector3 ControlPoints[] = {
				GVector3(0.0, 2.0, 4000.0), 
				GVector3(0.0, 1.0, 2000.0),
				GVector3(0.0, -1.0, 1000.0),
				GVector3(0.0, -2.0, 0.0)
		};

		// add basis
		curve_object->push_basis(CurveBasis::Bezier);

		// add control points and type
		curve_object->push_curve3(
			    Curve3Type(
				ControlPoints, // control points
				GScalar(0.05),  // width
				GScalar(1.0),  // opacity
				Color3f(0.2f, 0.4f, 0.7f) // color
			    )
		);

		//create out dir
		const path m_out_dir = absolute("unit tests/outputs/plugin/");
		remove_all(m_out_dir);
		foundation::sleep(50); // win and access
		create_directory(m_out_dir);

		// write a curve object
		ParamArray& params = curve_object->get_parameters();

		if (!params.strings().exist("filepath"))
		{
		    const string object_name = curve_object->get_name();
		    const string filename = object_name + ".binarycurve";

		    
		    // Write the curve file to disk.
		    const string filepath = (m_out_dir / filename).string();
		    CurveObjectWriter::write(curve_object.ref(), filepath.c_str());


		    // Add a file path parameter to the object.
		    params.insert("filepath", filename);
		}

	}

}
