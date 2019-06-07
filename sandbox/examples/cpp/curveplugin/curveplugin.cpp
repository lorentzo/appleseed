// appleseed.renderer headers.
#include "renderer/modeling/object/curveobject.h"
#include "renderer/utility/paramarray.h"
#include "renderer/global/globaltypes.h"
#include "renderer/modeling/object/curveobjectwriter.h"

// appleseed.foundation headers.
#include "foundation/utility/autoreleaseptr.h"

// standard headers.
#include <string>

using namespace renderer;
using namespace foundation;
using namespace std;

class CurveBuilder
{

	private:

		string m_curve_object_name;
		GVector3 *m_control_points;
	

    	public:
	
		// constructor 
		CurveBuilder(string curve_object_name, GVector3 *control_points){
			m_curve_object_name = curve_object_name;
			m_control_points = control_points;
		}

		// curve builder
		auto_release_ptr<CurveObject> build_curve(){


			// create an curve object.
			auto_release_ptr<CurveObject> curve_object(
				CurveObjectFactory().create(
				m_curve_object_name.c_str(), 
				ParamArray()
				)
			);

			// add basis
			curve_object->push_basis(CurveBasis::Bezier);

			// add control points and type
			curve_object->push_curve3(
			    Curve3Type(
			    m_control_points, // control points
			    GScalar(0.05),  // width
			    GScalar(1.0),  // opacity
			    Color3f(0.2f, 0.4f, 0.7f) // color
			    )
			);

			return curve_object;


		}

};

int main()
{

	string name = "curve";

	GVector3 ControlPoints[] = {
		    GVector3(0.0, 2.0, 4000.0), 
		    GVector3(0.0, 1.0, 2000.0),
		    GVector3(0.0, -1.0, 1000.0),
		    GVector3(0.0, -2.0, 0.0)
	};

	CurveBuilder curve_builder(name, ControlPoints);

	auto_release_ptr<CurveObject> curve(curve_builder.build_curve());

	CurveObjectWriter::write(curve.ref(), "curve.binarycurve");

}
