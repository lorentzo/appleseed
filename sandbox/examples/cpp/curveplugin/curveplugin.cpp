// appleseed.renderer headers.
#include "renderer/modeling/object/curveobject.h"
#include "renderer/utility/paramarray.h"
#include "renderer/global/globaltypes.h"
#include "renderer/modeling/object/curveobjectwriter.h"

// appleseed.foundation headers.
#include "foundation/utility/autoreleaseptr.h"

using namespace renderer;
using namespace foundation;

class CurveBulder
{

    public:

        auto_release_ptr<CurveObject> build_curve(){


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

            return curve_object

            
        }
};

int main()
{

    CurveBulder curve_builder;

    auto_release_ptr<CurveObject> curve(curve_builder.build_curve());

    CurveObjectWriter::write(curve.ref(), "output/curve.binarycurve");

}