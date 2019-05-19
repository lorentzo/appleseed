
// appleseed.renderer headers.
#include "renderer/global/globaltypes.h"
#include "renderer/kernel/intersection/intersector.h"
#include "renderer/kernel/intersection/tracecontext.h"
#include "renderer/kernel/shading/shadingpoint.h"
#include "renderer/kernel/shading/shadingray.h"
#include "renderer/kernel/texturing/texturecache.h"
#include "renderer/kernel/texturing/texturestore.h"
#include "renderer/modeling/object/object.h"
#include "renderer/modeling/object/curveobject.h"
#include "renderer/modeling/scene/assembly.h"
#include "renderer/modeling/scene/assemblyinstance.h"
#include "renderer/modeling/scene/containers.h"
#include "renderer/modeling/scene/objectinstance.h"
#include "renderer/modeling/scene/scene.h"
#include "renderer/modeling/scene/visibilityflags.h"
#include "renderer/utility/paramarray.h"
#include "renderer/utility/testutils.h"
#include "renderer/modeling/project/project.h"
#include "renderer/modeling/project/projectfilewriter.h"
#include "renderer/modeling/camera/camera.h"
#include "renderer/modeling/camera/pinholecamera.h"
#include "renderer/modeling/project/configuration.h"
#include "renderer/modeling/frame/frame.h"

// appleseed.foundation headers.
#include "foundation/math/matrix.h"
#include "foundation/math/transform.h"
#include "foundation/math/vector.h"
#include "foundation/utility/autoreleaseptr.h"
#include "foundation/utility/containers/dictionary.h"
#include "foundation/utility/test.h"
#include "foundation/utility/string.h"
#include "foundation/utility/searchpaths.h"
#include "foundation/utility/statistics.h"
#include "foundation/platform/types.h"
#include "foundation/utility/bufferedfile.h"

// boost
#include "boost/filesystem/operations.hpp" //?
#include "boost/filesystem/path.hpp"

// Standard headers.
#include <string>
#include <cstddef>


using namespace foundation;
using namespace renderer;
using namespace boost;
using namespace boost::filesystem;
using namespace std;

TEST_SUITE(Intersector)
{
    struct TestScene
      : public TestSceneBase
    {
        TestScene()
        {

            // NB: "project" variable is initialized as auto_release_ptr private member
            //      in TestSceneBaseProjectHolder which
            //      is parent of TestSceneBase which is parent of this class.
            // 
            //      In class TestSceneBase "project" is a public reference.

            // add base config
            m_project.add_default_configurations();

            // NB: "scene" variable is added to "project" variable in TestSceneBaseProjectHolder
            //      constructor.
            //
            //      In class TestSceneBase "scene" is a public reference.


            // Create an assembly
            auto_release_ptr<Assembly> assembly(
                AssemblyFactory().create("assembly", ParamArray()));

            // create an curve object.
            auto_release_ptr<CurveObject> curve_object(
                CurveObjectFactory().create(
                    "curve_non_uniform_coordinates", 
                    ParamArray()
                )
            );
            static const GVector3 ControlPoints[] = {
                GVector3(0.0, 2.0, 4000.0), 
                GVector3(0.0, 1.0, 2000.0),
                GVector3(0.0, -1.0, 1000.0),
                GVector3(0.0, -2.0, 0.0)
            };
            curve_object->push_basis(CurveBasis::Bezier);
            curve_object->push_curve3(
                Curve3Type(
                    ControlPoints, // control points
                    GScalar(0.05),  // width
                    GScalar(1.0),  // opacity
                    Color3f(0.2f, 0.4f, 0.7f) // color
                )
            );

            // Create an instance of the curve object.
            auto_release_ptr<ObjectInstance> curve_instance(
                ObjectInstanceFactory::create(
                    "curve_non_uniform_coordinates_instance",
                    ParamArray(),
                    "curve_non_uniform_coordinates",
                    Transformd::identity(),
                    StringDictionary()
                )
            );

            // add curve object to the assembly
            assembly->objects().insert(
		auto_release_ptr<Object>(
                    curve_object
		)
            );

            // add curve object instance to the assembly
            assembly->object_instances().insert(
                curve_instance
            );

            // create camera
            auto_release_ptr<Camera> camera(
                PinholeCameraFactory().create(
                    "camera",
                    ParamArray()
                        .insert("film_width", "0.025")
                        .insert("film_height", "0.025")
                        .insert("focal_length", "0.035")
                )
            ); // transformations?

            // add camera to the scene
            m_scene.cameras().insert(camera);

            // add frame definition
            auto_release_ptr<Frame> frame(
                FrameFactory::create(
                    "beauty", // name
                    ParamArray()
                        .insert("resolution", "1024 768")
                        .insert("camera", "camera")
                )

            );

            // add frame to the project
            m_project.set_frame(frame);

            // add assembly instance to the scene
            m_scene.assembly_instances().insert(
                auto_release_ptr<AssemblyInstance>(
                    AssemblyInstanceFactory::create(
                        "assembly_instance",
                        ParamArray(),
                        "assembly")));

            // add assembly to the scene
            m_scene.assemblies().insert(assembly);
        }
    };

    template <bool UseEmbree>
    struct Fixture
      : public StaticTestSceneContext<TestScene>
    {
        TraceContext    m_trace_context;
        TextureStore    m_texture_store;
        TextureCache    m_texture_cache;
        Intersector     m_intersector;

        Fixture()
          : m_trace_context(m_scene)
          , m_texture_store(m_scene)
          , m_texture_cache(m_texture_store)
          , m_intersector(m_trace_context, m_texture_cache)
        {
#ifdef APPLESEED_WITH_EMBREE
            m_trace_context.set_use_embree(UseEmbree);
#endif
            m_trace_context.update();
        }
    };

    TEST_CASE_F(intersect, Fixture<false>)
    {
        const ShadingRay ray(
            Vector3d(0.0, 0.0, 0.0),
            Vector3d(0.0, 0.0, 1.0),
            0.0,                                // tmin
            2.0,                                // tmax
            ShadingRay::Time(),
            VisibilityFlags::CameraRay,
            0);                                 // depth

        ShadingPoint shading_point;
        const bool hit = m_intersector.trace(ray, shading_point);

	// stats
	StatisticsVector statistic_vector = m_intersector.get_statistics();
        string statistics = statistic_vector.to_string().c_str();

	// save stats
	const char* Filename = "unit tests/outputs/test_stats.tmp";
	const size_t BufferSize = 4;
	BufferedFile file(
		Filename, 
		BufferedFile::TextType,
		BufferedFile::WriteMode,
		BufferSize
	);
	file.write(statistics);
	file.close();
	

        // save project
        const path m_out_dir = absolute("unit tests/outputs/test_scene_case/");
        remove_all(m_out_dir);
        foundation::sleep(50); // win and access
        create_directory(m_out_dir);

        Project& m_project = get_project();

        m_project.set_path((m_out_dir / "curve_non_uniform_coordinates_intersector.appleseed").string().c_str());
        m_project.search_paths().set_root_path(m_out_dir.string());

        ProjectFileWriter::write(
            m_project,
            (m_out_dir / "curve_non_uniform_coordinates_intersector.appleseed").string().c_str()
        );



        EXPECT_FALSE(hit);
    }

}
