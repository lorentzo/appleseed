
// appleseed.renderer headers.

#include "renderer/kernel/intersection/intersectionsettings.h" // ?

#include "renderer/modeling/object/object.h"
#include "renderer/modeling/object/curveobject.h"
#include "renderer/modeling/scene/objectinstance.h"

#include "renderer/modeling/project/project.h"
#include "renderer/modeling/project/projectfilewriter.h"

#include "renderer/modeling/scene/scene.h"
#include "renderer/modeling/scene/containers.h" //?

#include "renderer/modeling/scene/assembly.h"
#include "renderer/modeling/scene/assemblyinstance.h"

#include "renderer/utility/paramarray.h"

#include "renderer/modeling/camera/camera.h"
#include "renderer/modeling/camera/pinholecamera.h"

#include "renderer/modeling/frame/frame.h"

#include "renderer/modeling/environment/environment.h"

#include "renderer/modeling/project/configuration.h"


// appleseed.foundation headers.
#include "foundation/platform/thread.h" // ?
#include "foundation/utility/test.h"
#include "foundation/utility/autoreleaseptr.h"
#include "foundation/utility/searchpaths.h"

// Boost headers.
#include "boost/filesystem/operations.hpp" //?
#include "boost/filesystem/path.hpp"

// Standard headers.
#include <string>

using namespace boost;
using namespace boost::filesystem;
using namespace foundation;
using namespace renderer;
using namespace std;


TEST_SUITE(Koji_Bezier)
{

    TEST_CASE(my_curve){

        // output path for project
        // boost::filesystem::absolute(path)
        const path m_out_dir = absolute("unit tests/outputs/test_scene_case/");

        // reset output directory
        // boost::filesystem::remove_all(path), boost::filesystem::create_directory(path)
        remove_all(m_out_dir);
        foundation::sleep(50); // win and access
        create_directory(m_out_dir);

        // crate project.
        auto_release_ptr<Project> m_project = ProjectFactory::create("project");
        
        // create scene.
        auto_release_ptr<Scene> scene(SceneFactory::create());

        // create an assembly.
        auto_release_ptr<Assembly> assembly(
            AssemblyFactory().create(
                "assembly", 
                ParamArray()
            )
        );

        // create an object.
        auto_release_ptr<CurveObject> curve_object(
            CurveObjectFactory().create(
                "moja_krivulja", 
                ParamArray()
            )
        );

        // where is gvector3 defined?
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

        // Create an instance of the object.
        auto_release_ptr<ObjectInstance> curve_instance(
            ObjectInstanceFactory::create(
                "mojaKrivulja_inst",
                ParamArray(),
                "moja_krivulja",
                Transformd::identity(),
                StringDictionary()
            )
        );

        // Create an instance of the assembly.
        auto_release_ptr<AssemblyInstance> assembly_instance(
            AssemblyInstanceFactory::create(
                "assembly_inst",
                ParamArray(),
                "assembly"
            )
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

        // add frame definition
        auto_release_ptr<Frame> frame(
            FrameFactory::create(
                "beauty", // name
                ParamArray()
                    .insert("resolution", "1024 768")
                    .insert("camera", "camera")
            )

        );

        // environment definition
        auto_release_ptr<Environment> environment(
            EnvironmentFactory::create(
                "default", // name
                ParamArray()
            )

        );


        // add object to assembly
        assembly->objects().insert(auto_release_ptr<Object>(curve_object));

        // add object instance to assembly
        assembly->object_instances().insert(curve_instance);

        // add assembly to the scene
        scene->assemblies().insert(assembly);

        // add assembly instance to the scene
        scene->assembly_instances().insert(assembly_instance);

        // add camera to the scene
        scene->cameras().insert(camera);

        // add environment to the scene
        scene->set_environment(environment);

        // add scene to the project
        m_project->set_scene(scene);

        // add frame to the project
        m_project->set_frame(frame);

        // add base config
        m_project->add_default_configurations();
        
        // add paths to the project
        m_project->set_path((m_out_dir / "project.appleseed").string().c_str());
        m_project->search_paths().set_root_path(m_out_dir.string());

        // save project
        ProjectFileWriter::write(
            m_project.ref(),
            (m_out_dir / "project.appleseed").string().c_str()
        );

    }
}