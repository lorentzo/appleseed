
// appleseed.renderer headers. Only include header files from renderer/api/.
#include "renderer/api/bsdf.h" // lambertianbrdf
#include "renderer/api/camera.h"
#include "renderer/api/color.h" // colorEntity, colorspace
#include "renderer/api/environment.h"
#include "renderer/api/environmentedf.h"
#include "renderer/api/environmentshader.h"
#include "renderer/api/frame.h"
#include "renderer/api/light.h" 
#include "renderer/api/log.h"
#include "renderer/api/material.h" // material
#include "renderer/api/object.h" // object, meshobject, meshobjectreader
#include "renderer/api/project.h" // project
#include "renderer/api/rendering.h"
#include "renderer/api/scene.h" // scene, assembly
#include "renderer/api/surfaceshader.h" // surfaceshader
#include "renderer/api/utility.h" // paramArray

// appleseed.foundation headers.
#include "foundation/core/appleseed.h"
#include "foundation/math/matrix.h"
#include "foundation/math/scalar.h"
#include "foundation/math/transform.h" // transform
#include "foundation/math/vector.h"
#include "foundation/utility/containers/dictionary.h" // stringdictionary
#include "foundation/utility/log/consolelogtarget.h"
#include "foundation/utility/autoreleaseptr.h" // autoreleaseptr
#include "foundation/utility/searchpaths.h"

// Standard headers.
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

using namespace foundation;
using namespace renderer;
using namespace std;

auto_release_ptr<Project> build_project(string object_name)
{
    // Create an empty project. Add data folder folder.
    auto_release_ptr<Project> project(ProjectFactory::create("plugin"));
    project->search_paths().push_back_explicit_path("data");

    // Add default configurations to the project.
    project->add_default_configurations();

    // Set number of samples. Quality parameter: 
    // tradeoff: quality vs render time
    project->configurations()
        .get_by_name("final")->get_parameters()
            .insert_path("uniform_pixel_renderer.samples", "25");

    // Create a scene.
    auto_release_ptr<Scene> scene(SceneFactory::create());

    // Create an assembly.
    auto_release_ptr<Assembly> assembly(
        AssemblyFactory().create(
            "assembly",
            ParamArray()));

    
    //------------------------------------------------------------------------
    // Materials
    //------------------------------------------------------------------------

    // Create a color called "yellow" and insert it into the assembly.
    static const float YellowReflectance[] = {0.8f, 0.8f, 0.2f};
    assembly->colors().insert(
        ColorEntityFactory::create(
            "yellow",
            ParamArray()
                .insert("color_space", "srgb"),
            ColorValueArray(3, YellowReflectance)));

    // Create BRDF called "diffuse_yellow_brdf" and insert it into the assembly.
    // Bidirectional distribution function: describes how light is reflected
    // from surface given incoming and outcoming direction.
    assembly->bsdfs().insert(
        LambertianBRDFFactory().create(
            "diffuse_yellow_brdf",
            ParamArray()
                .insert("reflectance", "yellow")));

    // Create a physical surface shader and insert it into the assembly.
    assembly->surface_shaders().insert(
        PhysicalSurfaceShaderFactory().create(
            "physical_surface_shader",
            ParamArray()));

    // Create a material called "yellow_material" and insert it into the assembly.
    assembly->materials().insert(
        GenericMaterialFactory().create(
            "yellow_material",
            ParamArray()
                .insert("surface_shader", "physical_surface_shader")
                .insert("bsdf", "diffuse_yellow_brdf")));

    
    //------------------------------------------------------------------------
    // Geometry
    //------------------------------------------------------------------------

    MeshObjectArray objects;
    MeshObjectReader::read(
        project->search_paths(),
        object_name.c_str(),
        ParamArray()
            .insert("filename", object_name.c_str()),
        objects);
    
    // Insert all the objects into the scene.
    for (size_t i = 0; i < objects.size(); ++i)
    {
        // Insert the current object into the assebly.
        MeshObject* object = objects[i];
        assembly->objects().insert(auto_release_ptr<Object>(object));

        // Create an instance of current object and inset it into the assembly.
        const string instance_name = string(object->get_name()) + "_inst";
        assembly->object_instances().insert(
            ObjectInstanceFactory::create(
                instance_name.c_str(), // name of the instance
                ParamArray(),          // params of the instance 
                object->get_name(),    // name of the object of which the instance is made 
                Transformd::identity(), // transformations
                StringDictionary()
                    .insert("default", "yellow_material")
                    .insert("default2", "yellow_material")));

    }

  
    //------------------------------------------------------------------------
    // Light
    //------------------------------------------------------------------------

    // Create a color called "light_intensity" and insert it into the assembly.
    static const float LightRadiance[] = { 1.0f, 1.0f, 1.0f };
    assembly->colors().insert(
        ColorEntityFactory::create(
            "light_intensity",
            ParamArray()
                .insert("color_space", "srgb")
                .insert("multiplier", "30.0"),
            ColorValueArray(3, LightRadiance)));
        
    // Create a point light called "light" and insert it into the assembly.
    auto_release_ptr<Light> light(
        PointLightFactory().create(
            "light",
            ParamArray()
                .insert("intensity", "light_intensity")));
        
    light->set_transform(
        Transformd::from_local_to_parent(
            Matrix4d::make_translation(Vector3d(0.6, 2.0, 1.0))));
            
    assembly->lights().insert(light);


    //------------------------------------------------------------------------
    // Assembly instance
    //------------------------------------------------------------------------

    // Create an instance of the assembly and insert it into the scene.
    auto_release_ptr<AssemblyInstance> assembly_instance(
        AssemblyInstanceFactory::create(
            "assembly_inst",
            ParamArray(),
            "assembly"));
     
    assembly_instance
        ->transform_sequence()  
            .set_transform(
                0.0f,
                Transformd::identity());
            
    scene->assembly_instances().insert(assembly_instance);

    // Insert the assembly instance to the scene.
    scene->assemblies().insert(assembly);

    //------------------------------------------------------------------------
    // Environment
    //------------------------------------------------------------------------

    // Similar to the material creation.

    // Create a color called "sky_radiance" and insert it into the scene.
    static const float SkyRadiance[] = { 0.75f, 0.80f, 1.0f };
    scene->colors().insert(
        ColorEntityFactory::create(
            "sky_radiance",
            ParamArray()
                .insert("color_space", "srgb")
                .insert("multiplier", "0.5"),
            ColorValueArray(3, SkyRadiance)));

    // Create an environment EDF called "sky_edf" and insert it into the scene.
    scene->environment_edfs().insert(
        ConstantEnvironmentEDFFactory().create(
            "sky_edf",
            ParamArray()
                .insert("radiance", "sky_radiance")));

    // Create an environment shader called "sky_shader" and insert it into the scene.
    scene->environment_shaders().insert(
        EDFEnvironmentShaderFactory().create(
            "sky_shader",
            ParamArray()
                .insert("environment_edf", "sky_edf")));

    // Create an environment called "sky" and bind it to the scene.
    scene->set_environment(
        EnvironmentFactory::create(
            "sky",
            ParamArray()
                .insert("environment_edf", "sky_edf")
                .insert("environment_shader", "sky_shader")));

    //------------------------------------------------------------------------
    // Camera
    //------------------------------------------------------------------------

    // Create a pinhole camera with film dimensins 0.980 x 0735 in (24.892 x 18.669 mm)
    auto_release_ptr<Camera> camera(
        PinholeCameraFactory().create(
            "camera",
            ParamArray()
                .insert("film_dimensions", "0.024892 0.018669")
                .insert("focal_lenght", "0.035")));
    
    // Place and orient the camera.
    camera->transform_sequence().set_transform(
        0.0f,
        Transformd::from_local_to_parent(
            Matrix4d::make_rotation(Vector3d(1.0, 0.0, 0.0), deg_to_rad(-20.0)) *
            Matrix4d::make_translation(Vector3d(0.0, 0.8, 11.0))));

    // Bind the camera to the scene.
    scene->cameras().insert(camera);

    //------------------------------------------------------------------------
    // Frame
    //------------------------------------------------------------------------

    // Create frame and bind it to the project.
    project->set_frame(
        FrameFactory::create(
            "beauty",
            ParamArray()
                .insert("camera", "camera")
                .insert("resolution", "640 480")));
    
    // Bind the scene to the project
    project->set_scene(scene);

    return project;

}

int main(int argc, char* argv[])
{
    // log
    // todo

    // Build the project.
    // TODO: geometry object name will be passed from CLI.
    // TODO: enable additional project configurations that are red from file or CLI.
    auto_release_ptr<Project> project(build_project("scene.obj"));

    // Create the master renderer.
    DefaultRendererController renderer_controller;
    SearchPaths resource_search_paths;
    unique_ptr<MasterRenderer> renderer (
        new MasterRenderer(
            project.ref(),
            project->configurations().get_by_name("final")->get_inherited_parameters(),
            resource_search_paths));

    // Configure camera positions and orientations.
    // TODO: positions and orientations should be red from file.
    int n_states = 2;
    Vector3d states[][3] = {
        {Vector3d(5.0, 5.0, 5.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0)}, // origin, target, up
        {Vector3d(7.0, 7.0, 5.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0)}
    }; 

    // Perform rendering for existing states.
    for (int i = 0; i < n_states; ++i){

        // Change the camera position and orientation.
        project->get_scene()
                    ->cameras() // entity vector
                        .get_by_name("camera") 
                            ->transform_sequence().set_transform(
                                0.0f,
                                Transformd::from_local_to_parent( // foundation/math/matrix
                                    Matrix4d::make_lookat(
                                        states[i][0], // origin
                                        states[i][1], // target
                                        states[i][2]))); // up

        // Render the frame.
        renderer->render(renderer_controller);

        // Save the frame to disk.
        string render_file_name = "output/test" + to_string(i) + ".png";
        project->get_frame()->write_main_image(render_file_name.c_str());
    }

    // Save the project to disk.
    ProjectFileWriter::write(project.ref(), "output/test.appleseed");

    // Make sure to delete the master renderer before the project and the logger / log target.
    renderer.reset();

    return 0;
}