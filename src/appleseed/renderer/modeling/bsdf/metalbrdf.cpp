
//
// This source file is part of appleseed.
// Visit https://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2016-2018 Esteban Tovagliari, The appleseedhq Organization
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

// Interface header.
#include "metalbrdf.h"

// appleseed.renderer headers.
#include "renderer/kernel/lighting/scatteringmode.h"
#include "renderer/kernel/shading/directshadingcomponents.h"
#include "renderer/kernel/shading/shadingpoint.h"
#include "renderer/modeling/bsdf/bsdf.h"
#include "renderer/modeling/bsdf/bsdfsample.h"
#include "renderer/modeling/bsdf/bsdfwrapper.h"
#include "renderer/modeling/bsdf/fresnel.h"
#include "renderer/modeling/bsdf/microfacethelper.h"
#include "renderer/modeling/bsdf/specularhelper.h"
#include "renderer/utility/paramarray.h"

// appleseed.foundation headers.
#include "foundation/containers/dictionary.h"
#include "foundation/math/basis.h"
#include "foundation/math/dual.h"
#include "foundation/math/microfacet.h"
#include "foundation/math/vector.h"
#include "foundation/utility/api/specializedapiarrays.h"
#include "foundation/utility/makevector.h"

// Standard headers.
#include <algorithm>
#include <cmath>

// Forward declarations.
namespace foundation    { class IAbortSwitch; }
namespace renderer      { class Assembly; }
namespace renderer      { class Project; }

using namespace foundation;

namespace renderer
{

namespace
{
    //
    // Metal BRDF.
    //
    // References:
    //
    //   [1] Microfacet Models for Refraction through Rough Surfaces
    //       http://www.cs.cornell.edu/~srm/publications/EGSR07-btdf.pdf
    //
    //   [2] Revisiting Physically Based Shading at Imageworks
    //       http://blog.selfshadow.com/publications/s2017-shading-course/imageworks/s2017_pbs_imageworks_slides.pdf
    //
    //   [3] Practical multiple scattering compensation for microfacet models
    //       https://blog.selfshadow.com/publications/turquin/ms_comp_final.pdf
    //

    const char* Model = "metal_brdf";

    class MetalBRDFImpl
      : public BSDF
    {
      public:
        MetalBRDFImpl(
            const char*                 name,
            const ParamArray&           params)
          : BSDF(name, Reflective, ScatteringMode::Glossy | ScatteringMode::Specular, params)
        {
            m_inputs.declare("normal_reflectance", InputFormatSpectralReflectance);
            m_inputs.declare("edge_tint", InputFormatSpectralReflectance);
            m_inputs.declare("edge_tint_weight", InputFormatFloat, "1.0");
            m_inputs.declare("reflectance_multiplier", InputFormatFloat, "1.0");
            m_inputs.declare("roughness", InputFormatFloat, "0.15");
            m_inputs.declare("anisotropy", InputFormatFloat, "0.0");
            m_inputs.declare("energy_compensation", InputFormatFloat, "0.0");
        }

        void release() override
        {
            delete this;
        }

        const char* get_model() const override
        {
            return Model;
        }

        size_t compute_input_data_size() const override
        {
            return sizeof(InputValues);
        }

        void prepare_inputs(
            Arena&                      arena,
            const ShadingPoint&         shading_point,
            void*                       data) const override
        {
            InputValues* values = static_cast<InputValues*>(data);
            new (&values->m_precomputed) InputValues::Precomputed();

            values->m_roughness = std::max(values->m_roughness, shading_point.get_ray().m_min_roughness);

            if (values->m_edge_tint_weight != 0.0f)
            {
                fresnel_lazanyi_schlick_a(
                    values->m_precomputed.m_a,
                    values->m_normal_reflectance,
                    values->m_edge_tint,
                    values->m_edge_tint_weight);
            }
            else
                values->m_precomputed.m_a.set(0.0f);
        }

        void sample(
            SamplingContext&            sampling_context,
            const void*                 data,
            const bool                  adjoint,
            const bool                  cosine_mult,
            const LocalGeometry&        local_geometry,
            const Dual3f&               outgoing,
            const int                   modes,
            BSDFSample&                 sample) const override
        {
            const InputValues* values = static_cast<const InputValues*>(data);

            const FresnelConductorSchlickLazanyi f(
                values->m_normal_reflectance,
                values->m_precomputed.m_a,
                values->m_reflectance_multiplier);

            // If roughness is zero use reflection.
            if (values->m_roughness == 0.0f)
            {
                if (ScatteringMode::has_specular(modes))
                {
                    bool use_microfacet_normal_mapping = false;
                    if(use_microfacet_normal_mapping)
                    {
                        sample_microfacet_based_normal_mapping_specular(sampling_context, f, local_geometry, outgoing, sample);
                    }
                    else
                    {
                        SpecularBRDFHelper::sample(f, local_geometry, outgoing, sample);
                    }
                    sample.m_value.m_beauty = sample.m_value.m_glossy;
                }

                return;
            }

            if (ScatteringMode::has_glossy(modes))
            {
                float alpha_x, alpha_y;
                microfacet_alpha_from_roughness(
                    values->m_roughness,
                    values->m_anisotropy,
                    alpha_x,
                    alpha_y);

                MicrofacetBRDFHelper<GGXMDF>::sample(
                    sampling_context,
                    values->m_roughness,
                    alpha_x,
                    alpha_y,
                    f,
                    local_geometry,
                    outgoing,
                    sample);

                if (sample.get_mode() != ScatteringMode::None)
                {
                    apply_energy_compensation_factor(
                        values,
                        outgoing.get_value(),
                        local_geometry.m_shading_basis.get_normal(),
                        sample.m_value.m_glossy);

                    sample.m_min_roughness = values->m_roughness;
                }

                sample.m_value.m_beauty = sample.m_value.m_glossy;
            }
        }

        float evaluate(
            const void*                 data,
            const bool                  adjoint,
            const bool                  cosine_mult,
            const LocalGeometry&        local_geometry,
            const Vector3f&             outgoing,
            const Vector3f&             incoming,
            const int                   modes,
            DirectShadingComponents&    value) const override
        {
            if (!ScatteringMode::has_glossy(modes))
                return 0.0f;

            const InputValues* values = static_cast<const InputValues*>(data);

            float alpha_x, alpha_y;
            microfacet_alpha_from_roughness(
                values->m_roughness,
                values->m_anisotropy,
                alpha_x,
                alpha_y);

            const FresnelConductorSchlickLazanyi f(
                values->m_normal_reflectance,
                values->m_precomputed.m_a,
                values->m_reflectance_multiplier);

            bool use_microfacet_normal_mapping = true;
            float pdf = 0.0;
            if (use_microfacet_normal_mapping)
            {
                pdf = evaluate_microfacet_based_normal_mapping(
                    local_geometry,
                    outgoing,
                    incoming,
                    value,
                    alpha_x,
                    alpha_y,
                    f);
            } 
            else {
                pdf =
                    MicrofacetBRDFHelper<GGXMDF>::evaluate(
                        alpha_x,
                        alpha_y,
                        f,
                        local_geometry,
                        outgoing,
                        incoming,
                        value.m_glossy);
            }

            apply_energy_compensation_factor(
                values,
                outgoing,
                local_geometry.m_shading_basis.get_normal(),
                value.m_glossy);

            value.m_beauty = value.m_glossy;

            assert(pdf >= 0.0f);
            return pdf;
        }

        float evaluate_pdf(
            const void*                 data,
            const bool                  adjoint,
            const LocalGeometry&        local_geometry,
            const Vector3f&             outgoing,
            const Vector3f&             incoming,
            const int                   modes) const override
        {
            if (!ScatteringMode::has_glossy(modes))
                return 0.0f;

            const InputValues* values = static_cast<const InputValues*>(data);

            float alpha_x, alpha_y;
            microfacet_alpha_from_roughness(
                values->m_roughness,
                values->m_anisotropy,
                alpha_x,
                alpha_y);

            float pdf = 0.0;
            bool use_microfacet_normal_mapping = true;
            if (use_microfacet_normal_mapping)
            {
                pdf =
                    evaluate_pdf_microfacet_based_normal_mapping(
                        local_geometry,
                        outgoing,
                        incoming,
                        alpha_x,
                        alpha_y);
            }
            else
            {
                pdf =
                    MicrofacetBRDFHelper<GGXMDF>::pdf(
                        alpha_x,
                        alpha_y,
                        local_geometry,
                        outgoing,
                        incoming);
            }
            assert(pdf >= 0.0f);
            return pdf;
        }

      private:
        typedef MetalBRDFInputValues InputValues;

        static void apply_energy_compensation_factor(
            const InputValues*          values,
            const Vector3f&             outgoing,
            const Vector3f&             n,
            Spectrum&                   value)
        {
            if (values->m_energy_compensation != 0.0f)
            {
                const float Ess = get_directional_albedo(
                    std::abs(dot(outgoing, n)),
                    values->m_roughness);

                if (Ess == 0.0f)
                    return;

                Spectrum fms = values->m_normal_reflectance;
                fms *= values->m_energy_compensation * (1.0f - Ess) / Ess;
                fms += Spectrum(1.0f);
                value *= fms;
            }
        }

        // TODO:
        // Apply microfacet based normal mapping on pdf
        static void sample_microfacet_based_normal_mapping_specular(
            SamplingContext&                     sampling_context,
            const FresnelConductorSchlickLazanyi f, 
            const LocalGeometry&                 local_geometry, 
            const foundation::Dual3f&            outgoing, 
            BSDFSample&                          sample)
        {
            Vector3f& perturbed_normal = 
                const_cast<Vector3f&>(
                    reinterpret_cast<Vector3f&>(
                        local_geometry.m_shading_point->get_shading_normal()));

            // Test perturbed_normal for validity.
            if(cos_theta(perturbed_normal) <= 0 
                || std::abs(perturbed_normal.x) < 1e-6
                || std::abs(perturbed_normal.y) < 1e-6)
            {
                SpecularBRDFHelper::sample(f, local_geometry, outgoing, sample);
                return;
            }

            // calculate wi? See: specularhelper
            foundation::Vector3f incoming = foundation::reflect(outgoing.get_value(), perturbed_normal);
            BSDF::force_above_surface(incoming, local_geometry.m_geometric_normal);
            Vector3f tangent_world = wt(perturbed_normal);
            Basis3f geometric_basis = Basis3f(local_geometry.m_shading_point->get_geometric_normal());
            Basis3f shading_basis = Basis3f(local_geometry.m_shading_point->get_shading_normal());
            Spectrum value(1.0);
            // hitting wp?
            if (sampling_context.next2<float>() < lambda_p(perturbed_normal, incoming))
            {
                // Sample facet with perturbed normal
                SpecularBRDFHelper::sample(f, local_geometry, outgoing, sample);
                // sampling fail?
                if(foundation::is_zero(sample.m_value.m_glossy))
                {
                    return;
                }
                // sampling direction shadowed? 
                Vector3f outgoing_perturbed = shading_basis.transform_to_parent(outgoing.get_value());
                float G1_value =
                    G1(perturbed_normal, geometric_basis.transform_to_local((outgoing_perturbed)));
                if (sampling_context.next2<float>() > G1_value)
                {
                    // reflect on tangent facet: -outgoing?
                    Vector3f reflected = 
                        normalize(outgoing_perturbed + Vector3f(2.0) * dot(outgoing_perturbed, tangent_world) * tangent_world);
                    sample.m_value.m_glossy *=
                        G1(perturbed_normal, geometric_basis.transform_to_local(reflected));
                }
            }
            else
            {
                // One reflection on tangent facet. TODO: it should be used as incoming?
                Vector3f reflected = 
                    normalize(-incoming + Vector3f(2.0) * dot(incoming, tangent_world) * tangent_world);
                // Sample on wp.
                SpecularBRDFHelper::sample(f, local_geometry, outgoing, sample);
                // Sample fail
                if (foundation::is_zero(sample.m_value.m_glossy))
                {
                    return;
                }
                Vector3f outgoing_perturbed = shading_basis.transform_to_parent(outgoing.get_value());
                sample.m_value.m_glossy *= 
                    G1(perturbed_normal, geometric_basis.transform_to_local((outgoing_perturbed)));
            }
        }

        static float evaluate_microfacet_based_normal_mapping(
            const LocalGeometry&                 local_geometry,
            const Vector3f&                      outgoing,
            const Vector3f&                      incoming,
            DirectShadingComponents&             value,
            float                                alpha_x,
            float                                alpha_y,
            const FresnelConductorSchlickLazanyi f)
        {
            Spectrum final_value(0.0);
            float pdf = 0.0;
            Vector3f perturbed_normal = local_geometry.m_shading_point->get_shading_normal();

            // Test perturbed_normal for validity.
            if(cos_theta(perturbed_normal) <= 0 
                || std::abs(perturbed_normal.x) < 1e-6
                || std::abs(perturbed_normal.y) < 1e-6)
            {
                return MicrofacetBRDFHelper<GGXMDF>::evaluate(
                    alpha_x,
                    alpha_y,
                    f,
                    local_geometry,
                    outgoing,
                    incoming,
                    value.m_glossy);
            }

            //
            // i -> p -> o
            //

            // Calculate pdf and value using perturbed normal and wi, wo.
            Spectrum value_ipo(0.0);
            float pdf_ipo =
                MicrofacetBRDFHelper<GGXMDF>::evaluate(
                    alpha_x,
                    alpha_y,
                    f,
                    local_geometry,
                    outgoing,
                    incoming,
                    value_ipo);

            // Apply microfacet based mapping on value.
            Basis3f geometric_basis(local_geometry.m_shading_point->get_geometric_normal());
            Vector3f incoming_local = geometric_basis.transform_to_local(incoming);
            Vector3f outgoing_local = geometric_basis.transform_to_local(outgoing);
            final_value += value_ipo  
                        * lambda_p(perturbed_normal, incoming_local)
                        * G1(perturbed_normal, outgoing_local);
            
            // Apply microfacet based mapping on pdf.
            float probability_wp = lambda_p(perturbed_normal, incoming_local);
            if(probability_wp > 0.0)
            {
                pdf += pdf_ipo
                    * lambda_p(perturbed_normal, incoming_local)
                    * G1(perturbed_normal, outgoing_local);
            }

            //
            // i -> p -> t -> o
            //

            Vector3f tangent_world = wt(perturbed_normal);
            Vector3f outgoing_reflected = 
                normalize(outgoing - Vector3f(2.0) * dot(outgoing, tangent_world) * tangent_world);
            if(dot(outgoing, tangent_world) > 0)
            {
                // Calculate pdf and value using perturbed normal and wi, wor.
                Spectrum value_ipto(0.0);
                float pdf_ipto =
                    MicrofacetBRDFHelper<GGXMDF>::evaluate(
                        alpha_x,
                        alpha_y,
                        f,
                        local_geometry,
                        outgoing_reflected,
                        incoming,
                        value_ipto);

                // Apply microfacet based normal mapping on value.
                Vector3f outgoing_reflected_local = geometric_basis.transform_to_local(outgoing_reflected);
                final_value += value_ipto
                    * lambda_p(perturbed_normal, incoming_local)
                    * G1(perturbed_normal, outgoing_local)
                    * (1.0f - G1(perturbed_normal, outgoing_reflected_local));

                // Apply microfacet based normal mapping on pdf.
                if(dot(outgoing, tangent_world) > 1e-6)
                {
                    pdf += pdf_ipto
                        * lambda_p(perturbed_normal, incoming_local) 
                        * (1.0 - G1(perturbed_normal, outgoing_reflected_local));
                }
            }

            //
            // i -> t -> p -> o
            //

            if (dot(incoming, tangent_world) > 0)
            {
                Vector3f incoming_reflected
                    = normalize(incoming - Vector3f(2.0) * dot(incoming, tangent_world) * tangent_world);

                // Calculate pdf and value using perturbed normal and wi, wor.
                Spectrum value_itpo(0.0);
                float pdf_itpo =
                    MicrofacetBRDFHelper<GGXMDF>::evaluate(
                        alpha_x,
                        alpha_y,
                        f,
                        local_geometry,
                        outgoing,
                        incoming_reflected,
                        value_itpo);
                
                // Apply microfacet based normal mapping on value.
                Vector3f outgoing_reflected_local = geometric_basis.transform_to_local(outgoing_reflected);
                final_value += (1.0f - lambda_p(perturbed_normal, incoming_local))
                                * G1(perturbed_normal, outgoing_reflected_local)
                                * value_itpo;
                
                // Apply microfacet based normal mapping on pdf.
                if(lambda_p(perturbed_normal, incoming_local) < 1.0 && dot(incoming, tangent_world) > 1e-6)
                {
                    pdf += pdf_itpo      
                        * (1.0 - lambda_p(perturbed_normal, incoming_local)); 
                }            
            }
            value.m_glossy = final_value;
            return pdf;
        }

        static float evaluate_pdf_microfacet_based_normal_mapping(
            const LocalGeometry&                 local_geometry,
            const Vector3f&                      outgoing,
            const Vector3f&                      incoming,
            float                                alpha_x,
            float                                alpha_y)
        {
            float pdf = 0.0;
            Vector3f perturbed_normal = local_geometry.m_shading_point->get_shading_normal();

            // Test perturbed_normal for validity.
            if(cos_theta(perturbed_normal) <= 0 
                || std::abs(perturbed_normal.x) < 1e-6
                || std::abs(perturbed_normal.y) < 1e-6)
            {
                return MicrofacetBRDFHelper<GGXMDF>::pdf(
                        alpha_x,
                        alpha_y,
                        local_geometry,
                        outgoing,
                        incoming);
            }

            //
            // i -> p -> o
            //

            Basis3f geometric_basis(local_geometry.m_shading_point->get_geometric_normal());
            Vector3f incoming_local = geometric_basis.transform_to_local(incoming);
            Vector3f outgoing_local = geometric_basis.transform_to_local(outgoing);
            if (lambda_p(perturbed_normal, incoming_local) > 0.0)
            {
                const float pdf_ipo =
                    MicrofacetBRDFHelper<GGXMDF>::pdf(
                        alpha_x,
                        alpha_y,
                        local_geometry,
                        outgoing,
                        incoming);
                
                // Apply microfacet based normal mapping on pdf.
                pdf += pdf_ipo
                    * lambda_p(perturbed_normal, incoming_local)
                    * G1(perturbed_normal, outgoing_local);
            }

            //
            // i -> p -> t -> o
            //

            Vector3f tangent_world = wt(perturbed_normal);
            Vector3f outgoing_reflected = 
                normalize(outgoing - Vector3f(2.0) * dot(outgoing, tangent_world) * tangent_world);
            Vector3f outgoing_reflected_local =
                geometric_basis.transform_to_local(outgoing_reflected);
            if(dot(outgoing, tangent_world) > 1e-6)
            {
                const float pdf_ipto =
                    MicrofacetBRDFHelper<GGXMDF>::pdf(
                        alpha_x,
                        alpha_y,
                        local_geometry,
                        outgoing_reflected,
                        incoming);

                // Apply microfacet based normal mapping on pdf.
                pdf += pdf_ipto
                    * lambda_p(perturbed_normal, incoming_local)
                    * (1.0 - G1(perturbed_normal, outgoing_reflected_local));
            }

            //
            // i -> t -> p -> o
            //
            Vector3f incoming_reflected =
                normalize(incoming - Vector3f(2.0) * dot(incoming, tangent_world) * tangent_world);
            if(lambda_p(perturbed_normal, incoming_local) < 1.0 && dot(incoming, tangent_world) > 1e-6)
            {
                const float pdf_itpo =
                    MicrofacetBRDFHelper<GGXMDF>::pdf(
                        alpha_x,
                        alpha_y,
                        local_geometry,
                        outgoing,
                        incoming_reflected);
                
                // Apply microfacet based normal mapping on pdf.
                pdf += pdf_itpo
                    * (1.0 - lambda_p(perturbed_normal, incoming_local));
            }
            return pdf;
        }

        static Vector3f wt(Vector3f wp)
        {
            return normalize(Vector3f(-wp.x, -wp.y, 0.0));
        }

        static float sin_theta(Vector3f w)
        {
            float sin_theta2 = 1.0 - w.z * w.z;
            if (sin_theta2 <= 0.0)
                return 0.0;
            return std::sqrt(sin_theta2);
        }

        static float cos_theta(Vector3f w)
        {
            return w.z;
        }

        static float lambda_p(Vector3f wp, Vector3f wi)
        {
            float i_dot_p = dot(wp, wi);
            Vector3f wt = wt(wp);
            float t_dot_i = dot(wt, wi);
            return i_dot_p / (i_dot_p + t_dot_i * sin_theta(wp));
        }

        static float G1(Vector3f wp, Vector3f w)
        {
            float cos_theta_w = std::max(0.0f, cos_theta(w));
            float cos_theta_wp = std::max(0.0f, cos_theta(wp));
            float w_dot_wp = dot(w, wp);
            Vector3f wt = wt(wp);
            float w_dot_wt = dot(w, wt);
            return std::min(1.0,
                    cos_theta_w * cos_theta_wp / (w_dot_wp + w_dot_wt * sin_theta(wp)));
        }
    };

    typedef BSDFWrapper<MetalBRDFImpl> MetalBRDF;
}


//
// MetalBRDFFactory class implementation.
//

void MetalBRDFFactory::release()
{
    delete this;
}

const char* MetalBRDFFactory::get_model() const
{
    return Model;
}

Dictionary MetalBRDFFactory::get_model_metadata() const
{
    return
        Dictionary()
            .insert("name", Model)
            .insert("label", "Metal BRDF");
}

DictionaryArray MetalBRDFFactory::get_input_metadata() const
{
    DictionaryArray metadata;

    metadata.push_back(
        Dictionary()
            .insert("name", "normal_reflectance")
            .insert("label", "Normal Reflectance")
            .insert("type", "colormap")
            .insert("entity_types",
                Dictionary()
                    .insert("color", "Colors")
                    .insert("texture_instance", "Texture Instances"))
            .insert("use", "required")
            .insert("default", "0.92"));

    metadata.push_back(
        Dictionary()
            .insert("name", "edge_tint")
            .insert("label", "Edge Tint")
            .insert("type", "colormap")
            .insert("entity_types",
                Dictionary()
                    .insert("color", "Colors")
                    .insert("texture_instance", "Texture Instances"))
            .insert("use", "required")
            .insert("default", "0.98"));

    metadata.push_back(
        Dictionary()
            .insert("name", "edge_tint_weight")
            .insert("label", "Edge Tint Weight")
            .insert("type", "colormap")
            .insert("entity_types",
                Dictionary().insert("texture_instance", "Texture Instances"))
            .insert("use", "optional")
            .insert("min",
                Dictionary()
                    .insert("value", "0.0")
                    .insert("type", "hard"))
            .insert("max",
                Dictionary()
                    .insert("value", "1.0")
                    .insert("type", "hard"))
            .insert("default", "1.0"));

    metadata.push_back(
        Dictionary()
            .insert("name", "reflectance_multiplier")
            .insert("label", "Reflectance Multiplier")
            .insert("type", "colormap")
            .insert("entity_types",
                Dictionary().insert("texture_instance", "Texture Instances"))
            .insert("use", "optional")
            .insert("default", "1.0"));

    metadata.push_back(
        Dictionary()
            .insert("name", "roughness")
            .insert("label", "Roughness")
            .insert("type", "colormap")
            .insert("entity_types",
                Dictionary()
                    .insert("color", "Colors")
                    .insert("texture_instance", "Texture Instances"))
            .insert("use", "optional")
            .insert("min",
                Dictionary()
                    .insert("value", "0.0")
                    .insert("type", "hard"))
            .insert("max",
                Dictionary()
                    .insert("value", "1.0")
                    .insert("type", "hard"))
            .insert("default", "0.15"));

    metadata.push_back(
        Dictionary()
            .insert("name", "anisotropy")
            .insert("label", "Anisotropy")
            .insert("type", "colormap")
            .insert("entity_types",
                Dictionary()
                    .insert("color", "Colors")
                    .insert("texture_instance", "Texture Instances"))
            .insert("use", "optional")
            .insert("min",
                Dictionary()
                    .insert("value", "-1.0")
                    .insert("type", "hard"))
            .insert("max",
                Dictionary()
                    .insert("value", "1.0")
                    .insert("type", "hard"))
            .insert("default", "0.0"));

    metadata.push_back(
        Dictionary()
            .insert("name", "energy_compensation")
            .insert("label", "Energy Compensation")
            .insert("type", "numeric")
            .insert("min",
                Dictionary()
                    .insert("value", "0.0")
                    .insert("type", "hard"))
            .insert("max",
                Dictionary()
                    .insert("value", "1.0")
                    .insert("type", "hard"))
            .insert("use", "optional")
            .insert("default", "0.0"));

    return metadata;
}

auto_release_ptr<BSDF> MetalBRDFFactory::create(
    const char*         name,
    const ParamArray&   params) const
{
    return auto_release_ptr<BSDF>(new MetalBRDF(name, params));
}

}   // namespace renderer
