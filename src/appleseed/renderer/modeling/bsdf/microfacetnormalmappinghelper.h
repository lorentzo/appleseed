
//
// This source file is part of appleseed.
// Visit https://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2010-2013 Francois Beaune, Jupiter Jazz Limited
// Copyright (c) 2014-2018 Francois Beaune, The appleseedhq Organization
// Copyright (c) 2014-2018 Esteban Tovagliari, The appleseedhq Organization
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

#pragma once

// appleseed.renderer headers.
#include "renderer/global/globaltypes.h"
#include "renderer/kernel/lighting/scatteringmode.h"
#include "renderer/kernel/shading/directshadingcomponents.h"
#include "renderer/kernel/shading/shadingpoint.h"
#include "renderer/modeling/bsdf/bsdfsample.h"
#include "renderer/utility/shadowterminator.h"

// appleseed.foundation headers.
#include "foundation/math/basis.h"
#include "foundation/math/dual.h"
#include "foundation/math/vector.h"

namespace renderer
{

// MicrofacetNormalMappingHelper class corrects the usage of normal maps for Monte Carlo Path Tracing.
// Based on Microfacet based normal mapping (Heitz et al.).
// https://blogs.unity3d.com/2017/10/02/microfacet-based-normal-mapping-for-robust-monte-carlo-path-tracing/
// Paper/Mitsuba wi -> outgoing Appleseed.
// Paper/Mitsuba wo -> incoming Appleseed.
//

static float heaviside(float a)
{
	if(a < 0.0f)
	{
		return 0.0f;
	}
	else
	{
		return 1.0f;
	}
}

static float mdot(foundation::Vector3f a, foundation::Vector3f b)
{
	return std::min(std::abs(foundation::dot(a, b)), 1.0f);
}

static float lambda_p(foundation::Vector3f wp, foundation::Vector3f wi, foundation::Vector3f wg)
{
	if(dot(wi, wg) <= 0.0f)
		return 0.0f;

	float wi_dot_wp = dot(wi, wp);
	foundation::Vector3f wt = normalize(foundation::Vector3f(-wp.x, -wp.y, 0.0f));
	float wi_dot_wt = dot(wi, wt);
	float wp_dot_wg = dot(wp, wg);

	return wi_dot_wp / (wi_dot_wp + wi_dot_wt * std::sqrt(1.0f - std::pow(wp_dot_wg, 2.0f)));
}

static float lambda_t(foundation::Vector3f wp, foundation::Vector3f wi, foundation::Vector3f wg)
{
	if(dot(wi, wg) <= 0.0f)
		return 0.0f;
	
	float wi_dot_wp = dot(wi, wp);
	float wp_dot_wg = dot(wp, wg);
	foundation::Vector3f wt = normalize(foundation::Vector3f(-wp.x, -wp.y, 0.0f));
	float wi_dot_wt = dot(wi, wt);

	return (wi_dot_wt * std::sqrt(1.0f - std::pow(wp_dot_wg, 2.0f))) / (wi_dot_wp + wi_dot_wt * std::sqrt(1.0f - std::pow(wp_dot_wg, 2.0f)));
}

static float G1(foundation::Vector3f wp, foundation::Vector3f w, foundation::Vector3f wg, bool w_wp)
{
	if (dot(w, wg) <= 0.0f)
		return 0.0f;

	float H = 0.0f;
	foundation::Vector3f wt = normalize(foundation::Vector3f(-wp.x, -wp.y, 0.0f));

	if(w_wp)
	{
		H = heaviside(dot(w, wp));
	}
	else
	{
		H = heaviside(dot(w, wt));
	}

	float w_dot_wg = dot(w, wg);
	float wp_dot_wg = dot(wp, wg);
	float w_dot_wp = dot(w, wp);
	float w_dot_wt = dot(w, wt);

	return H * std::min(1.0f, (w_dot_wg * wp_dot_wg) / (w_dot_wp + w_dot_wt * std::sqrt(1.0f - std::pow(wp_dot_wg, 2.0f))));
}

template <typename BSDFImpl>
class MicrofacetNormalMappingHelper : public BSDFImpl
{
  public:
    using LocalGeometry = typename BSDFImpl::LocalGeometry;

	MicrofacetNormalMappingHelper() {}

    MicrofacetNormalMappingHelper(
    	const char*                     name,
      	const ParamArray&               params);

    void sample(
		SamplingContext&            sampling_context,
		const void*                 data,                       // input values
		const bool                  adjoint,                    // if true, use the adjoint scattering kernel
		const bool                  cosine_mult,                // if true, multiply by |cos(incoming, normal)|
		const LocalGeometry&        local_geometry,
		const foundation::Dual3f&   outgoing,                   // world space outgoing direction, unit-length
		const int                   modes,                      // allowed scattering modes
		BSDFSample&                 sample) const;

	float evaluate(
        const void*                 data,                       // input values
        const bool                  adjoint,                    // if true, use the adjoint scattering kernel
        const bool                  cosine_mult,                // if true, multiply by |cos(incoming, normal)|
        const LocalGeometry&        local_geometry,
        const foundation::Vector3f& outgoing,                   // world space outgoing direction, unit-length
        const foundation::Vector3f& incoming,                   // world space incoming direction, unit-length
        const int                   modes,                      // enabled scattering modes
        DirectShadingComponents&    value) const;               // BSDF value, or BSDF value * |cos(incoming, normal)|

	float evaluate_pdf(
        const void*                 data,                       // input values
        const bool                  adjoint,                    // if true, use the adjoint scattering kernel
        const LocalGeometry&        local_geometry,
        const foundation::Vector3f& outgoing,                   // world space outgoing direction, unit-length
        const foundation::Vector3f& incoming,                   // world space incoming direction, unit-length
        const int                   modes) const;           	// enabled scattering modes
};


//
// MicrofacetNormalMappingHelper class implementation.
//

template <typename BSDFImpl>
MicrofacetNormalMappingHelper<BSDFImpl>::MicrofacetNormalMappingHelper(
    const char*                     name,
    const ParamArray&               params)
  : BSDFImpl(name, params)
{
}

template <typename BSDFImpl>
void MicrofacetNormalMappingHelper<BSDFImpl>::sample(
	SamplingContext&                    sampling_context,
	const void*                         data,
	const bool                          adjoint,
	const bool                          cosine_mult,
	const LocalGeometry&                local_geometry,
	const foundation::Dual3f&           outgoing,
	const int                           modes,
	BSDFSample&                         sample) const
{
	// BSDF value.
	DirectShadingComponents final_sample_value;

	// World space original shading normal.
	foundation::Vector3f original_shading_normal(local_geometry.m_shading_point->get_original_shading_normal());

	// World space perturbed shading normal.
	foundation::Vector3f perturbed_shading_normal(local_geometry.m_shading_point->get_shading_normal());

	// World space tangent.
	foundation::Vector3f tangent = normalize(foundation::Vector3f(-perturbed_shading_normal.x, -perturbed_shading_normal.y, 0.0f));

	const float shadow_terminator_freq_mult = local_geometry.m_shading_point->get_object_instance().get_render_data().m_shadow_terminator_freq_mult;

	// check outgoing for validity. Intersection probability check.
	if (dot(outgoing.get_value(), original_shading_normal) <= 0.0f)
		return;

	// TODO: check perturbed shading normal.
	// ...

	// View direction is intersecting perturbed facet, sample its BRDF.
	if(sampling_context.next2<float>() < lambda_p(perturbed_shading_normal, outgoing.get_value(), original_shading_normal))
	{
		// i -> p -> o
		BSDFImpl::sample(
            sampling_context,
            data,
            adjoint,
            false,
            local_geometry,
            outgoing,
            modes,
            sample);

		final_sample_value = sample.m_value; // TODO: if sample_value == 0?

		// Sampled light direction is masked by tangent facet: i -> p -> t -> o.
		if(sampling_context.next2<float>() > G1(perturbed_shading_normal, sample.m_incoming.get_value(), original_shading_normal, true)) // TODO: G1(wo, wt) vs G1(wo, wp)
		{
			// World space incoming direction reflected on tangent facet.
			foundation::Vector3f incoming_reflected = 
                normalize(sample.m_incoming.get_value() -2.0f * dot(sample.m_incoming.get_value(), tangent) * tangent);

			final_sample_value *= G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, false) // TODO G1(w'o, wp) vs G1(w'o, wt)
				* shift_cos_in_fast(mdot(incoming_reflected, perturbed_shading_normal), shadow_terminator_freq_mult);
		}
	}
	else
	{
		// View direction is intersecting tangent facet, reflect. i -> t -> p -> o.
		// TODO: check outgoing vector orientation.
		foundation::Vector3f outgoing_reflected = 
            normalize(-outgoing.get_value() + 2.0f * dot(outgoing.get_value(), tangent) * tangent);

		// Sample perturbed facet BRDF.
		BSDFImpl::sample(
            sampling_context,
            data,
            adjoint,
            false,
            local_geometry,
            foundation::Dual3f(outgoing_reflected),
            modes,
            sample);
		
		final_sample_value = sample.m_value; // TODO: if sample_value == 0?

		// Masking of the perturbed facet.
		final_sample_value *= shift_cos_in_fast(mdot(sample.m_incoming.get_value(), perturbed_shading_normal), shadow_terminator_freq_mult)
			* G1(perturbed_shading_normal, sample.m_incoming.get_value(), original_shading_normal, true); // TODO G1(wo, wp) vs G1(wo, wt)
	}

	// Check the sampled incoming.
	if(dot(sample.m_incoming.get_value(), original_shading_normal) <= 0.0f)
	{
		return;
	}

	sample.m_value = final_sample_value;

	float pdf = evaluate_pdf(
		data,
        adjoint,
        local_geometry,
        outgoing.get_value(),
        sample.m_incoming.get_value(),
        modes);

	if(ScatteringMode::has_diffuse(modes))
	{
		sample.set_to_scattering(ScatteringMode::Diffuse, pdf);
	}
	if(ScatteringMode::has_glossy(modes))
	{
		sample.set_to_scattering(ScatteringMode::Glossy, pdf);
	}
	if(ScatteringMode::has_specular(modes))
	{
		sample.set_to_scattering(ScatteringMode::Specular, pdf);
	}
	if(ScatteringMode::has_volume(modes))
	{
		sample.set_to_scattering(ScatteringMode::Volume, pdf);
	}
}

template <typename BSDFImpl>
float MicrofacetNormalMappingHelper<BSDFImpl>::evaluate(
	const void*                 data,
	const bool                  adjoint,
	const bool                  cosine_mult,
	const LocalGeometry&        local_geometry,
	const foundation::Vector3f& outgoing,
	const foundation::Vector3f& incoming,
	const int                   modes,
	DirectShadingComponents&    value) const
{
	// BSDF value.
	DirectShadingComponents final_value;

	// World space original shading normal.
	foundation::Vector3f original_shading_normal(local_geometry.m_shading_point->get_original_shading_normal()); 

	// World space perturbed shading normal.
	foundation::Vector3f perturbed_shading_normal(local_geometry.m_shading_point->get_shading_normal());

	// World space tangent.
	foundation::Vector3f tangent = normalize(foundation::Vector3f(-perturbed_shading_normal.x, -perturbed_shading_normal.y, 0.0f));

	// World space outgoing direction reflected at tangent facet.
	foundation::Vector3f outgoing_reflected = 
		normalize(outgoing - 2.0f * dot(outgoing, tangent) * tangent);

	// World space incoming direction reflected at tangent facet.	
	foundation::Vector3f incoming_reflected =
		normalize(incoming - 2.0f * dot(incoming, tangent) * tangent);

	const float shadow_terminator_freq_mult = local_geometry.m_shading_point->get_object_instance().get_render_data().m_shadow_terminator_freq_mult;

	// Check incoming and outgoing for validity. Masking and intersection probability check.
	if (dot(incoming, original_shading_normal) <= 0.0f || dot(outgoing, original_shading_normal) <= 0.0f)
	{
		return 0.0f;
	}

	// TODO: Check perturbed shading normal.
	// ...

	// Case: i -> p -> o 
	DirectShadingComponents ipo_value;
	BSDFImpl::evaluate(
            data,
            adjoint,
            false,
            local_geometry,
            outgoing,
            incoming,
            modes,
            ipo_value); // fp(wi, wo, wp)

	ipo_value *= lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_p(wi)
    	* shift_cos_in_fast(mdot(incoming, perturbed_shading_normal), shadow_terminator_freq_mult) // <wo, wp>
        * G1(perturbed_shading_normal, incoming, original_shading_normal, true); // G1(wo, wp)*H(dot(wo, wp))
	
	final_value += ipo_value;

	// Case: i -> p -> t -> o
	if(dot(incoming, tangent) > 0.0f)
	{
		DirectShadingComponents value_ipto;
		BSDFImpl::evaluate(
            data,
            adjoint,
            false,
            local_geometry,
            outgoing,
            incoming_reflected,
            modes,
            value_ipto); // fp(wi, w'o, wp)

		value_ipto *= lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_p(wi)
			* shift_cos_in_fast(mdot(incoming_reflected, perturbed_shading_normal), shadow_terminator_freq_mult) // <w'o, wp>
			* (1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, true)) // 1-G1(w'o,wp) * H(dot(w'o, wp))
			* G1(perturbed_shading_normal, incoming, original_shading_normal, false); //G1(wo, wt) * H(dot(wo, wt))

		final_value += value_ipto;
	}

	// Case: i -> t -> p -> o
	if (dot(outgoing, tangent) > 0.0f)
	{
		DirectShadingComponents value_itpo;
		BSDFImpl::evaluate(
            data,
            adjoint,
            false,
            local_geometry,
            outgoing_reflected,
            incoming,
            modes,
            value_itpo); // fp(w'i, wo, wp)

		// Apply microfacet based normal mapping on value.
		value_itpo *= lambda_t(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_t(wi)
			* shift_cos_in_fast(mdot(incoming, perturbed_shading_normal), shadow_terminator_freq_mult) // <wo, wp>
			* G1(perturbed_shading_normal, incoming, original_shading_normal, true); // G1(wo, wp) * H(dot(wo, wp))
		    //* (1.0f - G1(perturbed_shading_normal, outgoing_reflected, original_shading_normal, false)); // 1-G1(w'i, wt) * H(dot(w'i, wt))

		final_value += value_itpo;
	}
	value = final_value;

	float pdf = evaluate_pdf(
		data,
        adjoint,
        local_geometry,
        outgoing,
        incoming,
        modes);

	return pdf;
}

template <typename BSDFImpl>
float MicrofacetNormalMappingHelper<BSDFImpl>::evaluate_pdf(
        const void*                 data,
        const bool                  adjoint,
        const LocalGeometry&        local_geometry,
        const foundation::Vector3f& outgoing,
        const foundation::Vector3f& incoming,
        const int                   modes) const
{
	// BSDF pdf.
	float final_pdf = 0.0f;

	// World space original shading normal.
	foundation::Vector3f original_shading_normal(local_geometry.m_shading_point->get_original_shading_normal());

	// World space perturbed shading normal.
	foundation::Vector3f perturbed_shading_normal(local_geometry.m_shading_point->get_shading_normal());

	// World space tangent.
	foundation::Vector3f tangent = normalize(foundation::Vector3f(-perturbed_shading_normal.x, -perturbed_shading_normal.y, 0.0f));

	// World space outgoing direction reflected at tangent facet.
	foundation::Vector3f outgoing_reflected = 
		normalize(outgoing - 2.0f * dot(outgoing, tangent) * tangent);

	// World space incoming direction reflected at tangent facet.
	foundation::Vector3f incoming_reflected =
		normalize(incoming - 2.0f * dot(incoming, tangent) * tangent);

	// Check incoming and outgoing for validity. Masking and intersection probability check.
	if (dot(incoming, original_shading_normal) <= 0.0f || dot(outgoing, original_shading_normal) <= 0.0f)
	{
		return 0.0f;
	}

	// TODO: check perturbed shading normal.
	// ...

	// Case: i -> p -> o
	if (lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) > 0.0f)
	{
		float pdf_ipo =
			BSDFImpl::evaluate_pdf(
            data,
            adjoint,
            local_geometry,
            outgoing,
            incoming,
            modes); // fp(wi, wo, wp)

		final_pdf += lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_p(wi)
			* pdf_ipo // fp(wi, wo, wp)
			* G1(perturbed_shading_normal, incoming, original_shading_normal, true); // G1(wo, wp) * H(dot(wo, wp))

		// Case: i -> p -> t -> o
		if(dot(incoming, tangent) > 1e-6)
		{
			float pdf_ipto =
				BSDFImpl::evaluate_pdf(
					data,
					adjoint,
					local_geometry,
					outgoing,
					incoming_reflected,
					modes); // fp(wi, w'o, wp)

			final_pdf += lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_p(wi)
				* pdf_ipto // fp(wi, w'o, wp)
				* G1(perturbed_shading_normal, incoming, original_shading_normal, false) // G1(wo, wt) * H(dot(wo, wt))
				* (1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, true)); // 1-G1(w'o, wp) * H(dot(w'o, wp))
		}
	}

	// Case: i -> t -> p -> o
	if(lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) < 1.0f && dot(outgoing, tangent) > 1e-6)
	{
		const float pdf_itpo =
			BSDFImpl::evaluate_pdf(
					data,
					adjoint,
					local_geometry,
					outgoing_reflected,
					incoming,
					modes); // fp(w'i, wo, wp)
		
		// Apply microfacet based normal mapping on pdf.
		final_pdf += lambda_t(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_t(wi)
			* pdf_itpo; // fp(w'i, wo, wp)
			// * G1(perturbed_shading_normal, incoming, original_shading_normal, true) // G1(wo, wp) * H(dot(wo, wp))
			// * (1.0f - G1(tangent, outgoing_reflected, original_shading_normal, false)); // 1-G1(w'i, wt) * H(dot(w'i, wt))
	}
	return final_pdf;
}

} // namespace renderer
