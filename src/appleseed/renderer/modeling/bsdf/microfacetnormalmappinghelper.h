
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

static float pdot(foundation::Vector3f a, foundation::Vector3f b) {
	return std::max(0.0f, dot(a, b));
}

static float mdot(foundation::Vector3f a, foundation::Vector3f b)
{
	return std::min(std::abs(foundation::dot(a, b)), 1.0f);
}

static float sin_theta(float cos_theta)
{
	return std::sqrt(1.0f - cos_theta * cos_theta);
}

static float lambda_p(foundation::Vector3f wp, foundation::Vector3f wi, foundation::Vector3f wg)
{
	float i_dot_p = pdot(wp, wi);
	foundation::Vector3f tangent = normalize(foundation::Vector3f(-wp.x, -wp.y, 0.0f));
	float t_dot_i = pdot(tangent, wi);
	float cos_theta_wp = pdot(wg, wp); // cos(theta)
	float sin_theta_wp = sin_theta(cos_theta_wp);
	float lambda = i_dot_p / (i_dot_p + t_dot_i * sin_theta_wp);        
	return lambda;
}

static float G1(foundation::Vector3f wp, foundation::Vector3f w, foundation::Vector3f wg)
{
	float cos_theta_w = pdot(w, wg);
	float cos_theta_wp = pdot(wp, wg);
	float sin_theta_wp = sin_theta(cos_theta_wp);
	float w_dot_wp = pdot(w, wp);
	foundation::Vector3f tangent = normalize(foundation::Vector3f(-wp.x, -wp.y, 0.0f));
	float w_dot_wt = pdot(w, tangent);
	float G = std::min(1.0f, cos_theta_w * cos_theta_wp / (w_dot_wp + w_dot_wt * sin_theta_wp));
	return G;
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
	// TODO: check outgoing for validity.
	// ...

	// BSDF value.
	DirectShadingComponents final_sample_value;

	// World space original shading normal.
	foundation::Vector3f original_shading_normal(local_geometry.m_shading_point->get_original_shading_normal());

	// World space perturbed shading normal.
	foundation::Vector3f perturbed_shading_normal(local_geometry.m_shading_point->get_shading_normal());

	// World space tangent.
	foundation::Vector3f tangent = normalize(foundation::Vector3f(-perturbed_shading_normal.x, -perturbed_shading_normal.y, 0.0f));

	const float shadow_terminator_freq_mult = local_geometry.m_shading_point->get_object_instance().get_render_data().m_shadow_terminator_freq_mult;

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
		if(sampling_context.next2<float>() > G1(perturbed_shading_normal, sample.m_incoming.get_value(), original_shading_normal))
		{
			// World space incoming direction reflected on tangent facet.
			foundation::Vector3f incoming_reflected = 
                normalize(sample.m_incoming.get_value() -2.0f * dot(sample.m_incoming.get_value(), tangent) * tangent);

			final_sample_value *= G1(perturbed_shading_normal, incoming_reflected, original_shading_normal)
				* shift_cos_in_fast(mdot(incoming_reflected, perturbed_shading_normal), shadow_terminator_freq_mult);
		}
	}
	else
	{
		// View direction is intersecting tangent facet, reflect. i -> t -> p -> o.
		// TODO: check outgoing vector orientation.
		foundation::Vector3f outgoing_reflected = 
            normalize(outgoing.get_value() - 2.0f * dot(outgoing.get_value(), tangent) * tangent);

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
			* G1(perturbed_shading_normal, sample.m_incoming.get_value(), original_shading_normal);
	}

	// TODO: check validity of the sampled incoming.
	// ...

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
	// TODO: Check incoming and outgoing for validity.
	// ...

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
            ipo_value);

	ipo_value *= lambda_p(perturbed_shading_normal, outgoing, original_shading_normal)
    	* shift_cos_in_fast(mdot(incoming, perturbed_shading_normal), shadow_terminator_freq_mult)
        * G1(perturbed_shading_normal, incoming, original_shading_normal); 
	
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
            value_ipto);

		value_ipto *= lambda_p(perturbed_shading_normal, outgoing, original_shading_normal)
			* shift_cos_in_fast(mdot(incoming_reflected, perturbed_shading_normal), shadow_terminator_freq_mult)
			* (1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal)) 
			* (G1(perturbed_shading_normal, incoming, original_shading_normal));

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
            value_itpo);

		// Apply microfacet based normal mapping on value.
		value_itpo *= (1.0f - lambda_p(perturbed_shading_normal, outgoing, original_shading_normal)) // lambda_t
			* shift_cos_in_fast(mdot(incoming, perturbed_shading_normal), shadow_terminator_freq_mult)
			* G1(perturbed_shading_normal, incoming, original_shading_normal);
			//* (1.0f - G1(perturbed_shading_normal, outgoing_reflected, original_shading_normal)); // should be 1.0f

		final_value += value_itpo;
	}
	value = final_value;

	return evaluate_pdf(
		data,
        adjoint,
        local_geometry,
        outgoing,
        incoming,
        modes);
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
	// TODO: check incoming and outgoing directions.
	// ...

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
            modes);

		final_pdf += lambda_p(perturbed_shading_normal, outgoing, original_shading_normal)
			* pdf_ipo
			* G1(perturbed_shading_normal, incoming, original_shading_normal);

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
					modes);

			final_pdf += lambda_p(perturbed_shading_normal, outgoing, original_shading_normal)
				* pdf_ipto
				* (G1(perturbed_shading_normal, incoming, original_shading_normal))
				* (1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal));
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
					modes);
		
		// Apply microfacet based normal mapping on pdf.
		final_pdf += (1.0f - lambda_p(perturbed_shading_normal, outgoing, original_shading_normal)) // lambda_t
			* pdf_itpo;
			//* G1(perturbed_shading_normal, incoming, original_shading_normal) // should be 1.0f
			//* (1.0f - G1(tangent, outgoing_reflected, original_shading_normal)); // should be 1.0f
	}
	return final_pdf;
}

} // namespace renderer
