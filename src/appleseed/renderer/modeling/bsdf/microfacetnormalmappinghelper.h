
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
#include "renderer/modeling/color/colorspace.h"

// appleseed.foundation headers.
#include "foundation/math/basis.h"
#include "foundation/math/dual.h"
#include "foundation/math/vector.h"

namespace renderer
{

// MicrofacetNormalMappingHelper class corrects the usage of normal maps for Monte Carlo Path Tracing.
// Based on Microfacet based normal mapping (Heitz et al.).
// https://blogs.unity3d.com/2017/10/02/microfacet-based-normal-mapping-for-robust-monte-carlo-path-tracing/
//
// Light: wi, i (paper, mitsuba). outgoing (appleseed)
// View: wo, o (paper, mitsuba). incoming (appleseed)

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

static float cdot(foundation::Vector3f a, foundation::Vector3f b)
{
	return std::max(foundation::dot(a, b), 0.0f);
}

// TODO: use only one lambda function with flag for wp or wt.
// Note: cdot(a,b) is used for safe calculation because it "checks" if vectors are dot(a,b)<0.
static float lambda_p(foundation::Vector3f wp, foundation::Vector3f wi, foundation::Vector3f wg)
{	
	float wi_dot_wp = cdot(wi, wp);
	foundation::Vector3f wt = normalize(foundation::Vector3f(-wp.x, -wp.y, 0.0f));
	float wi_dot_wt = cdot(wi, wt);
	float wp_dot_wg = cdot(wp, wg);

	float ap_wi = wi_dot_wp / wp_dot_wg;
	float at_wi = (wi_dot_wt * std::sqrt(1.0f - std::pow(wp_dot_wg, 2.0f))) / wp_dot_wg;

	return ap_wi / (ap_wi + at_wi);
}

static float lambda_t(foundation::Vector3f wp, foundation::Vector3f wi, foundation::Vector3f wg)
{
	foundation::Vector3f wt = normalize(foundation::Vector3f(-wp.x, -wp.y, 0.0f));
	
	float wi_dot_wp = cdot(wi, wp);
	float wi_dot_wt = cdot(wi, wt);
	float wp_dot_wg = cdot(wp, wg);

	float ap_wi = wi_dot_wp / wp_dot_wg;
	float at_wi = (wi_dot_wt * std::sqrt(1.0f - std::pow(wp_dot_wg, 2.0f))) / wp_dot_wg;

	return at_wi / (ap_wi + at_wi);
}

static float G1(foundation::Vector3f wp, foundation::Vector3f w, foundation::Vector3f wg, bool w_wp)
{
	if (dot(w, wg) <= 0.0f)
		return 0.0f;

	float H = 1.0f;
	foundation::Vector3f wt = normalize(foundation::Vector3f(-wp.x, -wp.y, 0.0f));
	
	if(w_wp)
	{
		H = heaviside(dot(w, wp));
	}
	else
	{
		H = heaviside(dot(w, wt));
	}
	
	float w_dot_wg = cdot(w, wg);
	float wp_dot_wg = cdot(wp, wg);
	float w_dot_wp = cdot(w, wp);
	float w_dot_wt = cdot(w, wt);

	float ap_w = w_dot_wp / wp_dot_wg;
	float at_w = (w_dot_wt * std::sqrt(1.0f - std::pow(wp_dot_wg, 2.0f))) / wp_dot_wg;

	return H * std::min(1.0f, w_dot_wg / (ap_w + at_w));
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
	// World space original shading normal.
	foundation::Vector3f original_shading_normal(local_geometry.m_shading_point->get_original_shading_normal()); 

	// Check outgoing for validity. Check required for masking and intersection probability.
	if (dot(outgoing.get_value(), original_shading_normal) <= 0.0f)
	{
		return;
	}

	// World space perturbed shading normal.
	foundation::Vector3f perturbed_shading_normal(local_geometry.m_shading_point->get_shading_normal());

	// TODO: dot(original_shading_normal, perturbed_shading_normal) <= 0.0f?
	// In this case perturbed is too similar to original and modification can not be used (tangent vector can not be constructed)
	if((dot(original_shading_normal, perturbed_shading_normal) > 1.0f - 1e-6)) // 10e-5
	{
		BSDFImpl::sample(
			sampling_context,
			data,
			adjoint,
			false,
			local_geometry,
			outgoing,
			modes,
			sample);

		/*sample.m_value.m_beauty.set(
			foundation::Color3f(0.0f, 0.18f, 0.88f), // blue
			g_std_lighting_conditions,
			Spectrum::Intent::Reflectance);*/
		
		return;
	}

	// World space tangent.
	foundation::Vector3f tangent = normalize(foundation::Vector3f(-perturbed_shading_normal.x, -perturbed_shading_normal.y, 0.0f));

	const float shadow_terminator_freq_mult = local_geometry.m_shading_point->get_object_instance().get_render_data().m_shadow_terminator_freq_mult;

	float final_pdf = 0.0f; 
	if (lambda_p(perturbed_shading_normal, outgoing.get_value(), original_shading_normal) > sampling_context.next2<float>())
	{
		// Case: hit perturbed facet.
		BSDFSample perturbed_facet_sample;
		float perturbed_facet_pdf = 0.0f;
		BSDFImpl::sample(
			sampling_context,
			data,
			adjoint,
			false,
			local_geometry,
			outgoing, // wi
			modes,
			perturbed_facet_sample); // fp(wi, wp) value

		perturbed_facet_sample.m_value *= 
			shift_cos_in_fast(mdot(perturbed_facet_sample.m_incoming.get_value(), perturbed_shading_normal), shadow_terminator_freq_mult); // cosine weight, non-adjoint <wo, wp>

		perturbed_facet_pdf = perturbed_facet_sample.get_probability(); // fp(wi, wp) pdf

		if(G1(perturbed_shading_normal, perturbed_facet_sample.m_incoming.get_value(), original_shading_normal, true) > sampling_context.next2<float>())
		{
			// Sampled incoming direction was reflected on tangent facet (see Fig 10, right).
			// Calculate reflected of incoming direction to find out the direction which intersects perturbed facet
			// and calculate shadowing for this case.
			foundation::Vector3f incoming_reflected = 
				normalize(perturbed_facet_sample.m_incoming.get_value() - 2.0f * dot(perturbed_facet_sample.m_incoming.get_value(), tangent) * tangent);
		
			perturbed_facet_sample.m_value *= 
				G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, true); // G1(wo, wp) * H(dot(wo, wp))
		
			perturbed_facet_pdf *=
				G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, true); // G1(wo, wp) * H(dot(wo, wp))
		}
		
		/* Color coding of case where perturbed facet is sampled.
		if(lambda_p(perturbed_shading_normal, outgoing.get_value(), original_shading_normal) > 0.0f && 
			G1(perturbed_shading_normal, perturbed_facet_sample.m_incoming.get_value(), original_shading_normal, true) >0.0f)
				sample.m_value.m_beauty.set(
					foundation::Color3f(0.1f, 0.8f, 0.0f), // green
					g_std_lighting_conditions,
					Spectrum::Intent::Reflectance);
		*/
		
		sample.m_incoming = perturbed_facet_sample.m_incoming;

		if (dot(sample.m_incoming.get_value(), original_shading_normal) <= 0.0f)
		{
			return;
		}

		sample.m_value = perturbed_facet_sample.m_value;
		final_pdf = perturbed_facet_pdf;

	} else 
	{
		// Case: hit tangent facet.
		// Tangent facet is specular. Sampling tangent facet is actually sampling the perturbed facet
		// with reflected outgoing direction. 
		// World space outgoing reflected.
		foundation::Vector3f outgoing_reflected = 
			normalize(outgoing.get_value() - 2.0f * dot(outgoing.get_value(), tangent) * tangent); // w'i

		BSDFSample tangent_facet_sample;
		float tangent_facet_pdf = 0.0f;
		BSDFImpl::sample(
			sampling_context,
			data,
			adjoint,
			false,
			local_geometry, 
			foundation::Dual3f(outgoing_reflected), // w'i
			modes,
			tangent_facet_sample); // fp(wo, wp). 

		tangent_facet_sample.m_value *= 
			shift_cos_in_fast(mdot(tangent_facet_sample.m_incoming.get_value(), perturbed_shading_normal), shadow_terminator_freq_mult) // cosine weight, non-adjoint <wo, wp>
			* G1(perturbed_shading_normal, tangent_facet_sample.m_incoming.get_value(), original_shading_normal, true); // G1(wo, wp) * H(dot(wo, wp))
			//* (1.0f - G1(perturbed_shading_normal, outgoing_reflected, original_shading_normal, false)); // 1-G1(w'i, wt) * H(dot(w'i, wt)) = 1
		
		tangent_facet_pdf = tangent_facet_sample.get_probability() // fp(w'i, wp)
			* G1(perturbed_shading_normal, tangent_facet_sample.m_incoming.get_value(), original_shading_normal, true); // G1(wo, wp) * H(dot(wo, wp))
			// * (1.0f - G1(perturbed_shading_normal, outgoing_reflected, original_shading_normal, false)); // 1-G1(w'i, wt) * H(dot(w'i, wt)) = 1
		
		/* Color code: sampling tangent facet.
		if(lambda_t(perturbed_shading_normal, outgoing.get_value(), original_shading_normal) > 0.0f && 
			G1(perturbed_shading_normal, tangent_facet_sample.m_incoming.get_value(), original_shading_normal, true) > 0.0f)
				sample.m_value.m_beauty.set(
					foundation::Color3f(0.8f, 0.8f, 0.1f), // yellow
					g_std_lighting_conditions,
					Spectrum::Intent::Reflectance);
		*/

		sample.m_incoming = tangent_facet_sample.m_incoming;

		if (dot(sample.m_incoming.get_value(), original_shading_normal) <= 0.0f)
		{
			return;
		}

		sample.m_value = tangent_facet_sample.m_value;
		final_pdf = tangent_facet_pdf;
	}

	// Final pdf.
	if(ScatteringMode::has_diffuse(modes))
	{
		sample.set_to_scattering(ScatteringMode::Diffuse, final_pdf);
	}
	if(ScatteringMode::has_glossy(modes))
	{
		sample.set_to_scattering(ScatteringMode::Glossy, final_pdf);
	}
	if(ScatteringMode::has_specular(modes))
	{
		sample.set_to_scattering(ScatteringMode::Specular, final_pdf);
	}
	if(ScatteringMode::has_volume(modes))
	{
		sample.set_to_scattering(ScatteringMode::Volume, final_pdf);
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
	// BSDF value and pdf.
	DirectShadingComponents final_value;
	float final_pdf = 0.0f;

	// World space original shading normal.
	foundation::Vector3f original_shading_normal(local_geometry.m_shading_point->get_original_shading_normal()); 

	// Check incoming and outgoing for validity. Check required for masking and intersection probability.
	if (dot(incoming, original_shading_normal) <= 0.0f || dot(outgoing, original_shading_normal) <= 0.0f)
	{
		return 0.0f;
	}

	// World space perturbed shading normal.
	foundation::Vector3f perturbed_shading_normal(local_geometry.m_shading_point->get_shading_normal());

	// TODO dot(original_shading_normal, perturbed_shading_normal) <= 0?
	// In this case perturbed is too similar to original and modification can not be used (tangent vector can not be constructed).
	if(dot(original_shading_normal, perturbed_shading_normal) > 1.0f - 1e-6)
	{
		float default_pdf = BSDFImpl::evaluate(
            data,
            adjoint,
            false,
            local_geometry,
            outgoing,
            incoming,
            modes,
            value);

		/*	
		value.m_beauty.set(
			foundation::Color3f(0.0f, 0.18f, 0.88f), // blue
			g_std_lighting_conditions,
			Spectrum::Intent::Reflectance);
		*/
		return default_pdf;
	}

	// World space tangent.
	foundation::Vector3f tangent = normalize(foundation::Vector3f(-perturbed_shading_normal.x, -perturbed_shading_normal.y, 0.0f));

	// Factor for cosine weight.
	const float shadow_terminator_freq_mult = local_geometry.m_shading_point->get_object_instance().get_render_data().m_shadow_terminator_freq_mult;

	// Case: i -> p -> o.
	DirectShadingComponents value_ipo;
	if (foundation::dot(incoming, perturbed_shading_normal) >= 0.0f) // cull check for non-adjoint, BSDF::reflective
	{
		float pdf_ipo = BSDFImpl::evaluate(
				data,
				adjoint,
				false,
				local_geometry,
				outgoing,
				incoming,
				modes,
				value_ipo); // fp(wi, wo, wp)


		value_ipo *= lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_p(wi)
			* shift_cos_in_fast(mdot(incoming, perturbed_shading_normal), shadow_terminator_freq_mult) // cosine weight, non-adjoint <wo, wp>
			* G1(perturbed_shading_normal, incoming, original_shading_normal, true); // G1(wo, wp)*H(dot(wo, wp))

		pdf_ipo *= lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_p(wi)
			* G1(perturbed_shading_normal, incoming, original_shading_normal, true); // G1(wo, wp)*H(dot(wo, wp))

		final_value += value_ipo;
		final_pdf += pdf_ipo;
	
		/*
		if(lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) > 0.0f && 
			G1(perturbed_shading_normal, incoming, original_shading_normal, true) >0.0f)
				final_value.m_beauty.set(
					foundation::Color3f(0.1f, 0.8f, 0.0f), // green
					g_std_lighting_conditions,
					Spectrum::Intent::Reflectance);
		*/
	}

	// Case: i -> p -> t -> o.
	// World space incoming direction reflected at tangent facet.	
	foundation::Vector3f incoming_reflected =
		normalize(incoming - 2.0f * dot(incoming, tangent) * tangent);

	DirectShadingComponents value_ipto;
	if (foundation::dot(incoming_reflected, perturbed_shading_normal) >= 0.0f) // cull check for non-adjoint, BSDF::reflective
	{
		float pdf_ipto = BSDFImpl::evaluate(
			data,
			adjoint,
			false,
			local_geometry,
			outgoing,
			incoming_reflected,
			modes,
			value_ipto); // fp(wi, w'o, wp)

		value_ipto *= lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_p(wi)
			* shift_cos_in_fast(mdot(incoming_reflected, perturbed_shading_normal), shadow_terminator_freq_mult) // cosine weight, non-adjoint <w'o, wp>
			* (1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, true)) // 1-G1(w'o,wp) * H(dot(w'o, wp))
			* G1(perturbed_shading_normal, incoming, original_shading_normal, false); //G1(wo, wt) * H(dot(wo, wt))

		pdf_ipto *= lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_p(wi)
			* (1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, true)) // 1-G1(w'o,wp) * H(dot(w'o, wp))
			* G1(perturbed_shading_normal, incoming, original_shading_normal, false); //G1(wo, wt) * H(dot(wo, wt))

		final_value += value_ipto;
		final_pdf += pdf_ipto;

		/*
		if(lambda_p(perturbed_shading_normal, outgoing, original_shading_normal) > 0.0f && 
			G1(perturbed_shading_normal, incoming, original_shading_normal, false) > 0.0f && 
			(1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, true)) > 0.0f)
				final_value.m_beauty.set(
					foundation::Color3f(0.8f, 0.16f, 0.24f), // cherry red
					g_std_lighting_conditions,
					Spectrum::Intent::Reflectance);
		*/
	}

	// Case: i -> t -> p -> o.
	// World space outgoing direction reflected at tangent facet.
	foundation::Vector3f outgoing_reflected = 
		normalize(outgoing - 2.0f * dot(outgoing, tangent) * tangent);

	DirectShadingComponents value_itpo;
	if (foundation::dot(incoming, perturbed_shading_normal) >= 0.0f) // cull check for non-adjoint, BSDF::reflective
	{
		float pdf_itpo = BSDFImpl::evaluate(
			data,
			adjoint,
			false,
			local_geometry,
			outgoing_reflected,
			incoming,
			modes,
			value_itpo); // fp(w'i, wo, wp)

		value_itpo *= lambda_t(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_t(wi)
			* shift_cos_in_fast(mdot(incoming, perturbed_shading_normal), shadow_terminator_freq_mult) // cosine weight, non-adjoint <wo, wp>
			* G1(perturbed_shading_normal, incoming, original_shading_normal, true); // G1(wo, wp) * H(dot(wo, wp))
			//* (1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, false)); // 1-G1(w'i, wt) * H(dot(w'i, wt)) = 1
		
		pdf_itpo *= lambda_t(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_t(wi)
			* G1(perturbed_shading_normal, incoming, original_shading_normal, true); // G1(wo, wp) * H(dot(wo, wp))
			//* (1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, false)); // 1-G1(w'i, wt) * H(dot(w'i, wt)) = 1

		final_value += value_itpo;
		final_pdf += pdf_itpo;
		/*
			if(lambda_t(perturbed_shading_normal, outgoing, original_shading_normal) > 0.0f && 
				G1(perturbed_shading_normal, incoming, original_shading_normal, true) > 0.0f)
					final_value.m_beauty.set(
						foundation::Color3f(0.8f, 0.8f, 0.1f), // yellow
						g_std_lighting_conditions,
						Spectrum::Intent::Reflectance);
		*/
	}
	/*
	final_value.m_beauty.set(
		foundation::Color3f(0.f, 0.f, 0.f), 
		g_std_lighting_conditions,
		Spectrum::Intent::Reflectance);
	*/
	value = final_value;
	
	return final_pdf;
}

template <typename BSDFImpl>
float MicrofacetNormalMappingHelper<BSDFImpl>::evaluate_pdf(
        const void*                 data,
        const bool                  adjoint,
        const LocalGeometry&        local_geometry,
        const foundation::Vector3f& outgoing, // wi, i - view
        const foundation::Vector3f& incoming, // wo, o - light
        const int                   modes) const
{
	// BSDF pdf.
	float final_pdf = 0.0f;

	// World space original shading normal.
	foundation::Vector3f original_shading_normal(local_geometry.m_shading_point->get_original_shading_normal());

	// Check incoming and outgoing for validity. Check required for masking and intersection probability.
	if (dot(incoming, original_shading_normal) <= 0.0f || dot(outgoing, original_shading_normal) <= 0.0f)
	{
		return 0.0f;
	}

	// World space perturbed shading normal.
	foundation::Vector3f perturbed_shading_normal(local_geometry.m_shading_point->get_shading_normal());

	// TODO: dot(original_shading_normal, perturbed_shading_normal) <= 0?
	// In this case perturbed is too similar to original and modification can not be used (tangent vector can not be constructed).
	if(dot(original_shading_normal, perturbed_shading_normal) > 1.0f - 1e-6)
	{
		return BSDFImpl::evaluate_pdf(
			data,
			adjoint,
			local_geometry,
			outgoing,
			incoming,
			modes);
	}

	// World space tangent.
	foundation::Vector3f tangent = normalize(foundation::Vector3f(-perturbed_shading_normal.x, -perturbed_shading_normal.y, 0.0f));

	// Case: i -> p -> o.
	if (foundation::dot(incoming, perturbed_shading_normal) >= 0.0f) // cull check for non-adjoint, BSDF::reflective
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
	}

	// Case: i -> p -> t -> o.
	// World space outgoing direction reflected at tangent facet.
	foundation::Vector3f incoming_reflected =
		normalize(incoming - 2.0f * dot(incoming, tangent) * tangent);

	if (foundation::dot(incoming_reflected, perturbed_shading_normal) >= 0.0f) // cull check for non-adjoint, BSDF::reflective
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
			* (1.0f - G1(perturbed_shading_normal, incoming_reflected, original_shading_normal, true)); // 1-G1(w'o, wp) * H(dot(w'o, wp)) = 1
	}

	// Case: i -> t -> p -> o.
	// World space incoming direction reflected at tangent facet.
	foundation::Vector3f outgoing_reflected = 
		normalize(outgoing - 2.0f * dot(outgoing, tangent) * tangent);

	if (foundation::dot(incoming, perturbed_shading_normal) >= 0.0f) // cull check for non-adjoint, BSDF::reflective
	{
		const float pdf_itpo =
			BSDFImpl::evaluate_pdf(
					data,
					adjoint,
					local_geometry,
					outgoing_reflected,
					incoming,
					modes); // fp(w'i, wo, wp)

		final_pdf += lambda_t(perturbed_shading_normal, outgoing, original_shading_normal) // lambda_t(wi)
			* pdf_itpo // fp(w'i, wo, wp)
			* G1(perturbed_shading_normal, incoming, original_shading_normal, true); // G1(wo, wp) * H(dot(wo, wp))
			// * (1.0f - G1(tangent, incoming_reflected, original_shading_normal, false)); // 1-G1(w'i, wt) * H(dot(w'i, wt)) = 1
	}

	return final_pdf;
}

} // namespace renderer
