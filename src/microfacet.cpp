/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
	Microfacet(const PropertyList &propList) {
		/* RMS surface roughness */
		m_alpha = propList.getFloat("alpha", 0.1f);

		/* Interior IOR (default: BK7 borosilicate optical glass) */
		m_intIOR = propList.getFloat("intIOR", 1.5046f);

		/* Exterior IOR (default: air) */
		m_extIOR = propList.getFloat("extIOR", 1.000277f);

		/* Albedo of the diffuse base material (a.k.a "kd") */
		m_kd = propList.getColor("kd", Color3f(0.5f));

		/* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
		m_ks = 1 - m_kd.maxCoeff();
	}

	/// Evaluate the BRDF for the given pair of directions
	Color3f eval(const BSDFQueryRecord &bRec) const {
		if (bRec.measure != ESolidAngle ||
		    Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
			return Color3f(0.0f);

		Vector3f H = (bRec.wo + bRec.wi).normalized();  // half-vector

		// NDF
		float D = beckmann(H, m_alpha);

		// shadow-masking
		float G = smithG1(bRec.wo, H, m_alpha) * smithG1(bRec.wi, H, m_alpha);

		// Fresnel
		float F = fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);

		return INV_PI * m_kd +
		       m_ks * (F * G * D) / (4.0f * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo));
	}

	/// Evaluate the sampling density of \ref sample() wrt. solid angles
	float pdf(const BSDFQueryRecord &bRec) const {
		if (bRec.measure != ESolidAngle ||
		    Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		Vector3f H = (bRec.wo + bRec.wi).normalized();  // half-vector

		float pdf_s = m_ks * Warp::squareToBeckmannPdf(H, m_alpha) / (4.0f * H.dot(bRec.wo));
		float pdf_d = (1.0f - m_ks) * INV_PI * Frame::cosTheta(bRec.wo);

		return pdf_s + pdf_d;
	}

	/// Sample the BRDF
	Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
		if (Frame::cosTheta(bRec.wi) <= 0) return Color3f(0.0f);

		bRec.measure = ESolidAngle;
		bRec.eta = 1.0f;

		if (sample.x() <= m_ks) {  // sample the specular component
			// remap the sample to [0,1], ref: PBR3 p.833
			Point2f _sample(sample.x() / m_ks, sample.y());

			Vector3f h = Warp::squareToBeckmann(_sample, m_alpha);
			float pdf = Warp::squareToBeckmannPdf(h, m_alpha);

			if (pdf == 0) return Color3f(0.0f);

			bRec.wo = reflect(bRec.wi, h);
			if (Frame::cosTheta(bRec.wo) <= 0) return Color3f(0.0f);
			pdf *= 1 / (4.0f * h.dot(bRec.wo));  // the Jacobian of the half direction mapping

			// NDF
			float D = beckmann(h, m_alpha);

			// shadow-masking
			float G = smithG1(bRec.wo, h, m_alpha) * smithG1(bRec.wi, h, m_alpha);

			// Fresnel
			float F = fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);

			return m_ks * (F * G * D) / (4.0f * pdf * Frame::cosTheta(bRec.wi));
		}
		else {  // sample the diffuse component
			// remap the sample to [0,1]
			Point2f _sample((sample.x() - m_ks) / (1.0f - m_ks), sample.y());

			bRec.wo = Warp::squareToCosineHemisphere(_sample);

			// (INV_PI * m_kd) / pdf_d() * cos(theta) == m_kd
			return m_kd;
		}
	}

	bool isDiffuse() const {
		/* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
		return true;
	}

	std::string toString() const {
		return tfm::format(
		  "Microfacet[\n"
		  "  alpha = %f,\n"
		  "  intIOR = %f,\n"
		  "  extIOR = %f,\n"
		  "  kd = %s,\n"
		  "  ks = %f\n"
		  "]",
		  m_alpha,
		  m_intIOR,
		  m_extIOR,
		  m_kd.toString(),
		  m_ks);
	}

private:
	float m_alpha;
	float m_intIOR, m_extIOR;
	float m_ks;
	Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
