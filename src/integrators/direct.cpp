#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/**
@brief Direct illumination integrator
*/
class DirectIntegrator : public Integrator {

public:
	/**
	@brief Sampling strategy
	*/
	enum EStrategy {
		EEmitter,
		EBSDF,
		EMIS
	};

	DirectIntegrator(const PropertyList& props) {
		auto strategyStr = props.getString("strategy", "mis");
		if (strategyStr == "emitter") {
			m_strategy = EEmitter;
		}
		else if (strategyStr == "bsdf") {
			m_strategy = EBSDF;
		}
		else if (strategyStr == "mis") {
			m_strategy = EMIS;
		}
		else {
			throw NoriException("DirectIntegrator: unknown sampling strategy!");
		}
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
		Intersection its;
		if (!scene->rayIntersect(ray, its)) {
			return Color3f(0.0f);
		}

		if (its.shape->isEmitter()) {
			return its.shape->getEmitter()->eval(its, -ray.d);
		}

		Color3f estimation = 0;

		if (m_strategy == EEmitter) {
			float weight;
			estimation += Li_emitter(scene, ray, its, sampler->next2D(), weight);
		}
		else if (m_strategy == EBSDF) {
			float weight;
			estimation += Li_bsdf(scene, ray, its, sampler->next2D(), weight);
		}
		else {
			// note we do not allocate samples for emitter and bsdf separately
			// we just rely on 'sample per pixel'

			float weight1 = 0.0f, weight2 = 0.0f;
			Color3f Li1 = Li_emitter(scene, ray, its, sampler->next2D(), weight1);
			Color3f Li2 = Li_bsdf(scene, ray, its, sampler->next2D(), weight2);
			estimation += weight1 * Li1 + weight2 * Li2;
		}

		return estimation;
	}

	Color3f Li_emitter(const Scene* scene, const Ray3f& ray,
	                   const Intersection& its, const Point2f& sample,
	                   float& weight) const {
		auto& emitterDistr = scene->getEmitterPDF();
		Point2f _sample(sample);

		// randomly pick an emitter
		float pickPdf;
		size_t index = emitterDistr.sampleReuse(_sample.x(), pickPdf);
		Emitter* emitter = scene->getEmitters()[index];

		// sample the emitter
		auto emitterSample = emitter->sample(its, _sample);
		emitterSample.pdf *= pickPdf;
		Color3f Ld = emitterSample.Le;
		if (Ld.isZero()) return Color3f(0.0f);

		// evaluate the bsdf
		auto bsdf = its.shape->getBSDF();
		BSDFQueryRecord bRec(its.toLocal(emitterSample.wi), its.toLocal(-ray.d),
		                     EMeasure::ESolidAngle, its.uv);
		Color3f bsdfVal = bsdf->eval(bRec);
		if (bsdfVal.isZero()) return Color3f(0.0f);

		// test visibility
		// '-Epsilon' to avoid hitting the emitter
		Ray3f shadowRay(its.p, emitterSample.wi, ray.mint, emitterSample.distance - Epsilon);
		if (scene->rayIntersect(shadowRay)) {
			return Color3f(0.0f);
		}

		if (emitter->isDelta()) {
			weight = 1.0f;  // no MIS for delta lights
		}
		else {
			float bsdfPdf = bsdf->pdf(bRec);
			weight = miWeight(emitterSample.pdf, bsdfPdf);
		}

		float cosThetai = clamp(its.shFrame.n.dot(emitterSample.wi), 0.0f, 1.0f);
		return (Ld * bsdfVal * cosThetai) / emitterSample.pdf;
	}

	Color3f Li_bsdf(const Scene* scene, const Ray3f& ray,
	                const Intersection& its, const Point2f& sample,
	                float& weight) const {
		auto bsdf = its.shape->getBSDF();
		BSDFQueryRecord bRec(its.toLocal(-ray.d), its.uv);

		// sample the bsdf
		Color3f bsdfVal = bsdf->sample(bRec, sample);
		if (bsdfVal.isZero()) return Color3f(0.0f);

		// find emitter along the reflected ray
		Vector3f wo = its.shFrame.toWorld(bRec.wo);
		Ray3f reflectedRay(its.p, wo, ray.mint, ray.maxt);
		Intersection its2;
		if (scene->rayIntersect(reflectedRay, its2)) {
			auto shape = its2.shape;
			if (shape->isEmitter()) {
				Color3f Ld = shape->getEmitter()->eval(its2, -reflectedRay.d);
				if (Ld.isZero()) return Color3f(0.0f);

				float emitterPdf = shape->pdf(its, its2) *
				                   scene->getEmitterPDF().getNormalization();

				// TODO delta BSDF
				float bsdfPdf = bsdf->pdf(bRec);

				weight = miWeight(bsdfPdf, emitterPdf);

				// note BSDF::sample() already returns eval() / pdf() * cos(theta)
				return Ld * bsdfVal;
			}
			else {
				return Color3f(0.0f);
			}
		}
		else {
			return Color3f(0.0f);
		}
	}

	float miWeight(float pdfA, float pdfB) const {
		return pdfA / (pdfA + pdfB);
	}

	std::string toString() const {
		std::string strategy;
		if (m_strategy == EEmitter) {
			strategy = "Emitter sampling";
		}
		else if (m_strategy == EBSDF) {
			strategy = "BSDF sampling";
		}
		else {
			strategy = "Multiple importance sampling";
		}

		return tfm::format(
		  "DirectIntegrator[\n"
		  "  strategy = %s\n"
		  "]",
		  strategy);
	}

private:
	EStrategy m_strategy;
};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END
