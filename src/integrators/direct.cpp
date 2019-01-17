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
		auto strategyStr = props.getString("strategy", "emitter");
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

		auto& emitters = scene->getEmitters();
		auto bsdf = its.shape->getBSDF();

		Color3f estimation = 0;

		if (m_strategy == EEmitter) {
			for (auto emitter : emitters) {
				auto emitterSample = emitter->sample(its, sampler->next2D());
				Color3f Ld = emitterSample.Le;

				BSDFQueryRecord bRec(its.toLocal(emitterSample.wi), its.toLocal(-ray.d),
				                     EMeasure::ESolidAngle, its.uv);
				auto bsdfVal = bsdf->eval(bRec);

				// test visibility
				// '-Epsilon' to avoid hitting the emitter
				Ray3f shadowRay(its.p, emitterSample.wi, ray.mint, emitterSample.distance - Epsilon);
				if (scene->rayIntersect(shadowRay)) {
					Ld = Color3f(0.0f);
				}

				float cosThetai = clamp(its.shFrame.n.dot(emitterSample.wi), 0.0f, 1.0f);
				estimation += (Ld * bsdfVal * cosThetai) / emitterSample.pdf;
			}
		}
		else if (m_strategy == EBSDF) {
			BSDFQueryRecord bRec(its.toLocal(-ray.d), its.uv);

			Color3f bsdfVal = bsdf->sample(bRec, sampler->next2D());
			Vector3f wo = its.shFrame.toWorld(bRec.wo);
			float bsdfPDF = bsdf->pdf(bRec);

			if (bsdfPDF > 0) {
				Ray3f reflectedRay(its.p, wo, ray.mint, ray.maxt);
				Intersection its2;
				Color3f Ld;
				if (scene->rayIntersect(reflectedRay, its2)) {
					auto shape = its2.shape;
					if (shape->isEmitter()) {
						Ld = shape->getEmitter()->eval(its2, -reflectedRay.d);
					}
					else {
						Ld = Color3f(0.0f);
					}
				}

				float cosThetai = clamp(its.shFrame.n.dot(wo), 0.0f, 1.0f);
				estimation += (Ld * bsdfVal * cosThetai) / bsdfPDF;
			}
		}
		else {
			// TODO
		}

		return estimation;
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
