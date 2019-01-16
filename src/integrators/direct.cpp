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
	DirectIntegrator(const PropertyList& props) {}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
		Intersection its;
		if (!scene->rayIntersect(ray, its)) {
			return Color3f(0.0f);
		}

		if (its.shape->isEmitter()) {
			return its.shape->getEmitter()->eval(its, -ray.d);
		}

		auto& emitters = scene->getEmitters();

		Color3f estimation = 0;
		for (auto emitter : emitters) {
			auto emitterSample = emitter->sample(its, sampler->next2D());
			Color3f Ld = emitterSample.Le;

			auto bsdf = its.shape->getBSDF();
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

		return estimation;
	}

	std::string toString() const {
		return "DirectIntegrator[]";
	}
};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END
