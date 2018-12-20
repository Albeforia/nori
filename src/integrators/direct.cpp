#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

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

		auto& emitters = scene->getEmitters();

		// assuming only point lights here
		Color3f estimation = 0;
		for (auto emitter : emitters) {
			EmitterQueryRecord eRec;
			auto Ld = emitter->sample(eRec, its);

			auto bsdf = its.shape->getBSDF();
			BSDFQueryRecord bRec(its.toLocal(eRec.wi), its.toLocal(-ray.d), EMeasure::ESolidAngle);
			auto bsdfVal = bsdf->eval(bRec);

			// test visibility
			Ray3f shadowRay(its.p, eRec.wi, ray.mint, eRec.distance);
			if (scene->rayIntersect(shadowRay)) {
				Ld = Color3f(0.0f);
			}

			estimation += (Ld * bsdfVal * its.shFrame.n.dot(eRec.wi)) / eRec.pdf;
		}

		return estimation;
	}

	std::string toString() const {
		return "DirectIntegrator[]";
	}
};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END
