#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/**
@brief Integrator that visualizes the average visibility of surface points
*/
class AverageVisibility : public Integrator {

public:
	AverageVisibility(const PropertyList& props) {
		m_length = std::max(0.0f, props.getFloat("length"));
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
		Intersection its;
		if (!scene->rayIntersect(ray, its)) {
			return Color3f(1.0f);
		}

		// sample a new ray on the local hemisphere
		Vector3f dir = Warp::squareToUniformHemisphere(sampler->next2D());
		Ray3f newRay(its.p, its.shFrame.toWorld(dir), ray.mint, m_length);

		if (scene->rayIntersect(newRay)) {
			return Color3f(0.0f);
		}
		else {
			return Color3f(1.0f);
		}
	}

	std::string toString() const {
		return tfm::format(
		  "AverageVisibility[\n"
		  "  length = %f\n"
		  "]",
		  m_length);
	}

private:
	float m_length;
};

NORI_REGISTER_CLASS(AverageVisibility, "av");
NORI_NAMESPACE_END