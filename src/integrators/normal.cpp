#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

/**
@brief Integrator that visualizes surface normals
*/
class NormalIntegrator : public Integrator {

public:
	NormalIntegrator(const PropertyList& props) {}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
		Intersection its;
		if (!scene->rayIntersect(ray, its)) {
			return Color3f(0.0f);
		}

		// return the component-wise absolute value of the shading normal as a color
		Normal3f n = its.shFrame.n.cwiseAbs();
		return Color3f(n.x(), n.y(), n.z());
	}

	std::string toString() const {
		return "NormalIntegrator[]";
	}
};

NORI_REGISTER_CLASS(NormalIntegrator, "normal");
NORI_NAMESPACE_END