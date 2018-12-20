#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

/*
@brief Sphere centered at origin
*/
class Sphere : public Shape {

public:
	Sphere(const PropertyList& props) :
	    Shape(props) {
		m_radius = props.getFloat("radius", 1.0f);

		Point3f center = (m_transform.getMatrix().col(3)).head<3>();
		m_bbox.min = center + Vector3f(-m_radius);
		m_bbox.max = center + Vector3f(m_radius);
	}

	bool rayIntersect(const Ray3f& ray, float& t,
	                  Normal3f& normal, Vector2f& uv) const override {
		// transform ray to object space
		Ray3f localRay = m_transform.inverse() * ray;

		// compute quadratic sphere coefficients
		float a = localRay.d.x() * localRay.d.x() + localRay.d.y() * localRay.d.y() +
		          localRay.d.z() * localRay.d.z();
		float b = 2 * (localRay.d.x() * localRay.o.x() + localRay.d.y() * localRay.o.y() +
		               localRay.d.z() * localRay.o.z());
		float c = localRay.o.x() * localRay.o.x() + localRay.o.y() * localRay.o.y() +
		          localRay.o.z() * localRay.o.z() - m_radius * m_radius;

		// solve quadratic equation for t values
		float t0, t1;
		if (!solveQuadratic(a, b, c, t0, t1)) {
			return false;
		}

		// check quadric shape t0 and t1 for nearest intersection
		if (t0 > localRay.maxt || t1 < localRay.mint) {
			return false;
		}
		if (t0 < localRay.mint) {
			if (localRay.maxt < t1) return false;
			t = t1;
		}
		else {
			t = t0;
		}

		auto localHit = localRay(t);
		auto dist = localHit.norm();
		localHit *= m_radius / dist;  // refine to be closer to the surface

		// find parametric representation of sphere hit
		if (localHit.x() == 0 && localHit.y() == 0) {
			localHit.x() = 1e-5f * m_radius;
		}
		auto phi = std::atan2(localHit.y(), localHit.x());
		if (phi < 0) phi += 2 * M_PI;
		auto theta = std::acos(clamp(localHit.z() / m_radius, -1.0f, 1.0f));
		uv.x() = phi / 2 * M_PI;
		uv.y() = theta / M_PI;

		Normal3f localNormal = localHit / dist;
		normal = m_transform * localNormal;

		return true;
	}

	void setHitInformation(const Ray3f& ray, const float& t, const RTCHit& hit,
	                       Intersection& its) const override {
		its.t = t;
		its.p = ray(t);
		its.uv.x() = hit.u;
		its.uv.y() = hit.v;
		its.geoFrame = Frame(Vector3f(hit.Ng_x, hit.Ng_y, hit.Ng_z));
		its.shFrame = its.geoFrame;
		its.shape = this;
	}

	std::string toString() const {
		return tfm::format(
		  "Sphere[\n"
		  "  radius = %f,\n"
		  "  transform = %s,\n"
		  "  aabb = %s,\n"
		  "  bsdf = %s,\n"
		  "  emitter = %s\n"
		  "]",
		  m_radius,
		  indent(m_transform.toString()),
		  indent(m_bbox.toString()),
		  m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
		  m_emitter ? indent(m_emitter->toString()) : std::string("null"));
	}

private:
	float m_radius;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
