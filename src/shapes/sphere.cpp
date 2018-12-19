#include <nori/shape.h>

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
		Normal3f localNormal = localHit.normalized();
		normal = m_transform * localNormal;

		return true;
	}

	void setHitInformation(const Ray3f& ray, const float& t, const RTCHit& hit,
	                       Intersection& its) const override {
		its.t = t;
		its.p = ray(t);
		// TODO calculate uv
		its.geoFrame = Frame(Vector3f(hit.Ng_x, hit.Ng_y, hit.Ng_z));
		its.shFrame = its.geoFrame;
		its.shape = this;
	}

	std::string toString() const {
		return tfm::format(
		  "Sphere[\n"
		  "  radius = %f,\n"
		  "  transform = %s,\n"
		  "  aabb = %s\n"
		  "]",
		  m_radius,
		  indent(m_transform.toString()),
		  indent(m_bbox.toString()));
	}

private:
	float m_radius;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
