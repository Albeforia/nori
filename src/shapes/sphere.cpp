#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/*
@brief Sphere centered at origin
*/
class Sphere : public Shape {

public:
	Sphere(const PropertyList& props) :
	    Shape(props) {
		m_radius = props.getFloat("radius", 1.0f);

		m_center = (m_transform.getMatrix().col(3)).head<3>();
		m_bbox.min = m_center + Vector3f(-m_radius);
		m_bbox.max = m_center + Vector3f(m_radius);
	}

	float area() const override {
		return 4 * M_PI * m_radius * m_radius;
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

		return true;
	}

	void setHitInformation(const Ray3f& ray, const float& t, const RTCHit& hit,
	                       Intersection& its) const override {
		its.t = t;
		its.p = ray(t);

		Vector3f hitdir = (its.p - m_center).normalized();

		// refine to be closer to the surface
		its.p = m_center + hitdir * m_radius;

		// find parametric representation of sphere hit
		auto localHit = m_transform.inverse() * Vector3f(its.p - m_center);
		if (localHit.x() == 0 && localHit.y() == 0) {
			localHit.x() = 1e-5f * m_radius;
		}
		auto phi = std::atan2(localHit.y(), localHit.x());
		if (phi < 0) phi += 2 * M_PI;
		auto theta = std::acos(clamp(localHit.z() / m_radius, -1.0f, 1.0f));
		its.uv.x() = phi * 0.5f * INV_PI;
		its.uv.y() = theta * INV_PI;

		its.geoFrame = Frame(hitdir);
		its.shFrame = its.geoFrame;
		its.shape = this;
	}

	ShapeSamplingResult sample(const Point2f& sample) const override {
		ShapeSamplingResult result;

		Vector3f v = Warp::squareToUniformSphere(sample);

		result.p = m_transform * Point3f(m_radius * v);
		result.n = (m_transform * Normal3f(v)).normalized();
		result.measure = EMeasure::EArea;

		return result;
	}

	ShapeSamplingResult sample(const Intersection& ref,
	                           const Point2f& sample) const override {
		Vector3f refToCenter = m_center - ref.p;
		float dc2 = refToCenter.squaredNorm();
		float radius2 = m_radius * m_radius;
		float sinThetaMax2 = radius2 / dc2;

		if (sinThetaMax2 > 1.0f) {  // ref.p is inside the sphere (m_radius > dc)
			// just sample the whole surface
			auto result = this->sample(sample);
			result.measure = ESolidAngle;
			return result;
		}
		else {
			float cosThetaMax = safe_sqrt(1 - sinThetaMax2);

			// sample a point on the extended cone resp. to ref.p
			float cosTheta = 1 - sample.x() * (1 - cosThetaMax);
			float sinTheta2 = 1 - cosTheta * cosTheta;
			float phi = 2.0f * M_PI * sample.y();

			// compute spherical coordinates for the actual point
			float dc = safe_sqrt(dc2);
			float ds = dc * cosTheta - safe_sqrt(radius2 - dc2 * sinTheta2);
			float cosAlpha = (dc2 + radius2 - ds * ds) / (2 * dc * m_radius);
			float sinAlpha = safe_sqrt(1 - cosAlpha * cosAlpha);
			// the point on the sphere and the point on the cone share the same phi

			// construct a coordinate frame at the sphere center with -refToCenter as z-axis
			// alpha and phi are local coordinates in fr
			Frame fr(-refToCenter / dc);
			Vector3f v = fr.toWorld(
			  Vector3f(sinAlpha * std::cos(phi), sinAlpha * std::sin(phi), cosAlpha));

			// fill the result
			ShapeSamplingResult result;
			result.p = m_transform * Point3f(m_radius * v);
			result.n = (m_transform * Normal3f(v)).normalized();
			result.measure = EMeasure::ESolidAngle;
			return result;
		}
	}

	float pdf(const Intersection& ref,
	          const ShapeSamplingResult& result) const override {
		Vector3f refToCenter = m_center - ref.p;
		float dc2 = refToCenter.squaredNorm();
		float radius2 = m_radius * m_radius;
		float sinThetaMax2 = radius2 / dc2;

		if (sinThetaMax2 > 1.0f) {
			// hint: uniformly sample a direction around the ref.p will lead to
			// unevenly distributed points on the sphere surface. Therefore the
			// pdf with respect to solid angles is not a constant
			return Shape::pdf(ref, result);
		}
		else {
			float cosThetaMax = safe_sqrt(1 - sinThetaMax2);

			// Warp::squareToUniformSphericalCapPdf() already
			// returns a value with respect to the solid angle measure
			return 1 / (2.0f * M_PI * (1 - cosThetaMax));
		}
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
	Point3f m_center;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
