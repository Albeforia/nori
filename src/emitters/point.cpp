#include <nori/emitter.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

/*
@brief Point light source which emits light uniformly in all directions
*/
class PointEmitter : public Emitter {

public:
	PointEmitter(const PropertyList& props) :
	    Emitter(EEmitterType::EDeltaPosition) {
		m_position = props.getPoint("position", Point3f{});
		m_power = props.getColor("power", 4 * M_PI);
	}

	/*
	For point lights, we could never hit or 'sample' a point on it
	*/
	Color3f eval(const ShapeSamplingResult& ss,
	             const Vector3f& v) const override {
		return Color3f(0.0f);
	}

	/*
	For point lights, we are actually sampling a delta distribution.
	The function always returns a single direction instead of using random samples.
	The delta distributions exist both in the Ld and pdf and they cancel out in the
	MC estimator of the LTE. Or we could consider the integral involving delta having
	analytic solution.
	Either way, the pdf is useless, so we set it to 1.
	*/
	EmitterSamplingResult sample(const Intersection& ref,
	                             const Point2f& sample) const override {
		EmitterSamplingResult result;

		result.wi = (m_position - ref.p);
		float dist = result.wi.norm();
		result.distance = dist;
		result.wi /= dist;
		result.pdf = 1.0f;

		// Radiant intensity I = power / (4*pi)
		result.Le = m_power / (4 * M_PI * dist * dist);

		return result;
	}

	std::string toString() const {
		return tfm::format(
		  "PointEmitter[\n"
		  "  position = %s,\n"
		  "  power = %s\n"
		  "]",
		  indent(m_position.toString()),
		  indent(m_power.toString()));
	}

private:
	Point3f m_position;
	Color3f m_power;  // Watts
};

NORI_REGISTER_CLASS(PointEmitter, "point");
NORI_NAMESPACE_END
