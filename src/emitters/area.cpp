#include <nori/emitter.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

/*
@brief Area light source
*/
class AreaEmitter : public Emitter {

public:
	AreaEmitter(const PropertyList& props) :
	    Emitter(EEmitterType::EArea) {
		m_radiance = props.getColor("radiance", 1.0f);
	}

	Color3f eval(const ShapeSamplingResult& ss,
	             const Vector3f& v) const override {
		return ss.n.dot(v) > 0 ? m_radiance : 0.0f;
	}

	EmitterSamplingResult sample(const Intersection& ref,
	                             const Point2f& sample) const override {
		EmitterSamplingResult result;

		auto shapeSample = m_shape->sample(ref, sample);

		result.wi = (shapeSample.p - ref.p);
		float dist = result.wi.norm();
		result.distance = dist;
		result.wi /= dist;
		result.Le = ref.shFrame.n.dot(result.wi) > 0 ? eval(shapeSample, -result.wi) : 0.0f;
		result.pdf = m_shape->pdf(ref, shapeSample);

		return result;
	}

	void setParent(NoriObject* parent) {
		Emitter::setParent(parent);

		if (parent->getClassType() == EShape) {
			Shape* shape = static_cast<Shape*>(parent);
			if (m_shape == shape)
				return;

			if (m_shape != nullptr)
				throw NoriException("An area light cannot be attached to multiple shapes");

			m_shape = shape;
			//m_power = m_radiance * M_PI * m_shape->area();
		}
		else {
			throw NoriException("An area light can only be attached to a shape instance");
		}
	}

	std::string toString() const {
		return tfm::format(
		  "AreaEmitter[\n"
		  "  radiance = %s\n"
		  "]",
		  indent(m_radiance.toString()));
	}

private:
	Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area");
NORI_NAMESPACE_END
