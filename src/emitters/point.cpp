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
		m_power = props.getColor("power", 15.0f);
	}

	virtual Color3f sample(EmitterQueryRecord& eRec, const Intersection& ref) const override {
		eRec.wi = (m_position - ref.p);
		float dist = eRec.wi.norm();
		eRec.wi /= dist;
		eRec.pdf = 1.0f;

		// Radiant intensity I = power / (4*pi)
		return m_power / (4 * M_PI * dist * dist);
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
