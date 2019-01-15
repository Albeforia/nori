#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

Shape::Shape(const PropertyList &props) {
	m_transform = props.getTransform("toWorld", Transform{});
}

Shape::~Shape() {
	if (m_bsdf) delete m_bsdf;
	if (m_emitter) delete m_emitter;
}

void Shape::activate() {
	if (!m_bsdf) {
		/* If no material was assigned, instantiate a diffuse BRDF */
		m_bsdf = static_cast<BSDF *>(
		  NoriObjectFactory::createInstance("diffuse", PropertyList()));
		m_bsdf->addChild("albedo",
		                 NoriObjectFactory::createInstance("constexture", PropertyList{}));
	}
}

float Shape::pdf(const ShapeSamplingResult &) const {
	return 1 / area();
}

float Shape::pdf(const Intersection &ref, const ShapeSamplingResult &result) const {
	// ref: PBR3 5.5.3
	Vector3f v = ref.p - result.p;
	return v.squaredNorm() / (std::abs(result.n.dot(v)) * area());
}

void Shape::addChild(const std::string &name, NoriObject *obj) {
	switch (obj->getClassType()) {
	case EBSDF:
		if (m_bsdf)
			throw NoriException(
			  "Mesh: tried to register multiple BSDF instances!");
		m_bsdf = static_cast<BSDF *>(obj);
		break;

	case EEmitter: {
		if (m_emitter)
			throw NoriException(
			  "Mesh: tried to register multiple Emitter instances!");
		m_emitter = static_cast<Emitter *>(obj);
	} break;

	default:
		throw NoriException("Mesh::addChild(<%s>) is not supported!",
		                    classTypeName(obj->getClassType()));
	}
}

NORI_NAMESPACE_END
