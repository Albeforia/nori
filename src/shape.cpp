#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

Shape::Shape(const PropertyList& props) {
	m_transform = props.getTransform("toWorld", Transform{});
}

NORI_NAMESPACE_END
