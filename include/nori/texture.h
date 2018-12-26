#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

/*
@brief Base class of 2D textures
*/
template<typename T>
class Texture2D : public NoriObject {

public:
	Texture2D(const PropertyList& props) {
		m_uvOffset = Point2f(
		  props.getFloat("uoffset", 0.0f),
		  props.getFloat("voffset", 0.0f));
		m_uvScale = Vector2f(
		  props.getFloat("uscale", 1.0f),
		  props.getFloat("vscale", 1.0f));
	}

	virtual ~Texture2D() {}

	/**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
	EClassType getClassType() const { return ETexture; }

	/*
	@brief Texture lookup
	*/
	virtual T eval(const Point2f& uv) const = 0;

protected:
	Point2f m_uvOffset;
	Vector2f m_uvScale;
};

NORI_NAMESPACE_END
