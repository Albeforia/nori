#pragma once

#include <nori/object.h>
#include <nori/bbox.h>

NORI_NAMESPACE_BEGIN

/**
@brief Base class of all shapes
*/
class Shape : public NoriObject {

public:
	Shape(const PropertyList& props);

	virtual ~Shape() {}

	/**
    @brief Return the type of object (i.e. Mesh/BSDF/etc.)
	provided by this instance
    */
	virtual EClassType getClassType() const override { return EShape; }

	/**
	@brief Get the name of this shape
	*/
	const std::string& getName() const { return m_name; }

	/**
	@brief Get the local transform of this shape
	*/
	const Transform& getTransform() const { return m_transform; }

	/**
	@brief Get the axis-aligned bounding box of this shape
	*/
	const BoundingBox3f& getBoundingBox() const { return m_bbox; }

protected:
	std::string m_name;

	Transform m_transform;

	BoundingBox3f m_bbox;
};

NORI_NAMESPACE_END
