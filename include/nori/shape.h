#pragma once

#include <nori/object.h>
#include <nori/bbox.h>
#include <nori/frame.h>
#include <embree3/rtcore_ray.h>

NORI_NAMESPACE_BEGIN

class Shape;

/**
 * \brief Intersection data structure
 *
 * This data structure records local information about a ray-triangle intersection.
 * This includes the position, traveled ray distance, uv coordinates, as well
 * as well as two local coordinate frames (one that corresponds to the true
 * geometry, and one that is used for shading computations).
 */
struct Intersection {
	/// Position of the surface intersection
	Point3f p;
	/// Unoccluded distance along the ray
	float t;
	/// UV coordinates, if any
	Point2f uv;
	/// Shading frame (based on the shading normal)
	Frame shFrame;
	/// Geometric frame (based on the true geometry)
	Frame geoFrame;
	/// Pointer to the associated mesh
	const Shape* shape;

	/// Create an uninitialized intersection record
	Intersection() :
	    shape(nullptr) {}

	/// Transform a direction vector into the local shading frame
	Vector3f toLocal(const Vector3f& d) const {
		return shFrame.toLocal(d);
	}

	/// Transform a direction vector from local to world coordinates
	Vector3f toWorld(const Vector3f& d) const {
		return shFrame.toWorld(d);
	}

	/// Return a human-readable summary of the intersection record
	std::string toString() const;
};

/**
@brief
*/
struct ShapeSamplingResult {
	/// Sampled surface position
	Point3f p;
	/// Sampled surface normal
	Normal3f n;
	/// Measure associated with the sample
	EMeasure measure;

	ShapeSamplingResult() = default;

	/// construct from an Intersection
	ShapeSamplingResult(const Intersection& its) :
	    p{its.p}, n{its.shFrame.n} {}
};

/**
@brief Base class of all shapes
*/
class Shape : public NoriObject {

public:
	Shape(const PropertyList& props);

	virtual ~Shape();

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

	/**
	@brief Get the surface area of this shape
	*/
	virtual float area() const = 0;

	/// Is this mesh an area emitter?
	bool isEmitter() const { return m_emitter != nullptr; }

	/// Return a pointer to an attached area emitter instance
	Emitter* getEmitter() { return m_emitter; }

	/// Return a pointer to an attached area emitter instance (const version)
	const Emitter* getEmitter() const { return m_emitter; }

	/// Return a pointer to the BSDF associated with this mesh
	const BSDF* getBSDF() const { return m_bsdf; }

	/// Register a child object (e.g. a BSDF) with the shape
	virtual void addChild(const std::string& name, NoriObject* child) override;

	/// Initialize internal data structures (called once by the XML parser)
	virtual void activate() override;

	/**
	@brief Ray-shape intersection test

	@param ray		The ray segment to be used for the intersection query
	@param t		The distance from the ray origin to the intersection point if success
	@param nomral	The surface normal at the intersection point
	@param uv		The uv coordinates of the intersection point

	@return			True if an intersection has been detected
	*/
	virtual bool rayIntersect(const Ray3f& ray, float& t,
	                          Normal3f& normal, Vector2f& uv) const { return false; }

	/**
	@brief Fill the intersection data when the closest hit has been found

	@param ray		The ray used for the intersection query
	@param t		The ray parameter for the hit point
	@param hit		The structure containing information about the hit point
	@param its		The intersection data to be filled
	*/
	virtual void setHitInformation(const Ray3f& ray, const float& t, const RTCHit& hit,
	                               Intersection& its) const {}

	/**
	@brief Sample a point on the surface with respect to surface area
	*/
	virtual ShapeSamplingResult sample(const Point2f& sample) const = 0;

	/**
	@brief Return the corresponding pdf with respect to surface area
	*/
	virtual float pdf(const ShapeSamplingResult&) const;

	/**
	@brief Sample a point on the surface with respect to 
	the solid angle subtended by the reference point
	*/
	virtual ShapeSamplingResult sample(const Intersection& ref,
	                                   const Point2f& sample) const = 0;

	/**
	@brief Return the corresponding pdf with respect to solid angles
	*/
	virtual float pdf(const Intersection& ref,
	                  const ShapeSamplingResult&) const;

protected:
	std::string m_name;

	Transform m_transform;

	BoundingBox3f m_bbox;

	BSDF* m_bsdf = nullptr;        ///< BSDF of the surface
	Emitter* m_emitter = nullptr;  ///< Associated emitter, if any
};

NORI_NAMESPACE_END
