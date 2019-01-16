/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

struct Intersection;

/*
@brief Convenience data structure for sampling a emitter
*/
struct EmitterSamplingResult {
	Color3f Le;
	Vector3f wi;
	float distance;
	float pdf;
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:
	/*
	@brief Type flags for Emitter
	*/
	enum EEmitterType {
		EDeltaPosition = 1,
		EDeltaDirection = 2,
		EArea = 4,
		EInfinite = 8
	};

	/*
	@brief Constructor
	*/
	Emitter(uint32_t flags) :
	    m_typeFlags{flags} {}

	/**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
	EClassType getClassType() const { return EEmitter; }

	/*
	@brief Check the type of the emitter
	*/
	bool isType(EEmitterType type) const { return (m_typeFlags & (int)type) != 0; }

	/**
	@brief Get the shape to which the emitter is currently attached
	*/
	Shape* getShape() { return m_shape; }

	/**
	@brief Get the shape to which the emitter is currently attached (const version)
	*/
	const Shape* getShape() const { return m_shape; }

	/**
	@brief Query the radiance

	@param ss	A sample or intersection point on this emitter
	@param v	An outgoing direction for the radiance

	@return The radiance emitted from the front side
	*/
	virtual Color3f eval(const ShapeSamplingResult& ss,
	                     const Vector3f& v) const = 0;

	/*
	@brief Sample the emitter.

	@param ref	A reference point on the surface 

	@return The radiance arriving at the reference point from the emitter,
	assuming there is no occlusion between the emitter and the point, and
	other information
	*/
	virtual EmitterSamplingResult sample(const Intersection& ref,
	                                     const Point2f& sample) const = 0;

protected:
	uint32_t m_typeFlags;

	Shape* m_shape = nullptr;
};

NORI_NAMESPACE_END
