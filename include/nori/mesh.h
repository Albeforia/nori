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
#include <nori/dpdf.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Triangle mesh
 *
 * This class stores a triangle mesh object and provides numerous functions
 * for querying the individual triangles. Subclasses of \c Mesh implement
 * the specifics of how to create its contents (e.g. by loading from an
 * external file)
 */
class Mesh : public Shape {
public:
	Mesh(const PropertyList &props);

	/// Release all memory
	virtual ~Mesh();

	/// Return the total number of triangles in this shape
	uint32_t getTriangleCount() const { return (uint32_t)m_F.cols(); }

	/// Return the total number of vertices in this shape
	uint32_t getVertexCount() const { return (uint32_t)m_V.cols(); }

	/// Return the surface area of the given triangle
	float triangleArea(uint32_t index) const;

	float area() const override { return m_area; }

	//// Return an axis-aligned bounding box containing the given triangle
	BoundingBox3f getBoundingBox(uint32_t index) const;

	//// Return the centroid of the given triangle
	Point3f getCentroid(uint32_t index) const;

	/** \brief Ray-triangle intersection test
     *
     * Uses the algorithm by Moeller and Trumbore discussed at
     * <tt>http://www.acm.org/jgt/papers/MollerTrumbore97/code.html</tt>.
     *
     * Note that the test only applies to a single triangle in the mesh.
     * An acceleration data structure like \ref BVH is needed to search
     * for intersections against many triangles.
     *
     * \param index
     *    Index of the triangle that should be intersected
     * \param ray
     *    The ray segment to be used for the intersection query
     * \param t
     *    Upon success, \a t contains the distance from the ray origin to the
     *    intersection point,
     * \param u
     *   Upon success, \c u will contain the 'U' component of the intersection
     *   in barycentric coordinates
     * \param v
     *   Upon success, \c v will contain the 'V' component of the intersection
     *   in barycentric coordinates
     * \return
     *   \c true if an intersection has been detected
     */
	bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const;

	/// Return a pointer to the vertex positions
	const MatrixXf &getVertexPositions() const { return m_V; }

	/// Return a pointer to the vertex normals (or \c nullptr if there are none)
	const MatrixXf &getVertexNormals() const { return m_N; }

	/// Return a pointer to the texture coordinates (or \c nullptr if there are none)
	const MatrixXf &getVertexTexCoords() const { return m_UV; }

	/// Return a pointer to the triangle vertex index list
	const MatrixXu &getIndices() const { return m_F; }

	/// Return a human-readable summary of this instance
	std::string toString() const;

	/**
     * \brief Return the type of object (i.e. Mesh/BSDF/etc.)
     * provided by this instance
     * */
	//EClassType getClassType() const override { return EMesh; }

	void setHitInformation(const Ray3f &ray, const float &t, const RTCHit &hit,
	                       Intersection &its) const override;

	ShapeSamplingResult sample(const Point2f &sample) const override;

	ShapeSamplingResult sample(const Intersection &ref,
	                           const Point2f &sample) const override;

protected:
	/// Create an empty mesh
	//Mesh();

protected:
	MatrixXf m_V;   ///< Vertex positions
	MatrixXf m_N;   ///< Vertex normals
	MatrixXf m_UV;  ///< Vertex texture coordinates
	MatrixXu m_F;   ///< Faces

	void buildSamplingTable();

private:
	Point3f sampleTriangle(uint32_t index, const Point2f &sample,
	                       Normal3f &normal) const;

	float m_area;
	DiscretePDF m_areaPDF;
};

NORI_NAMESPACE_END
