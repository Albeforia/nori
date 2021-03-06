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

#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh(const PropertyList &props) :
    Shape(props) {}

Mesh::~Mesh() {}

float Mesh::triangleArea(uint32_t index) const {
	uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

	const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

	return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

void Mesh::buildSamplingTable() {
	uint32_t triCount = getTriangleCount();

	if (triCount > 0) {
		m_areaPDF.reserve(triCount);
		for (uint32_t i = 0; i < triCount; i++) {
			m_areaPDF.append(triangleArea(i));
		}
		m_area = m_areaPDF.normalize();
	}
	else {
		m_area = 0.0f;
	}
}

bool Mesh::rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const {
	uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
	const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

	/* Find vectors for two edges sharing v[0] */
	Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

	/* Begin calculating determinant - also used to calculate U parameter */
	Vector3f pvec = ray.d.cross(edge2);

	/* If determinant is near zero, ray lies in plane of triangle */
	float det = edge1.dot(pvec);

	if (det > -1e-8f && det < 1e-8f)
		return false;
	float inv_det = 1.0f / det;

	/* Calculate distance from v[0] to ray origin */
	Vector3f tvec = ray.o - p0;

	/* Calculate U parameter and test bounds */
	u = tvec.dot(pvec) * inv_det;
	if (u < 0.0 || u > 1.0)
		return false;

	/* Prepare to test V parameter */
	Vector3f qvec = tvec.cross(edge1);

	/* Calculate V parameter and test bounds */
	v = ray.d.dot(qvec) * inv_det;
	if (v < 0.0 || u + v > 1.0)
		return false;

	/* Ray intersects triangle -> compute t */
	t = edge2.dot(qvec) * inv_det;

	return t >= ray.mint && t <= ray.maxt;
}

void Mesh::setHitInformation(const Ray3f &ray, const float &t, const RTCHit &hit,
                             Intersection &its) const {
	its.shape = this;
	its.t = t;

	const MatrixXf &V = m_V;
	const MatrixXf &N = m_N;
	const MatrixXf &UV = m_UV;
	const MatrixXu &F = m_F;

	// vertices of the triangle
	uint32_t idx0 = F(0, hit.primID),
	         idx1 = F(1, hit.primID),
	         idx2 = F(2, hit.primID);
	Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

	// compute the intersection position using barycentric coordinates
	// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/barycentric-coordinates
	auto hit_w = 1 - hit.u - hit.v;
	its.p = hit_w * p0 + hit.u * p1 + hit.v * p2;

	// compute proper texture coordinates
	if (UV.size() > 0) {
		its.uv = hit_w * UV.col(idx0) + hit.u * UV.col(idx1) + hit.v * UV.col(idx2);
	}
	else {
		its.uv = Point2f(hit.u, hit.v);
	}

	// compute the geometry frame
	its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());

	// compute the shading frame
	if (N.size() > 0) {
		/* Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

		its.shFrame = Frame(
		  (hit_w * N.col(idx0) +
		   hit.u * N.col(idx1) +
		   hit.v * N.col(idx2))
		    .normalized());
	}
	else {
		its.shFrame = its.geoFrame;
	}
}

BoundingBox3f Mesh::getBoundingBox(uint32_t index) const {
	BoundingBox3f result(m_V.col(m_F(0, index)));
	result.expandBy(m_V.col(m_F(1, index)));
	result.expandBy(m_V.col(m_F(2, index)));
	return result;
}

Point3f Mesh::getCentroid(uint32_t index) const {
	return (1.0f / 3.0f) *
	       (m_V.col(m_F(0, index)) +
	        m_V.col(m_F(1, index)) +
	        m_V.col(m_F(2, index)));
}

ShapeSamplingResult Mesh::sample(const Point2f &sample) const {
	ShapeSamplingResult result;

	// choose a triangle according to its area
	Point2f _sample(sample);
	auto index = m_areaPDF.sampleReuse(_sample.y());

	// sample a point on the triangle
	result.p = sampleTriangle(index, _sample, result.n);
	result.measure = EMeasure::EArea;

	return result;
}

ShapeSamplingResult Mesh::sample(const Intersection &ref, const Point2f &sample) const {
	// just fallback to sample with area
	auto areaSample = this->sample(sample);
	areaSample.measure = EMeasure::ESolidAngle;
	return areaSample;
}

Point3f Mesh::sampleTriangle(uint32_t index, const Point2f &sample,
                             Normal3f &normal) const {
	uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
	const Point3f &p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

	Point2f bary = Warp::squareToUniformTriangle(sample);
	auto w = 1 - bary.x() - bary.y();
	Point3f p = w * p0 + bary.x() * p1 + bary.y() * p2;

	if (m_N.size() > 0) {
		normal = Normal3f((w * m_N.col(i0) + bary.x() * m_N.col(i1) + bary.y() * m_N.col(i2))
		                    .normalized());
	}
	else {
		normal = Normal3f((p1 - p0).cross(p2 - p0).normalized());
	}

	return p;
}

std::string Mesh::toString() const {
	return tfm::format(
	  "Mesh[\n"
	  "  name = \"%s\",\n"
	  "  vertexCount = %i,\n"
	  "  triangleCount = %i,\n"
	  "  hasUV = %s,\n"
	  "  hasNormal = %s,\n"
	  "  transform = %s,\n"
	  "  aabb = %s,\n"
	  "  bsdf = %s,\n"
	  "  emitter = %s\n"
	  "]",
	  m_name,
	  m_V.cols(),
	  m_F.cols(),
	  m_UV.size() > 0 ? "yes" : "no",
	  m_N.size() > 0 ? "yes" : "no",
	  indent(m_transform.toString()),
	  indent(m_bbox.toString()),
	  m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
	  m_emitter ? indent(m_emitter->toString()) : std::string("null"));
}

std::string Intersection::toString() const {
	if (!shape)
		return "Intersection[invalid]";

	return tfm::format(
	  "Intersection[\n"
	  "  p = %s,\n"
	  "  t = %f,\n"
	  "  uv = %s,\n"
	  "  shFrame = %s,\n"
	  "  geoFrame = %s,\n"
	  "  shape = %s\n"
	  "]",
	  p.toString(),
	  t,
	  uv.toString(),
	  indent(shFrame.toString()),
	  indent(geoFrame.toString()),
	  shape ? shape->toString() : std::string("null"));
}

NORI_NAMESPACE_END
