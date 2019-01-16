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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/*
ref: PBR3 13.6
*/

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
	return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
	return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
	throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
	throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
	// TODO concentric mapping

	float r = safe_sqrt(sample.x());
	float theta = 2.0f * M_PI * sample.y();
	return Point2f(r * std::cos(theta), r * std::sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
	//return INV_PI;
	return p.squaredNorm() <= 1.0f ? INV_PI : 0;
}

Point2f Warp::squareToUniformTriangle(const Point2f &sample) {
	float tmp = safe_sqrt(sample.x());
	return Point2f(1 - tmp, tmp * sample.y());
}

float Warp::squareToUniformTrianglePdf(const Point2f &p) {
	return p.x() + p.y() < 1 ? 2.0f : 0;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
	float cosTheta = 1.0f - 2.0f * sample.x();
	float sinTheta = safe_sqrt(1 - cosTheta * cosTheta);
	float phi = 2.0f * M_PI * sample.y();

	return Vector3f(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
	return INV_FOURPI;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
	float cosTheta = sample.x();
	float sinTheta = safe_sqrt(1 - cosTheta * cosTheta);
	float phi = 2.0f * M_PI * sample.y();

	return Vector3f(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
	//return INV_TWOPI;
	return v.z() > 0 ? INV_TWOPI : 0;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
	// Malley's Method
	Point2f p = squareToUniformDisk(sample);
	float z = safe_sqrt(1 - p.x() * p.x() - p.y() * p.y());
	return Vector3f(p.x(), p.y(), z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
	return v.z() > 0 ? v.z() * INV_PI : 0;
}

Vector3f Warp::squareToUniformSphericalCap(const Point2f &sample, float cosThetaMax) {
	float cosTheta = 1 - sample.x() * (1 - cosThetaMax);
	float sinTheta = safe_sqrt(1 - cosTheta * cosTheta);
	float phi = 2.0f * M_PI * sample.y();

	return Vector3f(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
}

float Warp::squareToUniformSphericalCapPdf(const Vector3f &v, float cosThetaMax) {
	return v.z() > cosThetaMax ? 1 / (2.0f * M_PI * (1 - cosThetaMax)) : 0;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
	float tanTheta2 = -alpha * alpha * std::log(1.0f - sample.x());
	float cosTheta = 1.0f / safe_sqrt(1.0f + tanTheta2);
	float sinTheta = safe_sqrt(1 - cosTheta * cosTheta);
	float phi = 2.0f * M_PI * sample.y();

	return Vector3f(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
	if (m.z() <= 0) return 0;

	float cosTheta2Inv = 1 / (m.z() * m.z());
	float alpha2Inv = 1 / (alpha * alpha);
	return INV_PI * alpha2Inv * cosTheta2Inv * cosTheta2Inv *
	       std::exp(alpha2Inv * (1 - cosTheta2Inv)) * m.z();
}

NORI_NAMESPACE_END
