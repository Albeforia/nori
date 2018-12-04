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

#include <nori/scene.h>
#include <nori/bitmap.h>
#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/camera.h>
#include <nori/emitter.h>
#include <nori/device.h>

NORI_NAMESPACE_BEGIN

Scene::Scene(const PropertyList &) {
	m_accel = new Accel();
}

Scene::~Scene() {
	delete m_accel;
	delete m_sampler;
	delete m_camera;
	delete m_integrator;

	if (m_scene) {
		rtcReleaseScene(m_scene);
	}
}

bool Scene::rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const {
	RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	// ray initialization
	RTCRayHit rayhit;
	rayhit.ray.org_x = ray.o.x();
	rayhit.ray.org_y = ray.o.y();
	rayhit.ray.org_z = ray.o.z();
	rayhit.ray.dir_x = ray.d.x();
	rayhit.ray.dir_y = ray.d.y();
	rayhit.ray.dir_z = ray.d.z();
	rayhit.ray.tnear = ray.mint;
	rayhit.ray.tfar = ray.maxt;
	rayhit.ray.flags = 0;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

	// ray query
	if (shadowRay) {
		rtcOccluded1(m_scene, &context, &rayhit.ray);
		return rayhit.ray.tfar != ray.maxt;
	}
	else {
		rtcIntersect1(m_scene, &context, &rayhit);
	}

	auto &hit = rayhit.hit;
	if (hit.geomID != RTC_INVALID_GEOMETRY_ID) {
		its.t = rayhit.ray.tfar;

		// hit a mesh
		its.mesh = m_meshIDs.at(hit.geomID);
		const MatrixXf &V = its.mesh->getVertexPositions();
		const MatrixXf &N = its.mesh->getVertexNormals();
		const MatrixXf &UV = its.mesh->getVertexTexCoords();
		const MatrixXu &F = its.mesh->getIndices();

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

		return true;
	}

	return false;
}

void Scene::activate() {
	//m_accel->build();
	// use Embree instead

	if (!m_integrator)
		throw NoriException("No integrator was specified!");
	if (!m_camera)
		throw NoriException("No camera was specified!");

	if (!m_sampler) {
		/* Create a default (independent) sampler */
		m_sampler = static_cast<Sampler *>(
		  NoriObjectFactory::createInstance("independent", PropertyList()));
	}

	m_scene = rtcNewScene(EmbreeDevice::instance().device());
	auto sceneFlags = RTC_SCENE_FLAG_ROBUST;
	rtcSetSceneFlags(m_scene, sceneFlags);
	build();

	cout << endl;
	cout << "Configuration: " << toString() << endl;
	cout << endl;
}

void Scene::build() {
	for (auto mesh : m_meshes) {
		auto geom = rtcNewGeometry(EmbreeDevice::instance().device(), RTC_GEOMETRY_TYPE_TRIANGLE);
		rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0,
		                           RTC_FORMAT_FLOAT3, mesh->getVertexPositions().data(),
		                           0, sizeof(Eigen::Vector3f), mesh->getVertexCount());
		rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0,
		                           RTC_FORMAT_UINT3, mesh->getIndices().data(),
		                           0, sizeof(Eigen::Vector3i), mesh->getTriangleCount());
		rtcCommitGeometry(geom);
		auto geomID = rtcAttachGeometry(m_scene, geom);
		m_meshIDs[geomID] = mesh;
		rtcReleaseGeometry(geom);
	}

	rtcCommitScene(m_scene);
}

void Scene::addChild(NoriObject *obj) {
	switch (obj->getClassType()) {
	case EMesh: {
		Mesh *mesh = static_cast<Mesh *>(obj);
		m_accel->addMesh(mesh);
		m_meshes.push_back(mesh);
	} break;

	case EEmitter: {
		//Emitter *emitter = static_cast<Emitter *>(obj);
		/* TBD */
		throw NoriException("Scene::addChild(): You need to implement this for emitters");
	} break;

	case ESampler:
		if (m_sampler)
			throw NoriException("There can only be one sampler per scene!");
		m_sampler = static_cast<Sampler *>(obj);
		break;

	case ECamera:
		if (m_camera)
			throw NoriException("There can only be one camera per scene!");
		m_camera = static_cast<Camera *>(obj);
		break;

	case EIntegrator:
		if (m_integrator)
			throw NoriException("There can only be one integrator per scene!");
		m_integrator = static_cast<Integrator *>(obj);
		break;

	default:
		throw NoriException("Scene::addChild(<%s>) is not supported!",
		                    classTypeName(obj->getClassType()));
	}
}

std::string Scene::toString() const {
	std::string meshes;
	for (size_t i = 0; i < m_meshes.size(); ++i) {
		meshes += std::string("  ") + indent(m_meshes[i]->toString(), 2);
		if (i + 1 < m_meshes.size())
			meshes += ",";
		meshes += "\n";
	}

	return tfm::format(
	  "Scene[\n"
	  "  integrator = %s,\n"
	  "  sampler = %s\n"
	  "  camera = %s,\n"
	  "  meshes = {\n"
	  "  %s  }\n"
	  "]",
	  indent(m_integrator->toString()),
	  indent(m_sampler->toString()),
	  indent(m_camera->toString()),
	  indent(meshes, 2));
}

NORI_REGISTER_CLASS(Scene, "scene");
NORI_NAMESPACE_END
