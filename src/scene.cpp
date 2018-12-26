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

	for (auto p : m_shapes) delete p;
	for (auto p : m_emitters) delete p;

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
		m_shapeIDs.at(hit.geomID).shape->setHitInformation(ray, rayhit.ray.tfar, rayhit.hit, its);
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
	for (auto shape : m_shapes) {
		if (auto mesh = dynamic_cast<Mesh *>(shape)) {
			auto geom = rtcNewGeometry(EmbreeDevice::instance().device(), RTC_GEOMETRY_TYPE_TRIANGLE);
			rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0,
			                           RTC_FORMAT_FLOAT3, mesh->getVertexPositions().data(),
			                           0, sizeof(Eigen::Vector3f), mesh->getVertexCount());
			rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0,
			                           RTC_FORMAT_UINT3, mesh->getIndices().data(),
			                           0, sizeof(Eigen::Vector3i), mesh->getTriangleCount());
			rtcCommitGeometry(geom);
			auto geomID = rtcAttachGeometry(m_scene, geom);
			m_shapeIDs[geomID] = ShapeData{shape, geomID};
			rtcReleaseGeometry(geom);
		}
		else {
			auto geom = rtcNewGeometry(EmbreeDevice::instance().device(), RTC_GEOMETRY_TYPE_USER);
			auto geomID = rtcAttachGeometry(m_scene, geom);
			m_shapeIDs[geomID] = ShapeData{shape, geomID};
			rtcSetGeometryUserPrimitiveCount(geom, 1);
			rtcSetGeometryUserData(geom, &(m_shapeIDs.at(geomID)));
			rtcSetGeometryBoundsFunction(geom,
			                             [](const RTCBoundsFunctionArguments *args) {
				                             auto shape = ((ShapeData *)args->geometryUserPtr)->shape;
				                             auto &aabb = shape->getBoundingBox();
				                             args->bounds_o->lower_x = aabb.min.x();
				                             args->bounds_o->lower_y = aabb.min.y();
				                             args->bounds_o->lower_z = aabb.min.z();
				                             args->bounds_o->upper_x = aabb.max.x();
				                             args->bounds_o->upper_y = aabb.max.y();
				                             args->bounds_o->upper_z = aabb.max.z();
			                             },
			                             nullptr);
			rtcSetGeometryIntersectFunction(geom,
			                                [](const RTCIntersectFunctionNArguments *args) {
				                                auto valid = args->valid;
				                                if (!valid[0]) return;

				                                auto shape = ((ShapeData *)args->geometryUserPtr)->shape;
				                                auto &rayhit = *(RTCRayHit *)(args->rayhit);

				                                Ray3f ray(Point3f(rayhit.ray.org_x, rayhit.ray.org_y, rayhit.ray.org_z),
				                                          Vector3f(rayhit.ray.dir_x, rayhit.ray.dir_y, rayhit.ray.dir_z),
				                                          rayhit.ray.tnear, rayhit.ray.tfar);

				                                float t;
				                                Normal3f normal;
				                                Vector2f uv;
				                                if (shape->rayIntersect(ray, t, normal, uv)) {
					                                rayhit.ray.tfar = t;

					                                rayhit.hit.u = uv.x();
					                                rayhit.hit.v = uv.y();
					                                rayhit.hit.Ng_x = normal.x();
					                                rayhit.hit.Ng_y = normal.y();
					                                rayhit.hit.Ng_z = normal.z();
					                                rayhit.hit.instID[0] = args->context->instID[0];
					                                rayhit.hit.geomID = ((ShapeData *)args->geometryUserPtr)->geomID;
					                                rayhit.hit.primID = args->primID;
				                                }
			                                });
			rtcSetGeometryOccludedFunction(geom,
			                               [](const RTCOccludedFunctionNArguments *args) {
				                               auto valid = args->valid;
				                               if (!valid[0]) return;

				                               auto shape = ((ShapeData *)args->geometryUserPtr)->shape;
				                               auto &rtcRay = *(RTCRay *)(args->ray);

				                               Ray3f ray(Point3f(rtcRay.org_x, rtcRay.org_y, rtcRay.org_z),
				                                         Vector3f(rtcRay.dir_x, rtcRay.dir_y, rtcRay.dir_z),
				                                         rtcRay.tnear, rtcRay.tfar);

				                               float t;
				                               Normal3f normal;
				                               Vector2f uv;
				                               if (shape->rayIntersect(ray, t, normal, uv)) {
					                               rtcRay.tfar = -std::numeric_limits<float>::infinity();
				                               }
			                               });
			rtcCommitGeometry(geom);
			rtcReleaseGeometry(geom);
		}
	}

	rtcCommitScene(m_scene);
}

void Scene::addChild(const std::string &name, NoriObject *obj) {
	switch (obj->getClassType()) {
	case EShape: {
		auto shape = static_cast<Shape *>(obj);
		//m_accel->addMesh(mesh);
		m_shapes.push_back(shape);
	} break;

	case EEmitter: {
		auto emitter = static_cast<Emitter *>(obj);
		m_emitters.push_back(emitter);
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
	std::string shapes;
	for (size_t i = 0; i < m_shapes.size(); ++i) {
		shapes += std::string("  ") + indent(m_shapes[i]->toString(), 2);
		if (i + 1 < m_shapes.size())
			shapes += ",";
		shapes += "\n";
	}

	std::string emitters;
	for (size_t i = 0; i < m_emitters.size(); ++i) {
		emitters += std::string("  ") + indent(m_emitters[i]->toString(), 2);
		if (i + 1 < m_emitters.size())
			emitters += ",";
		emitters += "\n";
	}

	return tfm::format(
	  "Scene[\n"
	  "  integrator = %s,\n"
	  "  sampler = %s\n"
	  "  camera = %s,\n"
	  "  shapes = {\n"
	  "  %s  },\n"
	  "  emitters = {\n"
	  "  %s  }\n"
	  "]",
	  indent(m_integrator->toString()),
	  indent(m_sampler->toString()),
	  indent(m_camera->toString()),
	  indent(shapes, 2),
	  indent(emitters, 2));
}

NORI_REGISTER_CLASS(Scene, "scene");
NORI_NAMESPACE_END
