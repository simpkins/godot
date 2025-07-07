/**************************************************************************/
/*  jolt_soft_body_3d_settings.cpp                                        */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "jolt_soft_body_3d_settings.h"

#include "../jolt_project_settings.h"

#include "scene/resources/3d/soft_body_3d_settings.h"

namespace {
JPH::Float3 to_float3(const Vector3 &v) {
	return JPH::Float3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}
} //namespace

void JoltSoftBody3DVolume::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_vertices", "vertices"), &JoltSoftBody3DVolume::set_vertices);
	ClassDB::bind_method(D_METHOD("get_vertices"), &JoltSoftBody3DVolume::get_vertices);
	ClassDB::bind_method(D_METHOD("set_six_rest_volume", "six_rest_volume"), &JoltSoftBody3DVolume::set_six_rest_volume);
	ClassDB::bind_method(D_METHOD("get_six_rest_volume"), &JoltSoftBody3DVolume::get_six_rest_volume);
	ClassDB::bind_method(D_METHOD("set_compliance", "compliance"), &JoltSoftBody3DVolume::set_compliance);
	ClassDB::bind_method(D_METHOD("get_compliance"), &JoltSoftBody3DVolume::get_compliance);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR4I, "vertices"), "set_vertices", "get_vertices");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "six_rest_volume"), "set_six_rest_volume", "get_six_rest_volume");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "compliance"), "set_compliance", "get_compliance");
}

void JoltSoftBody3DVolume::set_vertices(Vector4i p_vertices) {
	if (p_vertices != vertices) {
		vertices = p_vertices;
		emit_changed();
	}
}

Vector4i JoltSoftBody3DVolume::get_vertices() const {
	return vertices;
}

void JoltSoftBody3DVolume::set_six_rest_volume(float p_six_volume) {
	if (p_six_volume != six_rest_volume) {
		six_rest_volume = p_six_volume;
		emit_changed();
	}
}

float JoltSoftBody3DVolume::get_six_rest_volume() const {
	return six_rest_volume;
}

void JoltSoftBody3DVolume::set_compliance(float p_compliance) {
	if (p_compliance != compliance) {
		compliance = p_compliance;
		emit_changed();
	}
}

float JoltSoftBody3DVolume::get_compliance() const {
	return compliance;
}

JoltSoftBody3DVolume::JoltSoftBody3DVolume() = default;

JoltSoftBody3DVolume::~JoltSoftBody3DVolume() = default;

void JoltSoftBody3DSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_volumes", "volumes"), &JoltSoftBody3DSettings::set_volumes);
	ClassDB::bind_method(D_METHOD("get_volumes"), &JoltSoftBody3DSettings::get_volumes);

	ClassDB::bind_method(D_METHOD("calculate_volume_constraint_volumes", "multiplier"),
			&JoltSoftBody3DSettings::calculate_volume_constraint_volumes, DEFVAL(1.0));

	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "volumes", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("JoltSoftBody3DVolume")), "set_volumes", "get_volumes");
}

Array JoltSoftBody3DSettings::get_volumes() const {
	return volumes;
}

void JoltSoftBody3DSettings::set_volumes(const Array &p_volumes) {
	const int num_invalid_elements = _set_array_helper<JoltSoftBody3DVolume>(volumes, p_volumes);
	ERR_FAIL_COND_MSG(num_invalid_elements != 0, "JoltSoftBody3DSettings: invalid elements present in volume array");
}

JoltSoftBody3DSettings::JoltSoftBody3DSettings() = default;

JoltSoftBody3DSettings::~JoltSoftBody3DSettings() = default;

void JoltSoftBody3DSettings::calculate_volume_constraint_volumes(float multiplier) {
	const Array body_vertices = get_vertices();
	const int vertex_count = body_vertices.size();

	auto get_vertex = [&](int volume_idx, int32_t vidx, Vector3 &r_vertex) {
		ERR_FAIL_COND_V_MSG((vidx < 0 || vidx >= vertex_count), false,
				vformat("volume %d contains an invalid vertex index %d", volume_idx, vidx));
		const Variant &elem = body_vertices[vidx];
		const Ref<SoftBody3DVertex> &vertex = elem;
		ERR_FAIL_COND_V_MSG(vertex.is_valid(), false,
				vformat("volume %d refers to invalid vertex %d", volume_idx, vidx));
		r_vertex = vertex->get_position();
		return true;
	};

	const int volume_count = volumes.size();
	for (int volume_idx = 0; volume_idx < volume_count; ++volume_idx) {
		const Variant &elem = volumes[volume_idx];
		const Ref<JoltSoftBody3DVolume> &volume = elem;
		if (!volume.is_valid()) {
			continue;
		}

		Vector3 v0;
		Vector3 v1;
		Vector3 v2;
		Vector3 v3;
		const Vector4i &volume_verts = volume->get_vertices();
		if (!(get_vertex(volume_idx, volume_verts.x, v0) && get_vertex(volume_idx, volume_verts.y, v1) &&
					get_vertex(volume_idx, volume_verts.z, v2) && get_vertex(volume_idx, volume_verts.w, v3))) {
			continue;
		}

		Vector3 dv1 = v1 - v0;
		Vector3 dv2 = v2 - v0;
		Vector3 dv3 = v3 - v0;
		volume->set_six_rest_volume(dv1.cross(dv2).dot(dv3) * multiplier);
	}
}

JPH::Ref<JPH::SoftBodySharedSettings> JoltSoftBody3DSettings::convert_settings(const SoftBody3DSettings *p_settings) {
	ERR_FAIL_NULL_V(p_settings, nullptr);

	JPH::Ref<JPH::SoftBodySharedSettings> settings = new JPH::SoftBodySharedSettings();

	settings->mVertexRadius = JoltProjectSettings::soft_body_point_radius;

	Array settings_verts = p_settings->get_vertices();
	JPH::Array<JPH::SoftBodySharedSettings::Vertex> &physics_vertices = settings->mVertices;
	physics_vertices.reserve(settings_verts.size());
	for (const Variant &elem : settings_verts) {
		const Ref<SoftBody3DVertex> &vertex = elem;
		if (!vertex.is_valid()) {
			// Add a default vertex, so that we still have a 1:1 mapping between vertex indices.
			physics_vertices.emplace_back(JPH::Float3(0.0, 0.0, 0.0), JPH::Float3(0.0f, 0.0f, 0.0f), 1.0f);
		} else {
			physics_vertices.emplace_back(to_float3(vertex->get_position()), to_float3(vertex->get_velocity()), vertex->get_inverse_mass());
		}
	}

	Array settings_edges = p_settings->get_edges();
	JPH::Array<JPH::SoftBodySharedSettings::Edge> &physics_edges = settings->mEdgeConstraints;
	physics_edges.reserve(settings_edges.size());
	for (const Variant &elem : settings_edges) {
		const Ref<SoftBody3DEdge> &edge = elem;
		if (!edge.is_valid()) {
			continue;
		}

		const Vector2i edge_verts = edge->get_vertices();
		JPH::SoftBodySharedSettings::Edge &physics_edge = physics_edges.emplace_back(edge_verts.x, edge_verts.y, 0.1);
		physics_edge.mRestLength = edge->get_rest_length();
		physics_edge.mCompliance = edge->get_compliance();
	}

	Array settings_faces = p_settings->get_faces();
	JPH::Array<JPH::SoftBodySharedSettings::Face> &physics_faces = settings->mFaces;
	for (const Variant &elem : settings_faces) {
		const Ref<SoftBody3DFace> &face = elem;
		if (!face.is_valid()) {
			continue;
		}

		const Vector3i &face_verts = face->get_vertices();
		if (face_verts.x == face_verts.y || face_verts.x == face_verts.z || face_verts.y == face_verts.z) {
			continue; // We skip degenerate faces, since they're problematic, and Jolt will assert about it anyway.
		}

		// Jolt uses a different winding order, so we swap the indices to account for that.
		physics_faces.emplace_back(face_verts.z, face_verts.y, face_verts.x);
	}

	const JoltSoftBody3DSettings *p_jolt_settings = Object::cast_to<JoltSoftBody3DSettings>(p_settings);
	if (p_jolt_settings != nullptr) {
		Array settings_volumes = p_jolt_settings->get_volumes();
		JPH::Array<JPH::SoftBodySharedSettings::Volume> &physics_volumes = settings->mVolumeConstraints;
		for (const Variant &elem : settings_volumes) {
			const Ref<JoltSoftBody3DVolume> &volume = elem;
			if (!volume.is_valid()) {
				continue;
			}

			const Vector4i &volume_verts = volume->get_vertices();
			if (volume_verts.x == volume_verts.y || volume_verts.x == volume_verts.z || volume_verts.x == volume_verts.w ||
					volume_verts.y == volume_verts.z || volume_verts.y == volume_verts.w || volume_verts.z == volume_verts.w) {
				continue; // Skip degenerate volumes
			}

			physics_volumes.emplace_back(volume_verts.z, volume_verts.y, volume_verts.x, volume_verts.w, volume->get_compliance());
			physics_volumes.back().mSixRestVolume = volume->get_six_rest_volume();
		}
	}

	Ref<SoftBody3DAutoConstraintSettings> auto_constraint_settings = p_settings->get_auto_constraint_settings();
	if (auto_constraint_settings.is_valid()) {
		JPH::SoftBodySharedSettings::VertexAttributes vertex_attrib;
		vertex_attrib.mCompliance = vertex_attrib.mShearCompliance = auto_constraint_settings->get_compliance();

		settings->CreateConstraints(&vertex_attrib, 1, JPH::SoftBodySharedSettings::EBendType::None);
		const float multiplier = auto_constraint_settings->get_edge_length_multiplier();
		if (multiplier != 1.0) {
			for (JPH::SoftBodySharedSettings::Edge &e : settings->mEdgeConstraints) {
				e.mRestLength *= multiplier;
			}
		}
	}

	settings->Optimize();
	return settings;
}
