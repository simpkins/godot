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
