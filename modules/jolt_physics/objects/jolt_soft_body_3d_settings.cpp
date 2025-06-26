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

#include "scene/3d/physics/soft_body_3d_settings.h"

JoltSoftBody3DSettings::JoltSoftBody3DSettings() = default;
JoltSoftBody3DSettings::~JoltSoftBody3DSettings() = default;

namespace {
JPH::Float3 to_float3(const Vector3 &v) {
	return JPH::Float3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}
} //namespace

void JoltSoftBody3DSettings::initialize(const SoftBody3DSettings &p_settings) {
	settings = new JPH::SoftBodySharedSettings();

	settings->mVertexRadius = JoltProjectSettings::soft_body_point_radius;

	Array settings_verts = p_settings.get_vertices();
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

	Array settings_edges = p_settings.get_edges();
	JPH::Array<JPH::SoftBodySharedSettings::Edge> &physics_edges = settings->mEdgeConstraints;
	physics_edges.reserve(settings_edges.size());
	for (const Variant &elem : settings_edges) {
		const Ref<SoftBody3DEdgeConstraint> &edge = elem;
		if (edge.is_valid()) {
			const Vector2i edge_verts = edge->get_vertices();
			physics_edges.emplace_back(edge_verts.x, edge_verts.y, 0.1);
		}
	}

	// TODO: add faces

	// Note: we do not use settings->CreateConstraints() here.
	// We assume that the caller has already defined precisely the constraints that
	// they want in the SoftBodySharedSettings.

	// Compute edge lengths.
	// TODO: should we put rest length in SoftBody3DEdgeConstraint?
	// We could perhaps add a helper function to SoftBody3DSettings to compute edge constraints automatically.
	for (JPH::SoftBodySharedSettings::Edge &e : settings->mEdgeConstraints) {
		JPH::SoftBodySharedSettings::Vertex v0 = physics_vertices[e.mVertex[0]];
		JPH::SoftBodySharedSettings::Vertex v1 = physics_vertices[e.mVertex[1]];
		e.mRestLength = (JPH::Vec3(v1.mPosition) - JPH::Vec3(v0.mPosition)).Length();
	}
#if 0
	int physics_index_count = 0;

	for (int i = 0; i < mesh_index_count; i += 3) {
		int physics_face[3];

		for (int j = 0; j < 3; ++j) {
			const int mesh_index = mesh_indices[i + j];
			const Vector3 vertex = mesh_vertices[mesh_index];

			HashMap<Vector3, int>::Iterator iter_physics_index = vertex_to_physics.find(vertex);

			if (iter_physics_index == vertex_to_physics.end()) {
				physics_vertices.emplace_back(JPH::Float3((float)vertex.x, (float)vertex.y, (float)vertex.z), JPH::Float3(0.0f, 0.0f, 0.0f), 1.0f);
				iter_physics_index = vertex_to_physics.insert(vertex, physics_index_count++);
			}

			physics_face[j] = iter_physics_index->value;
			mesh_to_physics[mesh_index] = iter_physics_index->value;
		}

		if (physics_face[0] == physics_face[1] || physics_face[0] == physics_face[2] || physics_face[1] == physics_face[2]) {
			continue; // We skip degenerate faces, since they're problematic, and Jolt will assert about it anyway.
		}

		// Jolt uses a different winding order, so we swap the indices to account for that.
		physics_faces.emplace_back((JPH::uint32)physics_face[2], (JPH::uint32)physics_face[1], (JPH::uint32)physics_face[0]);
	}

	// Pin whatever pinned vertices we have currently. This is used during the `Optimize` call below to order the
	// constraints. Note that it's fine if the pinned vertices change later, but that will reduce the effectiveness
	// of the constraints a bit.
	pin_vertices(*this, pinned_vertices, &mesh_to_physics, physics_vertices);

	// Since Godot's stiffness is input as a coefficient between 0 and 1, and Jolt uses actual stiffness for its
	// edge constraints, we crudely map one to the other with an arbitrary constant.
	const float stiffness = MAX(Math::pow(stiffness_coefficient, 3.0f) * 100000.0f, 0.000001f);
	const float inverse_stiffness = 1.0f / stiffness;

	JPH::SoftBodySharedSettings::VertexAttributes vertex_attrib;
	vertex_attrib.mCompliance = vertex_attrib.mShearCompliance = inverse_stiffness;

	new_settings.CreateConstraints(&vertex_attrib, 1, JPH::SoftBodySharedSettings::EBendType::None);
	float multiplier = 1.0f - shrinking_factor;
	for (JPH::SoftBodySharedSettings::Edge &e : new_settings.mEdgeConstraints) {
		e.mRestLength *= multiplier;
	}
#endif
	settings->Optimize();
}
