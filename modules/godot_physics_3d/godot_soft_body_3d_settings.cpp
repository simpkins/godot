/**************************************************************************/
/*  godot_soft_body_3d_settings.cpp                                       */
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

#include "godot_soft_body_3d_settings.h"

#include "scene/resources/3d/soft_body_3d_settings.h"

GodotSoftBody3DSettings::GodotSoftBody3DSettings() = default;
GodotSoftBody3DSettings::~GodotSoftBody3DSettings() = default;

void GodotSoftBody3DSettings::initialize(const SoftBody3DSettings &p_settings) {
	Array settings_verts = p_settings.get_vertices();
	const int vertex_count = settings_verts.size();
	vertices.resize(vertex_count);
	for (int vidx = 0; vidx < vertex_count; ++vidx) {
		const Variant &elem = settings_verts[vidx];
		const Ref<SoftBody3DVertex> &vertex = elem;
		if (!vertex.is_valid()) {
			ERR_PRINT(vformat("invalid soft body vertex %d", vidx));
			// We leave this vertex at its default settings
			continue;
		}

		vertices[vidx].position = vertex->get_position();
		vertices[vidx].velocity = vertex->get_velocity();
		vertices[vidx].inverse_mass = vertex->get_inverse_mass();
	}

	Array settings_edges = p_settings.get_edges();
	int max_edge_count = settings_edges.size();
	edges.resize(max_edge_count);
	int eidx = 0;
	for (const Variant &elem : settings_edges) {
		const Ref<SoftBody3DEdge> &settings_edge = elem;
		if (!settings_edge.is_valid()) {
			continue;
		}

		Vector2i edge_vertices = settings_edge->get_vertices();
		ERR_CONTINUE(edge_vertices.x < 0 || edge_vertices.x >= vertex_count);
		ERR_CONTINUE(edge_vertices.y < 0 || edge_vertices.y >= vertex_count);
		ERR_CONTINUE(edge_vertices.x == edge_vertices.y);

		Edge &edge = edges[eidx];
		++eidx;
		edge.v0 = edge_vertices.x;
		edge.v1 = edge_vertices.y;
		edge.rest_length = settings_edge->get_rest_length();
		real_t stiffness_coefficient = settings_edge->get_stiffness_coefficient();
		if (stiffness_coefficient == 0) {
			edge.inv_linear_stiffness = FLT_MAX;
		} else {
			edge.inv_linear_stiffness = 1.0 / stiffness_coefficient;
		}
	}
	edges.resize(eidx);

	Array settings_faces = p_settings.get_faces();
	int max_face_count = settings_faces.size();
	faces.resize(max_face_count);
	int fidx = 0;
	for (const Variant &elem : settings_faces) {
		const Ref<SoftBody3DFace> &settings_face = elem;
		if (!settings_face.is_valid()) {
			continue;
		}

		Vector3i face_vertices = settings_face->get_vertices();
		ERR_CONTINUE(face_vertices.x < 0 || face_vertices.x >= vertex_count);
		ERR_CONTINUE(face_vertices.y < 0 || face_vertices.y >= vertex_count);
		ERR_CONTINUE(face_vertices.z < 0 || face_vertices.z >= vertex_count);
		ERR_CONTINUE(face_vertices.x == face_vertices.y);
		ERR_CONTINUE(face_vertices.x == face_vertices.z);
		ERR_CONTINUE(face_vertices.y == face_vertices.z);

		Face &face = faces[fidx];
		++fidx;
		face.v0 = face_vertices.x;
		face.v1 = face_vertices.y;
		face.v2 = face_vertices.z;
	}
	faces.resize(fidx);
}
