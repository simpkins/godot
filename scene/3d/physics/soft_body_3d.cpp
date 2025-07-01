/**************************************************************************/
/*  soft_body_3d.cpp                                                      */
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

#include "soft_body_3d.h"
#include "soft_body_3d.compat.inc"

#include "scene/3d/physics/physics_body_3d.h"
#include "servers/physics_server_3d.h"

SoftBody3D::BufferData::BufferData() = default;

void SoftBody3D::BufferData::prepare(RID p_mesh, int p_surface, const PackedVector3Array &p_vertices, const PackedInt32Array &p_face_indices) {
	clear();

	ERR_FAIL_COND(!p_mesh.is_valid());

	mesh = p_mesh;
	surface = p_surface;

	RS::SurfaceData surface_data = RS::get_singleton()->mesh_get_surface(mesh, surface);

	uint32_t surface_offsets[RS::ARRAY_MAX];
	uint32_t vertex_stride;
	uint32_t normal_tangent_stride;
	uint32_t attrib_stride;
	uint32_t skin_stride;
	RS::get_singleton()->mesh_surface_make_offsets_from_format(surface_data.format, surface_data.vertex_count, surface_data.index_count, surface_offsets, vertex_stride, normal_tangent_stride, attrib_stride, skin_stride);

	buffer[0] = surface_data.vertex_data;
	vertex_count = surface_data.vertex_count;
	stride = vertex_stride;
	normal_stride = normal_tangent_stride;
	offset_vertices = surface_offsets[RS::ARRAY_VERTEX];
	offset_normal = surface_offsets[RS::ARRAY_NORMAL];

	buffer_curr = &buffer[0];
	buffer_prev = &buffer[1];

	face_indices = p_face_indices;
	compute_physics_vertex_mapping(p_vertices);
}

void SoftBody3D::BufferData::compute_physics_vertex_mapping(const PackedVector3Array &p_vertices) {
	const int render_vertex_count = p_vertices.size();
	HashMap<Vector3, int> vertex_by_coords;
	mesh_to_physics.resize(render_vertex_count);

	// Collapse mesh vertices in the same coordinates to a single physics vertex.
	// The physics servers currently do the same thing.  In the future it would be nice to only do this logic in one
	// place, and only tell the physics servers about the physics vertices, rather than making the physics servers also
	// do this logic.  See https://github.com/godotengine/godot-proposals/issues/12670a
	//
	// Note that it doesn't actually matter if our physics indices match the ones
	// chosen internally by the physics server.  We are using these values only for
	// ourself to compute normals for shading.
	const Vector3 *const render_vertices = p_vertices.ptr();
	for (int render_index = 0; render_index < render_vertex_count; ++render_index) {
		const Vector3 &render_vertex = render_vertices[render_index];
		HashMap<Vector3, int>::Iterator iter = vertex_by_coords.find(render_vertex);
		int physics_index = 0;
		if (iter == vertex_by_coords.end()) {
			physics_index = vertex_by_coords.size();
			vertex_by_coords.insert(render_vertex, physics_index);
		} else {
			physics_index = iter->value;
		}
		mesh_to_physics[render_index] = physics_index;
	}

	physics_vertex_count = vertex_by_coords.size();
}

void SoftBody3D::BufferData::clear() {
	aabb_prev = AABB();
	aabb_curr = AABB();
	buffer[0].resize(0);
	buffer[1].resize(0);
	buffer_curr = nullptr;
	buffer_prev = nullptr;
	buffer_interp.resize(0);
	vertex_count = 0;
	stride = 0;
	normal_stride = 0;
	offset_vertices = 0;
	offset_normal = 0;

	aabb_last = AABB();
	surface = 0;
	mesh = RID();
}

void SoftBody3D::BufferData::_fti_pump() {
	if (buffer_prev->is_empty()) {
		buffer_prev->resize(buffer_curr->size());
	}
	SWAP(buffer_prev, buffer_curr);
	aabb_prev = aabb_curr;
}

void SoftBody3D::BufferData::do_physics_interpolation(real_t p_interpolation_fraction) {
	const real_t f = p_interpolation_fraction;

	// Only do interpolation if the soft body received an update from the physics
	// engine in the most recent frame.  We do not want to perform interpolation
	// for sleeping soft bodies that have not received state updates recently.
	const uint64_t current_frame = Engine::get_singleton()->get_physics_frames();
	if (last_update_frame != current_frame) {
		return;
	}

	if (buffer_prev->size() != buffer_curr->size()) {
		// We need to have seen two body_state_changed() updates for this mesh before we
		// can do physics interpolation.
		return;
	}

	if (buffer_interp.is_empty()) {
		buffer_interp.resize(buffer_curr->size());
	}

	// AABB.
	AABB aabb_interp = aabb_curr;
	if (aabb_prev != AABB() && aabb_curr != AABB()) {
		aabb_interp = AABB(aabb_prev.position.lerp(aabb_curr.position, f), aabb_prev.size.lerp(aabb_curr.size, f));
	}

	const float *vertex_prev = reinterpret_cast<const float *>(buffer_prev->ptr() + offset_vertices);
	const float *vertex_curr = reinterpret_cast<const float *>(buffer_curr->ptr() + offset_vertices);
	float *vertex_interp = reinterpret_cast<float *>(buffer_interp.ptrw() + offset_vertices);

	uint32_t stride_units = stride / sizeof(float);

	const uint32_t *normal_prev = reinterpret_cast<const uint32_t *>(buffer_prev->ptr() + offset_normal);
	const uint32_t *normal_curr = reinterpret_cast<const uint32_t *>(buffer_curr->ptr() + offset_normal);
	uint32_t *normal_interp = reinterpret_cast<uint32_t *>(buffer_interp.ptrw() + offset_normal);
	uint32_t normal_stride_units = normal_stride / sizeof(uint32_t);

	for (uint32_t i = 0; i < vertex_count; i++) {
		// Vertex.
		vertex_interp[0] = Math::lerp(vertex_prev[0], vertex_curr[0], (float)f);
		vertex_interp[1] = Math::lerp(vertex_prev[1], vertex_curr[1], (float)f);
		vertex_interp[2] = Math::lerp(vertex_prev[2], vertex_curr[2], (float)f);

		vertex_prev += stride_units;
		vertex_curr += stride_units;
		vertex_interp += stride_units;

		// Normal.
		Vector2 prev = Vector2((normal_prev[0] & 0xffff) / 65535.0f, (normal_prev[0] >> 16) / 65535.0f);
		Vector2 curr = Vector2((normal_curr[0] & 0xffff) / 65535.0f, (normal_curr[0] >> 16) / 65535.0f);
		Vector2 interp = Vector3::octahedron_decode(prev).lerp(Vector3::octahedron_decode(curr), f).octahedron_encode();
		uint32_t n = 0;
		n |= (uint16_t)CLAMP(interp.x * 65535, 0, 65535);
		n |= (uint16_t)CLAMP(interp.y * 65535, 0, 65535) << 16;
		normal_interp[0] = n;

		normal_prev += normal_stride_units;
		normal_curr += normal_stride_units;
		normal_interp += normal_stride_units;
	}

	if (aabb_interp != aabb_last) {
		RS::get_singleton()->mesh_set_custom_aabb(mesh, aabb_interp);
		aabb_last = aabb_interp;
	}
	RS::get_singleton()->mesh_surface_update_vertex_region(mesh, surface, 0, buffer_interp);
}

void SoftBody3D::BufferData::disable_physics_interpolation_until_next_update() {
	// We only perform physics interpolation when last_update_frame is the current frame.
	// Setting the last_update_frame to an older frame will prevent us from doing interpolation
	// until after the next body_state_changed() call.
	last_update_frame = Engine::get_singleton()->get_physics_frames() - 2;
}

void SoftBody3D::BufferData::body_state_changed(PhysicsDirectSoftBodyState3D *p_state, bool p_interpolation_enabled) {
	if (p_interpolation_enabled) {
		_fti_pump();
		last_update_frame = Engine::get_singleton()->get_physics_frames();
	}

	PackedVector3Array vertices = p_state->get_vertices();
	const Vector3 *const vp = vertices.ptr();
	uint8_t *buffer_ptr = buffer_curr->ptrw();
	uint8_t *vertex_buffer = buffer_ptr + offset_vertices;
	for (uint32_t vidx = 0; vidx < vertex_count; ++vidx) {
		const Vector3 &v = vp[vidx];
		float as_floats[3] = { static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z) };
		memcpy(vertex_buffer, as_floats, sizeof(as_floats));
		vertex_buffer += stride;
	}

	_recompute_normals(buffer_ptr);
	RS::get_singleton()->mesh_surface_update_vertex_region(mesh, surface, 0, *buffer_curr);

	AABB aabb = p_state->get_aabb();
	if (aabb != aabb_last) {
		RS::get_singleton()->mesh_set_custom_aabb(mesh, aabb);
		aabb_last = aabb;
	}
}

void SoftBody3D::BufferData::_recompute_normals(uint8_t *p_buffer) {
	const bool smooth_shading = shading_mode == SoftBody3D::SHADING_SMOOTH;
	const int normal_buffer_size = smooth_shading ? physics_vertex_count : vertex_count;
	normal_compute_buffer.clear();
	normal_compute_buffer.resize(normal_buffer_size);

	const uint8_t *const vertex_buffer = p_buffer + offset_vertices;
	auto get_vertex = [&](int vidx) {
		const uint8_t *vertex_data = vertex_buffer + vidx * stride;
		Vector3 result;
		// Use memcpy here, since it is technically undefined behavior to access
		// a char buffer via a float* pointer.  (Only the reverse is allowed--any
		// data type can be accessed via a char* pointer, but not the other
		// way around.)  In practice most compilers are smart enough to optimize
		// this memcpy() to a simple read instructions.
		memcpy(&result.coord, vertex_data, sizeof(result.coord));
		return result;
	};

	// Compute vertex normals.
	//
	// Smooth shading:
	// - Each vertex gets the average normal from all physics faces it is connected to.
	// Flat shading
	// - Each vertex gets the average normal from all render faces it is connected to.
	//
	// Iterate over each face, and add the face normal to each of the face vertices.
	int num_face_indices = face_indices.size();
	for (int fidx = 0; fidx + 2 < num_face_indices; fidx += 3) {
		int v0_idx = face_indices[fidx];
		int v1_idx = face_indices[fidx + 1];
		int v2_idx = face_indices[fidx + 2];

		Vector3 v0 = get_vertex(v0_idx);
		Vector3 v1 = get_vertex(v1_idx);
		Vector3 v2 = get_vertex(v2_idx);
		const Vector3 face_normal = (v2 - v0).cross(v1 - v0).normalized();

		// When using smooth shading, normal_compute_buffer is using physics vertex indices.
		if (smooth_shading) {
			v0_idx = mesh_to_physics[v0_idx];
			v1_idx = mesh_to_physics[v1_idx];
			v2_idx = mesh_to_physics[v2_idx];
		}

		normal_compute_buffer[v0_idx] += face_normal;
		normal_compute_buffer[v1_idx] += face_normal;
		normal_compute_buffer[v2_idx] += face_normal;
	}
	// Normalize the vertex normals to have length 1.0
	for (Vector3 &n : normal_compute_buffer) {
		real_t len = n.length();
		// Some normals may have length 0 if the face was degenerate,
		// so don't divide by zero.
		if (len > CMP_EPSILON) {
			n /= len;
		}
	}

	// Now assign each mesh vertex the normal computed
	// from its physics vertex
	uint8_t *normal_buffer = p_buffer + offset_normal;
	for (uint32_t mesh_idx = 0; mesh_idx < vertex_count; ++mesh_idx, normal_buffer += normal_stride) {
		uint32_t normal_idx = smooth_shading ? mesh_to_physics[mesh_idx] : mesh_idx;
		Vector2 encoded = normal_compute_buffer[normal_idx].octahedron_encode();
		uint32_t value = 0;
		value |= (uint16_t)CLAMP(encoded.x * 65535, 0, 65535);
		value |= (uint16_t)CLAMP(encoded.y * 65535, 0, 65535) << 16;
		memcpy(normal_buffer, &value, sizeof(uint32_t));
	}
}

void SoftBody3D::BufferData::recompute_normals() {
	_recompute_normals(buffer_curr->ptrw());
	RS::get_singleton()->mesh_surface_update_vertex_region(mesh, surface, 0, *buffer_curr);
}

SoftBody3D::ShadingMode SoftBody3D::BufferData::get_shading_mode() const {
	return shading_mode;
}

void SoftBody3D::BufferData::set_shading_mode(ShadingMode p_mode) {
	shading_mode = p_mode;
	if (has_mesh()) {
		recompute_normals();
	}
}

SoftBody3D::PinnedPoint::PinnedPoint() {
}

SoftBody3D::PinnedPoint::PinnedPoint(const PinnedPoint &obj_tocopy) {
	point_index = obj_tocopy.point_index;
	spatial_attachment_path = obj_tocopy.spatial_attachment_path;
	spatial_attachment = obj_tocopy.spatial_attachment;
	offset = obj_tocopy.offset;
}

void SoftBody3D::PinnedPoint::operator=(const PinnedPoint &obj) {
	point_index = obj.point_index;
	spatial_attachment_path = obj.spatial_attachment_path;
	spatial_attachment = obj.spatial_attachment;
	offset = obj.offset;
}

void SoftBody3D::_update_pickable() {
	if (!is_inside_tree()) {
		return;
	}
	bool pickable = ray_pickable && is_visible_in_tree();
	PhysicsServer3D::get_singleton()->soft_body_set_ray_pickable(physics_rid, pickable);
}

bool SoftBody3D::_set(const StringName &p_name, const Variant &p_value) {
	String name = p_name;
	String which = name.get_slicec('/', 0);

	if ("pinned_points" == which) {
		return _set_property_pinned_points_indices(p_value);

	} else if ("attachments" == which) {
		int idx = name.get_slicec('/', 1).to_int();
		String what = name.get_slicec('/', 2);

		return _set_property_pinned_points_attachment(idx, what, p_value);
	}

	return false;
}

bool SoftBody3D::_get(const StringName &p_name, Variant &r_ret) const {
	String name = p_name;
	String which = name.get_slicec('/', 0);

	if ("pinned_points" == which) {
		Array arr_ret;
		const int pinned_points_indices_size = pinned_points.size();
		const PinnedPoint *r = pinned_points.ptr();
		arr_ret.resize(pinned_points_indices_size);

		for (int i = 0; i < pinned_points_indices_size; ++i) {
			arr_ret[i] = r[i].point_index;
		}

		r_ret = arr_ret;
		return true;

	} else if ("attachments" == which) {
		int idx = name.get_slicec('/', 1).to_int();
		String what = name.get_slicec('/', 2);

		return _get_property_pinned_points(idx, what, r_ret);
	}

	return false;
}

void SoftBody3D::_get_property_list(List<PropertyInfo> *p_list) const {
	const int pinned_points_indices_size = pinned_points.size();

	p_list->push_back(PropertyInfo(Variant::PACKED_INT32_ARRAY, PNAME("pinned_points")));

	for (int i = 0; i < pinned_points_indices_size; ++i) {
		const String prefix = vformat("%s/%d/", PNAME("attachments"), i);
		p_list->push_back(PropertyInfo(Variant::INT, prefix + PNAME("point_index")));
		p_list->push_back(PropertyInfo(Variant::NODE_PATH, prefix + PNAME("spatial_attachment_path")));
		p_list->push_back(PropertyInfo(Variant::VECTOR3, prefix + PNAME("offset")));
	}
}

bool SoftBody3D::_set_property_pinned_points_indices(const Array &p_indices) {
	const int p_indices_size = p_indices.size();

	{ // Remove the pined points on physics server that will be removed by resize
		const PinnedPoint *r = pinned_points.ptr();
		if (p_indices_size < pinned_points.size()) {
			for (int i = pinned_points.size() - 1; i >= p_indices_size; --i) {
				pin_point(r[i].point_index, false);
			}
		}
	}

	pinned_points.resize(p_indices_size);

	PinnedPoint *w = pinned_points.ptrw();
	int point_index;
	for (int i = 0; i < p_indices_size; ++i) {
		point_index = p_indices.get(i);
		if (w[i].point_index != point_index || pinned_points.size() < p_indices_size) {
			bool insert = false;
			if (w[i].point_index != -1 && p_indices.find(w[i].point_index) == -1) {
				pin_point(w[i].point_index, false);
				insert = true;
			}
			w[i].point_index = point_index;
			if (insert) {
				pin_point(w[i].point_index, true, NodePath(), i);
			} else {
				pin_point(w[i].point_index, true);
			}
		}
	}
	return true;
}

bool SoftBody3D::_set_property_pinned_points_attachment(int p_item, const String &p_what, const Variant &p_value) {
	if (pinned_points.size() <= p_item) {
		return false;
	}

	if ("spatial_attachment_path" == p_what) {
		PinnedPoint *w = pinned_points.ptrw();

		if (is_inside_tree()) {
			callable_mp(this, &SoftBody3D::_pin_point_deferred).call_deferred(Variant(w[p_item].point_index), true, p_value);
		} else {
			pin_point(w[p_item].point_index, true, p_value);
			_make_cache_dirty();
		}
	} else if ("offset" == p_what) {
		PinnedPoint *w = pinned_points.ptrw();
		w[p_item].offset = p_value;
	} else {
		return false;
	}

	return true;
}

bool SoftBody3D::_get_property_pinned_points(int p_item, const String &p_what, Variant &r_ret) const {
	if (pinned_points.size() <= p_item) {
		return false;
	}
	const PinnedPoint *r = pinned_points.ptr();

	if ("point_index" == p_what) {
		r_ret = r[p_item].point_index;
	} else if ("spatial_attachment_path" == p_what) {
		r_ret = r[p_item].spatial_attachment_path;
	} else if ("offset" == p_what) {
		r_ret = r[p_item].offset;
	} else {
		return false;
	}

	return true;
}

void SoftBody3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			if (mesh.is_valid() && !buffer_data.has_mesh()) {
				_convert_mesh(mesh);
			}
			RID space = get_world_3d()->get_space();
			PhysicsServer3D::get_singleton()->soft_body_set_space(physics_rid, space);
			_update_simulation_active();
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			if (simulation_active && is_physics_interpolated_and_enabled()) {
				buffer_data.do_physics_interpolation(Engine::get_singleton()->get_physics_interpolation_fraction());
			}
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (is_inside_tree()) {
				_update_physics_server();
			}
		} break;
		case NOTIFICATION_READY: {
			if (!parent_collision_ignore.is_empty()) {
				add_collision_exception_with(get_node(parent_collision_ignore));
			}
		} break;

		case NOTIFICATION_TRANSFORM_CHANGED: {
			if (Engine::get_singleton()->is_editor_hint()) {
				_reset_points_offsets();
				return;
			}
		} break;
		case NOTIFICATION_RESET_PHYSICS_INTERPOLATION: {
			buffer_data.disable_physics_interpolation_until_next_update();
			if (simulation_active) {
				// We'll get NOTIFICATION_RESET_PHYSICS_INTERPOLATION if the user calls
				// reset_physics_interpolation() explicitly, or if the global
				// SceneTree.physics_interpolated setting is changed.
				//
				// In case SceneTree.physics_interpolated has changed we need to update
				// our set_process_internal() setting.
				set_process_internal(is_physics_interpolated_and_enabled());
			}
		} break;
		case NOTIFICATION_VISIBILITY_CHANGED: {
			_update_pickable();
		} break;

		case NOTIFICATION_EXIT_WORLD: {
			PhysicsServer3D::get_singleton()->soft_body_set_space(physics_rid, RID());
			_update_simulation_active();
		} break;

		case NOTIFICATION_DISABLED: {
			buffer_data.disable_physics_interpolation_until_next_update();
			_update_simulation_active();
		} break;

		case NOTIFICATION_ENABLED: {
			_update_simulation_active();
		} break;
	}
}

void SoftBody3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_physics_rid"), &SoftBody3D::get_physics_rid);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "collision_mask"), &SoftBody3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &SoftBody3D::get_collision_mask);

	ClassDB::bind_method(D_METHOD("set_collision_layer", "collision_layer"), &SoftBody3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &SoftBody3D::get_collision_layer);

	ClassDB::bind_method(D_METHOD("set_collision_mask_value", "layer_number", "value"), &SoftBody3D::set_collision_mask_value);
	ClassDB::bind_method(D_METHOD("get_collision_mask_value", "layer_number"), &SoftBody3D::get_collision_mask_value);

	ClassDB::bind_method(D_METHOD("set_collision_layer_value", "layer_number", "value"), &SoftBody3D::set_collision_layer_value);
	ClassDB::bind_method(D_METHOD("get_collision_layer_value", "layer_number"), &SoftBody3D::get_collision_layer_value);

	ClassDB::bind_method(D_METHOD("set_parent_collision_ignore", "parent_collision_ignore"), &SoftBody3D::set_parent_collision_ignore);
	ClassDB::bind_method(D_METHOD("get_parent_collision_ignore"), &SoftBody3D::get_parent_collision_ignore);

	ClassDB::bind_method(D_METHOD("set_disable_mode", "mode"), &SoftBody3D::set_disable_mode);
	ClassDB::bind_method(D_METHOD("get_disable_mode"), &SoftBody3D::get_disable_mode);

	ClassDB::bind_method(D_METHOD("set_shading_mode", "mode"), &SoftBody3D::set_shading_mode);
	ClassDB::bind_method(D_METHOD("get_shading_mode"), &SoftBody3D::get_shading_mode);

	ClassDB::bind_method(D_METHOD("get_collision_exceptions"), &SoftBody3D::get_collision_exceptions);
	ClassDB::bind_method(D_METHOD("add_collision_exception_with", "body"), &SoftBody3D::add_collision_exception_with);
	ClassDB::bind_method(D_METHOD("remove_collision_exception_with", "body"), &SoftBody3D::remove_collision_exception_with);

	ClassDB::bind_method(D_METHOD("set_simulation_precision", "simulation_precision"), &SoftBody3D::set_simulation_precision);
	ClassDB::bind_method(D_METHOD("get_simulation_precision"), &SoftBody3D::get_simulation_precision);

	ClassDB::bind_method(D_METHOD("set_total_mass", "mass"), &SoftBody3D::set_total_mass);
	ClassDB::bind_method(D_METHOD("get_total_mass"), &SoftBody3D::get_total_mass);

	ClassDB::bind_method(D_METHOD("set_linear_stiffness", "linear_stiffness"), &SoftBody3D::set_linear_stiffness);
	ClassDB::bind_method(D_METHOD("get_linear_stiffness"), &SoftBody3D::get_linear_stiffness);

	ClassDB::bind_method(D_METHOD("set_shrinking_factor", "shrinking_factor"), &SoftBody3D::set_shrinking_factor);
	ClassDB::bind_method(D_METHOD("get_shrinking_factor"), &SoftBody3D::get_shrinking_factor);

	ClassDB::bind_method(D_METHOD("set_pressure_coefficient", "pressure_coefficient"), &SoftBody3D::set_pressure_coefficient);
	ClassDB::bind_method(D_METHOD("get_pressure_coefficient"), &SoftBody3D::get_pressure_coefficient);

	ClassDB::bind_method(D_METHOD("set_damping_coefficient", "damping_coefficient"), &SoftBody3D::set_damping_coefficient);
	ClassDB::bind_method(D_METHOD("get_damping_coefficient"), &SoftBody3D::get_damping_coefficient);

	ClassDB::bind_method(D_METHOD("set_drag_coefficient", "drag_coefficient"), &SoftBody3D::set_drag_coefficient);
	ClassDB::bind_method(D_METHOD("get_drag_coefficient"), &SoftBody3D::get_drag_coefficient);

	ClassDB::bind_method(D_METHOD("get_point_transform", "point_index"), &SoftBody3D::get_point_transform);

	ClassDB::bind_method(D_METHOD("apply_impulse", "point_index", "impulse"), &SoftBody3D::apply_impulse);
	ClassDB::bind_method(D_METHOD("apply_force", "point_index", "force"), &SoftBody3D::apply_force);
	ClassDB::bind_method(D_METHOD("apply_central_impulse", "impulse"), &SoftBody3D::apply_central_impulse);
	ClassDB::bind_method(D_METHOD("apply_central_force", "force"), &SoftBody3D::apply_central_force);

	ClassDB::bind_method(D_METHOD("set_point_pinned", "point_index", "pinned", "attachment_path", "insert_at"), &SoftBody3D::pin_point, DEFVAL(NodePath()), DEFVAL(-1));
	ClassDB::bind_method(D_METHOD("is_point_pinned", "point_index"), &SoftBody3D::is_point_pinned);

	ClassDB::bind_method(D_METHOD("set_ray_pickable", "ray_pickable"), &SoftBody3D::set_ray_pickable);
	ClassDB::bind_method(D_METHOD("is_ray_pickable"), &SoftBody3D::is_ray_pickable);

	ADD_GROUP("Collision", "collision_");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "parent_collision_ignore", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "CollisionObject3D"), "set_parent_collision_ignore", "get_parent_collision_ignore");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "simulation_precision", PROPERTY_HINT_RANGE, "1,100,1"), "set_simulation_precision", "get_simulation_precision");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "total_mass", PROPERTY_HINT_RANGE, "0.01,10000,1"), "set_total_mass", "get_total_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "linear_stiffness", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_linear_stiffness", "get_linear_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "shrinking_factor", PROPERTY_HINT_RANGE, "-1,1,0.01,or_less,or_greater"), "set_shrinking_factor", "get_shrinking_factor");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pressure_coefficient"), "set_pressure_coefficient", "get_pressure_coefficient");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping_coefficient", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_damping_coefficient", "get_damping_coefficient");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag_coefficient", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_drag_coefficient", "get_drag_coefficient");

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "ray_pickable"), "set_ray_pickable", "is_ray_pickable");

	ADD_PROPERTY(PropertyInfo(Variant::INT, "disable_mode", PROPERTY_HINT_ENUM, "Remove,KeepActive"), "set_disable_mode", "get_disable_mode");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "shading_mode", PROPERTY_HINT_ENUM, "Smooth,Flat"), "set_shading_mode", "get_shading_mode");

	BIND_ENUM_CONSTANT(DISABLE_MODE_REMOVE);
	BIND_ENUM_CONSTANT(DISABLE_MODE_KEEP_ACTIVE);
	BIND_ENUM_CONSTANT(SHADING_FLAT);
	BIND_ENUM_CONSTANT(SHADING_SMOOTH);
}

void SoftBody3D::_physics_interpolated_changed() {
	buffer_data.disable_physics_interpolation_until_next_update();
	if (simulation_active) {
		set_process_internal(is_physics_interpolated_and_enabled());
	}
	MeshInstance3D::_physics_interpolated_changed();
}

PackedStringArray SoftBody3D::get_configuration_warnings() const {
	PackedStringArray warnings = MeshInstance3D::get_configuration_warnings();

	if (mesh.is_null()) {
		warnings.push_back(RTR("This body will be ignored until you set a mesh."));
	}

	return warnings;
}

void SoftBody3D::_update_physics_server() {
	_update_cache_pin_points_datas();
	// Submit bone attachment
	const int pinned_points_indices_size = pinned_points.size();
	const PinnedPoint *r = pinned_points.ptr();
	for (int i = 0; i < pinned_points_indices_size; ++i) {
		if (r[i].spatial_attachment) {
			PhysicsServer3D::get_singleton()->soft_body_move_point(physics_rid, r[i].point_index, r[i].spatial_attachment->get_global_transform().xform(r[i].offset));
		}
	}
}

void SoftBody3D::_body_state_changed(PhysicsDirectSoftBodyState3D *p_state) {
	buffer_data.body_state_changed(p_state, is_physics_interpolated_and_enabled());
}

void SoftBody3D::_update_simulation_active() {
	// Note that the simulation is never active in the editor;
	// buffer_data.has_mesh() always returns false in the editor.

	const bool new_state = is_inside_tree() && buffer_data.has_mesh() && (is_enabled() || (disable_mode != DISABLE_MODE_REMOVE));
	if (new_state == simulation_active) {
		// We want to avoid calling soft_body_set_mesh() again if we are already active,
		// since resetting the mesh may reset vertex velocities and recompute rest edge lengths
		// from the current mesh state.
		return;
	}

	simulation_active = new_state;
	if (simulation_active) {
		PhysicsServer3D::get_singleton()->soft_body_set_mesh(physics_rid, mesh->get_rid());
		set_process_internal(is_physics_interpolated_and_enabled());
		set_physics_process_internal(true);
	} else {
		PhysicsServer3D::get_singleton()->soft_body_set_mesh(physics_rid, RID());
		set_process_internal(false);
		set_physics_process_internal(false);
	}
}

void SoftBody3D::set_mesh(const Ref<Mesh> &p_mesh) {
	if (p_mesh == mesh) {
		return;
	}

	_process_set_mesh(p_mesh);
	_update_simulation_active();

#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint()) {
		update_configuration_warnings();
	}
#endif
}

void SoftBody3D::_process_set_mesh(const Ref<Mesh> &p_mesh) {
	if (p_mesh.is_null()) {
		buffer_data.clear();
		MeshInstance3D::set_mesh(nullptr);
		return;
	}

	// We only support meshes where surface 0 is a PRIMITIVE_TRIANGLES surface
	if (p_mesh->get_surface_count() < 1 || p_mesh->surface_get_primitive_type(0) != Mesh::PRIMITIVE_TRIANGLES) {
		ERR_PRINT("SoftBody3D only supports triangle meshes");
		buffer_data.clear();
		MeshInstance3D::set_mesh(nullptr);
		return;
	}

	// If we are not currently in the scene tree, we do not know our global transform
	// and therefore cannot create the dynamic soft mesh yet.
	// Calling buffer_data.clear() will let us know that we still need to
	// convert the mesh later on receipt of NOTIFICATION_ENTER_WORLD.
	if (!is_inside_tree()) {
		buffer_data.clear();
		MeshInstance3D::set_mesh(p_mesh);
		return;
	}

	_convert_mesh(p_mesh);
}

void SoftBody3D::_convert_mesh(const Ref<Mesh> &p_mesh) {
#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint()) {
		// We do not perform soft body simulation in the editor, and never convert the mesh.
		if (p_mesh != mesh) {
			MeshInstance3D::set_mesh(p_mesh);
		}
		return;
	}
#endif

	// Get current mesh array and create new mesh array with necessary flag for SoftBody
	Array surface_arrays = p_mesh->surface_get_arrays(0);
	Array surface_blend_arrays = p_mesh->surface_get_blend_shape_arrays(0);
	Dictionary surface_lods = p_mesh->surface_get_lods(0);
	uint32_t surface_format = p_mesh->surface_get_format(0);

	surface_format |= Mesh::ARRAY_FLAG_USE_DYNAMIC_UPDATE;
	surface_format &= ~Mesh::ARRAY_FLAG_COMPRESS_ATTRIBUTES;

	// Update the mesh vertices with the current global transform,
	// since we use set_instance_use_identity_transform(true).
	_update_mesh_arrays_with_transform(surface_arrays, surface_format);

	Ref<ArrayMesh> soft_mesh;
	soft_mesh.instantiate();
	soft_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, surface_arrays, surface_blend_arrays, surface_lods, surface_format);
	soft_mesh->surface_set_material(0, p_mesh->surface_get_material(0));

	RID soft_mesh_rid = soft_mesh->get_rid();
	buffer_data.prepare(soft_mesh_rid, 0, surface_arrays[Mesh::ARRAY_VERTEX], surface_arrays[Mesh::ARRAY_INDEX]);
	buffer_data.recompute_normals();
	MeshInstance3D::set_mesh(soft_mesh);
	if (simulation_active) {
		PhysicsServer3D::get_singleton()->soft_body_set_mesh(physics_rid, soft_mesh_rid);
	}
}

void SoftBody3D::_update_mesh_arrays_with_transform(Array &surface_arrays, uint32_t surface_format) {
	Transform3D transform = get_global_transform();
	if (transform == Transform3D()) {
		return;
	}

	PackedVector3Array vertices = surface_arrays[Mesh::ARRAY_VERTEX];
	// Temporarily reset surface_arrays to avoid an unnecessary copy-on-write
	// when we we modify the array.
	surface_arrays[Mesh::ARRAY_VERTEX] = Variant();
	int vertex_count = vertices.size();
	Vector3 *vptr = vertices.ptrw();
	for (int vidx = 0; vidx < vertex_count; ++vidx) {
		vptr[vidx] = transform.xform(vptr[vidx]);
	}
	surface_arrays[Mesh::ARRAY_VERTEX] = vertices;

	if (surface_format & Mesh::ARRAY_FORMAT_NORMAL) {
		PackedVector3Array normals = surface_arrays[Mesh::ARRAY_NORMAL];
		Vector3 *nptr = vertices.ptrw();
		surface_arrays[Mesh::ARRAY_NORMAL] = Variant();
		for (int vidx = 0; vidx < vertex_count; ++vidx) {
			nptr[vidx] = transform.xform(nptr[vidx]);
		}
		surface_arrays[Mesh::ARRAY_NORMAL] = normals;
	}
}

void SoftBody3D::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
	PhysicsServer3D::get_singleton()->soft_body_set_collision_mask(physics_rid, p_mask);
}

uint32_t SoftBody3D::get_collision_mask() const {
	return collision_mask;
}

void SoftBody3D::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;
	PhysicsServer3D::get_singleton()->soft_body_set_collision_layer(physics_rid, p_layer);
}

uint32_t SoftBody3D::get_collision_layer() const {
	return collision_layer;
}

void SoftBody3D::set_collision_layer_value(int p_layer_number, bool p_value) {
	ERR_FAIL_COND_MSG(p_layer_number < 1, "Collision layer number must be between 1 and 32 inclusive.");
	ERR_FAIL_COND_MSG(p_layer_number > 32, "Collision layer number must be between 1 and 32 inclusive.");
	uint32_t collision_layer_new = get_collision_layer();
	if (p_value) {
		collision_layer_new |= 1 << (p_layer_number - 1);
	} else {
		collision_layer_new &= ~(1 << (p_layer_number - 1));
	}
	set_collision_layer(collision_layer_new);
}

bool SoftBody3D::get_collision_layer_value(int p_layer_number) const {
	ERR_FAIL_COND_V_MSG(p_layer_number < 1, false, "Collision layer number must be between 1 and 32 inclusive.");
	ERR_FAIL_COND_V_MSG(p_layer_number > 32, false, "Collision layer number must be between 1 and 32 inclusive.");
	return get_collision_layer() & (1 << (p_layer_number - 1));
}

void SoftBody3D::set_collision_mask_value(int p_layer_number, bool p_value) {
	ERR_FAIL_COND_MSG(p_layer_number < 1, "Collision layer number must be between 1 and 32 inclusive.");
	ERR_FAIL_COND_MSG(p_layer_number > 32, "Collision layer number must be between 1 and 32 inclusive.");
	uint32_t mask = get_collision_mask();
	if (p_value) {
		mask |= 1 << (p_layer_number - 1);
	} else {
		mask &= ~(1 << (p_layer_number - 1));
	}
	set_collision_mask(mask);
}

bool SoftBody3D::get_collision_mask_value(int p_layer_number) const {
	ERR_FAIL_COND_V_MSG(p_layer_number < 1, false, "Collision layer number must be between 1 and 32 inclusive.");
	ERR_FAIL_COND_V_MSG(p_layer_number > 32, false, "Collision layer number must be between 1 and 32 inclusive.");
	return get_collision_mask() & (1 << (p_layer_number - 1));
}

void SoftBody3D::set_disable_mode(DisableMode p_mode) {
	if (disable_mode == p_mode) {
		return;
	}

	disable_mode = p_mode;
	_update_simulation_active();
}

SoftBody3D::DisableMode SoftBody3D::get_disable_mode() const {
	return disable_mode;
}

void SoftBody3D::set_shading_mode(ShadingMode p_mode) {
	buffer_data.set_shading_mode(p_mode);
}

SoftBody3D::ShadingMode SoftBody3D::get_shading_mode() const {
	return buffer_data.get_shading_mode();
}

void SoftBody3D::set_parent_collision_ignore(const NodePath &p_parent_collision_ignore) {
	parent_collision_ignore = p_parent_collision_ignore;
}

const NodePath &SoftBody3D::get_parent_collision_ignore() const {
	return parent_collision_ignore;
}

void SoftBody3D::set_pinned_points_indices(Vector<SoftBody3D::PinnedPoint> p_pinned_points_indices) {
	pinned_points = p_pinned_points_indices;
	for (int i = pinned_points.size() - 1; 0 <= i; --i) {
		pin_point(p_pinned_points_indices[i].point_index, true);
	}
}

Vector<SoftBody3D::PinnedPoint> SoftBody3D::get_pinned_points_indices() {
	return pinned_points;
}

TypedArray<PhysicsBody3D> SoftBody3D::get_collision_exceptions() {
	List<RID> exceptions;
	PhysicsServer3D::get_singleton()->soft_body_get_collision_exceptions(physics_rid, &exceptions);
	TypedArray<PhysicsBody3D> ret;
	for (const RID &body : exceptions) {
		ObjectID instance_id = PhysicsServer3D::get_singleton()->body_get_object_instance_id(body);
		Object *obj = ObjectDB::get_instance(instance_id);
		PhysicsBody3D *physics_body = Object::cast_to<PhysicsBody3D>(obj);
		ret.append(physics_body);
	}
	return ret;
}

void SoftBody3D::add_collision_exception_with(Node *p_node) {
	ERR_FAIL_NULL(p_node);
	CollisionObject3D *collision_object = Object::cast_to<CollisionObject3D>(p_node);
	ERR_FAIL_NULL_MSG(collision_object, "Collision exception only works between two nodes that inherit from CollisionObject3D (such as Area3D or PhysicsBody3D).");
	PhysicsServer3D::get_singleton()->soft_body_add_collision_exception(physics_rid, collision_object->get_rid());
}

void SoftBody3D::remove_collision_exception_with(Node *p_node) {
	ERR_FAIL_NULL(p_node);
	CollisionObject3D *collision_object = Object::cast_to<CollisionObject3D>(p_node);
	ERR_FAIL_NULL_MSG(collision_object, "Collision exception only works between two nodes that inherit from CollisionObject3D (such as Area3D or PhysicsBody3D).");
	PhysicsServer3D::get_singleton()->soft_body_remove_collision_exception(physics_rid, collision_object->get_rid());
}

int SoftBody3D::get_simulation_precision() {
	return PhysicsServer3D::get_singleton()->soft_body_get_simulation_precision(physics_rid);
}

void SoftBody3D::set_simulation_precision(int p_simulation_precision) {
	PhysicsServer3D::get_singleton()->soft_body_set_simulation_precision(physics_rid, p_simulation_precision);
}

real_t SoftBody3D::get_total_mass() {
	return PhysicsServer3D::get_singleton()->soft_body_get_total_mass(physics_rid);
}

void SoftBody3D::set_total_mass(real_t p_total_mass) {
	PhysicsServer3D::get_singleton()->soft_body_set_total_mass(physics_rid, p_total_mass);
}

void SoftBody3D::set_linear_stiffness(real_t p_linear_stiffness) {
	PhysicsServer3D::get_singleton()->soft_body_set_linear_stiffness(physics_rid, p_linear_stiffness);
}

real_t SoftBody3D::get_linear_stiffness() {
	return PhysicsServer3D::get_singleton()->soft_body_get_linear_stiffness(physics_rid);
}

void SoftBody3D::set_shrinking_factor(real_t p_shrinking_factor) {
	PhysicsServer3D::get_singleton()->soft_body_set_shrinking_factor(physics_rid, p_shrinking_factor);
}

real_t SoftBody3D::get_shrinking_factor() {
	return PhysicsServer3D::get_singleton()->soft_body_get_shrinking_factor(physics_rid);
}

real_t SoftBody3D::get_pressure_coefficient() {
	return PhysicsServer3D::get_singleton()->soft_body_get_pressure_coefficient(physics_rid);
}

void SoftBody3D::set_pressure_coefficient(real_t p_pressure_coefficient) {
	PhysicsServer3D::get_singleton()->soft_body_set_pressure_coefficient(physics_rid, p_pressure_coefficient);
}

real_t SoftBody3D::get_damping_coefficient() {
	return PhysicsServer3D::get_singleton()->soft_body_get_damping_coefficient(physics_rid);
}

void SoftBody3D::set_damping_coefficient(real_t p_damping_coefficient) {
	PhysicsServer3D::get_singleton()->soft_body_set_damping_coefficient(physics_rid, p_damping_coefficient);
}

real_t SoftBody3D::get_drag_coefficient() {
	return PhysicsServer3D::get_singleton()->soft_body_get_drag_coefficient(physics_rid);
}

void SoftBody3D::set_drag_coefficient(real_t p_drag_coefficient) {
	PhysicsServer3D::get_singleton()->soft_body_set_drag_coefficient(physics_rid, p_drag_coefficient);
}

Vector3 SoftBody3D::get_point_transform(int p_point_index) {
	return PhysicsServer3D::get_singleton()->soft_body_get_point_global_position(physics_rid, p_point_index);
}

void SoftBody3D::apply_impulse(int p_point_index, const Vector3 &p_impulse) {
	PhysicsServer3D::get_singleton()->soft_body_apply_point_impulse(physics_rid, p_point_index, p_impulse);
}

void SoftBody3D::apply_force(int p_point_index, const Vector3 &p_force) {
	PhysicsServer3D::get_singleton()->soft_body_apply_point_force(physics_rid, p_point_index, p_force);
}

void SoftBody3D::apply_central_impulse(const Vector3 &p_impulse) {
	PhysicsServer3D::get_singleton()->soft_body_apply_central_impulse(physics_rid, p_impulse);
}

void SoftBody3D::apply_central_force(const Vector3 &p_force) {
	PhysicsServer3D::get_singleton()->soft_body_apply_central_force(physics_rid, p_force);
}

void SoftBody3D::pin_point_toggle(int p_point_index) {
	pin_point(p_point_index, !(-1 != _has_pinned_point(p_point_index)));
}

void SoftBody3D::pin_point(int p_point_index, bool pin, const NodePath &p_spatial_attachment_path, int p_insert_at) {
	ERR_FAIL_COND_MSG(p_insert_at < -1 || p_insert_at >= pinned_points.size(), "Invalid index for pin point insertion position.");
	_pin_point_on_physics_server(p_point_index, pin);
	if (pin) {
		_add_pinned_point(p_point_index, p_spatial_attachment_path, p_insert_at);
	} else {
		_remove_pinned_point(p_point_index);
	}
}

void SoftBody3D::_pin_point_deferred(int p_point_index, bool pin, const NodePath p_spatial_attachment_path) {
	pin_point(p_point_index, pin, p_spatial_attachment_path);
	_make_cache_dirty();
}

bool SoftBody3D::is_point_pinned(int p_point_index) const {
	return -1 != _has_pinned_point(p_point_index);
}

void SoftBody3D::set_ray_pickable(bool p_ray_pickable) {
	ray_pickable = p_ray_pickable;
	_update_pickable();
}

bool SoftBody3D::is_ray_pickable() const {
	return ray_pickable;
}

SoftBody3D::SoftBody3D() :
		physics_rid(PhysicsServer3D::get_singleton()->soft_body_create()) {
	PhysicsServer3D::get_singleton()->body_attach_object_instance_id(physics_rid, get_instance_id());
	PhysicsServer3D::get_singleton()->soft_body_set_state_sync_callback(physics_rid, callable_mp(this, &SoftBody3D::_body_state_changed));

	// We always render at the origin in game.
	// In the editor we don't run physics simulations, and we just render the input static mesh,
	// so we do want to render it at the initial transform where the simulation starts.
	set_instance_use_identity_transform(!Engine::get_singleton()->is_editor_hint());
}

SoftBody3D::~SoftBody3D() {
	ERR_FAIL_NULL(PhysicsServer3D::get_singleton());
	PhysicsServer3D::get_singleton()->free(physics_rid);
}

void SoftBody3D::_make_cache_dirty() {
	pinned_points_cache_dirty = true;
}

void SoftBody3D::_update_cache_pin_points_datas() {
	if (!pinned_points_cache_dirty) {
		return;
	}

	pinned_points_cache_dirty = false;

	PinnedPoint *w = pinned_points.ptrw();
	for (int i = pinned_points.size() - 1; 0 <= i; --i) {
		if (!w[i].spatial_attachment_path.is_empty()) {
			w[i].spatial_attachment = Object::cast_to<Node3D>(get_node(w[i].spatial_attachment_path));
		}
	}
}

void SoftBody3D::_pin_point_on_physics_server(int p_point_index, bool pin) {
	PhysicsServer3D::get_singleton()->soft_body_pin_point(physics_rid, p_point_index, pin);
}

void SoftBody3D::_add_pinned_point(int p_point_index, const NodePath &p_spatial_attachment_path, int p_insert_at) {
	SoftBody3D::PinnedPoint *pinned_point;
	if (-1 == _get_pinned_point(p_point_index, pinned_point)) {
		// Create new
		PinnedPoint pp;
		pp.point_index = p_point_index;
		pp.spatial_attachment_path = p_spatial_attachment_path;

		if (!p_spatial_attachment_path.is_empty() && has_node(p_spatial_attachment_path)) {
			pp.spatial_attachment = Object::cast_to<Node3D>(get_node(p_spatial_attachment_path));
			pp.offset = (pp.spatial_attachment->get_global_transform().affine_inverse() * get_global_transform()).xform(PhysicsServer3D::get_singleton()->soft_body_get_point_global_position(physics_rid, pp.point_index));
		}

		if (p_insert_at != -1) {
			pinned_points.insert(p_insert_at, pp);
		} else {
			pinned_points.push_back(pp);
		}

	} else {
		pinned_point->point_index = p_point_index;
		pinned_point->spatial_attachment_path = p_spatial_attachment_path;

		if (!p_spatial_attachment_path.is_empty() && has_node(p_spatial_attachment_path)) {
			Node3D *attachment_node = Object::cast_to<Node3D>(get_node(p_spatial_attachment_path));

			ERR_FAIL_NULL_MSG(attachment_node, "Attachment node path is invalid.");

			pinned_point->spatial_attachment = attachment_node;
			pinned_point->offset = (pinned_point->spatial_attachment->get_global_transform().affine_inverse() * get_global_transform()).xform(PhysicsServer3D::get_singleton()->soft_body_get_point_global_position(physics_rid, pinned_point->point_index));
		}
	}
}

void SoftBody3D::_reset_points_offsets() {
	if (!Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	const PinnedPoint *r = pinned_points.ptr();
	PinnedPoint *w = pinned_points.ptrw();
	for (int i = pinned_points.size() - 1; 0 <= i; --i) {
		if (!r[i].spatial_attachment) {
			if (!r[i].spatial_attachment_path.is_empty() && has_node(r[i].spatial_attachment_path)) {
				w[i].spatial_attachment = Object::cast_to<Node3D>(get_node(r[i].spatial_attachment_path));
			}
		}

		if (!r[i].spatial_attachment) {
			continue;
		}

		w[i].offset = (r[i].spatial_attachment->get_global_transform().affine_inverse() * get_global_transform()).xform(PhysicsServer3D::get_singleton()->soft_body_get_point_global_position(physics_rid, r[i].point_index));
	}
}

void SoftBody3D::_remove_pinned_point(int p_point_index) {
	const int id(_has_pinned_point(p_point_index));
	if (-1 != id) {
		pinned_points.remove_at(id);
	}
}

int SoftBody3D::_get_pinned_point(int p_point_index, SoftBody3D::PinnedPoint *&r_point) const {
	const int id = _has_pinned_point(p_point_index);
	if (-1 == id) {
		r_point = nullptr;
		return -1;
	} else {
		r_point = const_cast<SoftBody3D::PinnedPoint *>(&pinned_points.ptr()[id]);
		return id;
	}
}

int SoftBody3D::_has_pinned_point(int p_point_index) const {
	const PinnedPoint *r = pinned_points.ptr();
	for (int i = pinned_points.size() - 1; 0 <= i; --i) {
		if (p_point_index == r[i].point_index) {
			return i;
		}
	}
	return -1;
}
