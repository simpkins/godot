/**************************************************************************/
/*  soft_body_3d.h                                                        */
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

#pragma once

#include "scene/3d/mesh_instance_3d.h"

class PhysicsBody3D;
class PhysicsDirectSoftBodyState3D;

class SoftBody3D : public MeshInstance3D {
	GDCLASS(SoftBody3D, MeshInstance3D);

public:
	enum DisableMode {
		DISABLE_MODE_REMOVE,
		DISABLE_MODE_KEEP_ACTIVE,
	};

	enum ShadingMode {
		SHADING_SMOOTH,
		SHADING_FLAT,
	};

	struct PinnedPoint {
		int point_index = -1;
		NodePath spatial_attachment_path;
		Node3D *spatial_attachment = nullptr; // Cache
		Vector3 offset;

		PinnedPoint();
		PinnedPoint(const PinnedPoint &obj_tocopy);
		void operator=(const PinnedPoint &obj);
	};

private:
	// Information used to update the mesh vertex buffer in the RenderingServer
	class BufferData {
		RID mesh;
		int surface = 0;
		AABB aabb_prev;
		AABB aabb_curr;
		Vector<uint8_t> buffer[2];
		Vector<uint8_t> *buffer_prev = nullptr;
		Vector<uint8_t> *buffer_curr = nullptr;
		Vector<uint8_t> buffer_interp;
		uint32_t vertex_count = 0;
		uint32_t stride = 0;
		uint32_t normal_stride = 0;
		uint32_t offset_vertices = 0;
		uint32_t offset_normal = 0;

		// last_update_frame tracks the last physics frame where we
		// received a body_state_changed() call from the physics server,
		// when interpolation is enabled.
		uint64_t last_update_frame = 0;

		AABB aabb_last;

		ShadingMode shading_mode = SoftBody3D::SHADING_SMOOTH;

		// A mapping from the render mesh's vertex indices
		// to physics vertex indices.
		LocalVector<int> mesh_to_physics;
		// A vector to use for temporarily storing vertex normals in recompute_normals() while
		// we average normals. We store this as a member variable simply to avoid having to
		// re-allocate it each frame.
		LocalVector<Vector3> normal_compute_buffer;
		uint32_t physics_vertex_count = 0;
		PackedInt32Array face_indices;

		void _recompute_normals(uint8_t *p_buffer);
		void _fti_pump();

	public:
		BufferData();
		void prepare(RID p_mesh, int p_surface, const PackedVector3Array &p_vertices, const PackedInt32Array &p_face_indices);
		void clear();
		void compute_physics_vertex_mapping(const PackedVector3Array &p_vertices);
		void recompute_normals();
		void do_physics_interpolation(real_t p_interpolation_fraction);
		void body_state_changed(PhysicsDirectSoftBodyState3D *p_state, bool p_interpolation_enabled);
		void disable_physics_interpolation_until_next_update();

		void move_point(int index, const Vector3 &position);
		void commit_mesh_changes();

		bool has_mesh() const {
			return mesh != RID();
		}

		ShadingMode get_shading_mode() const;
		void set_shading_mode(ShadingMode p_shading_mode);
	};

	BufferData buffer_data;

	RID physics_rid;

	DisableMode disable_mode = DISABLE_MODE_REMOVE;

	uint32_t collision_mask = 1;
	uint32_t collision_layer = 1;
	NodePath parent_collision_ignore;
	Vector<PinnedPoint> pinned_points;
	bool pinned_points_cache_dirty = true;

	// The simulation is active when all of the following are true:
	// - we have a valid mesh set
	// - we are in the scene tree
	// - we are enabled, or disable_mode is DISABLE_MODE_KEEP_ACTIVE
	bool simulation_active = false;

	bool capture_input_on_drag = false;
	bool ray_pickable = true;

	Ref<ArrayMesh> debug_mesh_cache;
	class MeshInstance3D *debug_mesh = nullptr;

	void _update_pickable();

	void _update_pinned_points();
	void _do_physics_interpolation(real_t p_interpolation_fraction);
	void _body_state_changed(PhysicsDirectSoftBodyState3D *p_state);

	void _update_simulation_active();
	void _process_set_mesh(const Ref<Mesh> &_mesh);
	void _convert_mesh(const Ref<Mesh> &p_mesh);
	void _update_mesh_arrays_with_transform(Array &surface_arrays, uint32_t surface_format);

protected:
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;

	bool _set_property_pinned_points_indices(const Array &p_indices);
	bool _set_property_pinned_points_attachment(int p_item, const String &p_what, const Variant &p_value);
	bool _get_property_pinned_points(int p_item, const String &p_what, Variant &r_ret) const;

	void _notification(int p_what);
	static void _bind_methods();

	void _physics_interpolated_changed() override;

#ifndef DISABLE_DEPRECATED
	void _pin_point_bind_compat_94684(int p_point_index, bool pin, const NodePath &p_spatial_attachment_path = NodePath());
	static void _bind_compatibility_methods();
#endif

	PackedStringArray get_configuration_warnings() const override;

public:
	RID get_physics_rid() const { return physics_rid; }

	virtual void set_mesh(const Ref<Mesh> &p_mesh) override;

	void set_collision_mask(uint32_t p_mask);
	uint32_t get_collision_mask() const;

	void set_collision_layer(uint32_t p_layer);
	uint32_t get_collision_layer() const;

	void set_collision_layer_value(int p_layer_number, bool p_value);
	bool get_collision_layer_value(int p_layer_number) const;

	void set_collision_mask_value(int p_layer_number, bool p_value);
	bool get_collision_mask_value(int p_layer_number) const;

	void set_disable_mode(DisableMode p_mode);
	DisableMode get_disable_mode() const;

	void set_shading_mode(ShadingMode p_mode);
	ShadingMode get_shading_mode() const;

	void set_parent_collision_ignore(const NodePath &p_parent_collision_ignore);
	const NodePath &get_parent_collision_ignore() const;

	void set_pinned_points_indices(Vector<PinnedPoint> p_pinned_points_indices);
	Vector<PinnedPoint> get_pinned_points_indices();

	void set_simulation_precision(int p_simulation_precision);
	int get_simulation_precision();

	void set_total_mass(real_t p_total_mass);
	real_t get_total_mass();

	void set_linear_stiffness(real_t p_linear_stiffness);
	real_t get_linear_stiffness();

	void set_shrinking_factor(real_t p_shrinking_factor);
	real_t get_shrinking_factor();

	void set_pressure_coefficient(real_t p_pressure_coefficient);
	real_t get_pressure_coefficient();

	void set_damping_coefficient(real_t p_damping_coefficient);
	real_t get_damping_coefficient();

	void set_drag_coefficient(real_t p_drag_coefficient);
	real_t get_drag_coefficient();

	TypedArray<PhysicsBody3D> get_collision_exceptions();
	void add_collision_exception_with(Node *p_node);
	void remove_collision_exception_with(Node *p_node);

	Vector3 get_point_transform(int p_point_index);

	void pin_point_toggle(int p_point_index);
	void pin_point(int p_point_index, bool pin, const NodePath &p_spatial_attachment_path = NodePath(), int p_insert_at = -1);
	bool is_point_pinned(int p_point_index) const;

	void _pin_point_deferred(int p_point_index, bool pin, const NodePath p_spatial_attachment_path);

	void set_ray_pickable(bool p_ray_pickable);
	bool is_ray_pickable() const;

	void apply_impulse(int p_point_index, const Vector3 &p_impulse);
	void apply_force(int p_point_index, const Vector3 &p_force);
	void apply_central_impulse(const Vector3 &p_impulse);
	void apply_central_force(const Vector3 &p_force);

	SoftBody3D();
	~SoftBody3D();

private:
	void _make_cache_dirty();
	void _update_cache_pin_points_datas();

	void _pin_point_on_physics_server(int p_point_index, bool pin);
	void _add_pinned_point(int p_point_index, const NodePath &p_spatial_attachment_path, int p_insert_at = -1);
	void _reset_points_offsets();

	void _remove_pinned_point(int p_point_index);
	int _get_pinned_point(int p_point_index, PinnedPoint *&r_point) const;
	int _has_pinned_point(int p_point_index) const;
};

VARIANT_ENUM_CAST(SoftBody3D::DisableMode);
VARIANT_ENUM_CAST(SoftBody3D::ShadingMode);
