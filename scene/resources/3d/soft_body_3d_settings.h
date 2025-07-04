/**************************************************************************/
/*  soft_body_3d_settings.h                                               */
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

#include "core/io/resource.h"

// Jolt physics uses compliance values (inverse stiffness): https://en.wikipedia.org/wiki/Stiffness
// Godot physics uses a "linear stiffness" coefficient between 0.0 and 1.0.
//
// This structure helps provide a (somewhat arbitrary) mapping between the two systems.
// This allows users that are targeting one specific physics engine to input values for that engine, while also
// providing a way to map from one to the other for users that want to be able to switch from one to the other.
class SoftBody3DStiffness {
	real_t value = 0.0;

public:
	real_t get_raw_value() const;
	void set_raw_value(real_t p_value);
	bool is_coefficient() const;

	real_t get_compliance() const;
	void set_compliance(real_t p_compliance);
	real_t get_stiffness_coefficient() const;
	void set_stiffness_coefficient(real_t p_stiffness);
};

class SoftBody3DVertex : public Resource {
	GDCLASS(SoftBody3DVertex, Resource);

	Vector3 position; // Initial position
	Vector3 velocity; // Initial velocity
	real_t inverse_mass = 1.0f;

protected:
	static void _bind_methods();

public:
	void set_inverse_mass(real_t p_inverse_mass);
	real_t get_inverse_mass() const;

	void set_position(Vector3 p_position);
	Vector3 get_position() const;

	void set_velocity(Vector3 p_velocity);
	Vector3 get_velocity() const;
};

class SoftBody3DEdge : public Resource {
	GDCLASS(SoftBody3DEdge, Resource);

	Vector2i vertices;
	real_t rest_length = 1.0f;
	SoftBody3DStiffness stiffness;

protected:
	static void _bind_methods();

	void _set_raw_compliance(real_t p_compliance);
	real_t _get_raw_compliance() const;

public:
	void set_vertices(Vector2i p_vertices);
	Vector2i get_vertices() const;
	void set_rest_length(real_t p_rest_length);
	real_t get_rest_length() const;
	void set_compliance(real_t p_compliance);
	real_t get_compliance() const;
	void set_stiffness_coefficient(real_t p_stiffness_coefficient);
	real_t get_stiffness_coefficient() const;
};

// SoftBody3DFace defines a face in a soft body.
//
// Note: The Godot physics and Jolt physics implementations treat
// faces a bit differently.
// - Godot physics applies wind forces to faces, and pressure forces to vertices
// - Jolt physics applies pressure forces to faces.
class SoftBody3DFace : public Resource {
	GDCLASS(SoftBody3DFace, Resource);

	Vector3i vertices;

protected:
	static void _bind_methods();

public:
	void set_vertices(Vector3i p_vertices);
	Vector3i get_vertices() const;
};

class SoftBody3DAutoConstraintSettings : public Resource {
	GDCLASS(SoftBody3DAutoConstraintSettings, Resource);

	SoftBody3DStiffness stiffness;
	real_t edge_length_multiplier = 1.0;

protected:
	static void _bind_methods();

	void _set_raw_compliance(real_t p_compliance);
	real_t _get_raw_compliance() const;

public:
	void set_edge_length_multiplier(real_t p_multiplier);
	real_t get_edge_length_multiplier() const;
	void set_compliance(real_t p_compliance);
	real_t get_compliance() const;
	void set_stiffness_coefficient(real_t p_stiffness_coefficient);
	real_t get_stiffness_coefficient() const;
};

class SoftBody3DSettings : public Resource {
	GDCLASS(SoftBody3DSettings, Resource);

	// Note: even const SoftBody3DSettings objects are not thread-safe, similar to Mesh.
	// The physics_rid field is updated without locking in the const get_rid() method.
	mutable RID physics_rid;

	Callable _subresource_changed;

	Array vertices; // Array of SoftBody3DVertex
	Array edges; // Array of SoftBody3DEdge
	Array faces; // Array of SoftBody3DFace

	Ref<SoftBody3DAutoConstraintSettings> auto_constraint_settings;

protected:
	static void _bind_methods();

	void _on_subresource_changed();

	template <typename T>
	int _set_array_helper(Array &member_variable, const Array &p_parameter);

public:
	Array get_vertices() const;
	void set_vertices(const Array &p_vertices);
	Array get_edges() const;
	void set_edges(const Array &p_edges);
	Array get_faces() const;
	void set_faces(const Array &p_faces);
	Ref<SoftBody3DAutoConstraintSettings> get_auto_constraint_settings() const;
	void set_auto_constraint_settings(const Ref<SoftBody3DAutoConstraintSettings> &p_settings);

	virtual RID get_rid() const override;

	SoftBody3DSettings();
	~SoftBody3DSettings();
};

template <typename T>
int SoftBody3DSettings::_set_array_helper(Array &member_variable, const Array &new_value) {
	_on_subresource_changed();

	for (const Variant &elem : member_variable) {
		const Ref<T> &typed_element = elem;
		if (typed_element.is_valid()) {
			typed_element->disconnect_changed(_subresource_changed);
		}
	}
	member_variable = new_value.duplicate(false);
	int num_invalid_elements = 0;
	for (Variant &elem : member_variable) {
		const Ref<T> &typed_element = elem;
		if (typed_element.is_valid()) {
			typed_element->connect_changed(_subresource_changed);
		} else {
			++num_invalid_elements;
		}
	}

	emit_changed();
	return num_invalid_elements;
}
