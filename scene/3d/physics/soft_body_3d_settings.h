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

class SoftBody3DVertex : public Resource {
	GDCLASS(SoftBody3DVertex, Resource);

	Vector3 position; // Initial position
	Vector3 velocity; // Initial velocity
	float inverse_mass = 1.0f;

protected:
	static void _bind_methods();

public:
	void set_inverse_mass(float p_inverse_mass);
	float get_inverse_mass() const;

	void set_position(Vector3 p_position);
	Vector3 get_position() const;

	void set_velocity(Vector3 p_velocity);
	Vector3 get_velocity() const;
};

class SoftBody3DEdgeConstraint : public Resource {
	GDCLASS(SoftBody3DEdgeConstraint, Resource);

	Vector2i vertices;
	// float rest_length = 1.0f;
	// float compliance = 0.0f;

protected:
	static void _bind_methods();

public:
	void set_vertices(Vector2i p_vertices);
	Vector2i get_vertices() const;
};

// SoftBody3DFaceConstraint defines a face in a soft body.
//
// Note: The Godot physics and Jolt physics implementations treat
// faces a bit differently.
// - Godot physics applies wind forces to faces, and pressure forces to vertices
// - Jolt physics applies pressure forces to faces.
class SoftBody3DFaceConstraint : public Resource {
	GDCLASS(SoftBody3DFaceConstraint, Resource);

	Vector3i vertices;

protected:
	static void _bind_methods();

public:
	void set_vertices(Vector3i p_vertices);
	Vector3i get_vertices() const;
};

class SoftBody3DSettings : public Resource {
	GDCLASS(SoftBody3DSettings, Resource);

	// Note: SoftBody3DSettings objects are not thread-safe, similar to Mesh.
	// The physics_rid field is updated without locking in the const get_rid() method.
	mutable RID physics_rid;

	Callable _subresource_changed;

	Array vertices; // Array of SoftBody3DVertex
	Array edges; // Array of SoftBody3DEdgeConstraint
	Array faces; // Array of SoftBody3DFaceConstraint

	template <typename T>
	int _set_array_helper(Array &member_variable, const Array &p_parameter);

	void _on_subresource_changed();

protected:
	static void _bind_methods();

public:
	Array get_vertices() const;
	void set_vertices(const Array &p_vertices);
	Array get_edges() const;
	void set_edges(const Array &p_edges);
	Array get_faces() const;
	void set_faces(const Array &p_faces);

	virtual RID get_rid() const override;

	SoftBody3DSettings();
	~SoftBody3DSettings();
};
