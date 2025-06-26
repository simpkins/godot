/**************************************************************************/
/*  soft_body_3d_settings.cpp                                             */
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

#include "soft_body_3d_settings.h"

#include "servers/physics_server_3d.h"

void SoftBody3DVertex::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_inverse_mass", "inverse_mass"), &SoftBody3DVertex::set_inverse_mass);
	ClassDB::bind_method(D_METHOD("get_inverse_mass"), &SoftBody3DVertex::get_inverse_mass);
	ClassDB::bind_method(D_METHOD("set_position", "position"), &SoftBody3DVertex::set_position);
	ClassDB::bind_method(D_METHOD("get_position"), &SoftBody3DVertex::get_position);
	ClassDB::bind_method(D_METHOD("set_velocity", "velocity"), &SoftBody3DVertex::set_velocity);
	ClassDB::bind_method(D_METHOD("get_velocity"), &SoftBody3DVertex::get_velocity);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inverse_mass"), "set_inverse_mass", "get_inverse_mass");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position"), "set_position", "get_position");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "velocity"), "set_velocity", "get_velocity");
}

void SoftBody3DVertex::set_inverse_mass(float p_inverse_mass) {
	if (p_inverse_mass == inverse_mass) {
		return;
	}
	inverse_mass = p_inverse_mass;
	emit_changed();
}

float SoftBody3DVertex::get_inverse_mass() const {
	return inverse_mass;
}

void SoftBody3DVertex::set_position(Vector3 p_position) {
	if (p_position == position) {
		return;
	}
	position = p_position;
	emit_changed();
}

Vector3 SoftBody3DVertex::get_position() const {
	return position;
}

void SoftBody3DVertex::set_velocity(Vector3 p_velocity) {
	if (p_velocity == velocity) {
		return;
	}
	velocity = p_velocity;
	emit_changed();
}

Vector3 SoftBody3DVertex::get_velocity() const {
	return velocity;
}

void SoftBody3DEdgeConstraint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_vertices", "vertices"), &SoftBody3DEdgeConstraint::set_vertices);
	ClassDB::bind_method(D_METHOD("get_vertices"), &SoftBody3DEdgeConstraint::get_vertices);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "vertices"), "set_vertices", "get_vertices");
}

void SoftBody3DEdgeConstraint::set_vertices(Vector2i p_vertices) {
	if (p_vertices == vertices) {
		return;
	}
	vertices = p_vertices;
	emit_changed();
}

Vector2i SoftBody3DEdgeConstraint::get_vertices() const {
	return vertices;
}

void SoftBody3DFaceConstraint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_vertices", "vertices"), &SoftBody3DFaceConstraint::set_vertices);
	ClassDB::bind_method(D_METHOD("get_vertices"), &SoftBody3DFaceConstraint::get_vertices);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "vertices"), "set_vertices", "get_vertices");
}

void SoftBody3DFaceConstraint::set_vertices(Vector3i p_vertices) {
	if (p_vertices == vertices) {
		return;
	}
	vertices = p_vertices;
	emit_changed();
}

Vector3i SoftBody3DFaceConstraint::get_vertices() const {
	return vertices;
}

void SoftBody3DSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_vertices", "vertices"), &SoftBody3DSettings::set_vertices);
	ClassDB::bind_method(D_METHOD("get_vertices"), &SoftBody3DSettings::get_vertices);
	ClassDB::bind_method(D_METHOD("set_edges", "edges"), &SoftBody3DSettings::set_edges);
	ClassDB::bind_method(D_METHOD("get_edges"), &SoftBody3DSettings::get_edges);
	ClassDB::bind_method(D_METHOD("set_faces", "faces"), &SoftBody3DSettings::set_faces);
	ClassDB::bind_method(D_METHOD("get_faces"), &SoftBody3DSettings::get_faces);

	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "vertices", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("SoftBody3DVertex")), "set_vertices", "get_vertices");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "edges", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("SoftBody3DEdgeConstraint")), "set_edges", "get_edges");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "faces", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("SoftBody3DFaceConstraint")), "set_faces", "get_faces");
}

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

Array SoftBody3DSettings::get_vertices() const {
	// We duplicate the array so users cannot add/remove entries
	// without us knowing about it.
	return vertices.duplicate(false);
}

void SoftBody3DSettings::set_vertices(const Array &p_vertices) {
	const int num_invalid_elements = _set_array_helper<SoftBody3DVertex>(vertices, p_vertices);
	ERR_FAIL_COND_MSG(num_invalid_elements != 0, "SoftBody3DSettings: invalid elements present in vertex array");
}

Array SoftBody3DSettings::get_edges() const {
	return edges.duplicate(false);
}

void SoftBody3DSettings::set_edges(const Array &p_edges) {
	const int num_invalid_elements = _set_array_helper<SoftBody3DEdgeConstraint>(edges, p_edges);
	ERR_FAIL_COND_MSG(num_invalid_elements != 0, "SoftBody3DSettings: invalid elements present in edge constraints array");
}

Array SoftBody3DSettings::get_faces() const {
	return faces.duplicate(false);
}

void SoftBody3DSettings::set_faces(const Array &p_faces) {
	const int num_invalid_elements = _set_array_helper<SoftBody3DFaceConstraint>(faces, p_faces);
	ERR_FAIL_COND_MSG(num_invalid_elements != 0, "SoftBody3DSettings: invalid elements present in face constraints array");
}

RID SoftBody3DSettings::get_rid() const {
	if (!physics_rid.is_valid()) {
		physics_rid = PhysicsServer3D::get_singleton()->soft_body_settings_create(this);
	}
	return physics_rid;
}

void SoftBody3DSettings::_on_subresource_changed() {
	// Soft body settings in the PhysicsServer3D are read-only once created.  We only create a physics_rid when we are
	// about to instantiate a soft body object with the current settings.  Multiple soft bodies can then be created with
	// this settings RID, but if any of our fields are modified we need to stop using this physics_rid and create a new
	// one the next time we instantiate a soft body using our settings.
	if (physics_rid.is_valid()) {
		PhysicsServer3D::get_singleton()->free(physics_rid);
		physics_rid = RID();
	}
}

SoftBody3DSettings::SoftBody3DSettings() :
		_subresource_changed(callable_mp(this, &SoftBody3DSettings::_on_subresource_changed)) {
}

SoftBody3DSettings::~SoftBody3DSettings() {
	if (physics_rid.is_valid()) {
		PhysicsServer3D::get_singleton()->free(physics_rid);
	}
}
