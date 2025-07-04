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

real_t SoftBody3DStiffness::get_raw_value() const {
	return value;
}

void SoftBody3DStiffness::set_raw_value(real_t p_value) {
	value = p_value;
}

bool SoftBody3DStiffness::is_coefficient() const {
	// Rather than always converting the value either to a stiffness_coefficient or a compliance
	// value, we store the data as it was given to us so that the value can be retrieved losslessly
	// if the caller is always operating with the same API type.
	//
	// Neither stiffness_coefficient nor compliance is expected to be negative, so we use the sign
	// bit to track whether the stored value is compliance or stiffness_coefficient.
	return signbit(value);
}

real_t SoftBody3DStiffness::get_compliance() const {
	if (is_coefficient()) {
		// We use the mapping calculation from JoltSoftBody3D
		const real_t coefficient = CLAMP(-value, 0.0f, 1.0f);
		const real_t stiffness = MAX(Math::pow(coefficient, (real_t)3.0) * 100000.0, 0.000001);
		return 1.0 / stiffness;
	} else {
		return value;
	}
}

void SoftBody3DStiffness::set_compliance(real_t p_compliance) {
	value = (p_compliance <= 0 ? 0.0 : p_compliance);
}

real_t SoftBody3DStiffness::get_stiffness_coefficient() const {
	if (is_coefficient()) {
		return -value;
	} else {
		if (value <= 0.000001f) {
			return 1.0;
		}
		return cbrt(0.00001 / value);
	}
}

void SoftBody3DStiffness::set_stiffness_coefficient(real_t p_stiffness) {
	value = (p_stiffness <= 0 ? -0.0 : -p_stiffness);
}

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

void SoftBody3DVertex::set_inverse_mass(real_t p_inverse_mass) {
	if (p_inverse_mass == inverse_mass) {
		return;
	}
	inverse_mass = p_inverse_mass;
	emit_changed();
}

real_t SoftBody3DVertex::get_inverse_mass() const {
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

void SoftBody3DEdge::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_vertices", "vertices"), &SoftBody3DEdge::set_vertices);
	ClassDB::bind_method(D_METHOD("get_vertices"), &SoftBody3DEdge::get_vertices);
	ClassDB::bind_method(D_METHOD("set_rest_length", "rest_length"), &SoftBody3DEdge::set_rest_length);
	ClassDB::bind_method(D_METHOD("get_rest_length"), &SoftBody3DEdge::get_rest_length);
	ClassDB::bind_method(D_METHOD("set_compliance", "compliance"), &SoftBody3DEdge::set_compliance);
	ClassDB::bind_method(D_METHOD("get_compliance"), &SoftBody3DEdge::get_compliance);
	ClassDB::bind_method(D_METHOD("set_stiffness_coefficient", "stiffness_coefficient"), &SoftBody3DEdge::set_stiffness_coefficient);
	ClassDB::bind_method(D_METHOD("get_stiffness_coefficient"), &SoftBody3DEdge::get_stiffness_coefficient);
	ClassDB::bind_method(D_METHOD("_set_raw_compliance", "raw_compliance"), &SoftBody3DEdge::_set_raw_compliance);
	ClassDB::bind_method(D_METHOD("_get_raw_compliance"), &SoftBody3DEdge::_get_raw_compliance);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2I, "vertices"), "set_vertices", "get_vertices");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rest_length"), "set_rest_length", "get_rest_length");

	// compliance, stiffness_coefficient, and raw_compliance are all related.
	// compliance and stiffness_coefficient are for exposing via the API,
	// but raw_compliance is what get saved and loaded when serializing the resource.
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "compliance", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR),
			"set_compliance", "get_compliance");
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "stiffness_coefficient", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR),
			"set_stiffness_coefficient", "get_stiffness_coefficient");
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "raw_compliance", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_STORAGE | PROPERTY_USAGE_INTERNAL),
			"_set_raw_compliance", "_get_raw_compliance");
}

void SoftBody3DEdge::set_vertices(Vector2i p_vertices) {
	if (p_vertices == vertices) {
		return;
	}
	vertices = p_vertices;
	emit_changed();
}

Vector2i SoftBody3DEdge::get_vertices() const {
	return vertices;
}

void SoftBody3DEdge::set_rest_length(real_t p_rest_length) {
	rest_length = p_rest_length;
	emit_changed();
}

real_t SoftBody3DEdge::get_rest_length() const {
	return rest_length;
}

void SoftBody3DEdge::set_compliance(real_t p_compliance) {
	stiffness.set_compliance(p_compliance);
	emit_changed();
}

real_t SoftBody3DEdge::get_compliance() const {
	return stiffness.get_compliance();
}

void SoftBody3DEdge::set_stiffness_coefficient(real_t p_stiffness_coefficient) {
	stiffness.set_stiffness_coefficient(p_stiffness_coefficient);
	emit_changed();
}

real_t SoftBody3DEdge::get_stiffness_coefficient() const {
	return stiffness.get_stiffness_coefficient();
}

void SoftBody3DEdge::_set_raw_compliance(real_t p_compliance) {
	stiffness.set_raw_value(p_compliance);
	emit_changed();
}

real_t SoftBody3DEdge::_get_raw_compliance() const {
	return stiffness.get_raw_value();
}

void SoftBody3DFace::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_vertices", "vertices"), &SoftBody3DFace::set_vertices);
	ClassDB::bind_method(D_METHOD("get_vertices"), &SoftBody3DFace::get_vertices);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3I, "vertices"), "set_vertices", "get_vertices");
}

void SoftBody3DFace::set_vertices(Vector3i p_vertices) {
	if (p_vertices == vertices) {
		return;
	}
	vertices = p_vertices;
	emit_changed();
}

Vector3i SoftBody3DFace::get_vertices() const {
	return vertices;
}

void SoftBody3DAutoConstraintSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_edge_length_multiplier", "multiplier"), &SoftBody3DAutoConstraintSettings::set_edge_length_multiplier);
	ClassDB::bind_method(D_METHOD("get_edge_length_multiplier"), &SoftBody3DAutoConstraintSettings::get_edge_length_multiplier);
	ClassDB::bind_method(D_METHOD("set_compliance", "compliance"), &SoftBody3DAutoConstraintSettings::set_compliance);
	ClassDB::bind_method(D_METHOD("get_compliance"), &SoftBody3DAutoConstraintSettings::get_compliance);
	ClassDB::bind_method(D_METHOD("set_stiffness_coefficient", "stiffness_coefficient"), &SoftBody3DAutoConstraintSettings::set_stiffness_coefficient);
	ClassDB::bind_method(D_METHOD("get_stiffness_coefficient"), &SoftBody3DAutoConstraintSettings::get_stiffness_coefficient);
	ClassDB::bind_method(D_METHOD("_set_raw_compliance", "raw_compliance"), &SoftBody3DAutoConstraintSettings::_set_raw_compliance);
	ClassDB::bind_method(D_METHOD("_get_raw_compliance"), &SoftBody3DAutoConstraintSettings::_get_raw_compliance);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "edge_length_multiplier", PROPERTY_HINT_RANGE, "-1,1,0.01,or_less,or_greater"), "set_edge_length_multiplier", "get_edge_length_multiplier");
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "compliance", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR),
			"set_compliance", "get_compliance");
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "stiffness_coefficient", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR),
			"set_stiffness_coefficient", "get_stiffness_coefficient");
	ADD_PROPERTY(
			PropertyInfo(Variant::FLOAT, "raw_compliance", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_STORAGE | PROPERTY_USAGE_INTERNAL),
			"_set_raw_compliance", "_get_raw_compliance");
}

void SoftBody3DAutoConstraintSettings::set_edge_length_multiplier(real_t p_multiplier) {
	edge_length_multiplier = p_multiplier;
	emit_changed();
}

real_t SoftBody3DAutoConstraintSettings::get_edge_length_multiplier() const {
	return edge_length_multiplier;
}

void SoftBody3DAutoConstraintSettings::set_compliance(real_t p_compliance) {
	stiffness.set_compliance(p_compliance);
	emit_changed();
}

real_t SoftBody3DAutoConstraintSettings::get_compliance() const {
	return stiffness.get_compliance();
}

void SoftBody3DAutoConstraintSettings::set_stiffness_coefficient(real_t p_stiffness_coefficient) {
	stiffness.set_stiffness_coefficient(p_stiffness_coefficient);
	emit_changed();
}

real_t SoftBody3DAutoConstraintSettings::get_stiffness_coefficient() const {
	return stiffness.get_stiffness_coefficient();
}

void SoftBody3DAutoConstraintSettings::_set_raw_compliance(real_t p_compliance) {
	stiffness.set_raw_value(p_compliance);
	emit_changed();
}

real_t SoftBody3DAutoConstraintSettings::_get_raw_compliance() const {
	return stiffness.get_raw_value();
}

void SoftBody3DSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_vertices", "vertices"), &SoftBody3DSettings::set_vertices);
	ClassDB::bind_method(D_METHOD("get_vertices"), &SoftBody3DSettings::get_vertices);
	ClassDB::bind_method(D_METHOD("set_edges", "edges"), &SoftBody3DSettings::set_edges);
	ClassDB::bind_method(D_METHOD("get_edges"), &SoftBody3DSettings::get_edges);
	ClassDB::bind_method(D_METHOD("set_faces", "faces"), &SoftBody3DSettings::set_faces);
	ClassDB::bind_method(D_METHOD("get_faces"), &SoftBody3DSettings::get_faces);
	ClassDB::bind_method(D_METHOD("set_auto_constraint_settings", "settings"), &SoftBody3DSettings::set_auto_constraint_settings);
	ClassDB::bind_method(D_METHOD("get_auto_constraint_settings"), &SoftBody3DSettings::get_auto_constraint_settings);

	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "vertices", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("SoftBody3DVertex")), "set_vertices", "get_vertices");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "edges", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("SoftBody3DEdge")), "set_edges", "get_edges");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "faces", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("SoftBody3DFace")), "set_faces", "get_faces");
	ADD_PROPERTY(
			PropertyInfo(Variant::OBJECT, "auto_constraint_settings", PROPERTY_HINT_RESOURCE_TYPE, "SoftBody3DAutoConstraintSettings"),
			"set_auto_constraint_settings", "get_auto_constraint_settings");
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
	const int num_invalid_elements = _set_array_helper<SoftBody3DEdge>(edges, p_edges);
	ERR_FAIL_COND_MSG(num_invalid_elements != 0, "SoftBody3DSettings: invalid elements present in edge constraints array");
}

Array SoftBody3DSettings::get_faces() const {
	return faces.duplicate(false);
}

void SoftBody3DSettings::set_faces(const Array &p_faces) {
	const int num_invalid_elements = _set_array_helper<SoftBody3DFace>(faces, p_faces);
	ERR_FAIL_COND_MSG(num_invalid_elements != 0, "SoftBody3DSettings: invalid elements present in face constraints array");
}

Ref<SoftBody3DAutoConstraintSettings> SoftBody3DSettings::get_auto_constraint_settings() const {
	return auto_constraint_settings;
}

void SoftBody3DSettings::set_auto_constraint_settings(const Ref<SoftBody3DAutoConstraintSettings> &p_settings) {
	if (auto_constraint_settings.is_valid()) {
		auto_constraint_settings->disconnect_changed(_subresource_changed);
	}
	auto_constraint_settings = p_settings;
	if (auto_constraint_settings.is_valid()) {
		auto_constraint_settings->connect_changed(_subresource_changed);
	}
	_on_subresource_changed();
}

RID SoftBody3DSettings::get_rid() const {
	if (!physics_rid.is_valid()) {
		// TODO: Call soft_body_settings_create() once it has been added to the API
		// physics_rid = PhysicsServer3D::get_singleton()->soft_body_settings_create(this);
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
