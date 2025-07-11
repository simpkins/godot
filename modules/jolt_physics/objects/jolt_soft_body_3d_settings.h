/**************************************************************************/
/*  jolt_soft_body_3d_settings.h                                          */
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

#include "scene/resources/3d/soft_body_3d_settings.h"

#include "Jolt/Jolt.h"

#include "Jolt/Physics/SoftBody/SoftBodySharedSettings.h"

class JoltSoftBody3DVolume : public Resource {
	GDCLASS(JoltSoftBody3DVolume, Resource);

	Vector4i vertices;
	float six_rest_volume = 1.0f;
	float compliance = 0.0f;

protected:
	static void _bind_methods();

public:
	JoltSoftBody3DVolume();
	~JoltSoftBody3DVolume();

	void set_vertices(Vector4i p_vertices);
	Vector4i get_vertices() const;

	void set_six_rest_volume(float p_six_volume);
	float get_six_rest_volume() const;

	void set_compliance(float p_compliance);
	float get_compliance() const;
};

class JoltSoftBody3DSettings : public SoftBody3DSettings {
	GDCLASS(JoltSoftBody3DSettings, SoftBody3DSettings)

	Array volumes; // Array of JoltSoftBody3DVolume

protected:
	static void _bind_methods();

public:
	JoltSoftBody3DSettings();
	~JoltSoftBody3DSettings();

	Array get_volumes() const;
	void set_volumes(const Array &p_volumes);

	void calculate_volume_constraint_volumes(float multiplier = 1.0f);

	static JPH::Ref<JPH::SoftBodySharedSettings> convert_settings(const SoftBody3DSettings *p_settings);
};
