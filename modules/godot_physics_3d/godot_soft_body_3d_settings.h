/**************************************************************************/
/*  godot_soft_body_3d_settings.h                                         */
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

#include "core/math/vector3.h"
#include "core/templates/local_vector.h"

class SoftBody3DSettings;

class GodotSoftBody3DSettings {
public:
	struct Vertex {
		Vector3 position;
		Vector3 velocity;
		real_t inverse_mass = 1.0;
	};
	struct Edge {
		uint32_t v0 = 0;
		uint32_t v1 = 0;
		real_t rest_length = 1.0;
		real_t inv_linear_stiffness = 1.0;
	};
	struct Face {
		uint32_t v0 = 0;
		uint32_t v1 = 0;
		uint32_t v2 = 0;
	};

	LocalVector<Vertex> vertices;
	LocalVector<Edge> edges;
	LocalVector<Face> faces;

	GodotSoftBody3DSettings();
	~GodotSoftBody3DSettings();

	void initialize(const SoftBody3DSettings &p_settings);
};
