/**************************************************************************/
/*  test_soft_body_3d.cpp                                                 */
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

#include "tests/test_macros.h"

#include "scene/resources/3d/soft_body_3d_settings.h"

namespace TestSoftBody3D {
TEST_SUITE("[SoftBody3D]") {
	TEST_CASE("[SoftBody3DSettings] Compliance and linear_stiffness conversion") {
		Ref<SoftBody3DEdge> edge;
		edge.instantiate();
		CHECK_EQ(edge->get_compliance(), 0.0);
		CHECK_EQ(edge->get_stiffness_coefficient(), 1.0);

		// Test setting the compliance, and getting the value back
		edge->set_compliance(1.0);
		CHECK_EQ(edge->get_compliance(), 1.0);
		CHECK_NE(edge->get_stiffness_coefficient(), 1.0);
		CHECK_NE(edge->get_stiffness_coefficient(), 0.0);
		edge->set_compliance(100.0);
		CHECK_EQ(edge->get_compliance(), 100.0);
		edge->set_compliance(0.0);
		CHECK_EQ(edge->get_compliance(), 0.0);
		CHECK_EQ(edge->get_stiffness_coefficient(), 1.0);
		edge->set_compliance(-0.0);
		CHECK_EQ(edge->get_compliance(), 0.0);

		// We don't allow storing compliance values less than 0
		edge->set_compliance(-1.0);
		CHECK_EQ(edge->get_compliance(), 0.0);
		CHECK_EQ(edge->get_stiffness_coefficient(), 1.0);

		// Test setting the stiffness coefficient, and getting the value back
		edge->set_stiffness_coefficient(0.0);
		CHECK_EQ(edge->get_stiffness_coefficient(), 0.0);
		// stiffness of 0 corresponds to a very large compliance value
		CHECK_GT(edge->get_compliance(), 50000.0);
		edge->set_stiffness_coefficient(1.0);
		CHECK_EQ(edge->get_stiffness_coefficient(), 1.0);
		// Note: setting the stiffness_coefficient to 1 results in a compliance that small, but not 0.
		// (Perhaps we should change the mapping so that 1.0 does convert to a compliance of 0.0?)
		CHECK_LT(edge->get_compliance(), 0.0001);
		edge->set_stiffness_coefficient(0.5);
		CHECK_EQ(edge->get_stiffness_coefficient(), 0.5);
		edge->set_stiffness_coefficient(-0.0);
		CHECK_EQ(edge->get_stiffness_coefficient(), 0.0);

		// We don't allow storing stiffness coefficient values less than 0
		edge->set_stiffness_coefficient(-1.0);
		CHECK_EQ(edge->get_stiffness_coefficient(), 0.0);
		CHECK_GT(edge->get_compliance(), 50000.0);

		// Test that round-tripping a value from compliance -> stiffness_coefficient -> compliance
		// returns roughly the same value
		edge->set_compliance(0.5);
		edge->set_stiffness_coefficient(edge->get_stiffness_coefficient());
		CHECK(edge->get_compliance() == doctest::Approx(0.5));

		// Test round-tripping the other way
		edge->set_stiffness_coefficient(0.6789);
		edge->set_compliance(edge->get_compliance());
		CHECK(edge->get_stiffness_coefficient() == doctest::Approx(0.6789));
	}
}
} // namespace TestSoftBody3D
