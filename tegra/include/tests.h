#ifndef TESTS_H_INCLUDED
#define TESTS_H_INCLUDED

#include "Camera.h"
#include "frame.h"
#include "ViewCombination.h"

void test_feature_matching(Frame *left, Frame *right);

void testGPUTriangulation(Camera const &cam, Frame *left, Frame *right);

void test_show_features_iteratively(ViewCombination const& view);
void test_show_features_colored(ViewCombination const& view, size_t n = 30, bool destroy = false);

#endif
