// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include "chrono_distributed/collision/ChCollisionModelDistributed.h"

using namespace chrono;
using namespace collision;

ChCollisionModelDistributed::ChCollisionModelDistributed()
{
	aabb_valid = false;
}

ChCollisionModelDistributed::~ChCollisionModelDistributed() {}


bool ChCollisionModelDistributed::AddBox(double hx, double hy, double hz,
                        const ChVector<>& pos, const ChMatrix33<>& rot)
{
	// Generate 8 corners of the box
	ChVector<> rx = rot * ChVector<>(hx,0,0);
	ChVector<> ry = rot * ChVector<>(0,hy,0);
	ChVector<> rz = rot * ChVector<>(0,0,hz);

	ChVector<> v[8]; // Vertices of collision box
	v[0] = pos + rx + ry + rz;
	v[1] = pos + rx + ry - rz;
	v[2] = pos + rx - ry + rz;
	v[3] = pos + rx - ry - rz;
	v[4] = pos - rx + ry + rz;
	v[5] = pos - rx + ry - rz;
	v[6] = pos - rx - ry + rz;
	v[7] = pos - rx - ry - rz;

	// If this is the first shape being added to the model,
	// set the first reference points
	if (!aabb_valid)
	{
		aabb_min.Set(v[0]);
		aabb_max.Set(v[0]);
		aabb_valid = true;
	}

	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (v[i][j] < aabb_min[j])
			{
				aabb_min[j] = v[i][j];
			}
			else if (v[i][j] > aabb_max[j])
			{
				aabb_max[j] = v[i][j];
			}
		}
	}

	return this->ChCollisionModelParallel::AddBox(hx, hy, hz, pos, rot);
}

bool ChCollisionModelDistributed::AddSphere(double radius, const ChVector<>& pos = ChVector<>())
{
	ChVector<double> max = pos + ChVector<double>(radius, radius, radius);
	ChVector<double> min = pos - ChVector<double>(radius, radius, radius);

	// If this is the first shape being added to the model,
	// set the first reference points
	if (!aabb_valid)
	{
		aabb_min.Set(min);
		aabb_max.Set(max);
		aabb_valid = true;
	}

	for (int i = 0; i < 3; i++)
	{
		if (min[i] < aabb_min[i])
		{
			aabb_min[i] = min[i];
		}
		if (max[i] > aabb_max[i])
		{
			aabb_max[i] = max[i];
		}
	}

	return this->ChCollisionModelParallel::AddSphere(radius, pos);
}

bool ChCollisionModelDistributed::AddTriangle(ChVector<> A, ChVector<> B, ChVector<> C, const ChVector<>& pos = ChVector<>(), const ChMatrix33<>& rot = ChMatrix33<>(1))
{
	// TODO
	return this->ChCollisionModelParallel::AddTriangle(A,B,C,pos,rot);
}

void ChCollisionModelDistributed::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const
{
	bbmin.Set(aabb_min);
	bbmax.Set(aabb_max);
}