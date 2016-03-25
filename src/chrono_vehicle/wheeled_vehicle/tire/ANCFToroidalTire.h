// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Sample ANCF toroidal tire.
// This is a concrete ANCF tire class which uses a semi-toroidal tire mesh
// composed of single-layer ANCF shell elements.
//
// =============================================================================

#ifndef ANCF_TOROIDAL_TIRE_H
#define ANCF_TOROIDAL_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_fea/ChElementShellANCF.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// ANCF Toroidal Tire.
class CH_VEHICLE_API ANCFToroidalTire : public ChANCFTire {
  public:
    ANCFToroidalTire(const std::string& name);
    ~ANCFToroidalTire() {}

    virtual double GetRadius() const override { return m_rim_radius + m_height; }
    virtual double GetRimRadius() const override { return m_rim_radius; }
    virtual double GetWidth() const override { return 2 * m_height; }
    virtual double GetDefaultPressure() const override { return m_default_pressure; }
    virtual std::vector<std::shared_ptr<chrono::fea::ChNodeFEAbase>> GetConnectedNodes() const override;

    void SetRimRadius(double rimRadius_) { m_rim_radius = rimRadius_; }
    void SetHeight(double height_) { m_height = height_; }
    void SetThickness(double thickness_) { m_thickness = thickness_; }

    void SetDivCircumference(int divcirc_) { m_div_circumference = divcirc_; }
    void SetDivWidth(int divwidth_) { m_div_width = divwidth_; }

    void SetPressure(double pressure_) { m_default_pressure = pressure_; }
    void SetAlpha(double alpha_) { m_alpha = alpha_; }
    virtual void CreateMesh(const chrono::ChFrameMoving<>& wheel_frame, chrono::vehicle::VehicleSide side) override;

  private:
    double m_rim_radius;
    double m_height;
    double m_thickness;

    int m_div_circumference;
    int m_div_width;

    double m_default_pressure;
    double m_alpha;
};
/// @} vehicle_wheeled_tire
}  // end namespace vehicle
}  // end namespace chrono
#endif
