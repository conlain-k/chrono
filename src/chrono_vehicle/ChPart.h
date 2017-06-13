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
// Base class for all vehicle subsystems.
//
// =============================================================================

#ifndef CH_PART_H
#define CH_PART_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Base class for a vehicle subsystem.
/// It manages the part's name, visualization assets, and contact material.
class CH_VEHICLE_API ChPart {
  public:
    /// Construct a vehicle subsystem with the specified name.
    ChPart(const std::string& name  ///< [in] name of the subsystem
           );

    virtual ~ChPart() {}

    /// Get the name identifier for this subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const = 0;

    /// Set the visualization mode for this subsystem.
    void SetVisualizationType(VisualizationType vis);

    /// Add visualization assets to this subsystem, for the specified visualization mode.
    virtual void AddVisualizationAssets(VisualizationType vis) {}

    /// Remove all visualization assets from this subsystem.
    virtual void RemoveVisualizationAssets() {}

    /// Set coefficient of friction.
    /// The default value is 0.7
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set coefficient of restitution.
    /// The default value is 0.1
    void SetContactRestitutionCoefficient(float restitution_coefficient) { m_restitution = restitution_coefficient; }

    /// Set contact material properties.
    /// These values are used to calculate contact material coefficients (if the containing
    /// system is so configured and if the SMC contact method is being used).
    /// The default values are: Y = 2e5 and nu = 0.3
    void SetContactMaterialProperties(float young_modulus,  ///< [in] Young's modulus of elasticity
                                      float poisson_ratio   ///< [in] Poisson ratio
                                      );

    /// Set contact material coefficients.
    /// These values are used directly to compute contact forces (if the containing system
    /// is so configured and if the SMC contact method is being used).
    /// The default values are: kn=2e5, gn=40, kt=2e5, gt=20
    void SetContactMaterialCoefficients(float kn,  ///< [in] normal contact stiffness
                                        float gn,  ///< [in] normal contact damping
                                        float kt,  ///< [in] tangential contact stiffness
                                        float gt   ///< [in] tangential contact damping
                                        );

    /// Get coefficient of friction for contact material.
    float GetCoefficientFriction() const { return m_friction; }
    /// Get coefficient of restitution for contact material.
    float GetCoefficientRestitution() const { return m_restitution; }
    /// Get Young's modulus of elasticity for contact material.
    float GetYoungModulus() const { return m_young_modulus; }
    /// Get Poisson ratio for contact material.
    float GetPoissonRatio() const { return m_poisson_ratio; }
    /// Get normal stiffness coefficient for contact material.
    float GetKn() const { return m_kn; }
    /// Get tangential stiffness coefficient for contact material.
    float GetKt() const { return m_kt; }
    /// Get normal viscous damping coefficient for contact material.
    float GetGn() const { return m_gn; }
    /// Get tangential viscous damping coefficient for contact material.
    float GetGt() const { return m_gt; }

    /// Utility function for transforming inertia tensors between centroidal frames.
    /// It converts an inertia matrix specified in a centroidal frame aligned with the
    /// vehicle reference frame to an inertia matrix expressed in a centroidal body
    /// reference frame.
    static ChMatrix33<> TransformInertiaMatrix(
        const ChVector<>& moments,        ///< moments of inertia in vehicle-aligned centroidal frame
        const ChVector<>& products,       ///< products of inertia in vehicle-aligned centroidal frame
        const ChMatrix33<>& vehicle_rot,  ///< vehicle absolute orientation matrix
        const ChMatrix33<>& body_rot      ///< body absolute orientation matrix
        );

    /// Export this part's available output channels to the specified JSON object.
    /// The base class implementation only includes the part name.  Derived classes
    /// should override this function and first invoke the base class implementation.
    virtual void ExportOutputChannels(rapidjson::Document& jsonDocument) const;

  protected:
    static rapidjson::Value BodyOutputChannels(std::shared_ptr<ChBody> body,
                                               rapidjson::Document::AllocatorType& allocator);

    static rapidjson::Value JointOutputChannels(std::shared_ptr<ChLink> link,
                                                rapidjson::Document::AllocatorType& allocator);

    static rapidjson::Value MarkerOutputChannels(std::shared_ptr<ChMarker> marker,
                                                 rapidjson::Document::AllocatorType& allocator);

    std::string m_name;

    float m_friction;       ///< contact coefficient of friction
    float m_restitution;    ///< contact coefficient of restitution
    float m_young_modulus;  ///< contact material Young modulus
    float m_poisson_ratio;  ///< contact material Poisson ratio
    float m_kn;             ///< normal contact stiffness
    float m_gn;             ///< normal contact damping
    float m_kt;             ///< tangential contact stiffness
    float m_gt;             ///< tangential contact damping
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
