// FROZEN verbatim copy of CTRL::PhysicsBodySim (ExVectrControl 2c0c00e, lib/ExVectrControl/{include,src}/physics_bodysim.*),
// namespace renamed to legacy_frozen, member functions made inline (header-only), simulateForTime removed (needs
// ExVectrCore). Golden reference for the ExVectrDynamics equivalence tests: do NOT edit or "fix" this file.
#ifndef FROZEN_LEGACY_PHYSICS_BODYSIM_HPP
#define FROZEN_LEGACY_PHYSICS_BODYSIM_HPP
#include <math.h>
#include <stdint.h>


#include "ExVectrMath.hpp"


namespace VCTR
{

    namespace legacy_frozen
    {

        /**
         * @brief Rigid-body integrator for forces/torques on a body (6 DOF).
         *
         * Integration (since 2026-11-21, feature/sim-aero): semi-implicit (symplectic) Euler. The force and torque
         * sums of the step are held constant; first v and omega are advanced, then p with the new v and q with the
         * new omega. The rotation uses the full Euler equation I*domega/dt = tau - omega x (I*omega) with the full
         * inertia tensor (products of inertia allowed). Pure code (no scheduler, no wall clock): the caller chooses
         * the step, so a fixed step gives bit-identical runs.
         */
        class PhysicsBodySim 
        {
        protected:

            Math::Vector<float, 3> forceSumBody_; // Sum of forces in body frame
            Math::Vector<float, 3> torqueSumBody_; // Sum of torques in body frame

            Math::Vector<float, 3> forceSumNav_; // Sum of forces in Nav frame
            Math::Vector<float, 3> torqueSumNav_; // Sum of torques in Nav frame

            Math::Vector<float, 6> positionState_; // Position of the body in reference frame
            Math::Vector<float, 7> attitudeState_; // Attitude of the body in reference frame

            float mass_; // Mass of the body in kg
            Math::Matrix<float, 3, 3> inertiaTensor_; // Inertia tensor of the body in kg*m^2
            Math::Matrix<float, 3, 3> inertiaInverse_; // Cached inverse of inertiaTensor_


        public:
            
            /**
             * @brief Standard constructor. Sets state to 0 and covariance to 1000 as starting values.
             */
            PhysicsBodySim(float mass, const Math::Matrix<float, 3, 3>& inertiaTensor);

            /**
             * @brief adds a force to a body at a given position.
             * @param force The force to add in body frame.
             * @param position The position to add the force at in body frame.
             * @param bodyFrame If true, then the force is added in body frame. If false, then the force is added in reference frame (World frame).
             * @param isAccel If true, then the force is added as an acceleration (Ignoring mass).
             */
            void addForce(const Math::Vector<float, 3>& force, const Math::Vector<float, 3>& position = 0, bool bodyFrame = true, bool isAccel = false);

            /**
             * @brief adds a torque to a body at a given position.
             * @param torque The torque to add in body frame.
             * @param position The position to add the torque at in body frame.
             * @param bodyFrame If true, then the torque is added in body frame. If false, then the torque is added in reference frame (World frame).
             */
            void addTorque(const Math::Vector<float, 3>& torque, const Math::Vector<float, 3>& position = 0, bool bodyFrame = true, bool isAccel = false);

            /**
             * @brief Clears the forces and torques applied to the body.
             * @param force If true, then the forces are cleared.
             * @param torque If true, then the torques are cleared.
             */
            void clearForces(bool force = true, bool torque = true);

            void setMass(float mass) { mass_ = mass; } // Set the mass of the body in kg
            /// Sets the inertia tensor about the centre of mass in body axes, kg*m^2 (its inverse is cached).
            void setInertiaTensor(const Math::Matrix<float, 3, 3>& inertiaTensor);

            /**
             * @brief Sets the position state [V, P] of the body in reference frame.
             * @param positionState The position state to set in reference frame.
             */
            void setPositionState(const Math::Vector<float, 6>& positionState) { positionState_ = positionState; }
            
            void setPosition(const Math::Vector<float, 3>& position) { positionState_.block(position, 3, 0); } // Set the position of the body in reference frame
            void setVelocity(const Math::Vector<float, 3>& velocity) { positionState_.block(velocity); } // Set the velocity of the body in reference frame

            /**
             * @brief Sets the attitude state [W, Q] of the body in reference frame.
             * @param attitudeState The attitude state to set in reference frame.
             */
            void setAttitudeState(const Math::Vector<float, 7>& attitudeState) { attitudeState_ = attitudeState; }

            void setAttitude(const Math::Quat<float>& attitude) { attitudeState_.block(attitude, 3, 0); } // Set the attitude of the body in reference frame
            void setAngularVelocity(const Math::Vector<float, 3>& angularVelocity) { attitudeState_.block(angularVelocity); } // Set the angular velocity of the body in reference frame

            /**
             * @returns the current position state of the body in reference frame as [V, P].
             */
            Math::Vector<float, 6>& getPositionState() { return positionState_; } // Get the position state in reference frame

            Math::Vector<float, 3> getPosition() { return positionState_.block<3, 1>(3); } // Get the position of the body in reference frame
            Math::Vector<float, 3> getVelocity() { return positionState_.block<3, 1>(0); } // Get the velocity of the body in reference frame

            /**
             * @returns the current attitude state of the body in reference frame as [W, Q].
             */
            Math::Vector<float, 7>& getAttitudeState() { return attitudeState_; } // Get the attitude state in reference frame

            Math::Quat<float> getAttitude() { return attitudeState_.block<4, 1>(3); } // Get the attitude of the body in reference frame
            Math::Vector<float, 3> getAngularVelocity() { return attitudeState_.block<3, 1>(0); } // Get the angular velocity of the body in reference frame

            /**
             * @returns the current force sum of the body in body frame.
             */
            Math::Vector<float, 3>& getBodyForceSum() { return forceSumBody_; }
            Math::Vector<float, 3>& getNavForceSum() { return forceSumNav_; } // Get the force sum in reference frame

            /**
             * @returns the current torque sum of the body in body frame.
             */
            Math::Vector<float, 3>& getBodyTorqueSum() { return torqueSumBody_; }
            Math::Vector<float, 3>& getNavTorqueSum() { return torqueSumNav_; } // Get the torque sum in reference frame


            Math::Matrix<float, 3, 3> getInertiaTensor() { return inertiaTensor_; } // Get the inertia tensor of the body
            float getMass() { return mass_; } // Get the mass of the body

            /**
             * @brief Advances the state by one step of dt seconds with the current force and torque sums
             * (semi-implicit Euler, see the class comment). dt <= 0 or non-finite does nothing.
             */
            void step(float dt);


        private:

            

        };

    }
    
}


namespace VCTR { namespace legacy_frozen {


inline PhysicsBodySim::PhysicsBodySim(float mass,
                               const Math::Matrix<float, 3, 3> &inertiaTensor) {
  mass_ = mass;
  setInertiaTensor(inertiaTensor);

  positionState_ = {0, 0, 0, 0, 0, 0};
  attitudeState_ = {0, 0, 0, 1, 0, 0, 0};

  forceSumBody_ = 0;
  torqueSumBody_ = 0;
  forceSumNav_ = 0;
  torqueSumNav_ = 0;
}

inline void PhysicsBodySim::clearForces(bool force, bool torque) {
  if (force) {
    forceSumBody_ = 0;
    forceSumNav_ = 0;
  }
  if (torque) {
    torqueSumBody_ = 0;
    torqueSumNav_ = 0;
  }
}

inline void PhysicsBodySim::addForce(const Math::Vector<float, 3> &force,
                              const Math::Vector<float, 3> &position,
                              bool bodyFrame, bool isAccel) {
  auto forceTrue = force; // The force vector in body frame

  // Lets start by converting to a force if its an acceleration
  if (isAccel) {
    forceTrue = force * mass_; // Convert to force by multiplying with mass
  }

  if (bodyFrame) {
    forceSumBody_ = forceSumBody_ + forceTrue;
  } else {
    forceSumNav_ = forceSumNav_ + forceTrue;
  }

  // Now we need to add the force to the torque vector as well. We do this by
  // calculating the torque vector from the force vector and the position
  // vector.

  auto torque = position.cross(forceTrue);
  if (bodyFrame) {
    torqueSumBody_ = torqueSumBody_ + torque;
  } else {
    torqueSumNav_ = torqueSumNav_ + torque;
  }

  // LOG_MSG("Force body sum: %.2f %.2f %.2f\n", forceSumBody_(0),
  // forceSumBody_(1), forceSumBody_(2)); // Print the force vector to the
  // console LOG_MSG("Force nav sum: %.2f %.2f %.2f\n", forceSumNav_(0),
  // forceSumNav_(1), forceSumNav_(2)); // Print the force vector to the console
  // LOG_MSG("Torque body sum: %.2f %.2f %.2f\n", torqueSumBody_(0),
  // torqueSumBody_(1), torqueSumBody_(2)); // Print the torque vector to the
  // console LOG_MSG("Torque nav sum: %.2f %.2f %.2f\n", torqueSumNav_(0),
  // torqueSumNav_(1), torqueSumNav_(2)); // Print the torque vector to the
  // console
}

inline void PhysicsBodySim::addTorque(const Math::Vector<float, 3> &torque,
                               const Math::Vector<float, 3> &position,
                               bool bodyFrame, bool isAccel) {
  (void)position; // a pure torque is independent of where it acts
  auto torqueTrue = torque; // The torque vector in body frame

  // Lets start by converting to a force if its an acceleration
  if (isAccel) {
    torqueTrue = torque * mass_; // Convert to force by multiplying with mass
  }

  if (bodyFrame) {
    torqueSumBody_ = torqueSumBody_ + torqueTrue;
  } else {
    torqueSumNav_ = torqueSumNav_ + torqueTrue;
  }

  // LOG_MSG("Torque body sum: %.2f %.2f %.2f\n", torqueSumBody_(0),
  // torqueSumBody_(1), torqueSumBody_(2)); // Print the torque vector to the
  // console LOG_MSG("Torque nav sum: %.2f %.2f %.2f\n", torqueSumNav_(0),
  // torqueSumNav_(1), torqueSumNav_(2)); // Print the torque vector to the
  // console
}

inline void PhysicsBodySim::setInertiaTensor(
    const Math::Matrix<float, 3, 3> &inertiaTensor) {
  inertiaTensor_ = inertiaTensor;
  inertiaInverse_ = inertiaTensor.inverse();
}


inline void PhysicsBodySim::step(float dt) {
  if (!(dt > 0.0f) || !isfinite(dt))
    return;

  // Translation: world-frame force = R(q) * body force + world force.
  Math::Quat_F attQuat = attitudeState_.block<4, 1>(3, 0);
  const Math::Vector<float, 3> forceWorld =
      attQuat.rotate(forceSumBody_) + forceSumNav_;
  for (size_t i = 0; i < 3; i++) {
    positionState_(i) += forceWorld(i) / mass_ * dt; // v first
    positionState_(i + 3) += positionState_(i) * dt; // then p with the new v
  }

  // Rotation: I domega/dt = tau - omega x (I omega), body frame. A world-frame torque sum (unused by the
  // simulator) is rotated into the body frame.
  Math::Vector<float, 3> angVel = attitudeState_.block<3, 1>(0, 0);
  const Math::Vector<float, 3> torqueBody =
      torqueSumBody_ + attQuat.conjugate().rotate(torqueSumNav_);
  const Math::Vector<float, 3> angMom = inertiaTensor_ * angVel;
  const Math::Vector<float, 3> angAcc =
      inertiaInverse_ * (torqueBody - angVel.cross(angMom));
  angVel = angVel + angAcc * dt; // omega first

  // q <- q * dq(omega dt) with the new omega (body-frame rate, Hamilton).
  const float rate = angVel.magnitude();
  if (rate > 1e-9f) {
    attQuat = attQuat * Math::Quat_F(angVel / rate, rate * dt);
  }
  attQuat = attQuat.normalize();
  if (attQuat(0) < 0) { // keep the scalar part non-negative (same rotation)
    attQuat = -attQuat;
  }

  attitudeState_.block(angVel, 0, 0);
  attitudeState_.block(attQuat, 3, 0);
}


} } // namespace VCTR::legacy_frozen
#endif
