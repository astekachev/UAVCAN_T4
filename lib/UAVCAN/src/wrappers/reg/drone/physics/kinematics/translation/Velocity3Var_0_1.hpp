/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef ARDUINO_UAVCAN_TYPES_REG_DRONE_KINEMATICS_TRANSLATION_VELOCITY3VARTS_0_1_HPP_
#define ARDUINO_UAVCAN_TYPES_REG_DRONE_KINEMATICS_TRANSLATION_VELOCITY3VARTS_0_1_HPP_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <libcanard/canard.h>

#include <types/reg/drone/physics/kinematics/translation/Velocity3Var_0_1.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace reg {
namespace drone {
namespace physics {
namespace kinematics {
namespace translation {

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

template <CanardPortID ID>
class Velocity3Var_0_1
{

public:

  reg_drone_physics_kinematics_translation_Velocity3Var_0_1 data;

  static constexpr CanardPortID       PORT_ID          = ID;
  static constexpr size_t             MAX_PAYLOAD_SIZE = reg_drone_physics_kinematics_translation_Velocity3Var_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
  static constexpr CanardTransferKind TRANSFER_KIND    = CanardTransferKindMessage;


  Velocity3Var_0_1()
  {
    reg_drone_physics_kinematics_translation_Velocity3Var_0_1_initialize_(&data);
  }

  Velocity3Var_0_1(Velocity3Var_0_1 const & other)
  {
    memcpy(&data, &other.data, sizeof(data));
  }

  static Velocity3Var_0_1 deserialize(CanardTransfer const & transfer)
  {
    Velocity3Var_0_1 b;
    size_t inout_buffer_size_bytes = transfer.payload_size;
    reg_drone_physics_kinematics_translation_Velocity3Var_0_1_deserialize_(&b.data, (uint8_t *)(transfer.payload), &inout_buffer_size_bytes);
    return b;
  }

  size_t serialize(uint8_t * payload) const
  {
    size_t inout_buffer_size_bytes = Velocity3Var_0_1::MAX_PAYLOAD_SIZE;
    return (reg_drone_physics_kinematics_translation_Velocity3Var_0_1_serialize_(&data, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS) ? 0 : inout_buffer_size_bytes;
  }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* translation */
} /* kinematics */
} /* physics */
} /* drone */
} /* reg */

#endif /* ARDUINO_UAVCAN_TYPES_REG_DRONE_KINEMATICS_TRANSLATION_VELOCITY3VARTS_0_1_HPP_ */
