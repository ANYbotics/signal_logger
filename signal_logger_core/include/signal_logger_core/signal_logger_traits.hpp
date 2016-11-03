#pragma once

// signal logger
#include "signal_logger_core/LogElementTypes.hpp"

// kindr
#ifdef SILO_USE_KINDR
#include <kindr/Core>
#endif

// Eigen
#include <Eigen/Core>

// STL
#include <type_traits>

namespace signal_logger {

namespace traits {

// Eigen type traits
template<typename>
struct is_eigen_quaternion : std::false_type {};

template<typename PrimType_>
struct is_eigen_quaternion<Eigen::Quaternion<PrimType_>> : std::true_type {};

template<typename>
struct is_eigen_angle_axis : std::false_type {};

template<typename PrimType_>
struct is_eigen_angle_axis<Eigen::AngleAxis<PrimType_>> : std::true_type {};

template<typename>
struct is_eigen_vector3 : std::false_type {};

template<typename PrimType_>
struct is_eigen_vector3<Eigen::Matrix<PrimType_, 3, 1>> : std::true_type {};

// This excludes eigen vector 3 (They might be treated differently)
template<typename ValueType_, typename Enable_ = void>
struct is_eigen_matrix : std::false_type {};

template<typename ValueType_>
struct is_eigen_matrix<ValueType_, typename std::enable_if< std::is_base_of< Eigen::MatrixBase<ValueType_>, ValueType_ >::value
															&& !is_eigen_vector3<ValueType_>::value >::type > : std::true_type {};

// This excludes eigen vector 3 (They might be treated differently)
template<typename ValueType_, typename PrimType_, typename Enable_ = void>
struct is_eigen_matrix_of_scalar : std::false_type {};

template<typename ValueType_, typename PrimType_>
struct is_eigen_matrix_of_scalar<ValueType_, PrimType_, typename std::enable_if<
	std::is_base_of< Eigen::MatrixBase<ValueType_>, ValueType_ >::value && std::is_same<
	typename ValueType_::Scalar, PrimType_ >::value && !is_eigen_vector3<ValueType_>::value >::type > : std::true_type {};

// Kindr type traits
#ifdef SILO_USE_KINDR

template<typename>
struct is_kindr_vector3 : std::false_type {};

template<enum kindr::PhysicalType PhysicalType_, typename PrimType_>
struct is_kindr_vector3<kindr::Vector<PhysicalType_,PrimType_, 3>> : std::true_type {};

// This excludes kindr vectors with size 3 (They might be treated differently)
template<typename ValueType_, typename Enable_ = void>
struct is_kindr_vector : std::false_type {};

template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
struct is_kindr_vector<kindr::Vector<PhysicalType_,PrimType_, Dimension_>,
  typename std::enable_if<Dimension_ != 3>::type > : std::true_type {};

template<typename>
struct is_kindr_vector_at_position : std::false_type {};

template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
struct is_kindr_vector_at_position<signal_logger::KindrVectorAtPosition<kindr::Vector<PhysicalType_,PrimType_, Dimension_>>> : std::true_type {};

#endif

}

}
