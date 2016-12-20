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

//----------------------------------- EIGEN traits -------------------------------------//

//! isEigenQuaternion false type
template<typename>
struct is_eigen_quaternion : std::false_type {};

//! isEigenQuaternion true type
template<typename PrimType_>
struct is_eigen_quaternion<Eigen::Quaternion<PrimType_>> : std::true_type {};

//! isEigenAngleAxis false type
template<typename>
struct is_eigen_angle_axis : std::false_type {};

//! isEigenAngleAxis true type
template<typename PrimType_>
struct is_eigen_angle_axis<Eigen::AngleAxis<PrimType_>> : std::true_type {};

//! isEigenVector3 false type
template<typename>
struct is_eigen_vector3 : std::false_type {};

//! isEigenVector3 true type
template<typename PrimType_>
struct is_eigen_vector3<Eigen::Matrix<PrimType_, 3, 1>> : std::true_type {};

//! isEigenMatrix false type
template<typename ValueType_, typename Enable_ = void>
struct is_eigen_matrix : std::false_type {};

//! isEigenMatrix true type
template<typename ValueType_>
struct is_eigen_matrix<ValueType_, typename std::enable_if< std::is_base_of< Eigen::MatrixBase<ValueType_>,
                                                            ValueType_ >::value>::type > : std::true_type {};

//! isEigenMartix (All matrices/vectors except Eigen::Vector3 types) false type
template<typename ValueType_, typename Enable_ = void>
struct is_eigen_matrix_excluding_vector3 : std::false_type {};

//! isEigenMartix (All matrices/vectors except Eigen::Vector3 types) true type
template<typename ValueType_>
struct is_eigen_matrix_excluding_vector3<ValueType_, typename std::enable_if< is_eigen_matrix<ValueType_>::value
															&& !is_eigen_vector3<ValueType_>::value >::type > : std::true_type {};

//! isEigenOfScalarMartix (All matrices/vectors except Eigen::Vector3 types) false type
template<typename ValueType_, typename PrimType_, typename Enable_ = void>
struct is_eigen_matrix_of_scalar_excluding_vector3 : std::false_type {};

//! isEigenOfScalarMartix (All matrices/vectors except Eigen::Vector3 types) true type
template<typename ValueType_, typename PrimType_>
struct is_eigen_matrix_of_scalar_excluding_vector3<ValueType_, PrimType_, typename std::enable_if< is_eigen_matrix_excluding_vector3<ValueType_>::value &&
    std::is_same< typename ValueType_::Scalar, PrimType_ >::value>::type> : std::true_type {};

//-------------------------------------------------------------------------------------//


//----------------------------------- KINDR traits -------------------------------------//
#ifdef SILO_USE_KINDR

//! isKindrVector false type
template<typename>
struct is_kindr_vector : std::false_type {};

//! isKindrVector true type
template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
struct is_kindr_vector<kindr::Vector<PhysicalType_,PrimType_, Dimension_>> : std::true_type {};

//! isKindrVectorAtPosition false type
template<typename>
struct is_kindr_vector_at_position : std::false_type {};

//! isKindrVectorAtPosition true type
template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
struct is_kindr_vector_at_position<signal_logger::KindrVectorAtPosition<kindr::Vector<PhysicalType_,PrimType_, Dimension_>>> : std::true_type {};

#endif
//-------------------------------------------------------------------------------------//

} // end namespace traits

} // end namespace signal_logger
