/*!
 * @file     signal_logger_std_traits.hpp
 * @author   Gabriel Hottiger, Christian Gehring
 * @date     Sep 23, 2016
 * @brief    Write to binary file traits for all supported types.
 */

#pragma once
#include <signal_logger_core/signal_logger_traits.hpp>
#include <signal_logger_core/LogElementTypes.hpp>
#include <signal_logger_std/LogElementStd.hpp>
#include <fstream>
#include <typeinfo>
#include <message_logger/message_logger.hpp>

namespace signal_logger_std {

namespace traits {

using namespace signal_logger::traits;

// unknown types (such as long double)
template<typename ValueType_, typename Enable_ = void>
struct sls_typename_traits
{
  static std::string getTypeName() {
    return std::string("unknown");
  }
};

// double
template<typename ValueType_>
struct sls_typename_traits<ValueType_, typename std::enable_if<std::is_same<double, typename std::remove_cv<ValueType_>::type>::value>::type>
{
  static std::string getTypeName() {
    return std::string("double");
  }
};

// float
template<typename ValueType_>
struct sls_typename_traits<ValueType_, typename std::enable_if<std::is_same<float, typename std::remove_cv<ValueType_>::type>::value>::type>
{
  static std::string getTypeName() {
    return std::string("single");
  }
};

// integral unsigned types
template<typename ValueType_>
struct sls_typename_traits<ValueType_, typename std::enable_if<std::is_unsigned<ValueType_>::value && std::is_integral<ValueType_>::value>::type>
{
  static std::string getTypeName() {
    return(std::string("uint") + std::to_string(8 * sizeof(ValueType_)));
  }
};

// integral signed types
template<typename ValueType_>
struct sls_typename_traits<ValueType_, typename std::enable_if<std::is_signed<ValueType_>::value && std::is_integral<ValueType_>::value>::type>
{
  static std::string getTypeName() {
    return(std::string("int") + std::to_string(8 * sizeof(ValueType_)));
  }
};

// generic interface
template<typename ValueType_, typename ContainerType_, typename Enable_ = void> struct sls_traits;

/*******************************
 * Specializations: core types *
 *******************************/
template<typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<std::is_arithmetic<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_ * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {
    (*header) << name     << " " << sizeof(ValueType_) << " " << buffer.noTotalItems() << " "
              << divider  << " " << isBufferLooping    << " " << sls_typename_traits<ValueType_>::getTypeName() << std::endl;
    for (unsigned int i = 0; i<buffer.noTotalItems(); ++i)
    {
      binary->write(reinterpret_cast<const char*>(accessor(buffer.getPointerAtPosition((buffer.noTotalItems() - 1) - i))), sizeof(ValueType_) );
    }
  }
};
/********************************/

/*******************************
 * Specializations: enum types *
 *******************************/
template<typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<std::is_enum<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_ * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {
    // Lambda accessing the underlying type #TODO reinterpret_cast is kind of a hack
    auto getUnderlyingType = [accessor](const ContainerType_ * const v) { return reinterpret_cast<const typename std::underlying_type<ValueType_>::type *>(accessor(v)); };
    sls_traits<typename std::underlying_type<ValueType_>::type, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name, divider, isBufferLooping, getUnderlyingType);
  }
};
/********************************/

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <typename ContainerType_>
struct sls_traits<signal_logger::TimestampPair, ContainerType_>
{
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const signal_logger::TimestampPair * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })

  {
    // Get seconds from timestamp pair
    auto getSeconds = [accessor](const ContainerType_ * const v) { return &(accessor(v)->first); };
    sls_traits<typename signal_logger::TimestampPair::first_type, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name + "_s", divider, isBufferLooping, getSeconds);

    // Get nanoseconds from timestamp pair
    auto getNanoSeconds = [accessor](const ContainerType_ * const v) { return &(accessor(v)->second); };
    sls_traits<typename signal_logger::TimestampPair::second_type, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name + "_ns", divider, isBufferLooping, getNanoSeconds);
  }
};
/********************************/

/********************************
 * Specializations: eigen types *
 ********************************/
template<typename ContainerType_>
struct sls_traits<Eigen::Vector3d, ContainerType_>
{
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const Eigen::Vector3d *(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {
    // Suffix for vector name
    std::vector<std::string> suffix = {"_x", "_y", "_z"};

    // Loop through entries
    for (int r = 0; r < 3; ++r)
    {
      // Get xyz of the vector
      auto getXYZ = [r, accessor](const ContainerType_ * const v) { return &((*accessor(v))(r)); };
      sls_traits<double, ContainerType_>::writeLogElementToStreams(
          header, binary, buffer, name + suffix.at(r), divider, isBufferLooping, getXYZ);
    }

  }
};

template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<is_eigen_quaternion<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_ * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {
    // Get wxyz of the vector
    std::vector<std::string> suffix = {"_x", "_y", "_z", "_w"};
    for (int r = 0; r < 4; ++r)  {
      auto getXYZW = [r, accessor](const ContainerType_ * const v) { const typename ValueType_::Scalar & val = accessor(v)->coeffs()(r); return &val; };
      sls_traits<typename ValueType_::Scalar, ContainerType_>::writeLogElementToStreams(
          header, binary, buffer, name + suffix.at(r), divider, isBufferLooping, getXYZW);
    }
  }
};

template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_ * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {
    // Get angle of angle-axis
    auto getAngle = [accessor](const ContainerType_ * const v) { const typename ValueType_::Scalar & val = accessor(v)->angle();  return &val; };
    sls_traits<typename ValueType_::Scalar, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name + "_angle", divider, isBufferLooping, getAngle);

    // Get xyz of the vector
    std::vector<std::string> suffix = {"_x", "_y", "_z"};
    for (int r = 0; r < 3; ++r)  {
      auto getXYZ = [r, accessor](const ContainerType_ * const v) { return &(accessor(v)->axis()(r)); };
      sls_traits<typename ValueType_::Scalar, ContainerType_>::writeLogElementToStreams(
          header, binary, buffer, name + suffix.at(r), divider, isBufferLooping, getXYZ);
    }

  }
};

template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if< is_eigen_matrix_excluding_vector3<ValueType_>::value >::type>
{
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_ * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {
    for (int r = 0; r < accessor(buffer.getPointerAtPosition(0))->rows(); ++r)
    {
      for (int c = 0; c < accessor(buffer.getPointerAtPosition(0))->cols(); ++c)
      {
        std::string nameWithSuffix = name;
        if(accessor(buffer.getPointerAtPosition(0))->rows() > 1) nameWithSuffix += ("_" + std::to_string(r));
        if(accessor(buffer.getPointerAtPosition(0))->cols() > 1) nameWithSuffix += ("_" + std::to_string(c));
        // Get matrix entry
        auto getMatrixEntryRC = [r, c, accessor](const ContainerType_ * const v) { return &((*accessor(v))(r,c)); };
        sls_traits<typename ValueType_::Scalar, ContainerType_>::writeLogElementToStreams(
            header, binary, buffer, nameWithSuffix, divider, isBufferLooping, getMatrixEntryRC);
      }
    }
  }
};
/********************************/

#ifdef SILO_USE_KINDR
/********************************
 * Specializations: kindr types *
 ********************************/
template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if< std::is_base_of<kindr::RotationBase<ValueType_>,ValueType_>::value ||
  std::is_base_of<kindr::RotationDiffBase<ValueType_>,ValueType_>::value || is_kindr_vector<ValueType_>::value >::type>
{
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_ * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {
    // Get underlying eigen type
    auto getEigenType = [accessor](const ContainerType_ * const v) { return &(accessor(v)->toImplementation()); };
    sls_traits<typename ValueType_::Implementation, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name, divider, isBufferLooping, getEigenType);
  }
};


template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<is_kindr_homogeneous_transformation<ValueType_>::value>::type> {
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_ * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {

    // Get underlying position type
    auto getPositionType = [accessor](const ContainerType_ * const v) { return &(accessor(v)->getPosition()); };
    sls_traits<typename ValueType_::Position, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name + "_position", divider, isBufferLooping, getPositionType);

    // Get underlying rotation type
    auto getOrientationType = [accessor](const ContainerType_ * const v) { return &(accessor(v)->getRotation()); };
    sls_traits<typename ValueType_::Rotation, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name + "_orientation", divider, isBufferLooping, getOrientationType);
  }
};

template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<std::is_base_of<kindr::Twist<typename ValueType_::Scalar,typename ValueType_::PositionDiff,typename ValueType_::RotationDiff>,ValueType_>::value>::type> {
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_ * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {

    // Get underlying translational type
    auto getPositionDiffType = [accessor](const ContainerType_ * const v) { return &(accessor(v)->getTranslationalVelocity()); };
    sls_traits<typename ValueType_::PositionDiff, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name + "_linear", divider, isBufferLooping, getPositionDiffType);

    // Get underlying rotational type
    auto getRotationDiffType = [accessor](const ContainerType_ * const v) { return &(accessor(v)->getRotationalVelocity()); };
    sls_traits<typename ValueType_::RotationDiff, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name + "_angular", divider, isBufferLooping, getRotationDiffType);
  }
};

/********************************/

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  static void writeLogElementToStreams(std::stringstream* header,
                                       std::stringstream* binary,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_ * const(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {
    // Get kindr vector
    std::string vectorFrame = buffer.noTotalItems() ? accessor(buffer.getPointerAtPosition(0))->vectorFrame : "unknown";
    auto getKindrVector = [accessor](const ContainerType_ * const v) { return &(accessor(v)->vector); };
    sls_traits<typename ValueType_::VectorType, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name + "_vector_in_" + vectorFrame + "_frame", divider, isBufferLooping, getKindrVector);

    std::string positionFrame = buffer.noTotalItems() ? accessor(buffer.getPointerAtPosition(0))->positionFrame : "unknown";
    auto getKindrPosition = [accessor](const ContainerType_ * const v) { return &(accessor(v)->position); };
    sls_traits<typename signal_logger::KindrPositionD, ContainerType_>::writeLogElementToStreams(
        header, binary, buffer, name + "_at_position_in_" + positionFrame + "_frame", divider, isBufferLooping, getKindrPosition);
  }
};




#endif

} /* namespace traits */

} /* namespace signal_logger_std */

#ifdef SILO_STD_TRAITS_PLUGIN
#include SILO_STD_TRAITS_PLUGIN
#endif
