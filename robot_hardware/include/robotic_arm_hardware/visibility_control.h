#ifndef ROBOTIC_ARM__VISIBILITY_CONTROL_H_
#define ROBOTIC_ARM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROBOTIC_ARM_EXPORT __attribute__((dllexport))
#define ROBOTIC_ARM_IMPORT __attribute__((dllimport))
#else
#define ROBOTIC_ARM_EXPORT __declspec(dllexport)
#define ROBOTIC_ARM_IMPORT __declspec(dllimport)
#endif
#ifdef ROBOTIC_ARM_BUILDING_DLL
#define ROBOTIC_ARM_PUBLIC ROBOTIC_ARM_EXPORT
#else
#define ROBOTIC_ARM_PUBLIC ROBOTIC_ARM_IMPORT
#endif
#define ROBOTIC_ARM_PUBLIC_TYPE ROBOTIC_ARM_PUBLIC
#define ROBOTIC_ARM_LOCAL
#else
#define ROBOTIC_ARM_EXPORT __attribute__((visibility("default")))
#define ROBOTIC_ARM_IMPORT
#if __GNUC__ >= 4
#define ROBOTIC_ARM_PUBLIC __attribute__((visibility("default")))
#define ROBOTIC_ARM_LOCAL __attribute__((visibility("hidden")))
#else
#define ROBOTIC_ARM_PUBLIC
#define ROBOTIC_ARM_LOCAL
#endif
#define ROBOTIC_ARM_PUBLIC_TYPE
#endif

#endif  // ROBOTIC_ARM__VISIBILITY_CONTROL_H_