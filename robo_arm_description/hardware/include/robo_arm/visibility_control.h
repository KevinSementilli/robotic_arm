#ifndef robo_arm__VISIBILITY_CONTROL_H_
#define robo_arm__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define robo_arm_EXPORT __attribute__((dllexport))
#define robo_arm_IMPORT __attribute__((dllimport))
#else
#define robo_arm_EXPORT __declspec(dllexport)
#define robo_arm_IMPORT __declspec(dllimport)
#endif
#ifdef robo_arm_BUILDING_DLL
#define robo_arm_PUBLIC robo_arm_EXPORT
#else
#define robo_arm_PUBLIC robo_arm_IMPORT
#endif
#define robo_arm_PUBLIC_TYPE robo_arm_PUBLIC
#define robo_arm_LOCAL
#else
#define robo_arm_EXPORT __attribute__((visibility("default")))
#define robo_arm_IMPORT
#if __GNUC__ >= 4
#define robo_arm_PUBLIC __attribute__((visibility("default")))
#define robo_arm_LOCAL __attribute__((visibility("hidden")))
#else
#define robo_arm_PUBLIC
#define robo_arm_LOCAL
#endif
#define robo_arm_PUBLIC_TYPE
#endif

#endif  // robo_arm__VISIBILITY_CONTROL_H_