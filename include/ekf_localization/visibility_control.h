#ifndef EKF_LOCALIZATION__VISIBILITY_CONTROL_H_
#define EKF_LOCALIZATION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EKF_LOCALIZATION_EXPORT __attribute__ ((dllexport))
    #define EKF_LOCALIZATION_IMPORT __attribute__ ((dllimport))
  #else
    #define EKF_LOCALIZATION_EXPORT __declspec(dllexport)
    #define EKF_LOCALIZATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef EKF_LOCALIZATION_BUILDING_LIBRARY
    #define EKF_LOCALIZATION_PUBLIC EKF_LOCALIZATION_EXPORT
  #else
    #define EKF_LOCALIZATION_PUBLIC EKF_LOCALIZATION_IMPORT
  #endif
  #define EKF_LOCALIZATION_PUBLIC_TYPE EKF_LOCALIZATION_PUBLIC
  #define EKF_LOCALIZATION_LOCAL
#else
  #define EKF_LOCALIZATION_EXPORT __attribute__ ((visibility("default")))
  #define EKF_LOCALIZATION_IMPORT
  #if __GNUC__ >= 4
    #define EKF_LOCALIZATION_PUBLIC __attribute__ ((visibility("default")))
    #define EKF_LOCALIZATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EKF_LOCALIZATION_PUBLIC
    #define EKF_LOCALIZATION_LOCAL
  #endif
  #define EKF_LOCALIZATION_PUBLIC_TYPE
#endif

#endif  // EKF_LOCALIZATION__VISIBILITY_CONTROL_H_
