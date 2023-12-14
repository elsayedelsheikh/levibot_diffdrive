/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef LEVIBOT__VISIBILITY_CONTROL_H_
#define LEVIBOT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define LEVIBOT_EXPORT __attribute__((dllexport))
#define LEVIBOT_IMPORT __attribute__((dllimport))
#else
#define LEVIBOT_EXPORT __declspec(dllexport)
#define LEVIBOT_IMPORT __declspec(dllimport)
#endif
#ifdef LEVIBOT_BUILDING_DLL
#define LEVIBOT_PUBLIC LEVIBOT_EXPORT
#else
#define LEVIBOT_PUBLIC LEVIBOT_IMPORT
#endif
#define LEVIBOT_PUBLIC_TYPE LEVIBOT_PUBLIC
#define LEVIBOT_LOCAL
#else
#define LEVIBOT_EXPORT __attribute__((visibility("default")))
#define LEVIBOT_IMPORT
#if __GNUC__ >= 4
#define LEVIBOT_PUBLIC __attribute__((visibility("default")))
#define LEVIBOT_LOCAL __attribute__((visibility("hidden")))
#else
#define LEVIBOT_PUBLIC
#define LEVIBOT_LOCAL
#endif
#define LEVIBOT_PUBLIC_TYPE
#endif

#endif  // LEVIBOT__VISIBILITY_CONTROL_H_
