// declare bit operations on given enum.

// credit goes to emsr: http://stackoverflow.com/questions/12059774/c11-standard-conformant-bitmasks-using-enum-class
#ifndef ENUM_BIT_OPERATIONS

#define ENUM_BIT_OPERATIONS(T)                                                                                         \
  inline constexpr T operator&(T __x, T __y) { return static_cast<T>(static_cast<int>(__x) & static_cast<int>(__y)); } \
  inline constexpr T operator|(T __x, T __y) { return static_cast<T>(static_cast<int>(__x) | static_cast<int>(__y)); } \
  inline constexpr T operator^(T __x, T __y) {                                                                         \
    return static_cast<T>(static_cast<int>(__x) ^ static_cast<int>(__y));                                              \
  } inline constexpr T                                                                                                 \
  operator~(T __x) {                                                                                                   \
    return static_cast<T>(~static_cast<int>(__x));                                                                     \
  }                                                                                                                    \
  inline T& operator&=(T& __x, T __y) {                                                                                \
    __x = __x & __y;                                                                                                   \
    return __x;                                                                                                        \
  }                                                                                                                    \
  inline T& operator|=(T& __x, T __y)                                                                                  \
                                                                                                                       \
  {                                                                                                                    \
    __x = __x | __y;                                                                                                   \
    return __x;                                                                                                        \
  }                                                                                                                    \
  inline T& operator^=(T& __x, T __y) {                                                                                \
    __x = __x ^ __y;                                                                                                   \
    return __x;                                                                                                        \
  }

#endif
