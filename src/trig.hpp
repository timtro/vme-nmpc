
template <typename T>
constexpr T pi{std::acos(T(-1.0))};

template<typename T>
constexpr T pi_180{T(pi<T>/180)};

template<typename T>
T degToRad(T degs) {
  return degs * pi_180<T>;
}

template<typename T>
T radToDeg(T rads) {
  return rads / pi_180<T>;
}