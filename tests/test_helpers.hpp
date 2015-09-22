inline void printArray(std::valarray<float> a) {
  for (auto each:a)
    std::cout << each << ", ";

  std::cout<<std::endl;
}

template<typename T, typename V>
bool eachInArrayIsApprox(T array, V expectedValue, V absoluteError) {
  return std::all_of(
          std::begin(array), std::end(array), [expectedValue, absoluteError]
          (auto val) {
      return expectedValue-absoluteError <= std::abs(val) && std::abs(val) <=
                                                                  expectedValue+absoluteError;
  });
}