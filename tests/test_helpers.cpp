void printArray(std::valarray<float> a) {
  for (auto each:a)
    std::cout << each << ", ";

  std::cout<<std::endl;
}