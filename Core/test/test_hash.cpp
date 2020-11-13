#include <functional>
#include <iostream>
#include <string>
#include <string_view>

using namespace std;

auto test_hash_0(void) -> void {
	cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << ' ' << sizeof(size_t) << endl;
	
	std::string s = "Stand back! I've got jimmies!";
  std::cout << std::hash<std::string>{}(s) << '\n';
		
	s = "Another String";
  std::cout << std::hash<std::string>{}(s) << '\n';
}
